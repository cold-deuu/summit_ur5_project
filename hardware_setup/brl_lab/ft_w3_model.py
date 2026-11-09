#!/usr/bin/env python3
"""BRL_lab F/T model: tool gravity + wrist-3-angle dependent sensor offset.

The lab UR5e F/T reading shifts ~9 N with the wrist_3 joint angle (same vector at +-90 deg),
which a gravity-only model (ft_gravity.py) cannot fit. Model, in the wrench (wrist) frame:
  F = m*g + b0_F + sum_k (c_k,F*cos(k*q3) + s_k,F*sin(k*q3))
  T = h x g + b0_T + sum_k (c_k,T*cos(k*q3) + s_k,T*sin(k*q3))
g = gravity in the wrench frame, q3 = wrist_3 joint angle. Harmonics K=1..3, chosen by
leave-one-out error (lab data 2026-11-09: K=3 needed; the offset changes sharply between
-90 and -45 deg and repeats every ~180 deg). K=3 needs >= 7 distinct wrist_3 angles. Datasets use the ft_gravity.py / panel format (poses with joints[5] = q3).
"""
import math

import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def _harmonics(q3, order):
    columns = []
    for k in range(1, order + 1):
        columns += [math.cos(k * q3), math.sin(k * q3)]
    return columns


def _design(poses, order):
    force, torque = [], []
    for pose in poses:
        g = np.asarray(pose['gravity_wrist'], dtype=float)
        h = _harmonics(float(pose['joints'][5]), order)
        offset = np.hstack([np.eye(3)] + [value * np.eye(3) for value in h])
        force.append(np.column_stack([g, offset]))
        torque.append(np.column_stack([-skew(g), offset]))
    return np.vstack(force), np.vstack(torque)


def _condition(matrix):
    norms = np.linalg.norm(matrix, axis=0)
    if np.any(norms < 1e-10):
        return float('inf')
    singular = np.linalg.svd(matrix / norms, compute_uv=False)
    return float(singular[0] / max(singular[-1], 1e-16))


def solve(poses, order):
    force, torque = _design(poses, order)
    wrench = np.array([pose['wrench'] for pose in poses], dtype=float)
    conditions = [_condition(force), _condition(torque)]
    if max(conditions) > 1000:
        raise ValueError('Poses do not separate gravity and wrist_3 offset (condition %.0f)' % max(conditions))
    f = np.linalg.lstsq(force, wrench[:, :3].reshape(-1), rcond=None)[0]
    t = np.linalg.lstsq(torque, wrench[:, 3:].reshape(-1), rcond=None)[0]
    harmonics = []
    for k in range(order):
        cos_index, sin_index = 3 + 6 * k, 6 + 6 * k
        harmonics.append(dict(k=k + 1,
                              cos=np.r_[f[1 + cos_index:4 + cos_index], t[3 + cos_index:6 + cos_index]].tolist(),
                              sin=np.r_[f[1 + sin_index:4 + sin_index], t[3 + sin_index:6 + sin_index]].tolist()))
    return dict(effective_mass_kg=float(f[0]), first_moment_kg_m=t[:3].tolist(),
                bias=np.r_[f[1:4], t[3:6]].tolist(), harmonics=harmonics, conditions=conditions)


def predict(model, gravity_wrist, q3):
    g = np.asarray(gravity_wrist, dtype=float)
    value = np.r_[model['effective_mass_kg'] * g, np.cross(np.asarray(model['first_moment_kg_m']), g)]
    value = value + np.asarray(model['bias'], dtype=float)
    for term in model['harmonics']:
        value = value + math.cos(term['k'] * q3) * np.asarray(term['cos']) + math.sin(term['k'] * q3) * np.asarray(term['sin'])
    return value


def residuals(model, poses):
    errors = [np.asarray(p['wrench']) - predict(model, p['gravity_wrist'], p['joints'][5]) for p in poses]
    return [dict(label=p['label'], force_N=float(np.linalg.norm(e[:3])), torque_Nm=float(np.linalg.norm(e[3:])))
            for p, e in zip(poses, errors)]


def leave_one_out(poses, order):
    held = []
    for i, pose in enumerate(poses):
        other = solve(poses[:i] + poses[i + 1:], order)
        held += residuals(other, [pose])
    force = math.sqrt(np.mean([h['force_N'] ** 2 for h in held]))
    torque = math.sqrt(np.mean([h['torque_Nm'] ** 2 for h in held]))
    return dict(force_rms_N=force, torque_rms_Nm=torque, per_pose=held)


def fit(dataset, max_force=1.0, max_torque=0.08):
    poses = dataset['poses']
    angles = {round(math.degrees(p['joints'][5]) / 5) * 5 for p in poses}
    if len(poses) < 12 or len(angles) < 5:
        raise ValueError('Need >= 12 poses incl. the wrist_3 sweep (>= 5 distinct wrist_3 angles); have %d poses, %d angles'
                         % (len(poses), len(angles)))
    candidates = []
    distinct = len(angles)
    for order in (1, 2, 3):
        if 2 * order + 1 > distinct:
            break
        try:
            candidates.append((leave_one_out(poses, order), order))
        except (ValueError, np.linalg.LinAlgError):
            continue
    if not candidates:
        raise ValueError('Model could not be solved; add more distinct orientations')
    report, order = min(candidates, key=lambda c: c[0]['force_rms_N'] + 10 * c[0]['torque_rms_Nm'])
    if report['force_rms_N'] > max_force or report['torque_rms_Nm'] > max_torque:
        worst = max(report['per_pose'], key=lambda p: p['force_N'])
        raise ValueError('Held-out residual too large: %.2f N, %.3f Nm (worst %s %.2f N); check contact or a '
                         'restart of the UR program between captures' % (report['force_rms_N'], report['torque_rms_Nm'],
                                                                         worst['label'], worst['force_N']))
    model = solve(poses, order)
    model.update(schema='brl_lab_ft_w3', order=order, base_frame=dataset['base_frame'],
                 wrench_frame=dataset['wrench_frame'], gravity_base=dataset['gravity_base'],
                 wrist_3_joint='wrist_3_joint', validation=dict(accepted=True, leave_one_out=report),
                 interpretation='Residual tool gravity + wrist_3 dependent offset in the existing wrench stream.')
    mass = model['effective_mass_kg']
    model['effective_cog_m'] = (np.asarray(model['first_moment_kg_m']) / mass).tolist() if abs(mass) > 0.05 else None
    return model


def load(path):
    import yaml
    model = yaml.safe_load(open(path))
    if model.get('schema') != 'brl_lab_ft_w3' or model.get('validation', {}).get('accepted') is not True:
        raise ValueError('Not a validated brl_lab_ft_w3 model: %s' % path)
    return model
