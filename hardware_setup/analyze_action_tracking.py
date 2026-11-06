#!/usr/bin/env python3
"""Read-only analysis of latest recorded SE3 action; never connects to ROS."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    parser=argparse.ArgumentParser();parser.add_argument('path',nargs='?',default=str(Path.home()/'catkin_ws/artifacts/real_action_logs'))
    args=parser.parse_args();source=Path(args.path)
    if source.is_dir():
        if (source/'telemetry.csv').exists():source=source/'telemetry.csv'
        else:source=max(source.glob('*/telemetry.csv'),key=lambda p:p.stat().st_mtime)
    rows=list(csv.DictReader(source.open()));metadata=json.loads((source.parent/'metadata.json').read_text())
    def data(part,keys):return np.array([[float(r[k]) for k in keys] for r in part])
    full_time=data(rows,['time'])[:,0];dt=np.diff(full_time)
    ids=sorted({int(r['action_id']) for r in rows if r['mode']=='SE3' and not r['reason']})
    if not ids:raise RuntimeError('No valid SE3 action found')
    identifier=ids[-1];indices=np.array([i for i,r in enumerate(rows) if r['mode']=='SE3' and int(r['action_id'])==identifier and not r['reason']])
    part=[rows[i] for i in indices];t=data(part,['time'])[:,0];start_stamp=t[0];t-=start_stamp
    actual=data(part,['actual_'+a for a in 'xyz']);ref=data(part,['reference_'+a for a in 'xyz']);nom=data(part,['nominal_'+a for a in 'xyz'])
    pe=data(part,['translation_error_'+str(i) for i in range(3)]);re=data(part,['rotation_error_'+str(i) for i in range(3)])
    q=data(part,['joint_actual_'+str(i) for i in range(6)]);command=data(part,['joint_command_'+str(i) for i in range(6)])
    sample_period=float(np.median(np.diff(t)));integration_period=.01
    request=(command-q)/integration_period
    measured=savgol_filter(q,21,3,deriv=1,delta=sample_period,axis=0)
    delta=nom[-1]-nom[0];distance=float(np.linalg.norm(delta));direction=delta/distance
    nominal_progress=(nom-nom[0])@direction/distance
    actual_progress=(actual-actual[0])@direction/distance
    def crossing(values,level):
        hits=np.flatnonzero(values>=level)
        return float(t[hits[0]]) if len(hits) else None
    crosses=[]
    for level in [.1,.5,.9,.95,.99]:
        n=crossing(nominal_progress,level);a=crossing(actual_progress,level)
        crosses.append(dict(fraction=level,reference_s=n,actual_s=a,delay_s=None if a is None or n is None else a-n))
    moving=nominal_progress<.99999
    nominal_end=float(t[np.flatnonzero(~moving)[0]]) if (~moving).any() else float(t[-1])
    parameters=metadata['parameters'];cap=float(parameters['arm_velocity_limit'])
    decay_mask=(t>nominal_end+.2)&(t<t[-1]-.4)&(np.linalg.norm(pe,axis=1)>.002)
    decay_slope=np.polyfit(t[decay_mask],np.log(np.linalg.norm(pe[decay_mask],axis=1)),1)[0] if decay_mask.sum()>20 else None
    ratios=[]
    for i in range(6):
        mask=(t>.2)&(t<t[-1]-1)&(np.abs(request[:,i])>.03)&(np.abs(request[:,i])<cap-.02)
        ratio=float(np.dot(request[mask,i],measured[mask,i])/np.dot(request[mask,i],request[mask,i])) if mask.sum()>20 else None
        ratios.append(dict(joint_index=i,estimated_rate_ratio=ratio,peak_requested_rad_s=float(np.max(np.abs(request[:,i]))),
                           peak_measured_rad_s=float(np.max(np.abs(measured[:,i]))),saturation_fraction=float(np.mean(np.abs(request[:,i])>=cap-.01))))
    transitions=[]
    for i in range(1,len(rows)):
        if (rows[i]['mode'],rows[i]['action_id'])!=(rows[i-1]['mode'],rows[i-1]['action_id']):
            p0=data([rows[i-1]],['reference_'+a for a in 'xyz'])[0];p1=data([rows[i]],['reference_'+a for a in 'xyz'])[0]
            q0=Rotation.from_quat(data([rows[i-1]],['reference_'+a for a in ['qx','qy','qz','qw']])[0]);q1=Rotation.from_quat(data([rows[i]],['reference_'+a for a in ['qx','qy','qz','qw']])[0])
            transitions.append(dict(time_s=float(full_time[i]-full_time[0]),from_mode=rows[i-1]['mode'],to_mode=rows[i]['mode'],
                position_reference_jump_mm=((p1-p0)*1000).tolist(),rotation_reference_jump_deg=(q1*q0.inv()).as_rotvec().tolist()))
            transitions[-1]['rotation_reference_jump_deg']=(np.array(transitions[-1]['rotation_reference_jump_deg'])*180/np.pi).tolist()
    peak=int(np.argmax(np.linalg.norm(pe,axis=1)))
    report=dict(source=str(source.resolve()),action_id=identifier,log_samples=len(rows),log_duration_s=float(full_time[-1]-full_time[0]),
        dropped=metadata.get('dropped'),dt_min_median_max_s=np.quantile(dt,[0,.5,1]).tolist(),
        action_duration_s=float(t[-1]+sample_period),nominal_duration_s=nominal_end,estimated_post_goal_time_constant_s=None if decay_slope is None or decay_slope>=0 else float(-1/decay_slope),nominal_displacement_m=delta.tolist(),crossings=crosses,
        position_rms_norm_mm=float(np.sqrt(np.mean(np.sum(pe**2,axis=1)))*1000),
        moving_position_rms_norm_mm=float(np.sqrt(np.mean(np.sum(pe[moving]**2,axis=1)))*1000),
        max_position_error_mm=float(np.linalg.norm(pe[peak])*1000),peak_error_time_s=float(t[peak]),
        rotation_rms_norm_deg=float(np.sqrt(np.mean(np.sum(re**2,axis=1)))*180/np.pi),
        final_error_before_hold_mm=(pe[-1]*1000).tolist(),final_error_before_hold_norm_mm=float(np.linalg.norm(pe[-1])*1000),
        requested_to_measured_rate=ratios,mode_transitions=transitions,
        max_wrench_filtered=np.max(np.abs(data(part,['wrench_filtered_'+str(i) for i in range(6)])),axis=0).tolist(),
        max_admittance_displacement=np.max(np.abs(data(part,['admittance_displacement_'+str(i) for i in range(6)])),axis=0).tolist(),
        parameters=parameters,
        limitations=['Rate ratios are offline least-squares estimates from differentiated measured joint positions, not an identified driver transfer function.',
                     'CSV has no historical speed-scaling/controller-desired-state stream; low-level horizon causality cannot be isolated exactly.',
                     'HOLD reinitializes task reference; post-HOLD near-zero error does not mean the prior goal error was zero.'])
    out=source.parent/'analysis';out.mkdir(exist_ok=True);(out/'tracking_analysis.json').write_text(json.dumps(report,indent=2))
    fig,axes=plt.subplots(3,1,figsize=(11,10),sharex=True)
    axes[0].plot(t,nominal_progress*100,label='nominal trajectory');axes[0].plot(t,actual_progress*100,label='actual')
    axes[0].set_ylabel('Path progress [%]');axes[0].legend()
    for i,label in [(0,'X'),(2,'Z')]:axes[1].plot(t,(ref[:,i]-actual[:,i])*1000,label=label)
    axes[1].plot(t,np.linalg.norm(pe,axis=1)*1000,color='black',label='3D norm');axes[1].set_ylabel('Tracking error [mm]');axes[1].legend()
    axes[2].plot(t,np.linalg.norm(re,axis=1)*180/np.pi);axes[2].set_ylabel('Rotation error norm [deg]');axes[2].set_xlabel('Time from SE3 start [s]')
    for ax in axes:ax.grid(True);ax.axvline(nominal_end,color='.5',ls=':',label='nominal end')
    fig.suptitle('Latest SE3 action: reference vs measured (HOLD reset excluded)');fig.tight_layout();fig.savefig(out/'tracking_delay.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    for ax,i,label in zip(axes,[1,2,3],['shoulder lift','elbow','wrist 1']):
        ax.plot(t,request[:,i],label='HQP qdot reconstructed from (q_command-q)/0.01')
        ax.plot(t,measured[:,i],label='measured qdot (0.21 s Savitzky-Golay derivative)')
        ax.set_ylabel(label+' [rad/s]');ax.grid(True);ax.legend(fontsize=8)
    axes[-1].set_xlabel('Time from SE3 start [s]');fig.tight_layout();fig.savefig(out/'requested_vs_measured_speed.png',dpi=170);plt.close(fig)
    boundary=full_time[indices[-1]+1]-full_time[0] if indices[-1]+1<len(rows) else full_time[indices[-1]]-full_time[0]
    ft=full_time-full_time[0];mask=(ft>boundary-.5)&(ft<boundary+.5);part2=[r for r,k in zip(rows,mask) if k];tm=ft[mask]
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    for name in ['nominal','reference','actual']:axes[0].plot(tm,data(part2,[name+'_y'])[:,0]*1e6,label=name)
    axes[0].set_ylabel('Y [micrometre]');axes[0].legend()
    en=data(part2,['translation_error_'+str(i) for i in range(3)]);axes[1].plot(tm,np.linalg.norm(en,axis=1)*1000)
    axes[1].set_ylabel('Reference tracking error [mm]');axes[1].set_xlabel('Log time [s]')
    for ax in axes:ax.axvline(boundary,color='red',ls='--');ax.grid(True)
    fig.suptitle('SE3 -> HOLD: reference is reset to measured posture, not an actual pose step');fig.tight_layout();fig.savefig(out/'hold_reference_reset.png',dpi=170);plt.close(fig)
    print(json.dumps({k:report[k] for k in ['source','position_rms_norm_mm','max_position_error_mm','crossings','final_error_before_hold_norm_mm']},indent=2));print('Saved:',out)

if __name__=='__main__':main()
