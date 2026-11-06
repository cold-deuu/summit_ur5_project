import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

spec = importlib.util.spec_from_file_location('ft_gravity', Path(__file__).resolve().parents[1]/'ft_gravity.py')
gravity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gravity)


class GravityTests(unittest.TestCase):
    def dataset(self, mass=1.4, noise=0):
        truth = dict(effective_mass_kg=mass, first_moment_kg_m=[.08, -.025, .17],
                     bias=[1.2, -.4, 2.1, .02, -.06, .1])
        rng = np.random.default_rng(7)
        poses = []
        for i in range(12):
            g = rng.normal(size=3); g *= 9.80665/np.linalg.norm(g)
            observed = gravity.predict(truth, g)+rng.normal(size=6)*noise
            poses.append(dict(label=str(i), gravity_wrist=g.tolist(), wrench=observed.tolist()))
        return dict(base_frame='ur5e_base_link', wrench_frame='ur5e_wrist_3_link',
                    gravity_base=[0, 0, -9.80665], poses=poses), truth

    def test_recovers_mass_first_moment_and_bias_with_holdout(self):
        dataset, truth = self.dataset(noise=.001)
        model = gravity.fit(dataset, 1, .08)
        self.assertAlmostEqual(model['effective_mass_kg'], truth['effective_mass_kg'], places=3)
        np.testing.assert_allclose(model['first_moment_kg_m'], truth['first_moment_kg_m'], atol=.001)
        np.testing.assert_allclose(model['bias'], truth['bias'], atol=.002)
        self.assertTrue(model['validation']['accepted'])

    def test_signed_residual_does_not_assume_driver_uncompensated(self):
        dataset, _ = self.dataset(mass=-.4)
        model = gravity.fit(dataset, 1, .08)
        self.assertAlmostEqual(model['effective_mass_kg'], -.4)

    def test_external_contact_survives_compensation_at_unseen_orientation(self):
        dataset, truth = self.dataset()
        model = gravity.fit(dataset, 1, .08)
        g = np.array([2., -3., -9.]); g *= 9.80665/np.linalg.norm(g)
        contact = np.array([4., -2., 3., .2, .3, -.1])
        observed = gravity.predict(truth, g)+contact
        np.testing.assert_allclose(observed-gravity.predict(model, g), contact, atol=1e-10)

    def test_rejects_repeated_orientation(self):
        dataset, _ = self.dataset()
        dataset['poses'] = [dataset['poses'][0]]*8
        with self.assertRaisesRegex(ValueError, 'orientation diversity'):
            gravity.fit(dataset, 1, .08)

    def test_rejects_contact_contaminated_dataset(self):
        dataset, _ = self.dataset()
        dataset['poses'][2]['wrench'][0] += 25
        dataset['poses'][2]['wrench'][4] += 2
        with self.assertRaisesRegex(ValueError, 'Held-out residual'):
            gravity.fit(dataset, 1, .08)

    def test_configuration_rejects_unvalidated_or_nonfinite_model(self):
        dataset, _ = self.dataset()
        model = gravity.fit(dataset, 1, .08)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'model.yaml'
            path.write_text(yaml.safe_dump(model))
            gravity.load_model(path)
            model['effective_mass_kg'] = float('nan')
            path.write_text(yaml.safe_dump(model))
            with self.assertRaisesRegex(ValueError, 'Nonfinite'):
                gravity.load_model(path)
            model['validation']['accepted'] = False
            path.write_text(yaml.safe_dump(model))
            with self.assertRaisesRegex(ValueError, 'not passed'):
                gravity.load_model(path)


if __name__ == '__main__':
    unittest.main()
