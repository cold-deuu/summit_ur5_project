import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


spec = importlib.util.spec_from_file_location(
    'field_semantics', Path(__file__).resolve().parents[1]/'prepare_field_moveit_semantics.py')
semantics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(semantics)


class Master:
    def __init__(self, values):
        self.values = values

    def getParam(self, caller, key):
        return (1, '', self.values[key]) if key in self.values else (-1, 'Parameter not set: '+key, '')


class FieldSemanticTests(unittest.TestCase):
    def setUp(self):
        model = ET.Element('robot', name='live_brl_robot')
        parent = 'ur5e_base_link'
        ET.SubElement(model, 'link', name=parent)
        names = ['ur5e_'+name for name in ['shoulder_pan_joint', 'shoulder_lift_joint',
                 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']]
        for i, name in enumerate(names):
            child = 'ur5e_wrist_3_link' if i == 5 else 'arm_link_'+str(i)
            ET.SubElement(model, 'link', name=child)
            joint = ET.SubElement(model, 'joint', name=name, type='revolute')
            ET.SubElement(joint, 'parent', link=parent)
            ET.SubElement(joint, 'child', link=child)
            parent = child
        ET.SubElement(model, 'link', name='graps_link')
        joint = ET.SubElement(model, 'joint', name='tool_fixed', type='fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link='graps_link')
        self.urdf = ET.tostring(model, encoding='unicode')
        self.srdf = '''<robot name="old_name">
          <group name="manipulator"><chain base_link="ur5e_base_link" tip_link="graps_link"/></group>
          <disable_collisions link1="ur5e_base_link" link2="arm_link_0" reason="Adjacent"/>
          <end_effector name="invalid_self_group" parent_link="graps_link" group="manipulator"/>
        </robot>'''
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.fallback = Path(self.temp.name)/'field.srdf'
        self.fallback.write_text(self.srdf)

    def test_missing_semantic_parameter_uses_validated_fallback(self):
        master = Master({'/brl_robot/robot_description': self.urdf})
        result, used = semantics.from_master(master, '/brl_robot', self.fallback)
        root = ET.fromstring(result)
        self.assertTrue(used)
        self.assertEqual(root.get('name'), 'live_brl_robot')
        self.assertEqual(root.find('group/chain').get('tip_link'), 'graps_link')
        self.assertEqual(root.find('disable_collisions').attrib,
                         ET.fromstring(self.srdf).find('disable_collisions').attrib)
        self.assertFalse(root.findall('end_effector'))

    def test_live_semantics_take_precedence(self):
        live = self.srdf.replace('reason="Adjacent"', 'reason="ExistingFieldSetting"')
        master = Master({'/brl_robot/robot_description': self.urdf,
                         '/brl_robot/robot_description_semantic': live})
        result, used = semantics.from_master(master, '/brl_robot', Path('/nonexistent'))
        self.assertFalse(used)
        self.assertEqual(ET.fromstring(result).find('disable_collisions').get('reason'), 'ExistingFieldSetting')

    def test_missing_live_model_does_not_use_a_stale_model(self):
        with self.assertRaisesRegex(RuntimeError, 'start BRL bringup first'):
            semantics.from_master(Master({}), '/brl_robot', self.fallback)

    def test_rejects_collision_exclusions_for_other_links(self):
        with self.assertRaisesRegex(RuntimeError, 'link missing'):
            semantics.normalize(self.urdf, self.srdf.replace('link2="arm_link_0"', 'link2="other_robot"'))

    def test_rejects_wrong_joint_set(self):
        wrong = self.urdf.replace('ur5e_elbow_joint', 'other_elbow_joint')
        with self.assertRaisesRegex(RuntimeError, 'six BRL ur5e_ joints'):
            semantics.normalize(wrong, self.srdf)

    def test_rejects_disconnected_chain(self):
        broken = self.urdf.replace('link="arm_link_2"', 'link="disconnected"', 1)
        with self.assertRaisesRegex(RuntimeError, 'disconnected'):
            semantics.normalize(broken, self.srdf)


if __name__ == '__main__':
    unittest.main()
