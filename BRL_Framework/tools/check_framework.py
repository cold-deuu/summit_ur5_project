#!/usr/bin/env python3
"""Verify the 01 source snapshot and 03 offline replay contracts without ROS."""

import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'brl_offline_plan', ROOT / 'src/task_planner/src/task_planner/offline_plan.py')
plans = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = plans
spec.loader.exec_module(plans)


def check():
    for path in (ROOT / 'src').rglob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
    for path in (ROOT / 'src').rglob('package.xml'):
        ET.parse(path)
    ET.parse(ROOT / 'src/task_planner/launch/framework.launch')
    manifest = json.loads((ROOT / 'SOURCE_MANIFEST.json').read_text())
    preserved = 0
    for entry in manifest['files']:
        target = ROOT / entry['target']
        if entry['mode'] == 'byte-identical':
            assert hashlib.sha256(target.read_bytes()).hexdigest() == entry['source_sha256'], target
            preserved += 1
        elif entry.get('target_sha256'):
            assert hashlib.sha256(target.read_bytes()).hexdigest() == entry['target_sha256'], target
    print(f'PASS: syntax and {preserved} byte-identical source files')

    methods = {}
    tree = ast.parse((ROOT / 'src/task_planner/src/task_planner/ros_actions.py').read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            methods[node.name] = hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()
    for entry in manifest['extracted_methods']:
        assert methods[entry['name']] == entry['target_ast_sha256'], entry['name']
        if not entry['adaptation']:
            assert methods[entry['name']] == entry['source_ast_sha256'], entry['name']
    print(f"PASS: {len(manifest['extracted_methods'])} ROS methods extracted from 03")

    action_kinds = set()
    directory = ROOT / 'src/task_planner/templates/tomato/offline_plans'
    assert sorted(path.name for path in directory.iterdir()) == ['T1.yaml']
    for path in sorted(directory.glob('*.yaml')):
        planner = plans.OfflinePlanner(path)
        selected = []
        while True:
            action = planner.select_action(None)
            if action is None:
                break
            selected.append(action.name)
            action_kinds.add(plans.parse_action(action.name).kind)
        assert selected == [entry['name'] for entry in planner.offline_plan], path
        assert planner.select_action(None) is None
        parsed = [plans.parse_action(name) for name in selected]
        for index, action in enumerate(parsed):
            if action.kind == 'pick':
                assert index + 1 < len(parsed) and parsed[index + 1].kind == 'scan', path
                assert action.arguments[:2] == parsed[index + 1].arguments, path
    assert action_kinds == set(plans.ARITY) - {'navigate'}
    assert selected == [
        'prepare_nav(brl_robot)', 'detect(brl_robot, stem_01)',
        'pick(brl_robot, tomato1, stem_01)', 'scan(brl_robot, tomato1)', 'place(brl_robot, tomato1)',
        'pick(brl_robot, tomato2, stem_01)', 'scan(brl_robot, tomato2)', 'discard(brl_robot, tomato2)',
        'prepare_nav(brl_robot)',
    ]
    print('PASS: only T1.yaml remains; prepare_nav, detect, separate pick/scan, place and discard execute in file order')

    invalid = [
        'offline_actions: []',
        'offline_actions: [{index: 1, name: "unknown(brl_robot)"}]',
        'offline_actions: [{index: 1, name: "pick(brl_robot)"}]',
        'offline_actions: [{index: 1, name: "pick_n_scan(brl_robot, tomato1, stem_02)"}]',
        'offline_actions: [{index: 1, name: "scan(brl_robot, tomato1)"}, {index: 1, name: "scan(brl_robot, tomato2)"}]',
        'actions: [pick]',
    ]
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'plan.yaml'
        for contents in invalid:
            path.write_text(contents)
            try:
                plans.OfflinePlanner(path)
            except ValueError:
                continue
            raise AssertionError(f'Invalid plan accepted: {contents}')
        # Non-contiguous/out-of-order indices are metadata, as in 03 T1/T3.
        path.write_text('offline_actions:\n  - {index: 8, name: "scan(brl_robot, tomato1)"}\n'
                        '  - {index: 2, name: "place(brl_robot, tomato1)"}\n')
        planner = plans.OfflinePlanner(path)
        assert planner.select_action().name.startswith('scan(')
        assert planner.select_action().name.startswith('place(')
    print('PASS: invalid plans rejected before execution; index does not reorder steps')

    runtime = (ROOT / 'src/task_planner/src/task_planner/task_planner.py').read_text()
    assert plans.execution_action('navigate(brl_robot, stem_01, dock_station)').name == 'prepare_nav(brl_robot)'
    assert plans.execution_action('prepare_nav(brl_robot)').name == 'prepare_nav(brl_robot)'
    assert 'MoveBaseGoal' not in runtime and 'MoveBaseAction' not in runtime
    assert not (ROOT / 'src/feedback_manager').exists()
    for path in (ROOT / 'src').rglob('*.py'):
        text = path.read_text()
        assert "ServiceProxy('/cf/" not in text and 'call_feedback_manager(' not in text, path
    print('PASS: navigation steps execute prepare_nav; no base navigation or corrective-feedback clients')

    state_package = ROOT / 'src/knowledge_interface'
    assert not (state_package / 'src/knowledge_interface/knowledge_graph').exists()
    assert not (state_package / 'src/knowledge_interface/scene_graph/scene_synchronizer.py').exists()
    assert not (state_package / 'scripts/environment_manager_node.py').exists()
    for path in (ROOT / 'src').rglob('*.py'):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules = [name.name for name in node.names]
            elif isinstance(node, ast.ImportFrom):
                modules = [node.module or '']
            else:
                continue
            assert not any(module.split('.')[0] in {'neo4j', 'dotenv'} for module in modules), path
    requirements = (ROOT / 'requirements.txt').read_text().lower()
    assert 'neo4j' not in requirements and 'python-dotenv' not in requirements
    print('PASS: no database driver, synchronizer, credentials loader, or database CLI')


if __name__ == '__main__':
    check()
