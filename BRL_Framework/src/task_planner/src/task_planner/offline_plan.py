"""03's offline_actions replay, without online search or feedback dependencies."""

from dataclasses import dataclass
from pathlib import Path
import re
from types import SimpleNamespace

import yaml


ARITY = {"navigate": 3, "detect": 2, "pick": 3,
         "scan": 2, "place": 2, "discard": 2, "prepare_nav": 1}


@dataclass(frozen=True)
class OfflineAction:
    name: str

    @property
    def kind(self):
        return self.name.split("(", 1)[0]

    @property
    def arguments(self):
        return tuple(part.strip() for part in self.name.split("(", 1)[1][:-1].split(","))


def parse_action(name):
    if not isinstance(name, str):
        raise ValueError("Action name must be a string")
    match = re.fullmatch(r"([a-z_]+)\(([^()]*)\)", name.strip())
    if not match or match[1] not in ARITY:
        raise ValueError(f"Unsupported offline action: {name!r}")
    arguments = [part.strip() for part in match[2].split(",")]
    if len(arguments) != ARITY[match[1]] or any(
            not re.fullmatch(r"[A-Za-z0-9_.:/-]+", value) for value in arguments):
        raise ValueError(f"Invalid arguments for {name!r}")
    return OfflineAction(match[1] + "(" + ", ".join(arguments) + ")")


def execution_action(name):
    """Keep the source plan; execute its navigation steps as arm preparation."""
    action = parse_action(name)
    if action.kind == "navigate":
        return parse_action("prepare_nav(" + action.arguments[0] + ")")
    return action


class OfflinePlanner:
    def __init__(self, path):
        self.path = Path(path)
        config = self.load_config(self.path)
        if not isinstance(config, dict) or set(config) != {"offline_actions"}:
            raise ValueError("Expected the 03 format: offline_actions: [{index, name}, ...]")
        self.offline_plan = config["offline_actions"]
        if not isinstance(self.offline_plan, list) or not self.offline_plan:
            raise ValueError("offline_actions must be a nonempty list")
        actions, indices = [], set()
        for entry in self.offline_plan:
            if not isinstance(entry, dict) or set(entry) != {"index", "name"}:
                raise ValueError("Each offline action requires index and name")
            index = entry["index"]
            if type(index) is not int or index < 1 or index in indices:
                raise ValueError(f"Invalid or duplicate action index: {index!r}")
            indices.add(index)
            action = parse_action(entry["name"])
            # Keep source names and list order, as in 03. index is metadata.
            actions.append(OfflineAction(entry["name"]))
        self.env = SimpleNamespace(actions=actions)
        self.is_online = False
        self.step = 0

    def select_action(self, belief=None):
        # Offline branch copied from 03's Planner.select_action.
        if len(self.offline_plan) <= self.step:
            return None
        target_action_name = self.offline_plan[self.step]
        self.step += 1

        for action in self.env.actions:
            if action.name == target_action_name['name']:
                return action

        raise ValueError(f"There is no {target_action_name['name']} in available action")

    @staticmethod
    def load_config(path):
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)
        return cfg
