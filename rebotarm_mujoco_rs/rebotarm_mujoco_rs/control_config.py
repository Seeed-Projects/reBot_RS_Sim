"""Shared Web / ROS SI-unit defaults; ROS launch parameters can override gains."""
import json
from pathlib import Path

CONTROL_CONFIG = json.loads(Path(__file__).with_name("control_config.json").read_text(encoding="utf-8"))
JOINT_LIMITS = tuple(tuple(pair) for pair in CONTROL_CONFIG["jointLimits"])
