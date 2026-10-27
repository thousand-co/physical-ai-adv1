#!/usr/bin/env python3
"""Reproduce the scene-specific supervised classifier and synthetic validation."""
import sys
from pathlib import Path
root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'ros2_ws/src/final_project_ros'))
from final_project_ros.learned_color import train_model
print(train_model(root / 'ros2_ws/src/final_project_ros/models/color_tree.yml'))
