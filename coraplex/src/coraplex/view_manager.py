"""Minimal ViewManager — delegates to Arm.end_effector (manipulation/assembly stub)."""
from __future__ import annotations


class ViewManager:
    def get_end_effector_view(self, arm, robot=None):
        return arm.end_effector
