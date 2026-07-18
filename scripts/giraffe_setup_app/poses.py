"""ASCII pose guidance for the setup TUI (CLI-only graphics)."""

from __future__ import annotations

# Side view (left = base / clamp, right = gripper), roughly folded home.
HOME_POSE_ART = r"""
  HOME / ZERO POSE  (side view)

        [clamp]
           ||
           ||     shoulder ~ upright / slightly folded
          _||_
         /    \     elbow bent (~90°–folded)
        /      \
       |        `-.
       |           `-o  wrist level, gripper closed or consistent
       |              =
      ===  table

  Top-down (shoulder pan): arm roughly along a consistent forward axis
  you will remember (e.g. straight ahead from the clamp).
"""

HOME_POSE_CHECKLIST = [
    "Support the arm — torque will be OFF (may drop).",
    "Shoulder pan: choose a repeatable forward heading.",
    "Shoulder lift + elbow: folded / mid pose, not against hard stops.",
    "Wrists: neutral / level as best you can.",
    "Gripper: fully closed (or fully open — pick one and stick to it).",
    "Stable on the clamp; nothing in the workspace.",
]
