"""Load Unitree G1 humanoid model from MuJoCo Menagerie.

The G1 (29-DOF with hands) is a full-body humanoid with:
- 12 leg joints (6 per leg)
- 3 waist joints (yaw, roll, pitch)
- 14 arm joints (7 per arm: shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)
- 14 hand joints (7 per hand: thumb x3, middle x2, index x2)
- 43 total actuated joints
- freejoint at pelvis (7 DOF: 3 pos + 4 quat)
Total qpos: 50 (7 free + 43 joints)
"""

from pathlib import Path

MENAGERIE_DIR = Path(__file__).parent.parent.parent.parent / "assets" / "menagerie" / "unitree_g1"
G1_MODEL_FILE = MENAGERIE_DIR / "g1_with_hands.xml"
G1_SCENE_FILE = MENAGERIE_DIR / "scene_with_hands.xml"


def get_g1_model_path() -> Path:
    if not G1_MODEL_FILE.exists():
        raise FileNotFoundError(f"G1 model not found at {G1_MODEL_FILE}")
    return G1_MODEL_FILE


def get_g1_scene_path() -> Path:
    if not G1_SCENE_FILE.exists():
        raise FileNotFoundError(f"G1 scene not found at {G1_SCENE_FILE}")
    return G1_SCENE_FILE


def get_g1_asset_dir() -> Path:
    return MENAGERIE_DIR


# Body names from the G1 MJCF for camera attachment and reference
# The head mesh is attached directly to torso_link (no separate head body)
BODY_NAMES = {
    "pelvis": "pelvis",
    "torso": "torso_link",
    "head": "torso_link",  # head geom is on torso_link
    "left_hand": "left_wrist_yaw_link",  # palm geom is here
    "right_hand": "right_wrist_yaw_link",
    "left_foot": "left_ankle_roll_link",
    "right_foot": "right_ankle_roll_link",
}

# Camera mounting configurations for the G1
# Ego cam: mounted on torso_link looking forward-down (where head mesh is)
# Head mesh offset on torso: pos="0.0039635 0 -0.044"
# We place camera slightly forward and up from the head mesh position
CAMERA_CONFIGS = {
    "ego_cam": {
        "parent_body": "torso_link",
        "pos": "0.15 0 0.30",  # forward and above torso center (head height)
        "xyaxes": "0 -1 0 0.3 0 1",  # looking forward and slightly down
        "fovy": "90",
    },
    "wrist_cam_left": {
        "parent_body": "left_wrist_yaw_link",
        "pos": "0.06 0 0",
        "xyaxes": "0 -1 0 0 0 1",
        "fovy": "90",
    },
    "wrist_cam_right": {
        "parent_body": "right_wrist_yaw_link",
        "pos": "0.06 0 0",
        "xyaxes": "0 1 0 0 0 1",
        "fovy": "90",
    },
}

# Joint groups for organizing control
LEG_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]

WAIST_JOINTS = [
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
]

LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
]

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

LEFT_HAND_JOINTS = [
    "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint", "left_hand_middle_1_joint",
    "left_hand_index_0_joint", "left_hand_index_1_joint",
]

RIGHT_HAND_JOINTS = [
    "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
    "right_hand_index_0_joint", "right_hand_index_1_joint",
    "right_hand_middle_0_joint", "right_hand_middle_1_joint",
]

ALL_JOINTS = LEG_JOINTS + WAIST_JOINTS + LEFT_ARM_JOINTS + LEFT_HAND_JOINTS + RIGHT_ARM_JOINTS + RIGHT_HAND_JOINTS

# Actuator ordering matches the MJCF file (same order as ALL_JOINTS above)
# Total: 43 position actuators
NUM_ACTUATORS = 43

# Standing keyframe from the G1 model
STAND_QPOS = [
    # freejoint: x, y, z, qw, qx, qy, qz
    0, 0, 0.79, 1, 0, 0, 0,
    # legs (12)
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    # waist (3)
    0, 0, 0,
    # left arm (7) + left hand (7)
    0.2, 0.2, 0, 1.28, 0, 0, 0,
    0, 1.05, 0, 0, 0, 0, 0,
    # right arm (7) + right hand (7)
    0.2, -0.2, 0, 1.28, 0, 0, 0,
    0, -1.05, 0, 0, 0, 0, 0,
]

STAND_CTRL = [
    # legs (12)
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    # waist (3)
    0, 0, 0,
    # left arm (7) + left hand (7)
    0.2, 0.2, 0, 1.28, 0, 0, 0,
    0, 1.05, 0, 0, 0, 0, 0,
    # right arm (7) + right hand (7)
    0.2, -0.2, 0, 1.28, 0, 0, 0,
    0, -1.05, 0, 0, 0, 0, 0,
]
