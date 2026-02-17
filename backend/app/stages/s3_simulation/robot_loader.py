"""Build a 24-DOF dexterous hand on a 7-DOF arm for MuJoCo simulation."""

from lxml import etree


def _build_arm_xml(base_pos: str = "0.0 0 0.5") -> str:
    """Build XML snippet for a 7-DOF position-controlled arm.

    Joints: shoulder_pan, shoulder_lift, shoulder_roll, elbow,
            wrist_roll, wrist_pitch, wrist_yaw
    The arm positions the hand above the workspace.
    """
    return f"""
    <body name="arm_base" pos="{base_pos}">
      <geom name="arm_base_geom" type="cylinder" size="0.04 0.02"
            rgba="0.3 0.3 0.3 1" mass="2.0"/>

      <body name="arm_link1" pos="0 0 0.02">
        <joint name="shoulder_pan" type="hinge" axis="0 0 1"
               range="-3.14 3.14" damping="5.0"/>
        <geom name="arm_link1_geom" type="capsule" fromto="0 0 0 0 0 0.15"
              size="0.03" rgba="0.5 0.5 0.5 1" mass="1.0"/>

        <body name="arm_link2" pos="0 0 0.15">
          <joint name="shoulder_lift" type="hinge" axis="0 1 0"
                 range="-2.0 2.0" damping="5.0"/>
          <geom name="arm_link2_geom" type="capsule" fromto="0 0 0 0 0 0.15"
                size="0.03" rgba="0.5 0.5 0.5 1" mass="1.0"/>

          <body name="arm_link3" pos="0 0 0.15">
            <joint name="shoulder_roll" type="hinge" axis="0 0 1"
                   range="-3.14 3.14" damping="3.0"/>
            <geom name="arm_link3_geom" type="capsule" fromto="0 0 0 0 0 0.12"
                  size="0.025" rgba="0.5 0.5 0.5 1" mass="0.8"/>

            <body name="arm_link4" pos="0 0 0.12">
              <joint name="elbow" type="hinge" axis="0 1 0"
                     range="-2.5 2.5" damping="3.0"/>
              <geom name="arm_link4_geom" type="capsule" fromto="0 0 0 0 0 0.12"
                    size="0.022" rgba="0.5 0.5 0.5 1" mass="0.6"/>

              <body name="arm_link5" pos="0 0 0.12">
                <joint name="wrist_roll" type="hinge" axis="0 0 1"
                       range="-3.14 3.14" damping="1.0"/>
                <geom name="arm_link5_geom" type="capsule" fromto="0 0 0 0 0 0.06"
                      size="0.02" rgba="0.4 0.4 0.4 1" mass="0.4"/>

                <body name="arm_link6" pos="0 0 0.06">
                  <joint name="wrist_pitch" type="hinge" axis="0 1 0"
                         range="-1.57 1.57" damping="1.0"/>
                  <geom name="arm_link6_geom" type="capsule"
                        fromto="0 0 0 0 0 0.04" size="0.018"
                        rgba="0.4 0.4 0.4 1" mass="0.3"/>

                  <body name="end_effector" pos="0 0 0.04">
                    <joint name="wrist_yaw" type="hinge" axis="0 0 1"
                           range="-3.14 3.14" damping="1.0"/>
                    <site name="ee_site" pos="0 0 0" size="0.01"
                          rgba="1 0 0 0.5"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
    """


def _build_arm_actuators_xml() -> str:
    """Build actuator XML for the 7-DOF arm (position control)."""
    joints = [
        ("shoulder_pan", 100),
        ("shoulder_lift", 100),
        ("shoulder_roll", 80),
        ("elbow", 80),
        ("wrist_roll", 40),
        ("wrist_pitch", 40),
        ("wrist_yaw", 40),
    ]
    lines = []
    for name, kp in joints:
        lines.append(
            f'    <position name="act_{name}" joint="{name}" '
            f'kp="{kp}" ctrlrange="-3.14 3.14" forcerange="-50 50"/>'
        )
    return "\n".join(lines)


def load_shadow_hand() -> tuple[str, str, str, dict[str, bytes]]:
    """Build a 24-DOF dexterous hand model and return XML components.

    Returns:
        (hand_body_xml, hand_actuator_xml, hand_asset_xml, mesh_assets_dict)

    The hand is built procedurally with 5 fingers (thumb + 4 fingers),
    each with realistic joint ranges. No external mesh files are needed.
    """
    hand_asset_xml = ""

    # Finger definitions: (name, base_pos, is_thumb)
    fingers = [
        ("thumb", "0.02 -0.01 0.04", True),
        ("index", "0.04 -0.02 0.09", False),
        ("middle", "0.04 0.0 0.09", False),
        ("ring", "0.04 0.02 0.09", False),
        ("pinky", "0.035 0.04 0.085", False),
    ]

    finger_bodies = []
    finger_actuators = []

    for fname, fpos, is_thumb in fingers:
        if is_thumb:
            finger_bodies.append(f"""
        <body name="{fname}_base" pos="{fpos}" euler="0 0.5 -0.5">
          <joint name="{fname}_rotation" type="hinge" axis="0 0 1" range="-1.0 1.0" damping="0.2"/>
          <joint name="{fname}_abduction" type="hinge" axis="0 1 0" range="-0.5 0.5" damping="0.2"/>
          <geom name="{fname}_prox_geom" type="capsule" fromto="0 0 0 0.03 0 0" size="0.01"
                rgba="0.85 0.75 0.65 1" mass="0.02"/>
          <body name="{fname}_middle" pos="0.03 0 0">
            <joint name="{fname}_mcp" type="hinge" axis="0 1 0" range="0 1.57" damping="0.1"/>
            <geom name="{fname}_mid_geom" type="capsule" fromto="0 0 0 0.025 0 0" size="0.009"
                  rgba="0.85 0.75 0.65 1" mass="0.015"/>
            <body name="{fname}_distal" pos="0.025 0 0">
              <joint name="{fname}_pip" type="hinge" axis="0 1 0" range="0 1.3" damping="0.1"/>
              <geom name="{fname}_dist_geom" type="capsule" fromto="0 0 0 0.02 0 0" size="0.008"
                    rgba="0.85 0.75 0.65 1" mass="0.01"/>
              <site name="{fname}_tip" pos="0.02 0 0" size="0.005" rgba="1 0 0 0.5"/>
            </body>
          </body>
        </body>""")
            for jname in [f"{fname}_rotation", f"{fname}_abduction",
                          f"{fname}_mcp", f"{fname}_pip"]:
                finger_actuators.append(
                    f'    <position name="act_{jname}" joint="{jname}" '
                    f'kp="5" ctrlrange="-1.6 1.6" forcerange="-2 2"/>'
                )
        else:
            finger_bodies.append(f"""
        <body name="{fname}_base" pos="{fpos}">
          <joint name="{fname}_abduction" type="hinge" axis="1 0 0" range="-0.3 0.3" damping="0.2"/>
          <geom name="{fname}_prox_geom" type="capsule" fromto="0 0 0 0 0 0.04" size="0.009"
                rgba="0.85 0.75 0.65 1" mass="0.02"/>
          <body name="{fname}_middle" pos="0 0 0.04">
            <joint name="{fname}_mcp" type="hinge" axis="1 0 0" range="0 1.57" damping="0.1"/>
            <geom name="{fname}_mid_geom" type="capsule" fromto="0 0 0 0 0 0.03" size="0.008"
                  rgba="0.85 0.75 0.65 1" mass="0.015"/>
            <body name="{fname}_distal" pos="0 0 0.03">
              <joint name="{fname}_pip" type="hinge" axis="1 0 0" range="0 1.3" damping="0.1"/>
              <geom name="{fname}_dist_geom" type="capsule" fromto="0 0 0 0 0 0.025" size="0.007"
                    rgba="0.85 0.75 0.65 1" mass="0.01"/>
              <body name="{fname}_tip_body" pos="0 0 0.025">
                <joint name="{fname}_dip" type="hinge" axis="1 0 0" range="0 1.0" damping="0.05"/>
                <geom name="{fname}_tip_geom" type="capsule" fromto="0 0 0 0 0 0.02" size="0.006"
                      rgba="0.85 0.75 0.65 1" mass="0.005"/>
                <site name="{fname}_tip" pos="0 0 0.02" size="0.005" rgba="1 0 0 0.5"/>
              </body>
            </body>
          </body>
        </body>""")
            for jname in [f"{fname}_abduction", f"{fname}_mcp",
                          f"{fname}_pip", f"{fname}_dip"]:
                finger_actuators.append(
                    f'    <position name="act_{jname}" joint="{jname}" '
                    f'kp="5" ctrlrange="-1.6 1.6" forcerange="-2 2"/>'
                )

    hand_body_xml = f"""
      <body name="palm" pos="0 0 0" euler="3.14 0 0">
        <geom name="palm_geom" type="box" size="0.04 0.045 0.01"
              pos="0.02 0 0.05" rgba="0.85 0.75 0.65 1" mass="0.3"/>
        {"".join(finger_bodies)}
      </body>
    """

    hand_actuator_xml = "\n".join(finger_actuators)

    return hand_body_xml, hand_actuator_xml, hand_asset_xml, {}


# Arm joint names for external reference
ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "shoulder_roll",
    "elbow",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
]

# Finger joint names
FINGER_JOINT_NAMES = {
    "thumb": ["thumb_rotation", "thumb_abduction", "thumb_mcp", "thumb_pip"],
    "index": ["index_abduction", "index_mcp", "index_pip", "index_dip"],
    "middle": ["middle_abduction", "middle_mcp", "middle_pip", "middle_dip"],
    "ring": ["ring_abduction", "ring_mcp", "ring_pip", "ring_dip"],
    "pinky": ["pinky_abduction", "pinky_mcp", "pinky_pip", "pinky_dip"],
}


def get_arm_xml(base_pos: str = "0.0 0 0.5") -> str:
    """Return the 7-DOF arm body XML."""
    return _build_arm_xml(base_pos=base_pos)


def get_arm_actuators_xml() -> str:
    """Return the arm actuator XML."""
    return _build_arm_actuators_xml()
