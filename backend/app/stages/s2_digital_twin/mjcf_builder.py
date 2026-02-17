"""Programmatic MJCF XML generation for simulation."""

from lxml import etree

from app.stages.s3_simulation.robot_loader import (
    load_shadow_hand,
    get_arm_xml,
    get_arm_actuators_xml,
)


def build_mjcf(
    model_name: str,
    mass: float,
    inertia: dict,
    contact_params: dict,
    collision_mesh_files: list[str],
    visual_mesh_file: str,
    center_of_mass: list[float],
    environment: str = "open_space",
    object_position: list[float] | None = None,
    robot_position: list[float] | None = None,
) -> tuple[str, dict[str, bytes]]:
    """Build a complete MJCF XML string for the object + environment + robot.

    Returns (mjcf_xml_string, robot_mesh_assets_dict).
    """
    if object_position is None:
        object_position = [0.5, 0, 0.3]
    if robot_position is None:
        robot_position = [0.0, 0, 0.5]

    root = etree.Element("mujoco", model=model_name)

    # Compiler settings
    etree.SubElement(root, "compiler", angle="radian", meshdir="meshes")

    # Options
    etree.SubElement(root, "option", timestep="0.002", gravity="0 0 -9.81")

    # Visual settings
    visual = etree.SubElement(root, "visual")
    etree.SubElement(visual, "global", offwidth="1280", offheight="720")

    # Assets
    asset = etree.SubElement(root, "asset")
    etree.SubElement(asset, "texture", type="2d", name="grid", builtin="checker",
                     rgb1="0.2 0.2 0.2", rgb2="0.3 0.3 0.3", width="512", height="512")
    etree.SubElement(asset, "material", name="grid_mat", texture="grid",
                     texrepeat="8 8", reflectance="0.1")
    etree.SubElement(asset, "material", name="object_mat",
                     rgba="0.4 0.6 0.9 1.0", specular="0.5", shininess="0.5")

    # Visual mesh
    etree.SubElement(asset, "mesh", name="object_visual", file=visual_mesh_file)

    # Collision meshes
    for i, f in enumerate(collision_mesh_files):
        etree.SubElement(asset, "mesh", name=f"object_collision_{i}", file=f)

    # Load Shadow Hand model
    hand_body_xml, hand_actuator_xml, hand_asset_xml, robot_mesh_assets = (
        load_shadow_hand()
    )

    # Insert hand asset definitions (mesh refs, materials, etc.)
    if hand_asset_xml.strip():
        try:
            wrapped = f"<root>{hand_asset_xml}</root>"
            hand_asset_tree = etree.fromstring(wrapped)
            for child in hand_asset_tree:
                asset.append(child)
        except etree.XMLSyntaxError:
            pass  # Skip malformed asset XML

    # Worldbody
    worldbody = etree.SubElement(root, "worldbody")

    # Lights
    etree.SubElement(worldbody, "light", name="top_light",
                     pos="0 0 2.5", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    etree.SubElement(worldbody, "light", name="front_light",
                     pos="1 -1 1.5", dir="-0.5 0.5 -0.5", diffuse="0.5 0.5 0.5")

    # Ground
    etree.SubElement(worldbody, "geom", name="ground", type="plane",
                     size="2 2 0.01", material="grid_mat",
                     friction=f"{contact_params['friction'][0]} {contact_params['friction'][1]} {contact_params['friction'][2]}")

    # Environment elements
    _add_environment(worldbody, environment)

    # Overview camera (for recorded video)
    etree.SubElement(worldbody, "camera", name="overview_cam",
                     pos="1.2 -0.8 1.0", xyaxes="0.6 0.8 0 -0.3 0.2 0.9",
                     fovy="60")

    # Target object body
    obj_pos_str = " ".join(str(v) for v in object_position)
    obj_body = etree.SubElement(worldbody, "body", name="object",
                                pos=obj_pos_str)
    etree.SubElement(obj_body, "freejoint", name="object_joint")

    # Inertial
    cog_str = " ".join(f"{c:.6f}" for c in center_of_mass)
    diaginertia = f"{inertia['ixx']:.8f} {inertia['iyy']:.8f} {inertia['izz']:.8f}"
    etree.SubElement(obj_body, "inertial", pos=cog_str, mass=f"{mass:.6f}",
                     diaginertia=diaginertia)

    # Visual geom
    etree.SubElement(obj_body, "geom", name="object_visual", type="mesh",
                     mesh="object_visual", material="object_mat",
                     contype="0", conaffinity="0")

    # Collision geoms
    friction_str = " ".join(str(f) for f in contact_params["friction"])
    solref_str = " ".join(str(s) for s in contact_params["solref"])
    solimp_str = " ".join(str(s) for s in contact_params["solimp"])

    for i in range(len(collision_mesh_files)):
        etree.SubElement(obj_body, "geom", name=f"object_collision_{i}",
                         type="mesh", mesh=f"object_collision_{i}",
                         friction=friction_str,
                         solref=solref_str,
                         solimp=solimp_str,
                         condim=str(contact_params["condim"]))

    # Robot: 7-DOF arm with Shadow Hand attached at end-effector
    robot_pos_str = " ".join(str(v) for v in robot_position)
    arm_xml = get_arm_xml(base_pos=robot_pos_str)
    try:
        arm_tree = etree.fromstring(f"<root>{arm_xml}</root>")
        # Find the end_effector body and attach the hand
        for arm_body in arm_tree:
            worldbody.append(arm_body)
            # Find end_effector within the arm tree and attach hand
            ee = arm_body.find(".//body[@name='end_effector']")
            if ee is not None and hand_body_xml.strip():
                try:
                    hand_tree = etree.fromstring(f"<root>{hand_body_xml}</root>")
                    for hand_child in hand_tree:
                        ee.append(hand_child)
                except etree.XMLSyntaxError:
                    pass
                # Add wrist-mounted camera
                etree.SubElement(ee, "camera", name="wrist_cam",
                                 pos="0 0 0.05", xyaxes="1 0 0 0 -1 0",
                                 fovy="90")
    except etree.XMLSyntaxError:
        # Fallback: just add a static robot base
        robot_body = etree.SubElement(worldbody, "body", name="robot_mount",
                                      pos="0 0 0")
        etree.SubElement(robot_body, "geom", name="robot_base", type="cylinder",
                         size="0.06 0.05", rgba="0.3 0.3 0.3 1")

    # Actuators section
    actuator_el = etree.SubElement(root, "actuator")

    # Arm actuators
    arm_act_xml = get_arm_actuators_xml()
    try:
        arm_act_tree = etree.fromstring(f"<root>{arm_act_xml}</root>")
        for act in arm_act_tree:
            actuator_el.append(act)
    except etree.XMLSyntaxError:
        pass

    # Hand actuators
    if hand_actuator_xml.strip():
        try:
            hand_act_tree = etree.fromstring(f"<root>{hand_actuator_xml}</root>")
            for act in hand_act_tree:
                actuator_el.append(act)
        except etree.XMLSyntaxError:
            pass

    # Sensors for fingertip contact
    sensor = etree.SubElement(root, "sensor")
    for finger in ["thumb", "index", "middle", "ring", "pinky"]:
        etree.SubElement(sensor, "touch", name=f"{finger}_touch",
                         site=f"{finger}_tip")

    xml_str = etree.tostring(root, pretty_print=True, xml_declaration=True,
                             encoding="utf-8").decode()

    return xml_str, robot_mesh_assets


def _add_environment(worldbody, environment: str):
    """Add environment-specific elements."""
    env_builders = {
        "kitchen": _env_kitchen,
        "workshop": _env_workshop,
        "vehicle": _env_vehicle,
        "operating_room": _env_operating_room,
        "open_space": _env_open_space,
    }
    builder = env_builders.get(environment, _env_open_space)
    builder(worldbody)


def _env_kitchen(worldbody):
    """Kitchen environment with table and cabinet."""
    table = etree.SubElement(worldbody, "body", name="table", pos="0.5 0 0.35")
    etree.SubElement(table, "geom", type="box", size="0.4 0.3 0.02",
                     rgba="0.6 0.4 0.2 1", friction="0.5 0.005 0.0001")
    leg_positions = [("0.35 -0.25 -0.17",), ("0.35 0.25 -0.17",),
                     ("0.65 -0.25 -0.17",), ("0.65 0.25 -0.17",)]
    for i, (pos,) in enumerate(leg_positions):
        etree.SubElement(table, "geom", name=f"table_leg_{i}", type="cylinder",
                         pos=pos, size="0.02 0.17", rgba="0.6 0.4 0.2 1")


def _env_workshop(worldbody):
    """Workshop with workbench."""
    bench = etree.SubElement(worldbody, "body", name="workbench", pos="0.5 0 0.4")
    etree.SubElement(bench, "geom", type="box", size="0.6 0.4 0.03",
                     rgba="0.5 0.5 0.5 1", friction="0.6 0.005 0.0001")


def _env_vehicle(worldbody):
    """Vehicle interior surface."""
    surface = etree.SubElement(worldbody, "body", name="dashboard", pos="0.5 0 0.3")
    etree.SubElement(surface, "geom", type="box", size="0.5 0.3 0.02",
                     rgba="0.2 0.2 0.2 1", friction="0.4 0.005 0.0001")


def _env_operating_room(worldbody):
    """Operating room with sterile table."""
    table = etree.SubElement(worldbody, "body", name="surgical_table", pos="0.5 0 0.45")
    etree.SubElement(table, "geom", type="box", size="0.5 0.3 0.02",
                     rgba="0.9 0.9 0.95 1", friction="0.3 0.005 0.0001")


def _env_open_space(worldbody):
    """Open space — minimal environment, just the ground plane."""
    pass  # Ground already added
