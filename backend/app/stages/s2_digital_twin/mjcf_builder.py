"""Programmatic MJCF XML generation for humanoid manipulation scenes.

Composes a scene with the Unitree G1 humanoid (from MuJoCo Menagerie)
and a target object, with environment elements and cameras.
"""

from lxml import etree

from app.stages.s3_simulation.humanoid_loader import (
    get_g1_model_path,
    get_g1_asset_dir,
    CAMERA_CONFIGS,
    BODY_NAMES,
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
    """Build a complete MJCF XML string for G1 humanoid + object + environment.

    The G1 model is included via <include>, not generated procedurally.

    Returns (mjcf_xml_string, empty_assets_dict).
    The G1 meshes are loaded from disk by MuJoCo via the meshdir compiler
    setting, so no robot mesh assets need to be passed as binary blobs.
    """
    if object_position is None:
        object_position = [0.65, 0, 0.78]  # On table, in front of robot
    if robot_position is None:
        robot_position = [0.0, 0, 0.0]

    root = etree.Element("mujoco", model=model_name)

    # Compiler: angle in radian, meshdir for object meshes
    # The G1 model has its own meshdir set in the included file
    etree.SubElement(root, "compiler", angle="radian", meshdir="meshes")

    # Options
    etree.SubElement(root, "option", timestep="0.002", gravity="0 0 -9.81",
                     integrator="implicitfast")

    # Visual settings
    visual = etree.SubElement(root, "visual")
    etree.SubElement(visual, "global", offwidth="1280", offheight="720")

    # Assets - textures and materials for environment/object
    asset = etree.SubElement(root, "asset")
    etree.SubElement(asset, "texture", type="2d", name="grid", builtin="checker",
                     rgb1="0.2 0.2 0.2", rgb2="0.3 0.3 0.3", width="512", height="512")
    etree.SubElement(asset, "material", name="grid_mat", texture="grid",
                     texrepeat="8 8", reflectance="0.1")
    etree.SubElement(asset, "material", name="object_mat",
                     rgba="0.4 0.6 0.9 1.0", specular="0.5", shininess="0.5")

    # Object meshes
    etree.SubElement(asset, "mesh", name="object_visual", file=visual_mesh_file)
    for i, f in enumerate(collision_mesh_files):
        etree.SubElement(asset, "mesh", name=f"object_collision_{i}", file=f)

    # Include the G1 humanoid model from Menagerie
    # MuJoCo resolves the include relative to the main file, so we use
    # an absolute path. The G1 model defines its own meshdir for its assets.
    g1_path = str(get_g1_model_path())
    etree.SubElement(root, "include", file=g1_path)

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
                     friction=f"{contact_params['friction'][0]} "
                              f"{contact_params['friction'][1]} "
                              f"{contact_params['friction'][2]}")

    # Environment elements (table, etc.)
    _add_environment(worldbody, environment)

    # Overview camera (fixed world position, for video recording and planner)
    etree.SubElement(worldbody, "camera", name="overview_cam",
                     pos="1.5 -1.0 1.5", xyaxes="0.6 0.8 0 -0.3 0.2 0.9",
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

    # NOTE: Ego and wrist cameras are attached to the G1's body hierarchy.
    # We add them as top-level elements that MuJoCo will attach to the
    # included G1 bodies via the body="..." attribute.
    # However, <camera> elements in worldbody cannot reference included bodies
    # directly. Instead, we'll add them programmatically after model load
    # via the CameraManager, or define them in the MJCF using the <body>
    # attachment mechanism below.

    # We place cameras in dedicated wrapper bodies that we position
    # at the robot location. The CameraManager will look them up by name.
    # The ego cam is defined as a world-fixed camera approximating head view.
    robot_pos = robot_position or [0.0, 0, 0.0]
    ego_pos = f"{robot_pos[0] + 0.15} {robot_pos[1]} {robot_pos[2] + 1.1}"
    etree.SubElement(worldbody, "camera", name="ego_cam",
                     pos=ego_pos,
                     xyaxes="0 -1 0 0.3 0 1",
                     fovy="90")

    xml_str = etree.tostring(root, pretty_print=True, xml_declaration=True,
                             encoding="utf-8").decode()

    # No robot mesh assets to return - G1 loads from its own meshdir
    return xml_str, {}


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
    # Table positioned for G1 standing (table height ~0.75m)
    table = etree.SubElement(worldbody, "body", name="table", pos="0.6 0 0.375")
    etree.SubElement(table, "geom", type="box", size="0.4 0.3 0.375",
                     rgba="0.6 0.4 0.2 1", friction="0.5 0.005 0.0001")


def _env_workshop(worldbody):
    """Workshop with workbench."""
    bench = etree.SubElement(worldbody, "body", name="workbench", pos="0.6 0 0.4")
    etree.SubElement(bench, "geom", type="box", size="0.6 0.4 0.4",
                     rgba="0.5 0.5 0.5 1", friction="0.6 0.005 0.0001")


def _env_vehicle(worldbody):
    """Vehicle interior surface."""
    surface = etree.SubElement(worldbody, "body", name="dashboard", pos="0.6 0 0.35")
    etree.SubElement(surface, "geom", type="box", size="0.5 0.3 0.35",
                     rgba="0.2 0.2 0.2 1", friction="0.4 0.005 0.0001")


def _env_operating_room(worldbody):
    """Operating room with sterile table."""
    table = etree.SubElement(worldbody, "body", name="surgical_table", pos="0.6 0 0.425")
    etree.SubElement(table, "geom", type="box", size="0.5 0.3 0.425",
                     rgba="0.9 0.9 0.95 1", friction="0.3 0.005 0.0001")


def _env_open_space(worldbody):
    """Open space -- minimal environment, just the ground plane."""
    pass  # Ground already added
