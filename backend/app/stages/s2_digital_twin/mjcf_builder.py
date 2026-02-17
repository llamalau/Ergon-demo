"""Programmatic MJCF XML generation for simulation."""

from lxml import etree


def build_mjcf(
    model_name: str,
    mass: float,
    inertia: dict,
    contact_params: dict,
    collision_mesh_files: list[str],
    visual_mesh_file: str,
    center_of_mass: list[float],
    environment: str = "open_space",
) -> str:
    """Build a complete MJCF XML string for the object + environment + robot."""

    root = etree.Element("mujoco", model=model_name)

    # Compiler settings
    etree.SubElement(root, "compiler", angle="radian", meshdir="meshes")

    # Options
    option = etree.SubElement(root, "option", timestep="0.002", gravity="0 0 -9.81")

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

    # Worldbody
    worldbody = etree.SubElement(root, "worldbody")

    # Light
    etree.SubElement(worldbody, "light", name="top_light",
                     pos="0 0 2.5", dir="0 0 -1", diffuse="0.8 0.8 0.8")

    # Ground
    etree.SubElement(worldbody, "geom", name="ground", type="plane",
                     size="2 2 0.01", material="grid_mat",
                     friction=f"{contact_params['friction'][0]} {contact_params['friction'][1]} {contact_params['friction'][2]}")

    # Environment elements
    _add_environment(worldbody, environment)

    # Target object body
    obj_body = etree.SubElement(worldbody, "body", name="object",
                                pos="0.5 0 0.3")
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

    # Robot placeholder (Franka Panda to be included via include)
    robot_body = etree.SubElement(worldbody, "body", name="robot_mount",
                                  pos="0 0 0")
    etree.SubElement(robot_body, "geom", name="robot_base", type="cylinder",
                     size="0.06 0.05", rgba="0.3 0.3 0.3 1")

    return etree.tostring(root, pretty_print=True, xml_declaration=True,
                          encoding="utf-8").decode()


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
