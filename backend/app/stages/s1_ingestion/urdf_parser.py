"""URDF and MJCF XML format parsers."""

from lxml import etree


def parse_urdf(data: bytes) -> dict:
    """Parse URDF XML and extract robot structure."""
    root = etree.fromstring(data)
    robot_name = root.get("name", "unknown")

    links = []
    for link in root.findall(".//link"):
        link_info = {"name": link.get("name", "")}

        # Visual geometry
        visual = link.find("visual/geometry")
        if visual is not None:
            child = visual[0] if len(visual) else None
            if child is not None:
                link_info["visual_type"] = child.tag
                if child.tag == "mesh":
                    link_info["mesh_file"] = child.get("filename", "")

        # Collision geometry
        collision = link.find("collision/geometry")
        if collision is not None:
            child = collision[0] if len(collision) else None
            if child is not None:
                link_info["collision_type"] = child.tag

        # Inertial
        inertial = link.find("inertial")
        if inertial is not None:
            mass_elem = inertial.find("mass")
            if mass_elem is not None:
                link_info["mass"] = float(mass_elem.get("value", 0))

        links.append(link_info)

    joints = []
    for joint in root.findall(".//joint"):
        joint_info = {
            "name": joint.get("name", ""),
            "type": joint.get("type", ""),
        }
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None:
            joint_info["parent"] = parent.get("link", "")
        if child is not None:
            joint_info["child"] = child.get("link", "")
        joints.append(joint_info)

    return {
        "format": "urdf",
        "robot_name": robot_name,
        "num_links": len(links),
        "num_joints": len(joints),
        "links": links,
        "joints": joints,
    }


def parse_mjcf(data: bytes) -> dict:
    """Parse MJCF XML and extract model structure."""
    root = etree.fromstring(data)
    model_name = root.get("model", "unknown")

    bodies = []
    for body in root.iter("body"):
        body_info = {"name": body.get("name", "")}
        geoms = body.findall("geom")
        body_info["num_geoms"] = len(geoms)
        joints = body.findall("joint")
        body_info["num_joints"] = len(joints)
        bodies.append(body_info)

    actuators = []
    for actuator_section in root.findall(".//actuator"):
        for motor in actuator_section:
            actuators.append({
                "name": motor.get("name", ""),
                "type": motor.tag,
                "joint": motor.get("joint", ""),
            })

    return {
        "format": "mjcf",
        "model_name": model_name,
        "num_bodies": len(bodies),
        "num_actuators": len(actuators),
        "bodies": bodies,
        "actuators": actuators,
    }
