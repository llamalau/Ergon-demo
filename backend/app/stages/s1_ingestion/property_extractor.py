"""Extract physical properties from geometry data + material lookup."""

import numpy as np

# Default material database: material -> (density kg/m^3, friction, rigidity 0-1)
MATERIAL_DB = {
    "plastic_abs": (1040.0, 0.4, 0.6),
    "plastic_pla": (1240.0, 0.35, 0.55),
    "aluminum": (2700.0, 0.47, 0.9),
    "steel": (7800.0, 0.57, 0.95),
    "wood": (600.0, 0.45, 0.5),
    "rubber": (1100.0, 0.8, 0.15),
    "glass": (2500.0, 0.2, 0.95),
    "ceramic": (2300.0, 0.35, 0.9),
    "carbon_fiber": (1600.0, 0.3, 0.95),
    "default": (1000.0, 0.5, 0.7),
}


def extract_properties(geometry_info: dict, material: str = "default") -> dict:
    """Extract physical properties from parsed geometry and material.

    Returns mass, center of gravity, friction, rigidity, estimated inertia.
    """
    density, friction, rigidity = MATERIAL_DB.get(material, MATERIAL_DB["default"])

    volume = geometry_info.get("volume")
    if volume and volume > 0:
        mass = density * volume
    else:
        # Estimate from bounding box if volume unavailable
        extents = geometry_info.get("extents", [0.1, 0.1, 0.1])
        estimated_volume = extents[0] * extents[1] * extents[2]
        mass = density * estimated_volume
        volume = estimated_volume

    cog = geometry_info.get("center_of_mass", [0.0, 0.0, 0.0])

    # Estimate inertia tensor (box approximation from extents)
    extents = geometry_info.get("extents", [0.1, 0.1, 0.1])
    a, b, c = extents[0], extents[1], extents[2]
    ixx = (mass / 12.0) * (b**2 + c**2)
    iyy = (mass / 12.0) * (a**2 + c**2)
    izz = (mass / 12.0) * (a**2 + b**2)

    return {
        "material": material,
        "density": density,
        "mass": mass,
        "volume": volume,
        "center_of_mass": cog,
        "friction": friction,
        "rigidity": rigidity,
        "inertia_estimate": {
            "ixx": ixx, "iyy": iyy, "izz": izz,
            "ixy": 0.0, "ixz": 0.0, "iyz": 0.0,
        },
        "surface_area": geometry_info.get("surface_area"),
        "is_watertight": geometry_info.get("is_watertight", False),
    }
