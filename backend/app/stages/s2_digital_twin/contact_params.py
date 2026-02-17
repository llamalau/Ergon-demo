"""Generate MuJoCo contact parameters from material properties."""


# Default MuJoCo contact parameter presets by material category
CONTACT_PRESETS = {
    "hard": {  # metals, ceramics, glass
        "condim": 4,
        "friction": [0.5, 0.005, 0.0001],
        "solref": [0.02, 1.0],
        "solimp": [0.9, 0.95, 0.001],
    },
    "medium": {  # plastics, wood
        "condim": 4,
        "friction": [0.4, 0.005, 0.0001],
        "solref": [0.01, 0.8],
        "solimp": [0.85, 0.9, 0.001],
    },
    "soft": {  # rubber
        "condim": 4,
        "friction": [0.8, 0.01, 0.0001],
        "solref": [0.005, 0.5],
        "solimp": [0.7, 0.85, 0.001],
    },
}

MATERIAL_CATEGORY = {
    "aluminum": "hard",
    "steel": "hard",
    "glass": "hard",
    "ceramic": "hard",
    "carbon_fiber": "hard",
    "plastic_abs": "medium",
    "plastic_pla": "medium",
    "wood": "medium",
    "rubber": "soft",
    "default": "medium",
}


def compute_contact_params(material: str, friction_override: float | None = None) -> dict:
    """Compute MuJoCo contact parameters from material name.

    Returns condim, friction, solref, solimp values for MJCF.
    """
    category = MATERIAL_CATEGORY.get(material, "medium")
    params = dict(CONTACT_PRESETS[category])

    if friction_override is not None:
        params["friction"] = [friction_override, 0.005, 0.0001]

    return {
        "material": material,
        "category": category,
        **params,
    }
