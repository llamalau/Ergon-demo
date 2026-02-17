"""Apply user-specified property overrides to extracted properties."""


ALLOWED_OVERRIDES = {
    "mass", "friction", "rigidity", "material", "density",
    "center_of_mass", "inertia_override",
}


def apply_overrides(properties: dict, overrides: dict | None) -> dict:
    """Apply user overrides to the extracted property dict.

    Only fields in ALLOWED_OVERRIDES can be modified.
    """
    if not overrides:
        return properties

    result = dict(properties)

    for key, value in overrides.items():
        if key not in ALLOWED_OVERRIDES:
            continue

        if key == "material":
            from app.stages.s1_ingestion.property_extractor import MATERIAL_DB
            if value in MATERIAL_DB:
                density, friction, rigidity = MATERIAL_DB[value]
                result["material"] = value
                result["density"] = density
                result["friction"] = friction
                result["rigidity"] = rigidity
                # Recompute mass with new density
                if result.get("volume"):
                    result["mass"] = density * result["volume"]
        elif key == "inertia_override":
            if isinstance(value, dict):
                result["inertia_estimate"] = {**result.get("inertia_estimate", {}), **value}
        elif key == "center_of_mass":
            if isinstance(value, list) and len(value) == 3:
                result["center_of_mass"] = value
        else:
            result[key] = value

    return result
