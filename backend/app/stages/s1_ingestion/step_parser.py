"""STEP/IGES parser using pythonocc-core (if available) or trimesh fallback."""

import io
import tempfile
import os


def parse_step(data: bytes) -> dict:
    """Parse STEP file and extract geometry info.

    Attempts pythonocc first, falls back to trimesh CAD import.
    """
    try:
        return _parse_step_occ(data)
    except ImportError:
        return _parse_step_trimesh(data)


def parse_iges(data: bytes) -> dict:
    """Parse IGES file and extract geometry info."""
    try:
        return _parse_iges_occ(data)
    except ImportError:
        return _parse_iges_trimesh(data)


def _parse_step_occ(data: bytes) -> dict:
    """Parse STEP using pythonocc-core."""
    from OCP.STEPControl import STEPControl_Reader
    from OCP.BRep import BRep_Tool
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import brepgprop

    with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as f:
        f.write(data)
        f.flush()
        tmp_path = f.name

    try:
        reader = STEPControl_Reader()
        reader.ReadFile(tmp_path)
        reader.TransferRoots()
        shape = reader.OneShape()

        # Compute volume and surface properties
        props = GProp_GProps()
        brepgprop.VolumeProperties(shape, props)
        volume = props.Mass()
        cog = props.CentreOfMass()

        surface_props = GProp_GProps()
        brepgprop.SurfaceProperties(shape, surface_props)
        surface_area = surface_props.Mass()

        return {
            "format": "step",
            "volume": float(volume),
            "surface_area": float(surface_area),
            "center_of_mass": [cog.X(), cog.Y(), cog.Z()],
            "source": "pythonocc",
        }
    finally:
        os.unlink(tmp_path)


def _parse_iges_occ(data: bytes) -> dict:
    """Parse IGES using pythonocc-core."""
    from OCP.IGESControl import IGESControl_Reader
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import brepgprop

    with tempfile.NamedTemporaryFile(suffix=".iges", delete=False) as f:
        f.write(data)
        f.flush()
        tmp_path = f.name

    try:
        reader = IGESControl_Reader()
        reader.ReadFile(tmp_path)
        reader.TransferRoots()
        shape = reader.OneShape()

        props = GProp_GProps()
        brepgprop.VolumeProperties(shape, props)
        volume = props.Mass()
        cog = props.CentreOfMass()

        return {
            "format": "iges",
            "volume": float(volume),
            "center_of_mass": [cog.X(), cog.Y(), cog.Z()],
            "source": "pythonocc",
        }
    finally:
        os.unlink(tmp_path)


def _parse_step_trimesh(data: bytes) -> dict:
    """Fallback STEP parser using trimesh (limited support)."""
    import trimesh

    with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as f:
        f.write(data)
        f.flush()
        tmp_path = f.name

    try:
        mesh = trimesh.load(tmp_path)
        if isinstance(mesh, trimesh.Scene):
            meshes = list(mesh.geometry.values())
            mesh = trimesh.util.concatenate(meshes) if meshes else None
        if mesh is None:
            return {"format": "step", "error": "Could not parse geometry", "source": "trimesh"}
        return {
            "format": "step",
            "vertices": len(mesh.vertices),
            "faces": len(mesh.faces),
            "volume": float(mesh.volume) if mesh.is_watertight else None,
            "surface_area": float(mesh.area),
            "center_of_mass": mesh.center_mass.tolist(),
            "source": "trimesh",
        }
    finally:
        os.unlink(tmp_path)


def _parse_iges_trimesh(data: bytes) -> dict:
    """Fallback IGES parser."""
    return {"format": "iges", "error": "IGES parsing requires pythonocc-core", "source": "trimesh"}
