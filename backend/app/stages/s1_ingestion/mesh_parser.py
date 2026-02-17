import io

import numpy as np
import trimesh


def parse_stl(data: bytes) -> dict:
    """Parse STL file and extract geometry info."""
    mesh = trimesh.load(io.BytesIO(data), file_type="stl")
    return _extract_mesh_info(mesh)


def parse_obj(data: bytes) -> dict:
    """Parse OBJ file and extract geometry info."""
    mesh = trimesh.load(io.BytesIO(data), file_type="obj")
    return _extract_mesh_info(mesh)


def parse_mesh(data: bytes, file_format: str) -> dict:
    """Parse any mesh format supported by trimesh."""
    mesh = trimesh.load(io.BytesIO(data), file_type=file_format)
    return _extract_mesh_info(mesh)


def _extract_mesh_info(mesh) -> dict:
    """Extract geometric properties from a trimesh object."""
    if isinstance(mesh, trimesh.Scene):
        # Combine all geometries in scene
        meshes = list(mesh.geometry.values())
        if not meshes:
            raise ValueError("Empty scene — no geometry found")
        mesh = trimesh.util.concatenate(meshes)

    bounds = mesh.bounds.tolist()
    extents = mesh.extents.tolist()

    return {
        "vertices": len(mesh.vertices),
        "faces": len(mesh.faces),
        "bounds": {"min": bounds[0], "max": bounds[1]},
        "extents": extents,
        "volume": float(mesh.volume) if mesh.is_watertight else None,
        "surface_area": float(mesh.area),
        "center_of_mass": mesh.center_mass.tolist(),
        "is_watertight": bool(mesh.is_watertight),
        "euler_number": int(mesh.euler_number),
    }
