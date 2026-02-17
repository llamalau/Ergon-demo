"""Convex decomposition for collision meshes using CoACD or V-HACD via trimesh."""

import io
import numpy as np
import trimesh

# Must match the threshold in mesh_parser.py
_MAX_EXTENT_M = 2.0


def generate_collision_meshes(visual_mesh_data: bytes, file_format: str = "stl") -> dict:
    """Generate collision meshes via convex decomposition.

    Returns visual and collision mesh data.
    """
    mesh = trimesh.load(io.BytesIO(visual_mesh_data), file_type=file_format)
    if isinstance(mesh, trimesh.Scene):
        meshes = list(mesh.geometry.values())
        mesh = trimesh.util.concatenate(meshes) if meshes else None
    if mesh is None:
        raise ValueError("No geometry found for collision mesh generation")

    # Auto-scale mm → m (same heuristic as ingestion parser)
    max_extent = max(mesh.extents)
    if max_extent > _MAX_EXTENT_M:
        mesh.apply_scale(0.001)

    # Try convex decomposition; fall back to convex hull
    try:
        collision_parts = mesh.convex_decomposition(maxhulls=8)
        if not isinstance(collision_parts, list):
            collision_parts = [collision_parts]
    except Exception:
        collision_parts = [mesh.convex_hull]

    collision_meshes = []
    for i, part in enumerate(collision_parts):
        buf = io.BytesIO()
        part.export(buf, file_type="stl")
        collision_meshes.append({
            "index": i,
            "vertices": len(part.vertices),
            "faces": len(part.faces),
            "volume": float(part.volume) if part.is_watertight else None,
            "data": buf.getvalue(),
        })

    # Export visual mesh as well
    visual_buf = io.BytesIO()
    mesh.export(visual_buf, file_type="stl")

    return {
        "visual_mesh": visual_buf.getvalue(),
        "visual_info": {
            "vertices": len(mesh.vertices),
            "faces": len(mesh.faces),
        },
        "collision_meshes": collision_meshes,
        "num_collision_parts": len(collision_meshes),
    }
