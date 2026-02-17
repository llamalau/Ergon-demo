"""Compute inertia tensor from mesh geometry."""

import io
import numpy as np
import trimesh


def compute_inertia(mesh_data: bytes, mass: float, file_format: str = "stl") -> dict:
    """Compute accurate inertia tensor from mesh + mass.

    Uses trimesh's moment_inertia computation if the mesh is watertight,
    otherwise falls back to bounding-box approximation.
    """
    mesh = trimesh.load(io.BytesIO(mesh_data), file_type=file_format)
    if isinstance(mesh, trimesh.Scene):
        meshes = list(mesh.geometry.values())
        mesh = trimesh.util.concatenate(meshes) if meshes else None

    if mesh is None:
        raise ValueError("No geometry for inertia computation")

    if mesh.is_watertight and mesh.volume > 0:
        # Scale inertia tensor to desired mass
        density = mass / mesh.volume
        mesh.density = density
        inertia_tensor = mesh.moment_inertia
    else:
        # Bounding box approximation
        extents = mesh.extents
        a, b, c = extents
        ixx = (mass / 12.0) * (b**2 + c**2)
        iyy = (mass / 12.0) * (a**2 + c**2)
        izz = (mass / 12.0) * (a**2 + b**2)
        inertia_tensor = np.diag([ixx, iyy, izz])

    # Extract diagonal and off-diagonal
    return {
        "inertia_tensor": inertia_tensor.tolist(),
        "ixx": float(inertia_tensor[0, 0]),
        "iyy": float(inertia_tensor[1, 1]),
        "izz": float(inertia_tensor[2, 2]),
        "ixy": float(inertia_tensor[0, 1]),
        "ixz": float(inertia_tensor[0, 2]),
        "iyz": float(inertia_tensor[1, 2]),
        "principal_axes": _principal_axes(inertia_tensor),
    }


def _principal_axes(inertia_tensor: np.ndarray) -> dict:
    """Compute principal moments and axes."""
    eigenvalues, eigenvectors = np.linalg.eigh(inertia_tensor)
    return {
        "moments": eigenvalues.tolist(),
        "axes": eigenvectors.tolist(),
    }
