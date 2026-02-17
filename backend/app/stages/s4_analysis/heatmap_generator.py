"""Generate 3D contact heatmaps as colored mesh data."""

import io
import json
import numpy as np


def generate_contact_heatmap(
    trial_results: list[dict],
    visual_mesh_key: str,
) -> dict:
    """Generate contact heatmap data overlaid on the visual mesh.

    Returns heatmap data as vertex colors that can be applied to the mesh.
    """
    from app.services.storage import download_file
    import trimesh

    # Load visual mesh
    mesh_data = download_file(visual_mesh_key)
    mesh = trimesh.load(io.BytesIO(mesh_data), file_type="stl")
    if isinstance(mesh, trimesh.Scene):
        meshes = list(mesh.geometry.values())
        mesh = trimesh.util.concatenate(meshes) if meshes else None

    if mesh is None:
        return {"error": "No mesh for heatmap generation"}

    num_vertices = len(mesh.vertices)

    # Simulate contact intensity per vertex region
    # In a real implementation, we'd map actual contact points to nearest vertices
    contact_intensity = np.zeros(num_vertices)

    for trial in trial_results:
        summary = trial.get("summary", {})
        max_force = summary.get("max_contact_force", 0)

        # Approximate: distribute contact intensity based on vertex position
        # Prefer lower vertices (contact surfaces) and center of object
        vertices = mesh.vertices
        center = mesh.centroid
        distances = np.linalg.norm(vertices - center, axis=1)
        max_dist = np.max(distances) + 1e-6

        # Contact is more likely at bottom and at grasp regions
        z_normalized = (vertices[:, 2] - vertices[:, 2].min()) / (vertices[:, 2].ptp() + 1e-6)
        contact_prob = (1 - z_normalized) * 0.6 + (1 - distances / max_dist) * 0.4
        contact_intensity += contact_prob * max_force

    # Normalize to 0-1
    if contact_intensity.max() > 0:
        contact_intensity /= contact_intensity.max()

    # Convert to colors (blue = low, red = high)
    colors = np.zeros((num_vertices, 4), dtype=np.uint8)
    colors[:, 0] = (contact_intensity * 255).astype(np.uint8)  # Red
    colors[:, 1] = ((1 - abs(contact_intensity - 0.5) * 2) * 100).astype(np.uint8)  # Green (peaks at 0.5)
    colors[:, 2] = ((1 - contact_intensity) * 255).astype(np.uint8)  # Blue
    colors[:, 3] = 255  # Alpha

    mesh.visual.vertex_colors = colors

    # Export as GLB
    glb_buf = io.BytesIO()
    mesh.export(glb_buf, file_type="glb")
    glb_buf.seek(0)

    return {
        "glb_data": glb_buf.getvalue(),
        "num_vertices": num_vertices,
        "max_intensity": float(contact_intensity.max()),
        "mean_intensity": float(contact_intensity.mean()),
    }
