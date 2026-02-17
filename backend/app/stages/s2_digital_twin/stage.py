import json

from app.tasks.base_stage import BaseStage
from app.services.storage import download_file, upload_file
from app.stages.s2_digital_twin.collision_mesh import generate_collision_meshes
from app.stages.s2_digital_twin.inertia import compute_inertia
from app.stages.s2_digital_twin.contact_params import compute_contact_params
from app.stages.s2_digital_twin.mjcf_builder import build_mjcf


class DigitalTwinStage(BaseStage):
    stage_name = "digital_twin"
    stage_order = 1

    def run(self, context: dict) -> dict:
        job_id = context["job_id"]
        storage_key = context.get("storage_key", "")
        file_format = context.get("file_format", "stl")
        properties = context.get("physical_properties", {})
        environment = context.get("environment", "open_space")

        self.publish_progress(0.1, "Generating collision meshes")
        mesh_data = download_file(storage_key)
        collision_result = generate_collision_meshes(mesh_data, file_format)

        # Store meshes in MinIO
        visual_key = f"jobs/{job_id}/meshes/visual.stl"
        upload_file(visual_key, collision_result["visual_mesh"], "model/stl")

        collision_keys = []
        for cm in collision_result["collision_meshes"]:
            key = f"jobs/{job_id}/meshes/collision_{cm['index']}.stl"
            upload_file(key, cm["data"], "model/stl")
            collision_keys.append(key)

        self.publish_progress(0.4, "Computing inertia tensor")
        mass = properties.get("mass", 1.0)
        inertia = compute_inertia(collision_result["visual_mesh"], mass, "stl")

        self.publish_progress(0.6, "Computing contact parameters")
        material = properties.get("material", "default")
        contact = compute_contact_params(material, properties.get("friction"))

        self.publish_progress(0.8, "Building MJCF model with Shadow Hand")
        model_name = f"ergon_object_{job_id[:8]}"
        mjcf_xml, robot_mesh_assets = build_mjcf(
            model_name=model_name,
            mass=mass,
            inertia=inertia,
            contact_params=contact,
            collision_mesh_files=[f"collision_{i}.stl" for i in range(len(collision_keys))],
            visual_mesh_file="visual.stl",
            center_of_mass=properties.get("center_of_mass", [0, 0, 0]),
            environment=environment,
        )

        mjcf_key = f"jobs/{job_id}/model.xml"
        upload_file(mjcf_key, mjcf_xml.encode(), "application/xml")

        # Upload robot mesh assets to MinIO
        robot_mesh_keys = {}
        for rel_path, mesh_bytes in robot_mesh_assets.items():
            asset_key = f"jobs/{job_id}/robot_meshes/{rel_path}"
            upload_file(asset_key, mesh_bytes, "application/octet-stream")
            robot_mesh_keys[rel_path] = asset_key

        context["mjcf_key"] = mjcf_key
        context["visual_mesh_key"] = visual_key
        context["collision_mesh_keys"] = collision_keys
        context["robot_mesh_keys"] = robot_mesh_keys
        context["inertia"] = inertia
        context["contact_params"] = contact
        context["stage_result"] = {
            "mjcf_key": mjcf_key,
            "environment": environment,
            "num_collision_parts": collision_result["num_collision_parts"],
            "inertia_diagonal": [inertia["ixx"], inertia["iyy"], inertia["izz"]],
            "robot_type": "shadow_hand_7dof_arm",
        }

        return context
