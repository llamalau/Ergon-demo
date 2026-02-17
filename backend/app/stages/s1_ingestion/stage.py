from app.tasks.base_stage import BaseStage
from app.services.storage import download_file, upload_file
from app.stages.s1_ingestion.mesh_parser import parse_mesh
from app.stages.s1_ingestion.step_parser import parse_step, parse_iges
from app.stages.s1_ingestion.urdf_parser import parse_urdf, parse_mjcf
from app.stages.s1_ingestion.property_extractor import extract_properties
from app.stages.s1_ingestion.user_overrides import apply_overrides

import json


class IngestionStage(BaseStage):
    stage_name = "cad_ingestion"
    stage_order = 0

    def run(self, context: dict) -> dict:
        job_id = context["job_id"]
        storage_key = context.get("storage_key", "")
        file_format = context.get("file_format", "stl")
        overrides = context.get("property_overrides")
        material = context.get("material", "default")

        self.publish_progress(0.1, "Downloading CAD file")
        data = download_file(storage_key)

        self.publish_progress(0.3, f"Parsing {file_format} file")
        geometry_info = self._parse(data, file_format)

        self.publish_progress(0.6, "Extracting physical properties")
        properties = extract_properties(geometry_info, material=material)

        self.publish_progress(0.8, "Applying user overrides")
        properties = apply_overrides(properties, overrides)

        # Store parsed data
        result_key = f"jobs/{job_id}/ingestion_result.json"
        upload_file(result_key, json.dumps({
            "geometry": geometry_info,
            "properties": properties,
        }).encode(), "application/json")

        context["geometry_info"] = geometry_info
        context["physical_properties"] = properties
        context["ingestion_result_key"] = result_key
        context["stage_result"] = {
            "geometry_summary": {
                k: geometry_info[k] for k in ["vertices", "faces", "volume", "surface_area", "is_watertight"]
                if k in geometry_info
            },
            "properties_summary": {
                "mass": properties["mass"],
                "material": properties["material"],
                "friction": properties["friction"],
            },
        }

        return context

    def _parse(self, data: bytes, file_format: str) -> dict:
        parsers = {
            "stl": lambda d: parse_mesh(d, "stl"),
            "obj": lambda d: parse_mesh(d, "obj"),
            "step": parse_step,
            "stp": parse_step,
            "iges": parse_iges,
            "igs": parse_iges,
            "urdf": parse_urdf,
            "mjcf": parse_mjcf,
        }
        parser = parsers.get(file_format)
        if not parser:
            raise ValueError(f"Unsupported format: {file_format}")
        return parser(data)
