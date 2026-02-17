import json

from app.tasks.base_stage import BaseStage
from app.services.storage import upload_file
from app.stages.s4_analysis.telemetry_processor import aggregate_telemetry
from app.stages.s4_analysis.grasp_quality import compute_grasp_quality
from app.stages.s4_analysis.stability import compute_stability
from app.stages.s4_analysis.force_analysis import compute_force_metrics
from app.stages.s4_analysis.task_completion import compute_task_completion
from app.stages.s4_analysis.failure_modes import analyze_failure_modes
from app.stages.s4_analysis.heatmap_generator import generate_contact_heatmap


class AnalysisStage(BaseStage):
    stage_name = "analysis"
    stage_order = 3

    def run(self, context: dict) -> dict:
        job_id = context["job_id"]
        trial_results = context.get("simulation_results", [])
        visual_mesh_key = context.get("visual_mesh_key", "")

        self.publish_progress(0.1, "Aggregating telemetry data")
        aggregated = aggregate_telemetry(trial_results)

        self.publish_progress(0.3, "Computing grasp quality metrics")
        grasp = compute_grasp_quality(trial_results, aggregated)

        self.publish_progress(0.4, "Computing stability metrics")
        stability = compute_stability(trial_results, aggregated)

        self.publish_progress(0.5, "Analyzing force profiles")
        forces = compute_force_metrics(trial_results, aggregated)

        self.publish_progress(0.6, "Evaluating task completion")
        completion = compute_task_completion(trial_results, aggregated)

        self.publish_progress(0.7, "Analyzing failure modes")
        failures = analyze_failure_modes(trial_results, aggregated)

        self.publish_progress(0.85, "Generating contact heatmap")
        heatmap_key = None
        try:
            heatmap = generate_contact_heatmap(trial_results, visual_mesh_key)
            if "glb_data" in heatmap:
                heatmap_key = f"jobs/{job_id}/heatmap.glb"
                upload_file(heatmap_key, heatmap["glb_data"], "model/gltf-binary")
        except Exception:
            pass  # Heatmap generation is optional

        metrics = {
            "grasp_quality": grasp,
            "stability": stability,
            "force_analysis": forces,
            "task_completion": completion,
            "failure_modes": failures,
        }

        # Store analysis results
        analysis_key = f"jobs/{job_id}/analysis_result.json"
        upload_file(analysis_key, json.dumps(metrics, default=str).encode(), "application/json")

        context["metrics"] = metrics
        context["heatmap_key"] = heatmap_key
        context["analysis_result_key"] = analysis_key
        context["stage_result"] = {
            "grasp_quality_score": grasp["score"],
            "stability_score": stability["score"],
            "force_score": forces["score"],
            "completion_score": completion["score"],
            "num_failures": failures["total_failures"],
            "heatmap_available": heatmap_key is not None,
        }

        return context
