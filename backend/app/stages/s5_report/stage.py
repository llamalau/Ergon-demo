import json
import uuid

from app.tasks.base_stage import BaseStage
from app.services.storage import upload_file
from app.stages.s5_report.score_calculator import calculate_composite_score
from app.stages.s5_report.recommendations import generate_recommendations
from app.stages.s5_report.report_builder import build_report
from app.stages.s5_report.pdf_generator import generate_pdf
from app.core.database import SyncSession
from app.models.report import Report
from app.models.job import Job, JobStatus


class ReportStage(BaseStage):
    stage_name = "report"
    stage_order = 4

    def run(self, context: dict) -> dict:
        job_id = context["job_id"]
        metrics = context.get("metrics", {})
        geometry_info = context.get("geometry_info")
        properties = context.get("physical_properties")
        environment = context.get("environment", "open_space")
        heatmap_key = context.get("heatmap_key")

        self.publish_progress(0.2, "Calculating composite score")
        score_data = calculate_composite_score(metrics)

        self.publish_progress(0.4, "Generating recommendations")
        recommendations = generate_recommendations(metrics, score_data)

        self.publish_progress(0.6, "Building report")
        report_data = build_report(
            job_id=job_id,
            score_data=score_data,
            metrics=metrics,
            recommendations=recommendations,
            geometry_info=geometry_info,
            properties=properties,
            environment=environment,
            heatmap_key=heatmap_key,
        )

        # Store report JSON
        report_key = f"jobs/{job_id}/report.json"
        upload_file(report_key, json.dumps(report_data, default=str).encode(), "application/json")

        self.publish_progress(0.8, "Generating PDF")
        pdf_bytes = generate_pdf(report_data)
        pdf_key = f"jobs/{job_id}/report.pdf"
        upload_file(pdf_key, pdf_bytes, "application/pdf")

        # Save report to database
        self._save_report(
            job_id=job_id,
            overall_score=score_data["overall_score"],
            sub_scores=score_data["sub_scores"],
            metrics={name: {k: v for k, v in m.items() if k != "name"} for name, m in metrics.items() if isinstance(m, dict)},
            recommendations=recommendations,
            pdf_key=pdf_key,
        )

        # Mark job as completed
        self._complete_job(job_id)

        context["report_key"] = report_key
        context["pdf_key"] = pdf_key
        context["stage_result"] = {
            "overall_score": score_data["overall_score"],
            "grade": score_data["grade"],
            "num_recommendations": len(recommendations),
            "pdf_key": pdf_key,
        }

        return context

    def _save_report(self, job_id, overall_score, sub_scores, metrics, recommendations, pdf_key):
        from sqlalchemy import select
        with SyncSession() as db:
            report = Report(
                job_id=uuid.UUID(job_id),
                overall_score=overall_score,
                sub_scores=sub_scores,
                metrics=metrics,
                recommendations=recommendations,
                pdf_storage_key=pdf_key,
            )
            db.add(report)
            db.commit()

    def _complete_job(self, job_id):
        from sqlalchemy import select
        from datetime import datetime, timezone
        with SyncSession() as db:
            result = db.execute(select(Job).where(Job.id == uuid.UUID(job_id)))
            job = result.scalar_one_or_none()
            if job:
                job.status = JobStatus.COMPLETED
                job.completed_at = datetime.now(timezone.utc)
                db.commit()
