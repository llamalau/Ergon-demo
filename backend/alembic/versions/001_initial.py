"""Initial migration

Revision ID: 001
Revises:
Create Date: 2025-01-01 00:00:00.000000

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects.postgresql import UUID, JSON

revision: str = "001"
down_revision: Union[str, None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    op.create_table(
        "uploads",
        sa.Column("id", UUID(as_uuid=True), primary_key=True),
        sa.Column("filename", sa.String(512), nullable=False),
        sa.Column("original_filename", sa.String(512), nullable=False),
        sa.Column("content_type", sa.String(128), nullable=False),
        sa.Column("file_size", sa.Integer, nullable=False),
        sa.Column("storage_key", sa.String(1024), nullable=False),
        sa.Column("file_format", sa.String(32), nullable=False),
        sa.Column("description", sa.Text, nullable=True),
        sa.Column("created_at", sa.DateTime(timezone=True), server_default=sa.func.now()),
    )

    op.create_table(
        "jobs",
        sa.Column("id", UUID(as_uuid=True), primary_key=True),
        sa.Column("upload_id", UUID(as_uuid=True), sa.ForeignKey("uploads.id"), nullable=False),
        sa.Column("status", sa.Enum("pending", "running", "completed", "failed", name="jobstatus"), default="pending"),
        sa.Column("environment", sa.String(64), default="open_space"),
        sa.Column("config", JSON, nullable=True),
        sa.Column("error_message", sa.Text, nullable=True),
        sa.Column("celery_task_id", sa.String(256), nullable=True),
        sa.Column("created_at", sa.DateTime(timezone=True), server_default=sa.func.now()),
        sa.Column("started_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("completed_at", sa.DateTime(timezone=True), nullable=True),
    )

    op.create_table(
        "job_stages",
        sa.Column("id", UUID(as_uuid=True), primary_key=True),
        sa.Column("job_id", UUID(as_uuid=True), sa.ForeignKey("jobs.id"), nullable=False),
        sa.Column("stage_name", sa.String(64), nullable=False),
        sa.Column("stage_order", sa.Integer, nullable=False),
        sa.Column("status", sa.Enum("pending", "running", "completed", "failed", "skipped", name="stagestatus"), default="pending"),
        sa.Column("progress", sa.Float, default=0.0),
        sa.Column("message", sa.Text, nullable=True),
        sa.Column("result_data", JSON, nullable=True),
        sa.Column("error_message", sa.Text, nullable=True),
        sa.Column("started_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("completed_at", sa.DateTime(timezone=True), nullable=True),
    )

    op.create_table(
        "reports",
        sa.Column("id", UUID(as_uuid=True), primary_key=True),
        sa.Column("job_id", UUID(as_uuid=True), sa.ForeignKey("jobs.id"), unique=True, nullable=False),
        sa.Column("overall_score", sa.Float, nullable=False),
        sa.Column("sub_scores", JSON, nullable=False),
        sa.Column("metrics", JSON, nullable=False),
        sa.Column("recommendations", JSON, nullable=False),
        sa.Column("pdf_storage_key", sa.String(1024), nullable=True),
        sa.Column("created_at", sa.DateTime(timezone=True), server_default=sa.func.now()),
    )

    op.create_table(
        "telemetry_records",
        sa.Column("id", UUID(as_uuid=True), primary_key=True),
        sa.Column("job_id", UUID(as_uuid=True), sa.ForeignKey("jobs.id"), nullable=False),
        sa.Column("trial_index", sa.Integer, nullable=False),
        sa.Column("task_type", sa.String(64), nullable=False),
        sa.Column("storage_key", sa.String(1024), nullable=False),
        sa.Column("video_storage_key", sa.String(1024), nullable=True),
        sa.Column("summary", JSON, nullable=True),
        sa.Column("created_at", sa.DateTime(timezone=True), server_default=sa.func.now()),
    )


def downgrade() -> None:
    op.drop_table("telemetry_records")
    op.drop_table("reports")
    op.drop_table("job_stages")
    op.drop_table("jobs")
    op.drop_table("uploads")
    op.execute("DROP TYPE IF EXISTS jobstatus")
    op.execute("DROP TYPE IF EXISTS stagestatus")
