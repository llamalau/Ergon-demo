import uuid
import io

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.core.database import get_db
from app.models.report import Report
from app.schemas.report import ReportResponse
from app.services.storage import download_file

router = APIRouter(prefix="/reports", tags=["reports"])


@router.get("/{job_id}", response_model=ReportResponse)
async def get_report(job_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(Report).where(Report.job_id == job_id))
    report = result.scalar_one_or_none()
    if not report:
        raise HTTPException(status_code=404, detail="Report not found")
    return report


@router.get("/{job_id}/pdf")
async def get_report_pdf(job_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(Report).where(Report.job_id == job_id))
    report = result.scalar_one_or_none()
    if not report or not report.pdf_storage_key:
        raise HTTPException(status_code=404, detail="Report PDF not found")

    data = download_file(report.pdf_storage_key)
    return StreamingResponse(
        io.BytesIO(data),
        media_type="application/pdf",
        headers={"Content-Disposition": f'attachment; filename="ergon_report_{str(job_id)[:8]}.pdf"'},
    )
