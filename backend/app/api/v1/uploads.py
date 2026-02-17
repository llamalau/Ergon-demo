import uuid
from pathlib import Path

from fastapi import APIRouter, Depends, HTTPException, UploadFile, File, Form
from fastapi.responses import StreamingResponse
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
import io

from app.core.database import get_db
from app.models.upload import Upload
from app.schemas.upload import UploadResponse
from app.services.storage import upload_file, download_file

router = APIRouter(prefix="/uploads", tags=["uploads"])

ALLOWED_EXTENSIONS = {"stl", "obj", "step", "stp", "iges", "igs", "urdf", "xml", "mjcf"}


def detect_format(filename: str) -> str:
    ext = Path(filename).suffix.lower().lstrip(".")
    format_map = {"stp": "step", "igs": "iges", "xml": "mjcf"}
    return format_map.get(ext, ext)


@router.post("", response_model=UploadResponse, status_code=201)
async def create_upload(
    file: UploadFile = File(...),
    description: str | None = Form(None),
    db: AsyncSession = Depends(get_db),
):
    ext = Path(file.filename or "unknown").suffix.lower().lstrip(".")
    if ext not in ALLOWED_EXTENSIONS:
        raise HTTPException(status_code=400, detail=f"Unsupported file format: .{ext}")

    data = await file.read()
    file_format = detect_format(file.filename or "unknown")
    upload_id = uuid.uuid4()
    storage_key = f"uploads/{upload_id}/{file.filename}"

    upload_file(storage_key, data, content_type=file.content_type or "application/octet-stream")

    record = Upload(
        id=upload_id,
        filename=file.filename or "unknown",
        original_filename=file.filename or "unknown",
        content_type=file.content_type or "application/octet-stream",
        file_size=len(data),
        storage_key=storage_key,
        file_format=file_format,
        description=description,
    )
    db.add(record)
    await db.commit()
    await db.refresh(record)
    return record


@router.get("/{upload_id}", response_model=UploadResponse)
async def get_upload(upload_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(Upload).where(Upload.id == upload_id))
    upload = result.scalar_one_or_none()
    if not upload:
        raise HTTPException(status_code=404, detail="Upload not found")
    return upload


@router.get("/{upload_id}/download")
async def download_upload(upload_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(Upload).where(Upload.id == upload_id))
    upload = result.scalar_one_or_none()
    if not upload:
        raise HTTPException(status_code=404, detail="Upload not found")

    data = download_file(upload.storage_key)
    return StreamingResponse(
        io.BytesIO(data),
        media_type=upload.content_type,
        headers={"Content-Disposition": f'attachment; filename="{upload.original_filename}"'},
    )


@router.get("", response_model=list[UploadResponse])
async def list_uploads(db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(Upload).order_by(Upload.created_at.desc()))
    return result.scalars().all()
