import io
from minio import Minio
from minio.error import S3Error

from app.core.config import settings


def get_minio_client() -> Minio:
    return Minio(
        settings.MINIO_ENDPOINT,
        access_key=settings.MINIO_ROOT_USER,
        secret_key=settings.MINIO_ROOT_PASSWORD,
        secure=settings.MINIO_SECURE,
    )


def ensure_bucket(client: Minio | None = None) -> None:
    client = client or get_minio_client()
    bucket = settings.MINIO_BUCKET
    if not client.bucket_exists(bucket):
        client.make_bucket(bucket)


def upload_file(key: str, data: bytes, content_type: str = "application/octet-stream") -> str:
    client = get_minio_client()
    ensure_bucket(client)
    client.put_object(
        settings.MINIO_BUCKET,
        key,
        io.BytesIO(data),
        length=len(data),
        content_type=content_type,
    )
    return key


def download_file(key: str) -> bytes:
    client = get_minio_client()
    response = client.get_object(settings.MINIO_BUCKET, key)
    try:
        return response.read()
    finally:
        response.close()
        response.release_conn()


def file_exists(key: str) -> bool:
    client = get_minio_client()
    try:
        client.stat_object(settings.MINIO_BUCKET, key)
        return True
    except S3Error:
        return False


def check_minio_health() -> bool:
    try:
        client = get_minio_client()
        client.list_buckets()
        return True
    except Exception:
        return False
