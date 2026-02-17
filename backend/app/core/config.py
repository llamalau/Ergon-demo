from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    PROJECT_NAME: str = "Ergon"
    API_V1_PREFIX: str = "/api/v1"

    # Postgres
    DATABASE_URL: str = "postgresql+asyncpg://ergon:ergon_secret@postgres:5432/ergon"

    # Redis
    REDIS_URL: str = "redis://redis:6379/0"

    # MinIO
    MINIO_ENDPOINT: str = "minio:9000"
    MINIO_ROOT_USER: str = "ergon"
    MINIO_ROOT_PASSWORD: str = "ergon_secret"
    MINIO_BUCKET: str = "ergon-uploads"
    MINIO_SECURE: bool = False

    # Celery
    CELERY_BROKER_URL: str = "redis://redis:6379/0"
    CELERY_RESULT_BACKEND: str = "redis://redis:6379/1"

    # Security
    SECRET_KEY: str = "change-me-in-production"
    BACKEND_CORS_ORIGINS: list[str] = [
        "http://localhost:5173",
        "http://localhost:3000",
    ]

    model_config = {"env_file": ".env", "extra": "ignore"}


settings = Settings()
