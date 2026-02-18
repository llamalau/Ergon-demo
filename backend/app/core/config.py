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

    # VLM (Vision-Language Model) for high-level planner
    VLM_PROVIDER: str = ""  # "anthropic" or "openai"; empty = disabled
    VLM_API_KEY: str = ""
    VLM_MODEL: str = "claude-sonnet-4-5-20250929"

    # Low-level controller (OpenPI inference server)
    CONTROL_MODEL_URL: str = "http://localhost:8001"

    # SLURM cluster
    SLURM_HOST: str = ""  # e.g. "lreddy3@oddjobs.bmi.emory.edu"

    # Simulation defaults
    DEFAULT_MAX_ATTEMPTS: int = 3
    DEFAULT_MAX_STEPS: int = 5000

    # Low-level control model (OpenPI inference server on SLURM)
    CONTROL_MODEL_URL: str = "http://localhost:8001"
    CONTROL_MODEL_TIMEOUT: float = 5.0

    # Task execution
    DEFAULT_MAX_ATTEMPTS: int = 3
    DEFAULT_MAX_STEPS: int = 5000

    # Security
    SECRET_KEY: str = "change-me-in-production"
    BACKEND_CORS_ORIGINS: list[str] = [
        "http://localhost:5173",
        "http://localhost:3000",
    ]

    model_config = {"env_file": ".env", "extra": "ignore"}


settings = Settings()
