from contextlib import asynccontextmanager

import redis.asyncio as aioredis
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy import text

from app.core.config import settings
from app.core.database import engine
from app.core.logging import setup_logging, get_logger
from app.api.v1.router import api_router
from app.services.storage import ensure_bucket, check_minio_health

setup_logging()
log = get_logger("ergon")


async def _create_tables():
    """Create all database tables if they don't exist."""
    from app.core.database import Base
    from app.models import *  # noqa: F401,F403 — ensure all models registered
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


@asynccontextmanager
async def lifespan(app: FastAPI):
    log.info("starting_up", service=settings.PROJECT_NAME)
    await _create_tables()
    log.info("tables_ready")
    ensure_bucket()
    log.info("startup_complete")
    yield
    log.info("shutting_down")
    await engine.dispose()


app = FastAPI(title=settings.PROJECT_NAME, lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.BACKEND_CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


app.include_router(api_router)


@app.get("/health")
async def health():
    checks = {}

    # Database
    try:
        async with engine.connect() as conn:
            await conn.execute(text("SELECT 1"))
        checks["database"] = "ok"
    except Exception as e:
        checks["database"] = f"error: {e}"

    # Redis
    try:
        r = aioredis.from_url(settings.REDIS_URL)
        await r.ping()
        await r.aclose()
        checks["redis"] = "ok"
    except Exception as e:
        checks["redis"] = f"error: {e}"

    # MinIO
    checks["minio"] = "ok" if check_minio_health() else "error"

    all_ok = all(v == "ok" for v in checks.values())
    return {
        "status": "healthy" if all_ok else "degraded",
        "service": settings.PROJECT_NAME,
        "checks": checks,
    }
