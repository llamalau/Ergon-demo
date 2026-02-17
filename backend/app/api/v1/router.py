from fastapi import APIRouter

from app.api.v1.uploads import router as uploads_router
from app.api.v1.jobs import router as jobs_router
from app.api.v1.environments import router as environments_router
from app.api.v1.reports import router as reports_router

api_router = APIRouter(prefix="/api/v1")
api_router.include_router(uploads_router)
api_router.include_router(jobs_router)
api_router.include_router(environments_router)
api_router.include_router(reports_router)
