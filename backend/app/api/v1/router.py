from fastapi import APIRouter

from app.api.v1.uploads import router as uploads_router
from app.api.v1.jobs import router as jobs_router
from app.api.v1.environments import router as environments_router
from app.api.v1.playground import router as playground_router

api_router = APIRouter(prefix="/api/v1")
api_router.include_router(uploads_router)
api_router.include_router(jobs_router)
api_router.include_router(environments_router)
api_router.include_router(playground_router)
