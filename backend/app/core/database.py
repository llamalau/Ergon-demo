from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase, sessionmaker, Session

from app.core.config import settings

engine = create_async_engine(settings.DATABASE_URL, echo=False)
async_session = async_sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

# Synchronous engine for Celery workers (asyncpg doesn't work across event loops)
_sync_url = settings.DATABASE_URL.replace("+asyncpg", "+psycopg2")
sync_engine = create_engine(_sync_url, echo=False)
SyncSession = sessionmaker(sync_engine, class_=Session, expire_on_commit=False)


class Base(DeclarativeBase):
    pass


async def get_db() -> AsyncSession:  # type: ignore[misc]
    async with async_session() as session:
        yield session
