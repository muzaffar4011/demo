from qdrant_client import AsyncQdrantClient
from app.core.config import settings


def get_qdrant_client() -> AsyncQdrantClient:
    """
    Create and return an AsyncQdrantClient instance
    """
    client = AsyncQdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )
    return client