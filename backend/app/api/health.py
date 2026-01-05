from fastapi import APIRouter
from pydantic import BaseModel
from datetime import datetime
from app.core.config import settings
from app.services.rag_service import RAGService

router = APIRouter()

class HealthResponse(BaseModel):
    status: str
    models_loaded: bool
    vector_db_connected: bool
    timestamp: str


@router.get("/health", response_model=HealthResponse)
async def health_check():
    # Check if we can connect to the vector database
    try:
        rag_service = RAGService()
        vector_db_connected = await rag_service.check_vector_db_connection()
    except Exception:
        vector_db_connected = False

    # For now, assume models are loaded if we can initialize the service
    models_loaded = True

    return HealthResponse(
        status="healthy" if vector_db_connected else "unhealthy",
        models_loaded=models_loaded,
        vector_db_connected=vector_db_connected,
        timestamp=datetime.utcnow().isoformat()
    )