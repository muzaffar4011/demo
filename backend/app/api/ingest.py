from fastapi import APIRouter, HTTPException, BackgroundTasks, Depends
from pydantic import BaseModel, Field
from typing import List, Optional
import time
from app.services.document_service import DocumentService
from app.core.config import settings
from app.core.security import get_api_key

router = APIRouter()

class IngestRequest(BaseModel):
    source: str = Field(..., description="Source directory for documents")
    recursive: bool = Field(default=True, description="Whether to search recursively")
    filters: List[str] = Field(default=[".md", ".mdx"], description="File extensions to include")
    chunk_size: int = Field(default=1000, ge=100, le=4000, description="Size of text chunks")
    chunk_overlap: int = Field(default=200, ge=0, le=1000, description="Overlap between chunks")


class IngestResponse(BaseModel):
    status: str
    documents_processed: int
    chunks_created: int
    processing_time: float


@router.post("/ingest", response_model=IngestResponse)
async def ingest_endpoint(
    request: IngestRequest,
    background_tasks: BackgroundTasks,
    api_key: Optional[str] = Depends(get_api_key)
):
    start_time = time.time()

    try:
        document_service = DocumentService()

        # Process documents
        result = await document_service.ingest_documents(
            source=request.source,
            recursive=request.recursive,
            filters=request.filters,
            chunk_size=request.chunk_size,
            chunk_overlap=request.chunk_overlap
        )

        processing_time = time.time() - start_time

        return IngestResponse(
            status="processing",
            documents_processed=result["documents_processed"],
            chunks_created=result["chunks_created"],
            processing_time=round(processing_time, 2)
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error during ingestion: {str(e)}")