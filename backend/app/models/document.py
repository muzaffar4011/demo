from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime


class Document(BaseModel):
    id: str
    source_path: str
    title: str
    content: str
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class DocumentChunk(BaseModel):
    id: str
    document_id: str
    content: str
    embedding: Optional[list] = None  # Will be populated after embedding generation
    position: int
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)
    created_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }