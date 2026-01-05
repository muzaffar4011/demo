from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class EmbeddingModelConfig(BaseModel):
    id: str
    model_name: str = "openai/text-embedding-ada-002"  # Default to OpenAI embedding model
    dimensions: int = 1536  # text-embedding-ada-002 has 1536 dimensions
    chunk_size: int = 1000
    chunk_overlap: int = 200
    created_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class LLMModelConfig(BaseModel):
    id: str
    model_name: str = "qwen/qwen3-max"
    temperature: float = Field(default=0.7, ge=0.0, le=1.0)
    max_tokens: int = Field(default=1000, ge=100, le=4000)
    top_p: float = Field(default=1.0, ge=0.0, le=1.0)
    created_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }