from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime
import uuid


class ChatSession(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    user_id: Optional[str] = None
    title: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    is_active: bool = True

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class ChatMessage(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    session_id: str
    role: str  # "user" or "assistant"
    content: str
    context: Optional[str] = None
    sources: Optional[List[Dict[str, Any]]] = Field(default_factory=list)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    latency: Optional[float] = None

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    def __init__(self, **data):
        super().__init__(**data)
        # Validate role
        if self.role not in ["user", "assistant"]:
            raise ValueError(f"Role must be 'user' or 'assistant', got {self.role}")