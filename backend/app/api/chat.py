from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
import uuid
import time
from datetime import datetime
from app.services.rag_service import RAGService
from app.services.chat_history_service import ChatHistoryService
from app.models.chat import ChatSession, ChatMessage as ChatMessageModel
from app.core.config import settings
from app.core.security import get_api_key

router = APIRouter()

class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000)
    session_id: Optional[str] = Field(default=None)
    context: Optional[str] = Field(default=None, max_length=10000)
    llm_config: Optional[Dict[str, Any]] = Field(default_factory=dict)


class Source(BaseModel):
    document_id: str
    title: str
    url: str
    relevance_score: float


class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    session_id: str
    latency: float


class ChatHistoryMessage(BaseModel):
    id: str
    role: str
    content: str
    sources: Optional[List[Source]] = []
    created_at: str
    latency: Optional[float] = None


class ChatHistoryResponse(BaseModel):
    session_id: str
    messages: List[ChatHistoryMessage]


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    api_key: Optional[str] = Depends(get_api_key)
):
    start_time = time.time()

    # Validate session_id if provided
    if request.session_id:
        try:
            uuid.UUID(request.session_id)
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid session_id format")

    # Generate session_id if not provided
    session_id = request.session_id or str(uuid.uuid4())

    try:
        # Initialize services
        rag_service = RAGService()
        chat_history_service = ChatHistoryService()

        # Get or create session
        session = await chat_history_service.get_session(session_id)
        if not session:
            session = ChatSession(
                id=session_id,
                title=request.message[:50] if len(request.message) > 50 else request.message,
                created_at=datetime.utcnow()
            )
            await chat_history_service.save_session(session)

        # Save user message
        user_message = ChatMessageModel(
            session_id=session_id,
            role="user",
            content=request.message,
            context=request.context,
            created_at=datetime.utcnow()
        )
        await chat_history_service.save_message(user_message)
        print(f"[CHAT] Saved user message for session {session_id}")

        # Get RAG response
        response = await rag_service.get_response(
            query=request.message,
            context=request.context,
            session_id=session_id,
            model_config=request.llm_config
        )

        latency = time.time() - start_time

        # Save assistant message
        assistant_message = ChatMessageModel(
            session_id=session_id,
            role="assistant",
            content=response["response"],
            sources=response["sources"],
            latency=round(latency, 2),
            created_at=datetime.utcnow()
        )
        await chat_history_service.save_message(assistant_message)
        print(f"[CHAT] Saved assistant message for session {session_id}")

        # Update session
        await chat_history_service.save_session(session)
        print(f"[CHAT] Updated session {session_id}")

        return ChatResponse(
            response=response["response"],
            sources=response["sources"],
            session_id=session_id,
            latency=round(latency, 2)
        )
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Log the full error for debugging
        import traceback
        error_details = traceback.format_exc()
        print(f"Error processing chat request: {error_details}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing chat request: {str(e)}"
        )


@router.get("/chat/history/{session_id}", response_model=ChatHistoryResponse)
async def get_chat_history(
    session_id: str,
    api_key: Optional[str] = Depends(get_api_key)
):
    """Retrieve chat history for a session"""
    try:
        # Validate session_id format
        try:
            uuid.UUID(session_id)
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid session_id format")

        chat_history_service = ChatHistoryService()

        # Get session to verify it exists
        session = await chat_history_service.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        # Get messages
        messages = await chat_history_service.get_session_messages(session_id)

        # Convert to response format
        history_messages = []
        for msg in messages:
            sources = []
            if msg.sources:
                for src in msg.sources:
                    sources.append(Source(**src))

            history_messages.append(ChatHistoryMessage(
                id=msg.id,
                role=msg.role,
                content=msg.content,
                sources=sources,
                created_at=msg.created_at.isoformat(),
                latency=msg.latency
            ))

        return ChatHistoryResponse(
            session_id=session_id,
            messages=history_messages
        )

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        error_details = traceback.format_exc()
        print(f"Error retrieving chat history: {error_details}")
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving chat history: {str(e)}"
        )