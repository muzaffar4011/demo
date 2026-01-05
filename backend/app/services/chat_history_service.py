"""
Chat History Service - Manages chat sessions and message persistence
"""
from typing import List, Optional, Dict, Any
from datetime import datetime
from app.models.chat import ChatSession, ChatMessage
from app.core.database import get_qdrant_client
from qdrant_client.http import models
import json


class ChatHistoryService:
    """Service for managing chat history using Qdrant as a simple key-value store"""

    def __init__(self):
        self.client = get_qdrant_client()
        self.sessions_collection = "chat_sessions"
        self.messages_collection = "chat_messages"

    async def ensure_collections_exist(self):
        """Ensure chat history collections exist with proper indexes"""
        collections = await self.client.get_collections()
        collection_names = [col.name for col in collections.collections]

        # Create sessions collection if not exists
        if self.sessions_collection not in collection_names:
            await self.client.create_collection(
                collection_name=self.sessions_collection,
                vectors_config=models.VectorParams(size=1, distance=models.Distance.COSINE)
            )

        # Create messages collection if not exists
        if self.messages_collection not in collection_names:
            await self.client.create_collection(
                collection_name=self.messages_collection,
                vectors_config=models.VectorParams(size=1, distance=models.Distance.COSINE)
            )

            # Create index for session_id to enable filtering
            try:
                await self.client.create_payload_index(
                    collection_name=self.messages_collection,
                    field_name="session_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )
            except Exception as e:
                # Index might already exist, ignore the error
                print(f"Note: Could not create index (might already exist): {e}")
                pass

    async def save_session(self, session: ChatSession) -> ChatSession:
        """Save or update a chat session"""
        await self.ensure_collections_exist()

        session.updated_at = datetime.utcnow()

        await self.client.upsert(
            collection_name=self.sessions_collection,
            points=[
                models.PointStruct(
                    id=session.id,
                    vector=[0.0],  # Dummy vector since we're using it as key-value store
                    payload=session.dict()
                )
            ]
        )
        return session

    async def get_session(self, session_id: str) -> Optional[ChatSession]:
        """Retrieve a chat session by ID"""
        try:
            await self.ensure_collections_exist()

            point = await self.client.retrieve(
                collection_name=self.sessions_collection,
                ids=[session_id]
            )

            if point and len(point) > 0:
                return ChatSession(**point[0].payload)
            return None
        except Exception as e:
            print(f"Error retrieving session {session_id}: {e}")
            return None

    async def save_message(self, message: ChatMessage) -> ChatMessage:
        """Save a chat message"""
        await self.ensure_collections_exist()

        await self.client.upsert(
            collection_name=self.messages_collection,
            points=[
                models.PointStruct(
                    id=message.id,
                    vector=[0.0],  # Dummy vector
                    payload=message.dict()
                )
            ]
        )
        return message

    async def get_session_messages(self, session_id: str) -> List[ChatMessage]:
        """Retrieve all messages for a session"""
        try:
            await self.ensure_collections_exist()

            # Use scroll to get all messages for this session
            result = await self.client.scroll(
                collection_name=self.messages_collection,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="session_id",
                            match=models.MatchValue(value=session_id)
                        )
                    ]
                ),
                limit=1000,  # Max messages per session
                with_payload=True,
                with_vectors=False
            )

            messages = []
            if result and len(result) > 0:
                points = result[0]  # First element contains the points
                for point in points:
                    messages.append(ChatMessage(**point.payload))

            # Sort by created_at
            messages.sort(key=lambda m: m.created_at)
            return messages
        except Exception as e:
            print(f"Error retrieving messages for session {session_id}: {e}")
            import traceback
            traceback.print_exc()
            return []
