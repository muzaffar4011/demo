import asyncio
from typing import Dict, List, Any, Optional
from app.services.llm_service import LLMService
from app.services.embedding_service import EmbeddingService
from app.services.document_service import DocumentService
from app.core.database import get_qdrant_client
from app.core.config import settings
from qdrant_client.http import models
import uuid


class RAGService:
    def __init__(self):
        self.llm_service = LLMService()
        self.embedding_service = EmbeddingService()
        self.document_service = DocumentService()
        self.qdrant_client = get_qdrant_client()

    async def check_vector_db_connection(self) -> bool:
        """Check if we can connect to the vector database"""
        try:
            # Try to list collections to verify connection
            collections = await self.qdrant_client.get_collections()
            return True
        except Exception:
            return False

    async def get_response(
        self,
        query: str,
        context: Optional[str] = None,
        session_id: Optional[str] = None,
        model_config: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Main method to get response from the RAG system
        """
        # Generate embedding for the query
        query_embedding = await self.embedding_service.generate_embedding(query)

        # Search for relevant chunks in the vector database
        search_results = await self.search_relevant_chunks(query_embedding)

        # Format the context from retrieved chunks
        retrieved_context = self.format_retrieved_context(search_results)

        # Combine with any additional context provided by the user
        final_context = retrieved_context
        if context:
            final_context += f"\nAdditional context from selected text: {context}"

        # Generate response using the LLM
        response = await self.llm_service.generate_response(
            query=query,
            context=final_context,
            model_config=model_config
        )

        # Extract source information
        sources = self.extract_sources(search_results)

        return {
            "response": response,
            "sources": sources
        }

    async def search_relevant_chunks(self, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for relevant document chunks in the vector database
        """
        try:
            # Check if collection exists first
            collections = await self.qdrant_client.get_collections()
            collection_names = [col.name for col in collections.collections]
            
            if settings.QDRANT_COLLECTION_NAME not in collection_names:
                # Collection doesn't exist, return empty results
                print(f"Warning: Collection '{settings.QDRANT_COLLECTION_NAME}' not found. Available collections: {collection_names}")
                return []
            
            # Perform semantic search in Qdrant
            # AsyncQdrantClient.query_points takes a vector directly or a Query object
            query_result = await self.qdrant_client.query_points(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                query=query_embedding,  # Can pass vector directly
                limit=top_k,
                with_payload=True
            )

            results = []
            # Extract results from the query response
            # The response has a 'points' attribute with ScoredPoint objects
            for point in query_result.points:
                # ScoredPoint has id, score, and payload
                score = getattr(point, 'score', 0.0)
                
                results.append({
                    "id": str(point.id),
                    "document_id": point.payload.get("document_id", "") if point.payload else "",
                    "content": point.payload.get("content", "") if point.payload else "",
                    "title": point.payload.get("title", "") if point.payload else "",
                    "url": point.payload.get("url", "") if point.payload else "",
                    "relevance_score": float(score) if score else 0.0
                })

            return results
        except Exception as e:
            # Log the error but return empty results instead of crashing
            print(f"Error searching in vector database: {str(e)}")
            import traceback
            traceback.print_exc()
            return []

    def format_retrieved_context(self, search_results: List[Dict[str, Any]]) -> str:
        """
        Format the retrieved chunks into a context string for the LLM
        """
        if not search_results:
            return "No relevant information found in the book."

        context_parts = ["Relevant information from the Physical AI & Humanoid Robotics book:"]
        for result in search_results:
            context_parts.append(f"Document: {result['title']}")
            context_parts.append(f"Content: {result['content']}")
            context_parts.append("---")

        return "\n".join(context_parts)

    def extract_sources(self, search_results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Extract source information from search results
        """
        sources = []
        for result in search_results:
            source = {
                "document_id": result.get("document_id", ""),
                "title": result.get("title", ""),
                "url": result.get("url", ""),
                "relevance_score": result.get("relevance_score", 0.0)
            }
            sources.append(source)

        return sources