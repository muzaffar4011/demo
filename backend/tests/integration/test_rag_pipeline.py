import os
import pytest
import asyncio
# Set environment variables before importing the app
os.environ['OPENROUTER_API_KEY'] = 'test'
os.environ['QDRANT_URL'] = 'test'
os.environ['QDRANT_API_KEY'] = 'test'

from unittest.mock import AsyncMock, patch, MagicMock
from app.services.rag_service import RAGService


@pytest.mark.asyncio
async def test_full_rag_pipeline():
    """
    Integration test for the full RAG pipeline
    This test mocks external dependencies but tests the internal flow
    """
    with patch('app.services.rag_service.LLMService') as mock_llm_service, \
         patch('app.services.rag_service.EmbeddingService') as mock_embedding_service, \
         patch('app.services.rag_service.DocumentService') as mock_document_service, \
         patch('app.services.rag_service.get_qdrant_client') as mock_qdrant_client:

        # Setup mock objects
        mock_client = AsyncMock()
        search_result = [
            MagicMock(
                id="chunk-1",
                payload={
                    "document_id": "doc-123",
                    "content": "The ROS 2 node is a fundamental unit of a ROS graph.",
                    "title": "ROS Nodes Documentation",
                    "url": "/docs/ros-nodes"
                },
                score=0.98
            )
        ]
        mock_client.search.return_value = search_result
        mock_qdrant_client.return_value = mock_client

        mock_embedding = AsyncMock()
        mock_embedding.generate_embedding.return_value = [0.1, 0.2, 0.3, 0.4]
        mock_embedding_service.return_value = mock_embedding

        mock_llm = AsyncMock()
        mock_llm.generate_response.return_value = "A ROS 2 node is a fundamental unit of a ROS graph that communicates with other nodes via messages."
        mock_llm_service.return_value = mock_llm

        # Create RAG service and test the full pipeline
        rag_service = RAGService()

        # Test the full RAG process
        result = await rag_service.get_response(
            query="What is a ROS 2 node?",
            context=None,
            session_id="test-session"
        )

        # Verify the response structure
        assert "response" in result
        assert "sources" in result
        assert len(result["sources"]) == 1
        assert "ROS 2 node" in result["response"]
        assert result["sources"][0]["document_id"] == "doc-123"
        assert result["sources"][0]["title"] == "ROS Nodes Documentation"

        # Verify that the right methods were called
        mock_embedding.generate_embedding.assert_called_once()
        mock_client.search.assert_called_once()
        mock_llm.generate_response.assert_called_once()


@pytest.mark.asyncio
async def test_rag_pipeline_with_context():
    """
    Test RAG pipeline with additional context from selected text
    """
    with patch('app.services.rag_service.LLMService') as mock_llm_service, \
         patch('app.services.rag_service.EmbeddingService') as mock_embedding_service, \
         patch('app.services.rag_service.DocumentService') as mock_document_service, \
         patch('app.services.rag_service.get_qdrant_client') as mock_qdrant_client:

        # Setup mock objects
        mock_client = AsyncMock()
        search_result = [
            MagicMock(
                id="chunk-1",
                payload={
                    "document_id": "doc-123",
                    "content": "Forward kinematics is the process of calculating the position and orientation of the end effector.",
                    "title": "Forward Kinematics",
                    "url": "/docs/kinematics"
                },
                score=0.92
            )
        ]
        mock_client.search.return_value = search_result
        mock_qdrant_client.return_value = mock_client

        mock_embedding = AsyncMock()
        # Mock embedding for both query and context
        mock_embedding.generate_embedding.return_value = [0.1, 0.2, 0.3, 0.4]
        mock_embedding_service.return_value = mock_embedding

        mock_llm = AsyncMock()
        response_text = "Forward kinematics is the process of calculating the position and orientation of the end effector based on joint angles."
        mock_llm.generate_response.return_value = response_text
        mock_llm_service.return_value = mock_llm

        # Create RAG service and test with context
        rag_service = RAGService()

        result = await rag_service.get_response(
            query="Explain forward kinematics?",
            context="The robot arm has multiple joints that need to be considered.",
            session_id="test-session"
        )

        # Verify the response includes information related to the context
        assert "response" in result
        assert "Forward kinematics" in result["response"]
        assert len(result["sources"]) == 1

        # Verify that the LLM service was called (and thus received the combined context)
        mock_llm.generate_response.assert_called_once()
        args, kwargs = mock_llm.generate_response.call_args
        # The context should be passed to the LLM service