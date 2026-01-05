import os
import pytest
import asyncio
# Set environment variables before importing the app
os.environ['OPENROUTER_API_KEY'] = 'test'
os.environ['QDRANT_URL'] = 'test'
os.environ['QDRANT_API_KEY'] = 'test'

from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch
from app.main import app

client = TestClient(app)

@pytest.fixture
def mock_rag_service():
    with patch('app.api.chat.RAGService') as mock:
        yield mock

def test_chat_endpoint_success(mock_rag_service):
    # Mock the RAG service response
    mock_instance = AsyncMock()
    mock_instance.get_response.return_value = {
        "response": "This is a test response",
        "sources": [
            {
                "document_id": "doc-123",
                "title": "Test Document",
                "url": "/docs/test",
                "relevance_score": 0.95
            }
        ]
    }
    mock_rag_service.return_value = mock_instance

    # Test the chat endpoint
    response = client.post(
        "/api/chat",
        json={
            "message": "What is a test?",
            "session_id": "123e4567-e89b-12d3-a456-426614174000"  # Valid UUID
        },
        headers={"Authorization": "Bearer test"}
    )

    assert response.status_code == 200
    data = response.json()
    assert "response" in data
    assert "sources" in data
    assert "session_id" in data
    assert "latency" in data


def test_chat_endpoint_missing_message():
    response = client.post(
        "/api/chat",
        json={},
        headers={"Authorization": "Bearer test"}
    )

    assert response.status_code == 422  # Validation error


def test_chat_endpoint_invalid_session_id():
    response = client.post(
        "/api/chat",
        json={
            "message": "Test message",
            "session_id": "invalid-session-id"
        },
        headers={"Authorization": "Bearer test"}
    )

    # Should still work with invalid session ID format since it's just validated
    # If validation is strict, this might return 400
    assert response.status_code in [200, 400]