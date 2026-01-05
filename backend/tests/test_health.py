import os
import pytest
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
    with patch('app.api.health.RAGService') as mock:
        yield mock

def test_health_endpoint_healthy(mock_rag_service):
    # Mock the RAG service to return a successful connection
    mock_instance = AsyncMock()
    mock_instance.check_vector_db_connection.return_value = True
    mock_rag_service.return_value = mock_instance

    response = client.get("/api/health")

    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert data["models_loaded"] is True
    assert data["vector_db_connected"] is True
    assert "timestamp" in data


def test_health_endpoint_unhealthy(mock_rag_service):
    # Mock the RAG service to return a failed connection
    mock_instance = AsyncMock()
    mock_instance.check_vector_db_connection.return_value = False
    mock_rag_service.return_value = mock_instance

    response = client.get("/api/health")

    assert response.status_code == 200  # Health check itself succeeds
    data = response.json()
    assert data["status"] == "unhealthy"
    assert data["models_loaded"] is True  # Models are considered loaded if service initializes
    assert data["vector_db_connected"] is False
    assert "timestamp" in data