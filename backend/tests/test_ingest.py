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
def mock_document_service():
    with patch('app.api.ingest.DocumentService') as mock:
        yield mock

def test_ingest_endpoint_success(mock_document_service):
    # Mock the document service response
    mock_instance = AsyncMock()
    mock_instance.ingest_documents.return_value = {
        "documents_processed": 5,
        "chunks_created": 25
    }
    mock_document_service.return_value = mock_instance

    # Test the ingest endpoint
    response = client.post(
        "/api/ingest",
        json={
            "source": "test/source",
            "recursive": True,
            "filters": [".md", ".mdx"],
            "chunk_size": 1000,
            "chunk_overlap": 200
        },
        headers={"Authorization": "Bearer test"}
    )

    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "processing"
    assert "documents_processed" in data
    assert "chunks_created" in data
    assert "processing_time" in data


def test_ingest_endpoint_invalid_source(mock_document_service):
    response = client.post(
        "/api/ingest",
        json={
            "source": "",  # Invalid source
            "recursive": True
        },
        headers={"Authorization": "Bearer test"}
    )

    # Should return 422 for validation error or 500 for processing error
    assert response.status_code in [422, 500]


def test_ingest_endpoint_missing_required_fields():
    response = client.post(
        "/api/ingest",
        json={},
        headers={"Authorization": "Bearer test"}
    )

    assert response.status_code == 422  # Validation error