import os
import pytest
import asyncio
# Set environment variables before importing the app
os.environ['OPENROUTER_API_KEY'] = 'test'
os.environ['QDRANT_URL'] = 'test'
os.environ['QDRANT_API_KEY'] = 'test'

from unittest.mock import AsyncMock, patch, MagicMock
from app.services.rag_service import RAGService
from app.services.llm_service import LLMService
from app.services.embedding_service import EmbeddingService
from app.services.document_service import DocumentService


@pytest.fixture
def mock_llm_service():
    with patch('app.services.rag_service.LLMService') as mock:
        yield mock


@pytest.fixture
def mock_embedding_service():
    with patch('app.services.rag_service.EmbeddingService') as mock:
        yield mock


@pytest.fixture
def mock_document_service():
    with patch('app.services.rag_service.DocumentService') as mock:
        yield mock


@pytest.fixture
def mock_qdrant_client():
    with patch('app.services.rag_service.get_qdrant_client') as mock:
        yield mock


@pytest.mark.asyncio
async def test_rag_service_initialization(
    mock_llm_service,
    mock_embedding_service,
    mock_document_service,
    mock_qdrant_client
):
    # Test RAG service initialization
    rag_service = RAGService()

    assert isinstance(rag_service.llm_service, mock_llm_service.return_value.__class__)
    assert isinstance(rag_service.embedding_service, mock_embedding_service.return_value.__class__)
    assert isinstance(rag_service.document_service, mock_document_service.return_value.__class__)
    assert rag_service.qdrant_client == mock_qdrant_client.return_value


@pytest.mark.asyncio
async def test_check_vector_db_connection_success(
    mock_llm_service,
    mock_embedding_service,
    mock_document_service,
    mock_qdrant_client
):
    # Mock the qdrant client to return collections successfully
    mock_client = AsyncMock()
    mock_client.get_collections.return_value = MagicMock()
    mock_qdrant_client.return_value = mock_client

    rag_service = RAGService()
    result = await rag_service.check_vector_db_connection()

    assert result is True
    mock_client.get_collections.assert_called_once()


@pytest.mark.asyncio
async def test_check_vector_db_connection_failure(
    mock_llm_service,
    mock_embedding_service,
    mock_document_service,
    mock_qdrant_client
):
    # Mock the qdrant client to raise an exception
    mock_client = AsyncMock()
    mock_client.get_collections.side_effect = Exception("Connection failed")
    mock_qdrant_client.return_value = mock_client

    rag_service = RAGService()
    result = await rag_service.check_vector_db_connection()

    assert result is False


@pytest.mark.asyncio
async def test_get_response_success(
    mock_llm_service,
    mock_embedding_service,
    mock_document_service,
    mock_qdrant_client
):
    # Mock dependencies
    mock_client = AsyncMock()
    mock_client.search.return_value = [
        MagicMock(
            id="test-id",
            payload={
                "document_id": "doc-123",
                "content": "Test content",
                "title": "Test Document",
                "url": "/docs/test"
            },
            score=0.95
        )
    ]
    mock_qdrant_client.return_value = mock_client

    mock_embedding = AsyncMock()
    mock_embedding.generate_embedding.return_value = [0.1, 0.2, 0.3]
    mock_embedding_service.return_value = mock_embedding

    mock_llm = AsyncMock()
    mock_llm.generate_response.return_value = "Test response from LLM"
    mock_llm_service.return_value = mock_llm

    rag_service = RAGService()

    result = await rag_service.get_response(
        query="Test query",
        context="Test context"
    )

    assert "response" in result
    assert "sources" in result
    assert result["response"] == "Test response from LLM"
    assert len(result["sources"]) == 1
    assert result["sources"][0]["document_id"] == "doc-123"


@pytest.mark.asyncio
async def test_search_relevant_chunks(
    mock_llm_service,
    mock_embedding_service,
    mock_document_service,
    mock_qdrant_client
):
    # Mock the qdrant client search response
    mock_client = AsyncMock()
    search_result = [
        MagicMock(
            id="chunk-1",
            payload={
                "document_id": "doc-123",
                "content": "Relevant content 1",
                "title": "Document 1",
                "url": "/docs/doc1"
            },
            score=0.95
        ),
        MagicMock(
            id="chunk-2",
            payload={
                "document_id": "doc-456",
                "content": "Relevant content 2",
                "title": "Document 2",
                "url": "/docs/doc2"
            },
            score=0.87
        )
    ]
    mock_client.search.return_value = search_result
    mock_qdrant_client.return_value = mock_client

    rag_service = RAGService()
    results = await rag_service.search_relevant_chunks([0.1, 0.2, 0.3], top_k=2)

    assert len(results) == 2
    assert results[0]["id"] == "chunk-1"
    assert results[0]["document_id"] == "doc-123"
    assert results[0]["relevance_score"] == 0.95


def test_format_retrieved_context():
    rag_service = RAGService()

    search_results = [
        {
            "title": "Test Document 1",
            "content": "Content of document 1",
            "relevance_score": 0.95
        },
        {
            "title": "Test Document 2",
            "content": "Content of document 2",
            "relevance_score": 0.87
        }
    ]

    context = rag_service.format_retrieved_context(search_results)

    assert "Relevant information from the Physical AI & Humanoid Robotics book:" in context
    assert "Document: Test Document 1" in context
    assert "Content: Content of document 1" in context


def test_extract_sources():
    rag_service = RAGService()

    search_results = [
        {
            "document_id": "doc-123",
            "title": "Test Document",
            "url": "/docs/test",
            "relevance_score": 0.95
        }
    ]

    sources = rag_service.extract_sources(search_results)

    assert len(sources) == 1
    assert sources[0]["document_id"] == "doc-123"
    assert sources[0]["title"] == "Test Document"
    assert sources[0]["url"] == "/docs/test"
    assert sources[0]["relevance_score"] == 0.95