import os
# Set environment variables before importing the app
os.environ['OPENROUTER_API_KEY'] = 'test'
os.environ['QDRANT_URL'] = 'test'
os.environ['QDRANT_API_KEY'] = 'test'

from app.main import app
from fastapi.testclient import TestClient

def test_read_root():
    client = TestClient(app)
    response = client.get("/")
    assert response.status_code == 200
    assert "message" in response.json()
    assert "RAG Chatbot API" in response.json()["message"]

def test_health_endpoint():
    client = TestClient(app)
    response = client.get("/api/health")
    assert response.status_code in [200, 503]  # Could be 503 if Qdrant not configured