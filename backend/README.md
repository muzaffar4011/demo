# Backend API - RAG Chatbot

FastAPI backend for the Physical AI & Humanoid Robotics book RAG chatbot.

> **Note:** Agar koi issues aaye to `TROUBLESHOOTING_URDU.md` file check karein - saare issues aur unke solutions Roman Urdu mein explain kiye gaye hain.

## Prerequisites

- Python 3.10
- [uv](https://github.com/astral-sh/uv) package manager

## Setup

1. Install dependencies:
```bash
uv add -r requirements.txt
```

2. Create a `.env` file in the `backend` directory with the following variables:
```env
OPENROUTER_API_KEY=your_openrouter_api_key
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=physical_ai_book
EMBEDDING_MODEL=openai/text-embedding-ada-002
LLM_MODEL=qwen/qwen3-max

# Optional: Backend API key for authentication (leave empty for development)
# If not set, API endpoints will be accessible without authentication
BACKEND_API_KEY=your_backend_api_key_here
```

## Running the Backend

### Using uv (Recommended)
```bash
uv run uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### Using the virtual environment
```bash
# Activate the virtual environment
.venv\Scripts\activate  # Windows PowerShell
# or
source .venv/bin/activate  # Linux/Mac

# Run the server
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

## API Endpoints

- `GET /` - Root endpoint
- `GET /api/health` - Health check
- `POST /api/ingest` - Ingest documents (requires API key if `BACKEND_API_KEY` is set)
- `POST /api/chat` - Chat with the RAG system (requires API key if `BACKEND_API_KEY` is set)

### Authentication

If `BACKEND_API_KEY` is set in your `.env` file, you need to include it in the `Authorization` header:
```
Authorization: Bearer <your_backend_api_key>
```

If `BACKEND_API_KEY` is not set, authentication is disabled (development mode).

## Ingesting Documents

Before the chatbot can answer questions, you need to ingest your documents into the vector database.

### Using the API

Send a POST request to `/api/ingest`:

```bash
curl -X POST "http://localhost:8000/api/ingest" \
  -H "Content-Type: application/json" \
  -d '{
    "source": "path/to/your/documents",
    "recursive": true,
    "filters": [".md", ".mdx"],
    "chunk_size": 1000,
    "chunk_overlap": 200
  }'
```

Or use the Swagger UI at `http://localhost:8000/docs` to test the endpoint interactively.

**Parameters:**
- `source`: Path to the directory containing your documents (relative to backend directory or absolute path)
- `recursive`: Whether to search subdirectories (default: `true`)
- `filters`: File extensions to include (default: `[".md", ".mdx"]`)
- `chunk_size`: Size of text chunks in characters (default: `1000`)
- `chunk_overlap`: Overlap between chunks in characters (default: `200`)

**Example:**
If your documents are in a `docs` folder in the project root:
```json
{
  "source": "../docs",
  "recursive": true,
  "filters": [".md", ".mdx"]
}
```

## Development

The server will run on `http://localhost:8000` by default.

API documentation is available at:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

