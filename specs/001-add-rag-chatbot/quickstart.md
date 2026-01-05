# Quickstart Guide: RAG Chatbot for Physical AI & Humanoid Robotics

## Prerequisites
- Python 3.11 or higher
- Node.js 18 or higher
- Docker and Docker Compose
- OpenRouter API key with access to Qwen3 models
- Qdrant Cloud account (Free Tier)

## Getting Started

### 1. Clone the Repository
```bash
git clone <repository-url>
cd rag-chatbot
```

### 2. Backend Setup
1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create a virtual environment and install dependencies:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your OpenRouter API key and Qdrant configuration
   ```

### 3. Frontend (Docusaurus) Setup
1. Navigate to the docusaurus directory:
   ```bash
   cd docusaurus
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

### 4. Running the Services

#### Option A: Development Mode (Separate Services)
1. Start the backend:
   ```bash
   cd backend
   uvicorn app.main:app --reload --port 8000
   ```

2. In a separate terminal, start the Docusaurus site:
   ```bash
   cd docusaurus
   npm run start
   ```

#### Option B: Production Mode (Docker)
1. Build and start all services:
   ```bash
   docker-compose up --build
   ```

### 5. Initial Content Ingestion
1. After starting the backend, ingest the book content:
   ```bash
   # This will parse the existing Physical AI & Humanoid Robotics documentation
   curl -X POST http://localhost:8000/api/ingest -H "Content-Type: application/json" -d '{
     "source": "docusaurus/docs",
     "recursive": true
   }'
   ```

### 6. Using the Chatbot
1. Visit your Docusaurus site (typically http://localhost:3000)
2. Look for the floating chatbot widget in the bottom-right corner
3. Ask questions about the Physical AI & Humanoid Robotics content
4. Select text on the page and use it as context for more targeted questions

## Key Configuration Options

### Backend Configuration
- `OPENROUTER_API_KEY`: Your OpenRouter API key
- `QDRANT_URL`: URL for your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `EMBEDDING_MODEL`: Set to "qwen/qwen3-embedding-8b"
- `LLM_MODEL`: Set to "qwen/qwen3-max"
- `RATE_LIMIT_REQUESTS`: Number of requests per minute per user
- `CHUNK_SIZE`: Size of text chunks for embedding (default: 1000)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 200)

### Frontend Configuration
- `CHATBOT_API_URL`: URL of the backend API (default: http://localhost:8000)
- `CHATBOT_WIDGET_POSITION`: Position of the widget (default: bottom-right)

## Development Workflow

### Adding New Content
1. Add your Markdown/MDX files to the docusaurus/docs directory
2. Re-run the ingestion API endpoint to update the vector database
3. The chatbot will automatically have access to the new content

### Testing
1. Run backend tests:
   ```bash
   cd backend
   pytest
   ```

2. Run frontend tests:
   ```bash
   cd docusaurus
   npm test
   ```

### Customization
- Modify the chatbot components in `docusaurus/src/components/Chatbot/`
- Adjust the RAG pipeline in `backend/app/services/rag_service.py`
- Update document processing in `backend/app/utils/document_parser.py`

## Troubleshooting

### Common Issues
- **API Rate Limits**: Check your OpenRouter usage and upgrade if needed
- **Vector Search Poor Results**: Adjust chunk size and overlap parameters
- **Slow Responses**: Verify your Qdrant Cloud instance performance
- **Context Not Working**: Ensure text selection functionality is enabled in your browser

### Performance Tips
- Monitor token usage to manage costs
- Adjust the number of retrieved documents per query based on accuracy needs
- Implement caching for frequently asked questions
- Use appropriate embedding and LLM models for your use case