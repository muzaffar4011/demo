# RAG Chatbot for Physical AI & Humanoid Robotics Book

This repository contains a Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics book. The system uses Qwen3 models via OpenRouter API to provide accurate answers based on the book's content.

## Features

- **RAG-based Q&A**: Answers questions based on the full book content using retrieval-augmented generation
- **Contextual Queries**: Supports context from user-selected text on the page
- **Floating Chatbot Widget**: Modern, elegant chatbot interface with speech recognition support
- **Qwen3 Models**: Uses qwen/qwen3-embedding-8b for embeddings and qwen/qwen3-max for question answering
- **Qdrant Vector Database**: Efficient semantic search using Qdrant Cloud
- **Modern UI/UX**: Enhanced landing page with animated hero section and responsive feature cards
- **Dark Mode**: Full dark mode support across all components
- **Speech Recognition**: Voice input capability using Web Speech API
- **Responsive Design**: Optimized for all screen sizes from mobile to desktop

## Prerequisites

- Python 3.11 or higher
- Node.js 18 or higher
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
   cd docusaurus  # or wherever your Docusaurus site is located
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

## Configuration

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

## How to Get Required Keys

### OpenRouter API Key

1. Go to [OpenRouter](https://openrouter.ai/)
2. Sign up for an account
3. Navigate to the API Keys section
4. Create a new API key
5. Ensure you have access to Qwen3 models (qwen/qwen3-embedding-8b and qwen/qwen3-max)

### Qdrant Cloud Account

1. Go to [Qdrant Cloud](https://qdrant.tech/)
2. Sign up for an account
3. Create a new cloud instance
4. Note down the URL and API key
5. The free tier provides sufficient capacity for this implementation

## Testing

### Backend Tests

Run backend tests with pytest:

```bash
cd backend
pytest
```

### Frontend Tests

Run frontend tests:

```bash
cd docusaurus
npm test
```

## Deployment

### Backend Deployment

The backend can be deployed to various platforms:

- **Vercel**: Use the `vercel.json` configuration
- **Heroku**: Use the `Procfile` configuration
- **Docker**: Use the provided `Dockerfile` and `docker-compose.yml`

### Frontend Deployment

The Docusaurus site can be deployed to:

- **GitHub Pages**: Follow Docusaurus deployment guide
- **Vercel**: Use the standard Docusaurus Vercel configuration
- **Netlify**: Use the standard Docusaurus Netlify configuration

## Architecture

The system uses a hybrid architecture:

- **Backend**: FastAPI + LangChain for RAG pipeline, deployed separately
- **Frontend**: React-based floating widget integrated into Docusaurus
- **Vector Database**: Qdrant Cloud for efficient semantic search
- **Models**: Qwen3 models via OpenRouter API

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

## Development

### Adding New Content

1. Add your Markdown/MDX files to the docusaurus/docs directory
2. Re-run the ingestion API endpoint to update the vector database
3. The chatbot will automatically have access to the new content

### Customization

- Modify the chatbot components in `docusaurus/src/components/Chatbot/`
- Adjust the RAG pipeline in `backend/app/services/rag_service.py`
- Update document processing in `backend/app/utils/document_parser.py`

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.