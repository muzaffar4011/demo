# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics book using Qwen3 models via OpenRouter API, following the research findings in research.md. The solution includes a FastAPI backend with LangChain for the RAG pipeline, Qdrant Cloud for vector storage, and a React-based floating chat widget integrated into the Docusaurus site. The system will use qwen/qwen3-embedding-8b for document embeddings and qwen/qwen3-max for question answering, with support for contextual queries based on user-selected text from the current page.

## Technical Context

**Language/Version**: Python 3.11+, TypeScript/JavaScript for frontend components, compatible with Node.js 18+
**Primary Dependencies**: FastAPI, LangChain, OpenAI Python SDK, Qdrant client, React, Docusaurus
**Storage**: Qdrant Cloud vector database for embeddings, with local fallback option
**Testing**: pytest for backend, Jest for frontend components, integration tests for RAG pipeline
**Target Platform**: Cloud-based backend service (Linux), with Docusaurus frontend integration
**Project Type**: Hybrid web application with separate backend service and frontend Docusaurus integration
**Performance Goals**: <5s response time for queries, 90% accuracy on book-related questions, support 100 concurrent users
**Constraints**: Must integrate with existing Docusaurus site, OpenRouter API rate limits, Qdrant Cloud Free Tier limitations
**Scale/Scope**: Single book knowledge base (Physical AI & Humanoid Robotics), horizontal scaling capability for additional books

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

For educational documentation projects:
- All content must prioritize clarity and accessibility for students with varying technical backgrounds
- Technical accuracy must be validated through testing or expert review
- Documentation must follow logical progression from foundational to advanced concepts
- Content must use inclusive language and accommodate diverse learning styles
- All code examples must include clear explanations and expected outputs
- Visual aids must enhance understanding with proper alt-text for accessibility
- Content must follow Docusaurus Markdown/MDX standards with proper navigation structure

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### RAG Chatbot Backend Service
```text
backend/
├── app/
│   ├── main.py              # FastAPI application entrypoint
│   ├── api/
│   │   ├── __init__.py
│   │   ├── chat.py          # Chat endpoint with RAG functionality
│   │   └── ingest.py        # Document ingestion endpoint
│   ├── models/
│   │   ├── __init__.py
│   │   ├── document.py      # Document and chunk data models
│   │   ├── chat.py          # Chat session and message models
│   │   └── embedding.py     # Embedding configuration models
│   ├── services/
│   │   ├── __init__.py
│   │   ├── rag_service.py   # Core RAG service implementation
│   │   ├── embedding_service.py # Embedding generation and management
│   │   ├── document_service.py # Document processing and storage
│   │   └── llm_service.py   # LLM interaction service
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config.py        # Application configuration
│   │   ├── database.py      # Qdrant connection management
│   │   └── security.py      # API key management and rate limiting
│   └── utils/
│       ├── __init__.py
│       ├── document_parser.py # Markdown/MDX parsing utilities
│       └── validators.py     # Input validation utilities
├── tests/
│   ├── __init__.py
│   ├── test_chat.py         # Chat API tests
│   ├── test_ingest.py       # Ingestion API tests
│   ├── test_rag_service.py  # RAG service unit tests
│   └── integration/         # Integration tests
│       ├── __init__.py
│       └── test_rag_pipeline.py
├── requirements.txt         # Python dependencies
├── Dockerfile              # Container configuration
└── docker-compose.yml      # Service orchestration
```

### Docusaurus Frontend Integration
```text
docusaurus/
├── src/
│   ├── components/
│   │   ├── Chatbot/
│   │   │   ├── Chatbot.tsx          # Main chatbot component
│   │   │   ├── ChatbotWidget.tsx    # Floating widget implementation
│   │   │   ├── ChatWindow.tsx       # Chat interface
│   │   │   ├── Message.tsx          # Individual message display
│   │   │   └── InputArea.tsx        # Message input with context support
│   │   └── Theme/
│   │       └── Root.tsx             # Theme wrapper for chatbot initialization
│   └── theme/
│       └── MDXComponents.js         # Enhanced MDX components with text selection support
├── static/
│   └── js/
│       └── chatbot-context.js       # Client-side context capture
└── docusaurus.config.js             # Updated with chatbot plugin
```

**Structure Decision**: Hybrid architecture with separate backend service for RAG processing and frontend components for Docusaurus integration. This allows independent scaling and maintenance of the RAG system while providing seamless integration with the existing documentation site.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
