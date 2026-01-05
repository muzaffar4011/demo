# Data Model: RAG Chatbot for Physical AI & Humanoid Robotics

## Document Entity
- **Fields**:
  - id (string): unique identifier for the document
  - source_path (string): path to the original Markdown/MDX file
  - title (string): document title
  - content (string): full text content of the document
  - metadata (object): additional information (author, date, etc.)
  - created_at (datetime): timestamp of document ingestion
  - updated_at (datetime): timestamp of last update
- **Validation**: source_path and content are required
- **Relationships**: Contains many DocumentChunks

## DocumentChunk Entity
- **Fields**:
  - id (string): unique identifier for the chunk
  - document_id (string): reference to parent document
  - content (string): chunked text content
  - embedding (array of floats): vector representation of content
  - position (integer): order of chunk in original document
  - metadata (object): additional information (section title, etc.)
  - created_at (datetime): timestamp of chunk creation
- **Validation**: content and embedding are required
- **Relationships**: Belongs to one Document, linked to many similar chunks via vector similarity

## ChatSession Entity
- **Fields**:
  - id (string): unique identifier for the session
  - user_id (string): identifier for the user (optional for anonymous)
  - title (string): session title (auto-generated from first query)
  - created_at (datetime): timestamp of session creation
  - updated_at (datetime): timestamp of last interaction
  - is_active (boolean): whether the session is currently active
- **Validation**: At least one of user_id or session identifier is required
- **Relationships**: Contains many ChatMessages

## ChatMessage Entity
- **Fields**:
  - id (string): unique identifier for the message
  - session_id (string): reference to parent chat session
  - role (enum): "user" or "assistant"
  - content (string): message text content
  - context (string): additional context from page selection (optional)
  - sources (array of objects): references to source documents/chunks
  - created_at (datetime): timestamp of message creation
  - latency (float): time taken to generate response (optional)
- **Validation**: role must be "user" or "assistant", content is required
- **Relationships**: Belongs to one ChatSession

## EmbeddingModelConfig Entity
- **Fields**:
  - id (string): unique identifier for the configuration
  - model_name (string): name of the embedding model (e.g., "qwen/qwen3-embedding-8b")
  - dimensions (integer): number of dimensions in the embedding vector
  - chunk_size (integer): maximum number of tokens per chunk
  - chunk_overlap (integer): number of overlapping tokens between chunks
  - created_at (datetime): timestamp of configuration creation
- **Validation**: model_name and dimensions are required
- **Relationships**: Used by many DocumentChunks for embedding generation

## LLMModelConfig Entity
- **Fields**:
  - id (string): unique identifier for the configuration
  - model_name (string): name of the LLM (e.g., "qwen/qwen3-max")
  - temperature (float): temperature setting for response generation (0.0-1.0)
  - max_tokens (integer): maximum tokens in response
  - top_p (float): nucleus sampling parameter (0.0-1.0)
  - created_at (datetime): timestamp of configuration creation
- **Validation**: model_name is required
- **Relationships**: Used by ChatMessages for response generation

## APIKey Entity
- **Fields**:
  - id (string): unique identifier for the API key
  - key_hash (string): hashed value of the API key
  - provider (string): service provider (e.g., "openrouter", "qdrant")
  - is_active (boolean): whether the key is currently active
  - created_at (datetime): timestamp of key creation
  - last_used_at (datetime): timestamp of last usage
  - rate_limit (integer): requests per minute allowed
- **Validation**: key_hash and provider are required
- **Relationships**: Used by services for external API access

## RateLimit Entity
- **Fields**:
  - id (string): unique identifier for the rate limit record
  - identifier (string): user IP or API key identifier
  - endpoint (string): API endpoint being rate limited
  - requests_count (integer): number of requests in the current window
  - window_start (datetime): start time of the current rate limit window
  - expires_at (datetime): when the rate limit window expires
- **Validation**: identifier and endpoint are required
- **Relationships**: Tracks usage for APIKey entities