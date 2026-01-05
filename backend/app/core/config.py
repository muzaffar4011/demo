from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # OpenRouter configuration
    OPENROUTER_API_KEY: str
    OPENROUTER_BASE_URL: str = "https://openrouter.ai/api/v1"

    # Qdrant configuration
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION_NAME: str = "physical_ai_book"  # Default collection name

    # Backend API authentication
    BACKEND_API_KEY: Optional[str] = None  # Optional for development, set in production

    # Model configuration
    # Using OpenAI embedding model which is widely available on OpenRouter
    EMBEDDING_MODEL: str = "openai/text-embedding-ada-002"
    LLM_MODEL: str = "qwen/qwen3-max"

    # Rate limiting
    RATE_LIMIT_REQUESTS: int = 10  # requests per minute
    RATE_LIMIT_WINDOW: int = 60  # seconds

    # Chunking configuration
    DEFAULT_CHUNK_SIZE: int = 1000
    DEFAULT_CHUNK_OVERLAP: int = 200

    class Config:
        env_file = ".env"


settings = Settings()