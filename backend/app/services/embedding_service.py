import asyncio
import httpx
from typing import List, Dict, Any
from app.core.config import settings
from app.models.embedding import EmbeddingModelConfig


class EmbeddingService:
    def __init__(self):
        # Default embedding model configuration
        self.config = EmbeddingModelConfig(
            id="default-embedding-config",
            model_name=settings.EMBEDDING_MODEL,
            dimensions=1536  # Default dimension, will be updated based on actual model
        )

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for the given text using the OpenRouter API directly
        """
        try:
            async with httpx.AsyncClient(timeout=60.0) as client:  # 60 second timeout
                response = await client.post(
                    f"{settings.OPENROUTER_BASE_URL}/embeddings",
                    headers={
                        "Authorization": f"Bearer {settings.OPENROUTER_API_KEY}",
                        "Content-Type": "application/json"
                    },
                    json={
                        "model": self.config.model_name,
                        "input": text
                    }
                )

                if response.status_code != 200:
                    raise Exception(f"API request failed with status {response.status_code}: {response.text}")

                data = response.json()
                embedding = data["data"][0]["embedding"]
                return embedding
        except Exception as e:
            raise Exception(f"Error generating embedding: {str(e)}")

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts
        """
        embeddings = []
        for text in texts:
            embedding = await self.generate_embedding(text)
            embeddings.append(embedding)

        return embeddings