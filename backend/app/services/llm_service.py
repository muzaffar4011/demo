import asyncio
import httpx
from typing import Dict, Any, Optional
from app.core.config import settings
from app.models.embedding import LLMModelConfig


class LLMService:
    def __init__(self):
        # Default LLM model configuration
        self.config = LLMModelConfig(
            id="default-llm-config",
            model_name=settings.LLM_MODEL
        )

    async def generate_response(
        self,
        query: str,
        context: str,
        model_config: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Generate response using the LLM with the provided context
        """
        try:
            # Use provided config or default
            config = model_config or {}
            temperature = config.get("temperature", 0.7)  # Default temperature
            max_tokens = config.get("max_tokens", 1000)  # Default max tokens

            # Create the system message with context
            system_message = f"""
            You are an AI assistant specialized in Physical AI & Humanoid Robotics.
            Use the following information to answer the user's question.
            If the information doesn't contain the answer, say "I don't have enough information from the book to answer this question."
            """

            # Create the full prompt with context and query
            user_message = f"""
            Context information:
            {context}

            User question: {query}
            """

            # Call the OpenRouter API to generate response using HTTP request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{settings.OPENROUTER_BASE_URL}/chat/completions",
                    headers={
                        "Authorization": f"Bearer {settings.OPENROUTER_API_KEY}",
                        "Content-Type": "application/json"
                    },
                    json={
                        "model": self.config.model_name,
                        "messages": [
                            {"role": "system", "content": system_message},
                            {"role": "user", "content": user_message}
                        ],
                        "temperature": temperature,
                        "max_tokens": max_tokens,
                        "top_p": config.get("top_p", 1.0)
                    }
                )

                if response.status_code != 200:
                    raise Exception(f"API request failed with status {response.status_code}: {response.text}")

                data = response.json()
                return data["choices"][0]["message"]["content"].strip()
        except Exception as e:
            raise Exception(f"Error generating response: {str(e)}")