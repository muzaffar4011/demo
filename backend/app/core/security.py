from fastapi import HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi import Security
from app.core.config import settings
import time
from typing import Dict, Optional
from collections import defaultdict


class RateLimiter:
    def __init__(self):
        self.requests = defaultdict(list)

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if the identifier is allowed to make a request based on rate limits
        """
        current_time = time.time()
        # Clean old requests outside the window
        self.requests[identifier] = [
            req_time for req_time in self.requests[identifier]
            if current_time - req_time < settings.RATE_LIMIT_WINDOW
        ]

        # Check if we're under the limit
        if len(self.requests[identifier]) < settings.RATE_LIMIT_REQUESTS:
            self.requests[identifier].append(current_time)
            return True

        return False


# Global rate limiter instance
rate_limiter = RateLimiter()

# Security scheme - auto_error=False allows optional authentication in dev mode
security = HTTPBearer(auto_error=False)


async def get_api_key(
    request: Request,
    credentials: Optional[HTTPAuthorizationCredentials] = Security(security)
) -> Optional[str]:
    """
    Validate the API key from the Authorization header.
    If BACKEND_API_KEY is not set, authentication is disabled (development mode).
    """
    # If no backend API key is configured, skip authentication (development mode)
    if not settings.BACKEND_API_KEY:
        return None
    
    # If credentials are not provided, raise error
    if not credentials:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="API key required. Provide it in the Authorization header as: Bearer <your-api-key>"
        )
    
    # Validate the API key
    if credentials.credentials != settings.BACKEND_API_KEY:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API Key"
        )

    # Check rate limit
    client_ip = request.client.host if request.client else "unknown"
    if not rate_limiter.is_allowed(client_ip):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Rate limit exceeded"
        )

    return credentials.credentials