import re
import uuid
from typing import Optional


def validate_uuid(uuid_string: str) -> bool:
    """
    Validate if the given string is a valid UUID
    """
    try:
        uuid.UUID(uuid_string)
        return True
    except ValueError:
        return False


def validate_message_length(message: str, min_length: int = 1, max_length: int = 2000) -> bool:
    """
    Validate message length is within specified bounds
    """
    return min_length <= len(message) <= max_length


def validate_context_length(context: Optional[str], max_length: int = 10000) -> bool:
    """
    Validate context length if provided
    """
    if context is None:
        return True
    return len(context) <= max_length


def validate_temperature(temperature: float) -> bool:
    """
    Validate temperature is between 0.0 and 1.0
    """
    return 0.0 <= temperature <= 1.0


def validate_max_tokens(max_tokens: int) -> bool:
    """
    Validate max_tokens is between 100 and 4000
    """
    return 100 <= max_tokens <= 4000


def validate_url(url: str) -> bool:
    """
    Basic URL validation
    """
    pattern = re.compile(
        r'^https?://'  # http:// or https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
        r'localhost|'  # localhost...
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
        r'(?::\d+)?'  # optional port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)
    return pattern.match(url) is not None