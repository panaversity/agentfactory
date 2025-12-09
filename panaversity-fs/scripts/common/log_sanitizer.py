"""Logging utilities with sensitive data sanitization.

Prevents API keys, tokens, and other secrets from appearing in logs.
"""

import re
import logging
from typing import Any


# Patterns for sensitive data
SENSITIVE_PATTERNS = [
    # API keys in various formats
    (re.compile(r'(api[_-]?key[=:\s]+)[\w\-]+', re.IGNORECASE), r'\1***REDACTED***'),
    # Bearer tokens
    (re.compile(r'(Bearer\s+)[\w\-\.]+', re.IGNORECASE), r'\1***REDACTED***'),
    # Authorization headers
    (re.compile(r'(Authorization[:\s]+)[^\s,\n]+', re.IGNORECASE), r'\1***REDACTED***'),
    # Password fields
    (re.compile(r'(password[=:\s]+)[\w\-]+', re.IGNORECASE), r'\1***REDACTED***'),
    # Token fields
    (re.compile(r'(token[=:\s]+)[\w\-]+', re.IGNORECASE), r'\1***REDACTED***'),
    # Access keys (AWS-style)
    (re.compile(r'(access[_-]?key[=:\s]+)[\w\-]+', re.IGNORECASE), r'\1***REDACTED***'),
    # Secret keys
    (re.compile(r'(secret[_-]?key[=:\s]+)[\w\-]+', re.IGNORECASE), r'\1***REDACTED***'),
]


def sanitize_message(message: str) -> str:
    """Sanitize sensitive data from a log message.

    Args:
        message: Log message that may contain sensitive data

    Returns:
        Sanitized message with secrets redacted
    """
    sanitized = message
    for pattern, replacement in SENSITIVE_PATTERNS:
        sanitized = pattern.sub(replacement, sanitized)
    return sanitized


def sanitize_dict(data: dict[str, Any]) -> dict[str, Any]:
    """Sanitize sensitive data from a dictionary.

    Args:
        data: Dictionary that may contain sensitive values

    Returns:
        New dictionary with sensitive values redacted
    """
    sanitized = {}
    sensitive_keys = {
        'api_key', 'apikey', 'api-key',
        'password', 'passwd', 'pwd',
        'token', 'access_token', 'refresh_token',
        'secret', 'secret_key', 'client_secret',
        'authorization',
    }

    for key, value in data.items():
        if key.lower() in sensitive_keys:
            sanitized[key] = '***REDACTED***'
        elif isinstance(value, dict):
            sanitized[key] = sanitize_dict(value)
        elif isinstance(value, str):
            sanitized[key] = sanitize_message(value)
        else:
            sanitized[key] = value

    return sanitized


class SanitizingFormatter(logging.Formatter):
    """Logging formatter that sanitizes sensitive data."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record with sanitization.

        Args:
            record: Log record to format

        Returns:
            Formatted and sanitized log message
        """
        # Sanitize the message
        original_msg = record.getMessage()
        record.msg = sanitize_message(str(record.msg))

        # Format normally
        formatted = super().format(record)

        # Restore original message (in case record is reused)
        record.msg = original_msg

        return formatted


def get_sanitizing_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """Get a logger with automatic sensitive data sanitization.

    Args:
        name: Logger name (usually __name__)
        level: Logging level (default: INFO)

    Returns:
        Configured logger with sanitizing formatter
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Only add handler if logger doesn't have one
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(SanitizingFormatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        logger.addHandler(handler)

    return logger
