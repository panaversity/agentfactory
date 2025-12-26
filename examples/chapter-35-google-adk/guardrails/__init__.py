"""
Google ADK Guardrails & Callbacks Examples

This module demonstrates production-grade security patterns for AI agents:
- Input validation (before_model_callback)
- Tool protection (before_tool_callback)
- Audit logging (after callbacks)
- Layered defense-in-depth
- Async callback patterns

Each module focuses on one clear pattern with test cases.
"""

from .input_validation import (
    block_dangerous_keywords,
    block_pii_patterns,
    rate_limit_by_user,
    content_length_limit,
)

from .tool_protection import (
    protect_admin_tools,
    validate_tool_arguments,
    restrict_tools_by_role,
)

from .audit_logging import (
    log_model_calls,
    log_tool_executions,
    track_token_usage,
)

from .layered_defense import (
    create_layered_agent,
)

from .async_callbacks import (
    async_content_moderation,
    async_with_timeout,
)

__all__ = [
    # Input validation
    "block_dangerous_keywords",
    "block_pii_patterns",
    "rate_limit_by_user",
    "content_length_limit",
    # Tool protection
    "protect_admin_tools",
    "validate_tool_arguments",
    "restrict_tools_by_role",
    # Audit logging
    "log_model_calls",
    "log_tool_executions",
    "track_token_usage",
    # Layered defense
    "create_layered_agent",
    # Async patterns
    "async_content_moderation",
    "async_with_timeout",
]
