"""
State Management Examples for Google ADK

This module demonstrates state management patterns for ADK agents:
- SessionService implementations (InMemory, Firestore, VertexAI)
- Stateful tools with ToolContext
- State migration between backends
- MCP integration patterns
- Multi-user/tenant state isolation
"""

from .session_services import (
    create_session_service,
    demonstrate_session_lifecycle,
)
from .stateful_tools import (
    get_user_preference,
    set_user_preference,
    add_to_history,
    get_history,
    create_stateful_agent,
)
from .mcp_integration import (
    create_mcp_filesystem_agent,
    create_mcp_with_native_tools_agent,
)

__all__ = [
    # Session services
    "create_session_service",
    "demonstrate_session_lifecycle",
    # Stateful tools
    "get_user_preference",
    "set_user_preference",
    "add_to_history",
    "get_history",
    "create_stateful_agent",
    # MCP integration
    "create_mcp_filesystem_agent",
    "create_mcp_with_native_tools_agent",
]
