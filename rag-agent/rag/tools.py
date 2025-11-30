"""
RAG search tool for OpenAI Agents/ChatKit integration.

Provides a function tool that agents can call to search educational content.
"""

from typing import Optional
import logging
from models.search import SearchQuery, SearchResponse
from core.config import get_settings
from .search import RoboLearnSearch

logger = logging.getLogger(__name__)


# Global search client instance
_search_client: Optional[RoboLearnSearch] = None
_search_client_error: Optional[str] = None


def get_search_client() -> Optional[RoboLearnSearch]:
    """Get or create the search client instance. Returns None if Qdrant is unavailable."""
    global _search_client, _search_client_error
    if _search_client is not None:
        return _search_client
    if _search_client_error is not None:
        return None  # Already failed, don't retry
    
    try:
        _search_client = RoboLearnSearch()
        return _search_client
    except Exception as e:
        _search_client_error = str(e)
        logger.warning(f"Failed to initialize search client: {e}. RAG search will be unavailable.")
        return None


def search_tool(
    query: str,
    hardware_tier: Optional[int] = None,
    module: Optional[str] = None,
    chapter_min: Optional[int] = None,
    chapter_max: Optional[int] = None,
    lesson: Optional[int] = None,
    limit: int = 5,
) -> dict:
    """
    Search tool for agents to query RoboLearn educational content.
    
    This function can be registered as a tool with OpenAI Agents SDK or ChatKit.
    
    Args:
        query: Search query text (required, min 3 characters)
        hardware_tier: Optional hardware tier filter (1-4)
        module: Optional module filter (ros2, gazebo, isaac, vla)
        chapter_min: Optional minimum chapter number (0-20)
        chapter_max: Optional maximum chapter number (0-20)
        lesson: Optional specific lesson number (0-15)
        limit: Maximum number of results (1-20, default 5)
    
    Returns:
        dict: Search response with results, citations, and lesson URLs
        
    Example:
        >>> result = search_tool("How do I create a ROS 2 node?")
        >>> print(result["results"][0]["citation"])
        "Workspaces and Packages | Module ROS2"
    """
    try:
        client = get_search_client()
        if client is None:
            return {
                "error": "Search service unavailable (Qdrant connection failed)",
                "query": query,
                "results": [],
                "total_found": 0,
            }
        
        settings = get_settings()
        search_query = SearchQuery(
            text=query,
            book_id=settings.book_id,
            hardware_tier_filter=hardware_tier,
            module_filter=module,
            chapter_min=chapter_min,
            chapter_max=chapter_max,
            lesson_filter=lesson,
            limit=limit,
        )
        
        response = client.search(search_query)
        
        # Convert Pydantic model to dict for tool response
        return response.model_dump()
    except Exception as e:
        logger.error(f"Search tool error: {e}", exc_info=True)
        return {
            "error": str(e),
            "query": query,
            "results": [],
            "total_found": 0,
        }


# Tool schema for OpenAI Agents SDK / ChatKit registration
SEARCH_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "search_robolearn_content",
        "description": "Search RoboLearn educational content including lessons, chapters, and modules. Returns relevant content chunks with citations and navigation URLs.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "Search query text (e.g., 'How do I create a ROS 2 node?')",
                    "minLength": 3,
                },
                "hardware_tier": {
                    "type": "integer",
                    "description": "Optional: Filter by hardware tier (1-4). Lower tiers include higher tier content.",
                    "minimum": 1,
                    "maximum": 4,
                },
                "module": {
                    "type": "string",
                    "description": "Optional: Filter by module (ros2, gazebo, isaac, vla)",
                    "enum": ["ros2", "gazebo", "isaac", "vla"],
                },
                "chapter_min": {
                    "type": "integer",
                    "description": "Optional: Minimum chapter number (inclusive)",
                    "minimum": 0,
                    "maximum": 20,
                },
                "chapter_max": {
                    "type": "integer",
                    "description": "Optional: Maximum chapter number (inclusive)",
                    "minimum": 0,
                    "maximum": 20,
                },
                "lesson": {
                    "type": "integer",
                    "description": "Optional: Specific lesson number",
                    "minimum": 0,
                    "maximum": 15,
                },
                "limit": {
                    "type": "integer",
                    "description": "Maximum number of results to return",
                    "minimum": 1,
                    "maximum": 20,
                    "default": 5,
                },
            },
            "required": ["query"],
        },
    },
}

