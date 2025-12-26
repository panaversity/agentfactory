"""
Multi-User State Management for Google ADK

Demonstrates patterns for managing state in multi-tenant applications:
- User-scoped session isolation
- Shared state across users (with proper access control)
- Session cleanup and expiration
- Rate limiting per user
- Cross-user analytics (aggregate only)

These patterns are essential for production deployments where
multiple users interact with the same agent application.

Usage:
    export GOOGLE_API_KEY=your_api_key
    python multi_user_state.py
"""

import asyncio
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional

from dotenv import load_dotenv
from google.adk import Runner
from google.adk.agents import Agent
from google.adk.sessions import InMemorySessionService
from google.adk.tools.tool_context import ToolContext
from google.genai import types

# Load environment variables
load_dotenv()


# =============================================================================
# USER-SCOPED SESSION MANAGEMENT
# =============================================================================


class MultiUserSessionManager:
    """
    Manages sessions for multiple users with isolation and cleanup.

    Features:
    - Each user gets isolated session state
    - Automatic session expiration
    - Usage tracking per user
    - Cleanup of stale sessions
    """

    def __init__(
        self,
        session_service: InMemorySessionService,
        app_name: str,
        session_ttl_minutes: int = 60,
        max_sessions_per_user: int = 5,
    ):
        self.session_service = session_service
        self.app_name = app_name
        self.session_ttl = timedelta(minutes=session_ttl_minutes)
        self.max_sessions_per_user = max_sessions_per_user
        self._session_metadata: Dict[str, Dict] = {}  # session_id -> metadata

    async def get_or_create_session(
        self,
        user_id: str,
        initial_state: Optional[Dict] = None,
    ):
        """
        Get existing session or create new one for user.

        Enforces max sessions per user by removing oldest if limit reached.
        """
        # Try to get existing active session
        sessions = await self.session_service.list_sessions(
            app_name=self.app_name,
            user_id=user_id,
        )

        # Filter to non-expired sessions
        active_sessions = []
        now = datetime.now()

        for s in sessions.sessions:
            metadata = self._session_metadata.get(s.id, {})
            created_at = metadata.get("created_at", now)
            if now - created_at < self.session_ttl:
                active_sessions.append((s, created_at))

        # Sort by creation time (oldest first)
        active_sessions.sort(key=lambda x: x[1])

        # Enforce max sessions per user
        while len(active_sessions) >= self.max_sessions_per_user:
            oldest_session, _ = active_sessions.pop(0)
            await self.session_service.delete_session(
                app_name=self.app_name,
                user_id=user_id,
                session_id=oldest_session.id,
            )
            print(f"[Cleanup] Removed oldest session {oldest_session.id} for user {user_id}")

        # Return most recent active session or create new
        if active_sessions:
            most_recent = active_sessions[-1][0]
            print(f"[Session] Returning existing session {most_recent.id} for {user_id}")
            return most_recent

        # Create new session
        default_state = {
            "user_id": user_id,
            "created_at": now.isoformat(),
            "interaction_count": 0,
            "preferences": {},
        }
        if initial_state:
            default_state.update(initial_state)

        session = await self.session_service.create_session(
            app_name=self.app_name,
            user_id=user_id,
            state=default_state,
        )

        self._session_metadata[session.id] = {"created_at": now}
        print(f"[Session] Created new session {session.id} for {user_id}")

        return session

    async def cleanup_expired_sessions(self, user_id: Optional[str] = None):
        """
        Remove expired sessions for a user or all users.

        Args:
            user_id: Specific user to clean up, or None for all users
        """
        now = datetime.now()
        cleaned = 0

        # Get all sessions to clean
        sessions_to_check = []

        if user_id:
            user_sessions = await self.session_service.list_sessions(
                app_name=self.app_name,
                user_id=user_id,
            )
            sessions_to_check = [(user_id, s) for s in user_sessions.sessions]
        else:
            # In production, you'd iterate over known users
            # For demo, we track users in metadata
            known_users = set(
                m.get("user_id") for m in self._session_metadata.values() if m.get("user_id")
            )
            for uid in known_users:
                user_sessions = await self.session_service.list_sessions(
                    app_name=self.app_name,
                    user_id=uid,
                )
                sessions_to_check.extend((uid, s) for s in user_sessions.sessions)

        for uid, session in sessions_to_check:
            metadata = self._session_metadata.get(session.id, {})
            created_at = metadata.get("created_at", now)

            if now - created_at > self.session_ttl:
                await self.session_service.delete_session(
                    app_name=self.app_name,
                    user_id=uid,
                    session_id=session.id,
                )
                del self._session_metadata[session.id]
                cleaned += 1

        return cleaned


# =============================================================================
# SHARED VS ISOLATED STATE PATTERNS
# =============================================================================


class SharedStateManager:
    """
    Manages shared state that can be accessed across users.

    Use cases:
    - Feature flags
    - Global configuration
    - Aggregate statistics (anonymized)
    - Rate limit pools
    """

    def __init__(self):
        self._shared_state: Dict[str, Any] = {
            "feature_flags": {
                "new_ui": True,
                "beta_features": False,
            },
            "rate_limits": {
                "requests_per_minute": 60,
                "tokens_per_hour": 100000,
            },
            "aggregate_stats": {
                "total_sessions": 0,
                "total_queries": 0,
            },
        }
        self._user_usage: Dict[str, Dict] = {}  # Per-user rate limiting

    def get_feature_flag(self, flag_name: str) -> bool:
        """Get a feature flag value."""
        return self._shared_state["feature_flags"].get(flag_name, False)

    def get_rate_limit(self, limit_name: str) -> int:
        """Get a rate limit value."""
        return self._shared_state["rate_limits"].get(limit_name, 0)

    def track_usage(self, user_id: str, tokens: int = 0):
        """Track usage for rate limiting."""
        now = datetime.now()
        if user_id not in self._user_usage:
            self._user_usage[user_id] = {
                "minute_count": 0,
                "minute_start": now,
                "hour_tokens": 0,
                "hour_start": now,
            }

        usage = self._user_usage[user_id]

        # Reset minute counter if needed
        if now - usage["minute_start"] > timedelta(minutes=1):
            usage["minute_count"] = 0
            usage["minute_start"] = now

        # Reset hour counter if needed
        if now - usage["hour_start"] > timedelta(hours=1):
            usage["hour_tokens"] = 0
            usage["hour_start"] = now

        usage["minute_count"] += 1
        usage["hour_tokens"] += tokens

        # Update aggregate stats
        self._shared_state["aggregate_stats"]["total_queries"] += 1

    def check_rate_limit(self, user_id: str) -> Dict[str, Any]:
        """Check if user is within rate limits."""
        if user_id not in self._user_usage:
            return {"allowed": True, "reason": "No usage recorded"}

        usage = self._user_usage[user_id]
        limits = self._shared_state["rate_limits"]

        if usage["minute_count"] >= limits["requests_per_minute"]:
            return {
                "allowed": False,
                "reason": "Rate limit exceeded (requests/minute)",
                "reset_in_seconds": 60 - (datetime.now() - usage["minute_start"]).seconds,
            }

        if usage["hour_tokens"] >= limits["tokens_per_hour"]:
            return {
                "allowed": False,
                "reason": "Rate limit exceeded (tokens/hour)",
                "reset_in_seconds": 3600 - (datetime.now() - usage["hour_start"]).seconds,
            }

        return {"allowed": True}

    def get_aggregate_stats(self) -> Dict[str, int]:
        """Get anonymized aggregate statistics."""
        return self._shared_state["aggregate_stats"].copy()


# =============================================================================
# MULTI-TENANT TOOLS
# =============================================================================


# Global shared state manager (in production, use Redis or similar)
_shared_state = SharedStateManager()


def check_feature_flag(feature_name: str, tool_context: ToolContext) -> Dict:
    """
    Check if a feature is enabled.

    Args:
        feature_name: Name of the feature flag to check

    Returns:
        Feature flag status.
    """
    enabled = _shared_state.get_feature_flag(feature_name)
    return {
        "feature": feature_name,
        "enabled": enabled,
    }


def get_user_stats(tool_context: ToolContext) -> Dict:
    """
    Get statistics for the current user's session.

    Returns:
        User's session statistics.
    """
    state = tool_context.state

    return {
        "user_id": state.get("user_id"),
        "session_created": state.get("created_at"),
        "interaction_count": state.get("interaction_count", 0),
        "preferences_set": len(state.get("preferences", {})),
    }


def increment_interaction(tool_context: ToolContext) -> Dict:
    """
    Track an interaction for the current session.

    Returns:
        Updated interaction count.
    """
    current = tool_context.state.get("interaction_count", 0)
    tool_context.state["interaction_count"] = current + 1

    # Also track globally
    user_id = tool_context.state.get("user_id", "unknown")
    _shared_state.track_usage(user_id)

    return {
        "session_interactions": tool_context.state["interaction_count"],
        "rate_limit_status": _shared_state.check_rate_limit(user_id),
    }


def get_global_stats(tool_context: ToolContext) -> Dict:
    """
    Get anonymized global statistics.

    Returns:
        Aggregate statistics (no PII).
    """
    return {
        "aggregate": _shared_state.get_aggregate_stats(),
        "note": "These are anonymized aggregate statistics",
    }


# =============================================================================
# MULTI-TENANT AGENT
# =============================================================================


def create_multi_user_agent() -> Agent:
    """Create an agent configured for multi-user deployments."""
    return Agent(
        name="multi_user_assistant",
        model="gemini-2.5-flash",
        instruction="""You are a helpful assistant in a multi-user environment.

Each user has isolated session state. You can:
- Check feature flags (shared across all users)
- Get user-specific statistics
- Track interactions (for rate limiting)
- View anonymized global stats

Never share one user's data with another.
Always respect rate limits and feature flags.""",
        tools=[
            check_feature_flag,
            get_user_stats,
            increment_interaction,
            get_global_stats,
        ],
    )


# =============================================================================
# DEMONSTRATION
# =============================================================================


async def demonstrate_multi_user():
    """Demonstrate multi-user session management."""

    print("\n" + "=" * 60)
    print("MULTI-USER STATE MANAGEMENT DEMO")
    print("=" * 60)

    session_service = InMemorySessionService()
    session_manager = MultiUserSessionManager(
        session_service=session_service,
        app_name="multi_user_demo",
        session_ttl_minutes=30,
        max_sessions_per_user=3,
    )

    agent = create_multi_user_agent()
    runner = Runner(
        app_name="multi_user_demo",
        agent=agent,
        session_service=session_service,
    )

    # Simulate multiple users
    users = ["alice", "bob", "charlie"]

    print("\n1. Creating sessions for multiple users...")
    print("-" * 40)

    sessions = {}
    for user_id in users:
        session = await session_manager.get_or_create_session(
            user_id=user_id,
            initial_state={"preferences": {"theme": "dark" if user_id == "alice" else "light"}},
        )
        sessions[user_id] = session

    # Simulate interactions
    print("\n2. Users interacting with the agent...")
    print("-" * 40)

    queries = {
        "alice": "What are my user stats?",
        "bob": "Is the new_ui feature enabled?",
        "charlie": "Show me the global statistics",
    }

    for user_id, query in queries.items():
        print(f"\n[{user_id}] Query: {query}")

        message = types.Content(
            role="user",
            parts=[types.Part(text=query)],
        )

        response = ""
        async for event in runner.run_async(
            user_id=user_id,
            session_id=sessions[user_id].id,
            new_message=message,
        ):
            if event.content and event.content.parts:
                for part in event.content.parts:
                    if part.text:
                        response += part.text

        print(f"[{user_id}] Response: {response[:200]}...")

    # Show session isolation
    print("\n3. Verifying session isolation...")
    print("-" * 40)

    for user_id in users:
        session = await session_service.get_session(
            app_name="multi_user_demo",
            user_id=user_id,
            session_id=sessions[user_id].id,
        )
        print(f"[{user_id}] Preferences: {session.state.get('preferences')}")
        print(f"[{user_id}] Interactions: {session.state.get('interaction_count', 0)}")

    # Demonstrate rate limiting
    print("\n4. Rate Limiting...")
    print("-" * 40)

    for _ in range(5):
        _shared_state.track_usage("alice", tokens=100)

    status = _shared_state.check_rate_limit("alice")
    print(f"Alice rate limit status: {status}")

    # Show aggregate stats
    print("\n5. Global Statistics...")
    print("-" * 40)

    stats = _shared_state.get_aggregate_stats()
    print(f"Total queries across all users: {stats['total_queries']}")

    # Cleanup demonstration
    print("\n6. Session Cleanup...")
    print("-" * 40)

    cleaned = await session_manager.cleanup_expired_sessions()
    print(f"Cleaned up {cleaned} expired sessions")


async def demonstrate_session_limits():
    """Demonstrate session limits per user."""

    print("\n" + "=" * 60)
    print("SESSION LIMITS DEMO")
    print("=" * 60)

    session_service = InMemorySessionService()
    session_manager = MultiUserSessionManager(
        session_service=session_service,
        app_name="limits_demo",
        max_sessions_per_user=2,  # Only allow 2 sessions per user
    )

    user_id = "power_user"

    print(f"\nCreating sessions for {user_id} (max 2 allowed)...")

    for i in range(4):
        session = await session_manager.get_or_create_session(
            user_id=user_id,
            initial_state={"session_number": i + 1},
        )
        print(f"  Session {i + 1}: {session.id}")

        # Small delay to ensure different creation times
        await asyncio.sleep(0.1)

    # Verify only 2 sessions exist
    all_sessions = await session_service.list_sessions(
        app_name="limits_demo",
        user_id=user_id,
    )
    print(f"\nActive sessions: {len(all_sessions.sessions)}")
    print("(Oldest sessions were automatically removed)")


if __name__ == "__main__":
    print("Google ADK Multi-User State Management Examples")
    print("-" * 40)

    asyncio.run(demonstrate_multi_user())
    asyncio.run(demonstrate_session_limits())
