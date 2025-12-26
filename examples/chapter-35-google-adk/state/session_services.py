"""
SessionService Implementations for Google ADK

Demonstrates different session storage backends:
- InMemorySessionService: Development and testing
- FirestoreSessionService: Production persistence with Google Cloud
- VertexAiSessionService: Managed sessions for Agent Engine deployment

Usage:
    # Development
    export ADK_SESSION_BACKEND=memory
    python session_services.py

    # Production with Firestore
    export ADK_SESSION_BACKEND=firestore
    export GOOGLE_CLOUD_PROJECT=your-project-id
    export FIRESTORE_DATABASE=your-database  # Optional, defaults to (default)
    python session_services.py

    # Vertex AI (requires Agent Engine deployment)
    export ADK_SESSION_BACKEND=vertex
    export GOOGLE_CLOUD_PROJECT=your-project-id
    export GOOGLE_CLOUD_LOCATION=us-central1
    python session_services.py
"""

import asyncio
import os
from typing import Literal, Optional

from dotenv import load_dotenv
from google.adk import Runner
from google.adk.agents import Agent
from google.adk.sessions import InMemorySessionService
from google.genai import types

# Load environment variables
load_dotenv()

# Type alias for session backends
SessionBackend = Literal["memory", "firestore", "vertex"]


def create_session_service(
    backend: Optional[SessionBackend] = None,
    project: Optional[str] = None,
    location: Optional[str] = None,
    database: Optional[str] = None,
):
    """
    Factory function to create appropriate SessionService based on environment.

    Args:
        backend: Session backend type. If None, reads from ADK_SESSION_BACKEND env var.
        project: Google Cloud project ID for Firestore/Vertex AI.
        location: Google Cloud location for Vertex AI (e.g., 'us-central1').
        database: Firestore database name (defaults to '(default)').

    Returns:
        SessionService instance configured for the specified backend.

    Raises:
        ValueError: If required configuration is missing for production backends.
        ImportError: If production dependencies are not installed.
    """
    # Determine backend from parameter or environment
    backend = backend or os.getenv("ADK_SESSION_BACKEND", "memory")

    if backend == "memory":
        print("[SessionService] Using InMemorySessionService (development)")
        return InMemorySessionService()

    elif backend == "firestore":
        # Firestore requires google-cloud-firestore
        try:
            from google.adk.sessions import FirestoreSessionService
        except ImportError as e:
            raise ImportError(
                "FirestoreSessionService requires google-cloud-firestore. "
                "Install with: pip install google-cloud-firestore"
            ) from e

        project = project or os.getenv("GOOGLE_CLOUD_PROJECT")
        if not project:
            raise ValueError(
                "GOOGLE_CLOUD_PROJECT environment variable required for Firestore"
            )

        database = database or os.getenv("FIRESTORE_DATABASE", "(default)")

        print(f"[SessionService] Using FirestoreSessionService")
        print(f"  Project: {project}")
        print(f"  Database: {database}")

        return FirestoreSessionService(
            project=project,
            database=database,
        )

    elif backend == "vertex":
        # Vertex AI requires google-cloud-aiplatform
        try:
            from google.adk.sessions import VertexAiSessionService
        except ImportError as e:
            raise ImportError(
                "VertexAiSessionService requires google-cloud-aiplatform. "
                "Install with: pip install google-cloud-aiplatform[adk,agent_engines]"
            ) from e

        project = project or os.getenv("GOOGLE_CLOUD_PROJECT")
        location = location or os.getenv("GOOGLE_CLOUD_LOCATION", "us-central1")

        if not project:
            raise ValueError(
                "GOOGLE_CLOUD_PROJECT environment variable required for Vertex AI"
            )

        print(f"[SessionService] Using VertexAiSessionService")
        print(f"  Project: {project}")
        print(f"  Location: {location}")

        return VertexAiSessionService(
            project=project,
            location=location,
        )

    else:
        raise ValueError(
            f"Unknown session backend: {backend}. "
            "Valid options: memory, firestore, vertex"
        )


async def demonstrate_session_lifecycle():
    """
    Demonstrate full session lifecycle: create, update, get, list, delete.

    This shows how sessions persist across agent interactions and how to
    manage session state programmatically.
    """
    # Create a simple agent
    agent = Agent(
        name="session_demo_agent",
        model="gemini-2.5-flash",
        instruction="You are a helpful assistant. Remember user preferences.",
    )

    # Create session service based on environment
    session_service = create_session_service()

    # Create runner
    runner = Runner(
        app_name="session_demo",
        agent=agent,
        session_service=session_service,
    )

    app_name = "session_demo"
    user_id = "demo_user_123"

    print("\n" + "=" * 60)
    print("SESSION LIFECYCLE DEMONSTRATION")
    print("=" * 60)

    # 1. CREATE SESSION
    print("\n1. Creating new session...")
    session = await session_service.create_session(
        app_name=app_name,
        user_id=user_id,
        state={"theme": "dark", "language": "en"},  # Initial state
    )
    print(f"   Session ID: {session.id}")
    print(f"   User ID: {session.user_id}")
    print(f"   App Name: {session.app_name}")
    print(f"   Initial State: {session.state}")

    # 2. GET SESSION
    print("\n2. Retrieving session...")
    retrieved = await session_service.get_session(
        app_name=app_name,
        user_id=user_id,
        session_id=session.id,
    )
    print(f"   Retrieved Session ID: {retrieved.id}")
    print(f"   State: {retrieved.state}")

    # 3. RUN AGENT (state gets modified via tools)
    print("\n3. Running agent with session...")
    message = types.Content(
        role="user",
        parts=[types.Part(text="Hello! Remember that my favorite color is blue.")],
    )

    response_text = ""
    async for event in runner.run_async(
        user_id=user_id,
        session_id=session.id,
        new_message=message,
    ):
        if event.content and event.content.parts:
            for part in event.content.parts:
                if part.text:
                    response_text += part.text

    print(f"   Agent response: {response_text[:100]}...")

    # 4. LIST SESSIONS
    print("\n4. Listing all sessions for user...")
    sessions = await session_service.list_sessions(
        app_name=app_name,
        user_id=user_id,
    )
    print(f"   Found {len(sessions.sessions)} session(s)")
    for s in sessions.sessions:
        print(f"   - Session: {s.id}")

    # 5. GET UPDATED SESSION STATE
    print("\n5. Getting updated session state...")
    updated_session = await session_service.get_session(
        app_name=app_name,
        user_id=user_id,
        session_id=session.id,
    )
    print(f"   Current State: {updated_session.state}")
    print(f"   Events in history: {len(updated_session.events)}")

    # 6. DELETE SESSION
    print("\n6. Deleting session...")
    await session_service.delete_session(
        app_name=app_name,
        user_id=user_id,
        session_id=session.id,
    )
    print("   Session deleted")

    # Verify deletion
    try:
        await session_service.get_session(
            app_name=app_name,
            user_id=user_id,
            session_id=session.id,
        )
        print("   ERROR: Session still exists!")
    except Exception:
        print("   Verified: Session no longer exists")

    print("\n" + "=" * 60)
    print("SESSION LIFECYCLE COMPLETE")
    print("=" * 60)


async def demonstrate_session_state_sharing():
    """
    Show how state is shared across multiple agent turns within a session.
    """
    session_service = create_session_service()

    agent = Agent(
        name="stateful_agent",
        model="gemini-2.5-flash",
        instruction="""You are a shopping assistant.
        Keep track of items the user wants to buy.
        When asked, summarize their shopping list.""",
    )

    runner = Runner(
        app_name="shopping_app",
        agent=agent,
        session_service=session_service,
    )

    # Create session with empty cart
    session = await session_service.create_session(
        app_name="shopping_app",
        user_id="shopper_1",
        state={"cart": [], "total": 0.0},
    )

    print("\n" + "=" * 60)
    print("MULTI-TURN STATE SHARING")
    print("=" * 60)

    messages = [
        "I want to buy some apples",
        "Also add milk to my cart",
        "What's in my cart so far?",
    ]

    for i, msg in enumerate(messages, 1):
        print(f"\n--- Turn {i} ---")
        print(f"User: {msg}")

        message = types.Content(
            role="user",
            parts=[types.Part(text=msg)],
        )

        response = ""
        async for event in runner.run_async(
            user_id="shopper_1",
            session_id=session.id,
            new_message=message,
        ):
            if event.content and event.content.parts:
                for part in event.content.parts:
                    if part.text:
                        response += part.text

        print(f"Agent: {response[:200]}...")

        # Check session state after each turn
        current_session = await session_service.get_session(
            app_name="shopping_app",
            user_id="shopper_1",
            session_id=session.id,
        )
        print(f"State: {current_session.state}")

    # Cleanup
    await session_service.delete_session(
        app_name="shopping_app",
        user_id="shopper_1",
        session_id=session.id,
    )


if __name__ == "__main__":
    print("Google ADK Session Services Demo")
    print("-" * 40)

    # Run lifecycle demo
    asyncio.run(demonstrate_session_lifecycle())

    # Uncomment to run state sharing demo
    # asyncio.run(demonstrate_session_state_sharing())
