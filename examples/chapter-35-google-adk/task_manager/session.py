"""
Session Service Configuration - Environment-based session management.

Provides the appropriate SessionService based on the environment:
- development: InMemorySessionService (fast, ephemeral)
- production: FirestoreSessionService (persistent, scalable)
- vertexai: VertexAiSessionService (managed, production-grade)
"""

import os
from typing import Literal

from google.adk.sessions import InMemorySessionService

# Conditional imports for production services
# These require additional dependencies
try:
    from google.adk.sessions import FirestoreSessionService
    FIRESTORE_AVAILABLE = True
except ImportError:
    FIRESTORE_AVAILABLE = False

try:
    from google.adk.sessions import VertexAiSessionService
    VERTEXAI_AVAILABLE = True
except ImportError:
    VERTEXAI_AVAILABLE = False


EnvironmentType = Literal["development", "production", "vertexai"]


def get_session_service(environment: EnvironmentType = "development"):
    """Get the appropriate SessionService based on environment.

    Args:
        environment: The deployment environment.
            - "development": Uses InMemorySessionService (default)
            - "production": Uses FirestoreSessionService
            - "vertexai": Uses VertexAiSessionService

    Returns:
        Configured SessionService instance.

    Raises:
        ValueError: If required dependencies are not installed.
        ValueError: If required environment variables are missing.

    Environment Variables:
        For "production" (Firestore):
            - GOOGLE_CLOUD_PROJECT: GCP project ID
            - FIRESTORE_DATABASE: Firestore database name (default: "(default)")

        For "vertexai":
            - GOOGLE_CLOUD_PROJECT: GCP project ID
            - GOOGLE_CLOUD_LOCATION: GCP region (default: "us-central1")

    Example:
        # Development (default)
        session_service = get_session_service()

        # Production with Firestore
        session_service = get_session_service("production")

        # Vertex AI managed
        session_service = get_session_service("vertexai")
    """
    if environment == "development":
        return InMemorySessionService()

    elif environment == "production":
        if not FIRESTORE_AVAILABLE:
            raise ValueError(
                "FirestoreSessionService requires google-cloud-firestore. "
                "Install with: pip install google-adk[firestore]"
            )

        project = os.getenv("GOOGLE_CLOUD_PROJECT")
        if not project:
            raise ValueError(
                "GOOGLE_CLOUD_PROJECT environment variable is required "
                "for production environment."
            )

        database = os.getenv("FIRESTORE_DATABASE", "(default)")

        return FirestoreSessionService(
            project=project,
            database=database
        )

    elif environment == "vertexai":
        if not VERTEXAI_AVAILABLE:
            raise ValueError(
                "VertexAiSessionService requires google-cloud-aiplatform. "
                "Install with: pip install google-adk[vertexai]"
            )

        project = os.getenv("GOOGLE_CLOUD_PROJECT")
        if not project:
            raise ValueError(
                "GOOGLE_CLOUD_PROJECT environment variable is required "
                "for Vertex AI environment."
            )

        location = os.getenv("GOOGLE_CLOUD_LOCATION", "us-central1")

        return VertexAiSessionService(
            project=project,
            location=location
        )

    else:
        raise ValueError(
            f"Unknown environment: {environment}. "
            "Must be one of: development, production, vertexai"
        )


def get_environment_from_env() -> EnvironmentType:
    """Detect environment from environment variables.

    Checks ENV or ENVIRONMENT variable, defaulting to "development".

    Returns:
        The detected environment type.
    """
    env = os.getenv("ENV") or os.getenv("ENVIRONMENT", "development")
    env_lower = env.lower()

    if env_lower in ("prod", "production"):
        return "production"
    elif env_lower in ("vertex", "vertexai", "vertex_ai"):
        return "vertexai"
    else:
        return "development"


# Convenience function for auto-detection
def get_auto_session_service():
    """Get SessionService with auto-detected environment.

    Uses get_environment_from_env() to determine the environment,
    then returns the appropriate session service.

    Returns:
        Configured SessionService for the detected environment.
    """
    environment = get_environment_from_env()
    return get_session_service(environment)
