import logging
import os

from dotenv import load_dotenv
from fastapi import FastAPI
from contextlib import asynccontextmanager

from chatkit_store import (
    PostgresStore,
    CachedPostgresStore,
    StoreConfig,
)
from chatkit_server import create_chatkit_server
from core.config import get_settings

logger = logging.getLogger(__name__)

load_dotenv()

# Global store and server instances
postgres_store: PostgresStore | CachedPostgresStore | None = None
chatkit_server = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage app lifecycle including ChatKit."""
    global postgres_store, chatkit_server

    # Startup
    logger.info("Starting RoboLearn RAG agent application...")
    
    # Initialize RAG search client (non-blocking, fails gracefully)
    try:
        from rag.tools import get_search_client
        client = get_search_client()
        if client is not None:
            logger.info("RAG search client initialized")
        else:
            logger.warning("RAG search client unavailable (Qdrant connection failed)")
    except Exception as e:
        logger.warning(f"RAG initialization failed (non-critical): {e}")

    # Initialize PostgreSQL store for ChatKit
    try:
        settings = get_settings()
        
        # Check for database URL - prefer CHATKIT_STORE_DATABASE_URL, fallback to DATABASE_URL
        chatkit_db_url = os.getenv("CHATKIT_STORE_DATABASE_URL")
        database_url = chatkit_db_url or settings.database_url
        
        if not database_url:
            logger.warning("DATABASE_URL or CHATKIT_STORE_DATABASE_URL not configured - ChatKit will not be available")
            yield
            return
        
        # Use CachedPostgresStore if Redis is available, otherwise use PostgresStore
        # For now, use PostgresStore without Redis caching
        logger.info("Initializing PostgreSQL store for ChatKit")
        
        # Create store config
        # If CHATKIT_STORE_DATABASE_URL is set, StoreConfig will read it automatically
        # Otherwise, pass database_url explicitly
        if chatkit_db_url:
            # StoreConfig will automatically read CHATKIT_STORE_DATABASE_URL from env
            store_config = StoreConfig()
        else:
            # Pass database_url explicitly from DATABASE_URL
            store_config = StoreConfig(
                database_url=database_url,
            )
        postgres_store = PostgresStore(config=store_config)

        await postgres_store.initialize_schema()
        logger.info("PostgreSQL store initialized successfully")

        # Pre-warm connection pool to avoid 7+ second first request
        logger.info("Pre-warming database connections...")
        await postgres_store._warm_connection_pool()
        logger.info("Database connections warmed up")

        # Create ChatKit server
        chatkit_server = create_chatkit_server(postgres_store)
        logger.info("ChatKit server created successfully")

        # Store in app state for access in routes
        app.state.postgres_store = postgres_store
        app.state.chatkit_server = chatkit_server

    except Exception as e:
        logger.error(f"Failed to initialize ChatKit stores: {e}")
        logger.warning("ChatKit endpoints will not be available")

    yield

    # Shutdown
    logger.info("Shutting down application...")

    # Close PostgreSQL store
    if postgres_store:
        try:
            await postgres_store.close()
            logger.info("PostgreSQL store closed")
        except Exception as e:
            logger.error(f"Error closing PostgreSQL store: {e}")

