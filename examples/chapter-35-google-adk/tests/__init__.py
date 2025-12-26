"""
Chapter 35: Google ADK Integration Tests

This package contains comprehensive tests for the Google ADK examples:
- TaskManager agent tests
- Workflow agent tests (Sequential, Parallel, Loop)
- Callback/guardrail tests
- State management tests
- ADK evaluation integration tests

Usage:
    # Run all tests (mocked, no API required)
    pytest tests/

    # Run only unit tests
    pytest tests/ -m "not slow"

    # Run integration tests (requires GOOGLE_API_KEY)
    pytest tests/ -m slow

    # Run specific test file
    pytest tests/test_taskmanager.py -v

Environment:
    GOOGLE_API_KEY: Required for integration tests
    GOOGLE_GENAI_USE_VERTEXAI: Set to TRUE for Vertex AI
"""
