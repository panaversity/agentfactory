"""
Async Callback Patterns

Demonstrates async callback patterns for scenarios requiring:
- External API calls (content moderation services)
- Database lookups
- Timeout handling
- Concurrent validation checks

Key Pattern:
- ADK supports both sync and async callbacks
- Async callbacks allow non-blocking I/O operations
- Always implement timeouts to prevent hanging

Security Rationale:
Async callbacks enable integration with external security services
(content moderation APIs, threat detection, etc.) without blocking
the agent's processing pipeline.
"""

import asyncio
import time
from typing import Optional, Dict, Any
from dataclasses import dataclass

from google.adk.agents.callback_context import CallbackContext
from google.adk.models.llm_request import LlmRequest
from google.adk.models.llm_response import LlmResponse
from google.adk.tools.base_tool import BaseTool
from google.adk.tools.tool_context import ToolContext
from google.genai import types


# =============================================================================
# Pattern 1: Async Content Moderation
# =============================================================================

@dataclass
class ModerationResult:
    """Result from content moderation service."""
    is_safe: bool
    categories: Dict[str, float]
    flagged_categories: list[str]


async def _call_moderation_api(text: str) -> ModerationResult:
    """
    Simulate calling an external content moderation API.

    In production, this would call:
    - Google Cloud Natural Language API
    - OpenAI Moderation API
    - Perspective API
    - Custom moderation service
    """
    # Simulate API latency
    await asyncio.sleep(0.05)  # 50ms

    # Simulate moderation check
    text_lower = text.lower()
    categories = {
        "hate": 0.1 if "hate" not in text_lower else 0.9,
        "violence": 0.1 if "kill" not in text_lower else 0.85,
        "sexual": 0.05,
        "harassment": 0.1 if "harass" not in text_lower else 0.8,
        "self_harm": 0.02,
    }

    # Flag categories above threshold
    threshold = 0.7
    flagged = [cat for cat, score in categories.items() if score > threshold]

    return ModerationResult(
        is_safe=len(flagged) == 0,
        categories=categories,
        flagged_categories=flagged
    )


async def async_content_moderation(
    callback_context: CallbackContext,
    llm_request: LlmRequest
) -> Optional[LlmResponse]:
    """
    Async before_model_callback for content moderation.

    Calls external moderation API to check content safety.
    This is non-blocking and allows other operations to proceed.

    Returns:
        LlmResponse if content is unsafe
        None if content is safe
    """
    # Extract user message
    user_message = ""
    if llm_request.contents:
        for content in reversed(llm_request.contents):
            if content.role == "user" and content.parts:
                for part in content.parts:
                    if part.text:
                        user_message = part.text
                        break
                break

    if not user_message:
        return None  # No message to moderate

    try:
        # Call moderation API asynchronously
        result = await _call_moderation_api(user_message)

        if not result.is_safe:
            flagged = ", ".join(result.flagged_categories)
            return LlmResponse(
                content=types.Content(
                    role="model",
                    parts=[types.Part(
                        text=f"I cannot process this request. The content was "
                             f"flagged for: {flagged}. Please rephrase your message."
                    )]
                )
            )

        return None  # Content is safe

    except Exception as e:
        # Log error but allow request (fail open for availability)
        # In high-security contexts, you might want to fail closed
        print(f"Moderation API error: {e}")
        return None


# =============================================================================
# Pattern 2: Async with Timeout
# =============================================================================

class TimeoutError(Exception):
    """Custom timeout error for callbacks."""
    pass


async def _slow_external_check(text: str, delay: float = 2.0) -> bool:
    """Simulate a slow external service."""
    await asyncio.sleep(delay)
    return True


async def async_with_timeout(
    callback_context: CallbackContext,
    llm_request: LlmRequest,
    timeout_seconds: float = 1.0
) -> Optional[LlmResponse]:
    """
    Async callback with timeout protection.

    Prevents external service delays from blocking the agent.
    If timeout occurs, the request proceeds (fail open).

    Args:
        callback_context: ADK callback context
        llm_request: The incoming request
        timeout_seconds: Maximum time to wait for external check

    Returns:
        LlmResponse if blocked
        None if allowed (including timeout)
    """
    user_message = _extract_message(llm_request)

    try:
        # Wrap external call in timeout
        result = await asyncio.wait_for(
            _slow_external_check(user_message),
            timeout=timeout_seconds
        )

        if not result:
            return LlmResponse(
                content=types.Content(
                    role="model",
                    parts=[types.Part(text="Request blocked by external check.")]
                )
            )

        return None  # Allowed

    except asyncio.TimeoutError:
        # Log timeout but allow request to proceed
        print(f"External check timed out after {timeout_seconds}s, allowing request")
        return None

    except Exception as e:
        print(f"External check error: {e}")
        return None


# =============================================================================
# Pattern 3: Concurrent Validation Checks
# =============================================================================

async def _check_blocklist(text: str) -> bool:
    """Check against blocklist database."""
    await asyncio.sleep(0.02)
    blocked_phrases = ["spam", "scam", "phishing"]
    return not any(phrase in text.lower() for phrase in blocked_phrases)


async def _check_rate_limit_async(user_id: str) -> bool:
    """Check rate limit from distributed cache (Redis)."""
    await asyncio.sleep(0.01)
    # Simulate Redis lookup
    return True  # Not rate limited


async def _check_user_reputation(user_id: str) -> float:
    """Get user reputation score from database."""
    await asyncio.sleep(0.03)
    # Simulate database lookup
    return 0.8  # 80% reputation


async def concurrent_validation(
    callback_context: CallbackContext,
    llm_request: LlmRequest
) -> Optional[LlmResponse]:
    """
    Run multiple validation checks concurrently.

    This pattern is efficient when you have multiple independent checks
    that can run in parallel rather than sequentially.

    Benefits:
    - Faster overall validation (parallel vs sequential)
    - Efficient use of I/O bound operations
    - Single callback handles all checks
    """
    user_id = callback_context.invocation_context.user_id or "anonymous"
    user_message = _extract_message(llm_request)

    # Run all checks concurrently
    try:
        results = await asyncio.gather(
            _check_blocklist(user_message),
            _check_rate_limit_async(user_id),
            _check_user_reputation(user_id),
            return_exceptions=True  # Don't fail if one check errors
        )

        blocklist_ok, rate_limit_ok, reputation = results

        # Handle any exceptions
        if isinstance(blocklist_ok, Exception):
            blocklist_ok = True  # Fail open
        if isinstance(rate_limit_ok, Exception):
            rate_limit_ok = True
        if isinstance(reputation, Exception):
            reputation = 0.5  # Default reputation

        # Check blocklist
        if not blocklist_ok:
            return LlmResponse(
                content=types.Content(
                    role="model",
                    parts=[types.Part(text="Your message contains blocked content.")]
                )
            )

        # Check rate limit
        if not rate_limit_ok:
            return LlmResponse(
                content=types.Content(
                    role="model",
                    parts=[types.Part(text="Rate limit exceeded. Please wait.")]
                )
            )

        # Check reputation
        if reputation < 0.3:
            return LlmResponse(
                content=types.Content(
                    role="model",
                    parts=[types.Part(
                        text="Your account has been flagged. Please contact support."
                    )]
                )
            )

        return None  # All checks passed

    except Exception as e:
        print(f"Concurrent validation error: {e}")
        return None


# =============================================================================
# Pattern 4: Async Tool Callback
# =============================================================================

async def async_tool_authorization(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext
) -> Optional[Dict]:
    """
    Async before_tool_callback for checking authorization.

    Useful when authorization requires:
    - Database lookup for permissions
    - External IAM service call
    - Dynamic policy evaluation
    """
    user_id = tool_context.state.get("user_id", "anonymous")

    try:
        # Simulate async permission check
        has_permission = await _check_tool_permission(user_id, tool.name)

        if not has_permission:
            return {
                "status": "error",
                "code": "UNAUTHORIZED",
                "message": f"User {user_id} lacks permission for {tool.name}"
            }

        return None  # Authorized

    except asyncio.TimeoutError:
        # Fail closed for authorization (security critical)
        return {
            "status": "error",
            "code": "AUTH_TIMEOUT",
            "message": "Authorization service timed out"
        }


async def _check_tool_permission(user_id: str, tool_name: str) -> bool:
    """Simulate permission check against IAM service."""
    await asyncio.sleep(0.02)
    # Simulate permission lookup
    return True


# =============================================================================
# Pattern 5: Error Handling in Async Context
# =============================================================================

class ValidationError(Exception):
    """Custom validation error."""
    def __init__(self, message: str, code: str):
        self.message = message
        self.code = code
        super().__init__(message)


async def async_with_error_handling(
    callback_context: CallbackContext,
    llm_request: LlmRequest
) -> Optional[LlmResponse]:
    """
    Demonstrates proper error handling in async callbacks.

    Error handling strategies:
    1. Catch specific exceptions for specific responses
    2. Use try/except/finally for cleanup
    3. Log all errors for debugging
    4. Decide on fail-open vs fail-closed per error type
    """
    user_message = _extract_message(llm_request)

    try:
        # Validation that might raise custom errors
        await _validate_with_exceptions(user_message)
        return None  # Validation passed

    except ValidationError as e:
        # Known validation error - block with specific message
        return LlmResponse(
            content=types.Content(
                role="model",
                parts=[types.Part(text=f"Validation failed: {e.message}")]
            )
        )

    except asyncio.TimeoutError:
        # Timeout - fail open with logging
        print("Validation timed out, allowing request")
        return None

    except ConnectionError:
        # Network error - fail open but log
        print("Network error during validation, allowing request")
        return None

    except Exception as e:
        # Unknown error - log and fail open (or closed for high security)
        print(f"Unexpected validation error: {type(e).__name__}: {e}")
        # For high security, you might want to block:
        # return LlmResponse(content=types.Content(
        #     role="model",
        #     parts=[types.Part(text="An error occurred. Please try again.")]
        # ))
        return None

    finally:
        # Cleanup code (close connections, release resources, etc.)
        pass


async def _validate_with_exceptions(text: str):
    """Validation that raises exceptions."""
    await asyncio.sleep(0.01)

    if "forbidden" in text.lower():
        raise ValidationError("Content contains forbidden words", "FORBIDDEN_CONTENT")

    if len(text) > 50000:
        raise ValidationError("Content too long", "CONTENT_TOO_LONG")


# =============================================================================
# Helper Functions
# =============================================================================

def _extract_message(llm_request: LlmRequest) -> str:
    """Extract the last user message from request."""
    if not llm_request.contents:
        return ""

    for content in reversed(llm_request.contents):
        if content.role == "user" and content.parts:
            for part in content.parts:
                if part.text:
                    return part.text

    return ""


# =============================================================================
# Factory Functions
# =============================================================================

def create_async_before_model_callback(
    enable_moderation: bool = True,
    enable_concurrent_checks: bool = True,
    timeout_seconds: float = 2.0
):
    """
    Create an async before_model_callback with configurable features.

    Example:
        agent = Agent(
            name="secure_agent",
            model="gemini-2.5-flash",
            instruction="You are helpful.",
            before_model_callback=create_async_before_model_callback(
                enable_moderation=True,
                timeout_seconds=1.5
            )
        )
    """
    async def async_callback(
        callback_context: CallbackContext,
        llm_request: LlmRequest
    ) -> Optional[LlmResponse]:

        try:
            # Wrap all checks in overall timeout
            async with asyncio.timeout(timeout_seconds):

                # Content moderation
                if enable_moderation:
                    result = await async_content_moderation(
                        callback_context, llm_request
                    )
                    if result:
                        return result

                # Concurrent validation checks
                if enable_concurrent_checks:
                    result = await concurrent_validation(
                        callback_context, llm_request
                    )
                    if result:
                        return result

                return None

        except asyncio.TimeoutError:
            print(f"Overall callback timeout after {timeout_seconds}s")
            return None  # Fail open

    return async_callback


# =============================================================================
# Test Cases
# =============================================================================

if __name__ == "__main__":
    """
    Test async callback patterns.
    Run: python async_callbacks.py
    """
    from unittest.mock import MagicMock

    def create_mock_request(text: str) -> LlmRequest:
        """Helper to create mock request."""
        request = MagicMock(spec=LlmRequest)
        content = MagicMock()
        content.role = "user"
        part = MagicMock()
        part.text = text
        content.parts = [part]
        request.contents = [content]
        return request

    def create_mock_context(user_id: str = "test_user") -> CallbackContext:
        """Helper to create mock callback context."""
        context = MagicMock(spec=CallbackContext)
        context.invocation_context = MagicMock()
        context.invocation_context.user_id = user_id
        return context

    async def run_tests():
        print("=" * 60)
        print("Testing Async Callback Patterns")
        print("=" * 60)

        # Test 1: Content moderation - safe content
        print("\n1. Testing async content moderation (safe content):")
        request = create_mock_request("Hello, how are you today?")
        context = create_mock_context()
        start = time.time()
        result = await async_content_moderation(context, request)
        elapsed = (time.time() - start) * 1000
        print(f"   Input: 'Hello, how are you today?'")
        print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
        print(f"   Latency: {elapsed:.1f}ms")
        assert result is None, "Safe content should be allowed"

        # Test 2: Content moderation - unsafe content
        print("\n2. Testing async content moderation (unsafe content):")
        request = create_mock_request("I hate everyone and want to kill them")
        result = await async_content_moderation(context, request)
        print(f"   Input: 'I hate everyone and want to kill them'")
        print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
        assert result is not None, "Unsafe content should be blocked"

        # Test 3: Timeout handling
        print("\n3. Testing timeout handling:")
        request = create_mock_request("Normal message")
        start = time.time()
        result = await async_with_timeout(context, request, timeout_seconds=0.5)
        elapsed = (time.time() - start) * 1000
        print(f"   Timeout set: 500ms")
        print(f"   External service delay: 2000ms")
        print(f"   Actual wait: {elapsed:.0f}ms")
        print(f"   Result: {'BLOCKED' if result else 'ALLOWED (timeout, fail open)'}")
        assert elapsed < 1000, "Should timeout around 500ms"

        # Test 4: Concurrent validation
        print("\n4. Testing concurrent validation:")
        request = create_mock_request("Normal legitimate request")
        start = time.time()
        result = await concurrent_validation(context, request)
        elapsed = (time.time() - start) * 1000
        print(f"   Running 3 checks concurrently")
        print(f"   Total time: {elapsed:.1f}ms (not sum of individual times)")
        print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
        assert result is None, "Normal request should pass"

        # Test 5: Concurrent validation - blocked
        print("\n5. Testing concurrent validation (blocklist hit):")
        request = create_mock_request("This is a spam scam message")
        result = await concurrent_validation(context, request)
        print(f"   Input: 'This is a spam scam message'")
        print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
        assert result is not None, "Blocked content should be caught"

        # Test 6: Error handling
        print("\n6. Testing error handling (custom validation error):")
        request = create_mock_request("This contains forbidden content")
        result = await async_with_error_handling(context, request)
        print(f"   Input: 'This contains forbidden content'")
        print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
        assert result is not None, "Forbidden content should be blocked"

        # Test 7: Factory function
        print("\n7. Testing factory-created callback:")
        callback = create_async_before_model_callback(
            enable_moderation=True,
            enable_concurrent_checks=True,
            timeout_seconds=3.0
        )
        request = create_mock_request("Hello world")
        result = await callback(context, request)
        print(f"   Created callback with moderation + concurrent checks")
        print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
        assert result is None, "Normal message should pass all checks"

        print("\n" + "=" * 60)
        print("All async tests passed!")
        print("=" * 60)

    # Run async tests
    asyncio.run(run_tests())
