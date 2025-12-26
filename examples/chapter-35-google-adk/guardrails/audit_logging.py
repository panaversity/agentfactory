"""
Audit Logging with After Callbacks

Demonstrates observability patterns using after_model_callback and
after_tool_callback for logging, monitoring, and debugging.

Key Pattern:
- After callbacks are for OBSERVATION, not blocking
- They receive both request and response data
- Use for logging, metrics, alerting, and debugging

Security Rationale:
Audit logging creates an immutable record of agent activities for:
- Security incident investigation
- Compliance requirements (SOX, HIPAA, GDPR)
- Performance monitoring and optimization
- Cost tracking and attribution
"""

import json
import logging
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Optional, Dict, Any, List
from collections import defaultdict

from google.adk.agents.callback_context import CallbackContext
from google.adk.models.llm_request import LlmRequest
from google.adk.models.llm_response import LlmResponse
from google.adk.tools.base_tool import BaseTool
from google.adk.tools.tool_context import ToolContext


# =============================================================================
# Logging Configuration
# =============================================================================

# Configure structured logger
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger("adk.audit")


# =============================================================================
# Data Classes for Structured Logging
# =============================================================================

@dataclass
class ModelCallLog:
    """Structured log entry for model calls."""
    timestamp: str
    user_id: str
    session_id: str
    agent_name: str
    model: str
    input_tokens: int
    output_tokens: int
    latency_ms: float
    success: bool
    error_message: Optional[str] = None

    def to_json(self) -> str:
        return json.dumps(asdict(self), default=str)


@dataclass
class ToolCallLog:
    """Structured log entry for tool executions."""
    timestamp: str
    user_id: str
    session_id: str
    tool_name: str
    arguments: Dict[str, Any]
    result_summary: str
    latency_ms: float
    success: bool
    error_message: Optional[str] = None

    def to_json(self) -> str:
        # Redact sensitive arguments
        safe_args = _redact_sensitive_args(self.arguments)
        data = asdict(self)
        data["arguments"] = safe_args
        return json.dumps(data, default=str)


@dataclass
class AggregatedMetrics:
    """Aggregated metrics for monitoring dashboards."""
    total_model_calls: int = 0
    total_tool_calls: int = 0
    total_input_tokens: int = 0
    total_output_tokens: int = 0
    total_errors: int = 0
    model_latencies: List[float] = field(default_factory=list)
    tool_latencies: List[float] = field(default_factory=list)
    calls_by_user: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    calls_by_tool: Dict[str, int] = field(default_factory=lambda: defaultdict(int))


# Global metrics store (use Redis/CloudWatch in production)
_metrics = AggregatedMetrics()


# =============================================================================
# Pattern 1: Log All Model Calls with Timing
# =============================================================================

# Store start times for latency calculation
_request_start_times: Dict[str, float] = {}


def log_model_calls_before(
    callback_context: CallbackContext,
    llm_request: LlmRequest
) -> None:
    """
    Before callback to record start time for latency measurement.

    Note: This is a helper used with log_model_calls_after.
    Returns None to allow request to proceed.
    """
    request_id = id(llm_request)
    _request_start_times[str(request_id)] = time.time()
    return None


def log_model_calls(
    callback_context: CallbackContext,
    llm_response: LlmResponse
) -> LlmResponse:
    """
    After model callback to log all model interactions.

    Logs:
    - User and session identification
    - Token usage (input/output)
    - Response latency
    - Success/failure status

    Returns:
        The original response (unchanged)
    """
    # Get timing (if before callback was used)
    # In real implementation, use callback_context or request correlation
    latency_ms = 0.0

    # Extract context information
    invocation = callback_context.invocation_context
    user_id = invocation.user_id or "unknown"
    session_id = invocation.session_id or "unknown"
    agent_name = invocation.agent_name or "unknown"

    # Estimate tokens (in real implementation, use response metadata)
    input_tokens = 0
    output_tokens = 0
    if llm_response.content and llm_response.content.parts:
        for part in llm_response.content.parts:
            if part.text:
                # Rough estimate: 4 chars per token
                output_tokens += len(part.text) // 4

    # Create structured log
    log_entry = ModelCallLog(
        timestamp=datetime.utcnow().isoformat(),
        user_id=user_id,
        session_id=session_id,
        agent_name=agent_name,
        model=getattr(invocation, 'model', 'gemini'),
        input_tokens=input_tokens,
        output_tokens=output_tokens,
        latency_ms=latency_ms,
        success=True,
        error_message=None
    )

    # Log structured entry
    logger.info(f"MODEL_CALL | {log_entry.to_json()}")

    # Update metrics
    _metrics.total_model_calls += 1
    _metrics.total_input_tokens += input_tokens
    _metrics.total_output_tokens += output_tokens
    _metrics.calls_by_user[user_id] += 1
    if latency_ms > 0:
        _metrics.model_latencies.append(latency_ms)

    return llm_response


# =============================================================================
# Pattern 2: Log Tool Executions with Results
# =============================================================================

_tool_start_times: Dict[str, float] = {}


def log_tool_start(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext
) -> None:
    """
    Before tool callback to record start time.
    Returns None to allow execution.
    """
    call_id = f"{tool.name}_{id(args)}"
    _tool_start_times[call_id] = time.time()
    return None


def log_tool_executions(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext,
    tool_response: Dict[str, Any]
) -> Dict[str, Any]:
    """
    After tool callback to log all tool executions.

    Logs:
    - Tool name and (redacted) arguments
    - Execution result summary
    - Latency
    - Success/failure status

    Returns:
        The original tool response (unchanged)
    """
    # Calculate latency
    call_id = f"{tool.name}_{id(args)}"
    start_time = _tool_start_times.pop(call_id, None)
    latency_ms = (time.time() - start_time) * 1000 if start_time else 0.0

    # Determine success
    success = True
    error_message = None
    if isinstance(tool_response, dict):
        if tool_response.get("status") == "error":
            success = False
            error_message = tool_response.get("message", "Unknown error")

    # Create result summary (don't log full response for large data)
    result_summary = _summarize_result(tool_response)

    # Get user/session from tool context state
    user_id = tool_context.state.get("user_id", "unknown")
    session_id = tool_context.state.get("session_id", "unknown")

    # Create structured log
    log_entry = ToolCallLog(
        timestamp=datetime.utcnow().isoformat(),
        user_id=user_id,
        session_id=session_id,
        tool_name=tool.name,
        arguments=args,
        result_summary=result_summary,
        latency_ms=latency_ms,
        success=success,
        error_message=error_message
    )

    # Log structured entry
    logger.info(f"TOOL_CALL | {log_entry.to_json()}")

    # Update metrics
    _metrics.total_tool_calls += 1
    _metrics.calls_by_tool[tool.name] += 1
    if latency_ms > 0:
        _metrics.tool_latencies.append(latency_ms)
    if not success:
        _metrics.total_errors += 1

    return tool_response


# =============================================================================
# Pattern 3: Track Token Usage
# =============================================================================

@dataclass
class TokenUsageReport:
    """Token usage tracking for cost management."""
    total_input_tokens: int
    total_output_tokens: int
    estimated_cost_usd: float
    by_user: Dict[str, int]
    by_model: Dict[str, int]


# Approximate pricing (check current Google pricing)
TOKEN_COSTS = {
    "gemini-2.0-flash": {"input": 0.0001, "output": 0.0004},  # per 1K tokens
    "gemini-2.5-flash": {"input": 0.00015, "output": 0.0006},
    "gemini-2.5-pro": {"input": 0.00125, "output": 0.005},
}


def track_token_usage(
    callback_context: CallbackContext,
    llm_response: LlmResponse
) -> LlmResponse:
    """
    Track token usage for cost management and budgeting.

    In production, integrate with:
    - CloudWatch/Datadog for metrics
    - Billing systems for chargebacks
    - Alerting for budget thresholds
    """
    invocation = callback_context.invocation_context
    user_id = invocation.user_id or "unknown"
    model = getattr(invocation, 'model', 'gemini-2.0-flash')

    # Extract token counts from response metadata if available
    # This is illustrative - real implementation uses response metadata
    output_tokens = 0
    if llm_response.content and llm_response.content.parts:
        for part in llm_response.content.parts:
            if part.text:
                output_tokens += len(part.text) // 4

    # Track by user
    user_token_key = f"tokens:{user_id}"
    _metrics.calls_by_user[user_token_key] = (
        _metrics.calls_by_user.get(user_token_key, 0) + output_tokens
    )

    # Log token event
    logger.debug(f"TOKEN_USAGE | user={user_id} model={model} tokens={output_tokens}")

    return llm_response


def get_token_usage_report() -> TokenUsageReport:
    """Generate a token usage report for monitoring."""
    total_input = _metrics.total_input_tokens
    total_output = _metrics.total_output_tokens

    # Estimate cost (using default model pricing)
    costs = TOKEN_COSTS.get("gemini-2.0-flash", {"input": 0.0001, "output": 0.0004})
    estimated_cost = (
        (total_input / 1000) * costs["input"] +
        (total_output / 1000) * costs["output"]
    )

    return TokenUsageReport(
        total_input_tokens=total_input,
        total_output_tokens=total_output,
        estimated_cost_usd=estimated_cost,
        by_user=dict(_metrics.calls_by_user),
        by_model={}  # Would track per-model in production
    )


# =============================================================================
# Pattern 4: Error Tracking and Alerting
# =============================================================================

# Error threshold for alerting
ERROR_THRESHOLD_PERCENT = 10.0
ERROR_WINDOW_SIZE = 100  # Last N requests

_recent_results: List[bool] = []  # True = success, False = error


def track_errors_after_model(
    callback_context: CallbackContext,
    llm_response: LlmResponse
) -> LlmResponse:
    """
    Track errors and trigger alerts when error rate exceeds threshold.

    Integrates with alerting systems (PagerDuty, Slack, etc.) in production.
    """
    # Determine if this response indicates an error
    is_error = False
    if llm_response.content and llm_response.content.parts:
        for part in llm_response.content.parts:
            if part.text and "error" in part.text.lower():
                is_error = True
                break

    # Track in sliding window
    _recent_results.append(not is_error)
    if len(_recent_results) > ERROR_WINDOW_SIZE:
        _recent_results.pop(0)

    # Calculate error rate
    if len(_recent_results) >= 10:  # Minimum sample size
        error_rate = (1 - sum(_recent_results) / len(_recent_results)) * 100

        if error_rate > ERROR_THRESHOLD_PERCENT:
            logger.warning(
                f"ERROR_RATE_ALERT | rate={error_rate:.1f}% threshold={ERROR_THRESHOLD_PERCENT}%"
            )
            # In production: trigger PagerDuty, send Slack alert, etc.

    return llm_response


def track_errors_after_tool(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext,
    tool_response: Dict[str, Any]
) -> Dict[str, Any]:
    """
    Track tool errors for monitoring and alerting.
    """
    is_error = False
    if isinstance(tool_response, dict):
        if tool_response.get("status") == "error":
            is_error = True
            logger.error(
                f"TOOL_ERROR | tool={tool.name} "
                f"error={tool_response.get('message', 'unknown')}"
            )

    if is_error:
        _metrics.total_errors += 1

    return tool_response


# =============================================================================
# Helper Functions
# =============================================================================

def _redact_sensitive_args(args: Dict[str, Any]) -> Dict[str, Any]:
    """Redact sensitive information from arguments before logging."""
    sensitive_keys = {
        "password", "secret", "token", "api_key", "apikey",
        "ssn", "credit_card", "card_number", "cvv"
    }

    redacted = {}
    for key, value in args.items():
        if any(s in key.lower() for s in sensitive_keys):
            redacted[key] = "[REDACTED]"
        elif isinstance(value, str) and len(value) > 100:
            redacted[key] = f"{value[:50]}...[truncated]"
        else:
            redacted[key] = value

    return redacted


def _summarize_result(result: Any) -> str:
    """Create a brief summary of tool result for logging."""
    if result is None:
        return "null"

    if isinstance(result, dict):
        status = result.get("status", "unknown")
        if "items" in result:
            count = len(result["items"]) if isinstance(result["items"], list) else "?"
            return f"status={status}, items={count}"
        if "data" in result:
            return f"status={status}, has_data=true"
        return f"status={status}"

    if isinstance(result, list):
        return f"list[{len(result)} items]"

    if isinstance(result, str):
        if len(result) > 50:
            return f"string[{len(result)} chars]"
        return result

    return str(type(result).__name__)


# =============================================================================
# Metrics Export
# =============================================================================

def get_metrics_summary() -> Dict[str, Any]:
    """Get aggregated metrics for monitoring dashboards."""
    model_p50 = 0.0
    model_p95 = 0.0
    if _metrics.model_latencies:
        sorted_latencies = sorted(_metrics.model_latencies)
        n = len(sorted_latencies)
        model_p50 = sorted_latencies[n // 2]
        model_p95 = sorted_latencies[int(n * 0.95)] if n > 1 else sorted_latencies[0]

    tool_p50 = 0.0
    tool_p95 = 0.0
    if _metrics.tool_latencies:
        sorted_latencies = sorted(_metrics.tool_latencies)
        n = len(sorted_latencies)
        tool_p50 = sorted_latencies[n // 2]
        tool_p95 = sorted_latencies[int(n * 0.95)] if n > 1 else sorted_latencies[0]

    error_rate = 0.0
    total_calls = _metrics.total_model_calls + _metrics.total_tool_calls
    if total_calls > 0:
        error_rate = (_metrics.total_errors / total_calls) * 100

    return {
        "total_model_calls": _metrics.total_model_calls,
        "total_tool_calls": _metrics.total_tool_calls,
        "total_errors": _metrics.total_errors,
        "error_rate_percent": error_rate,
        "tokens": {
            "input": _metrics.total_input_tokens,
            "output": _metrics.total_output_tokens,
        },
        "latency_ms": {
            "model_p50": model_p50,
            "model_p95": model_p95,
            "tool_p50": tool_p50,
            "tool_p95": tool_p95,
        },
        "top_tools": dict(sorted(
            _metrics.calls_by_tool.items(),
            key=lambda x: x[1],
            reverse=True
        )[:5]),
        "top_users": dict(sorted(
            _metrics.calls_by_user.items(),
            key=lambda x: x[1],
            reverse=True
        )[:5]),
    }


def reset_metrics():
    """Reset metrics (useful for testing)."""
    global _metrics
    _metrics = AggregatedMetrics()


# =============================================================================
# Test Cases
# =============================================================================

if __name__ == "__main__":
    """
    Test cases demonstrating audit logging patterns.
    Run: python audit_logging.py
    """
    from unittest.mock import MagicMock

    def create_mock_response(text: str) -> LlmResponse:
        """Helper to create mock response."""
        response = MagicMock(spec=LlmResponse)
        content = MagicMock()
        content.role = "model"
        part = MagicMock()
        part.text = text
        content.parts = [part]
        response.content = content
        return response

    def create_mock_callback_context(
        user_id: str = "test_user",
        session_id: str = "session_123"
    ) -> CallbackContext:
        """Helper to create mock callback context."""
        context = MagicMock(spec=CallbackContext)
        context.invocation_context = MagicMock()
        context.invocation_context.user_id = user_id
        context.invocation_context.session_id = session_id
        context.invocation_context.agent_name = "test_agent"
        return context

    def create_mock_tool(name: str) -> BaseTool:
        """Helper to create mock tool."""
        tool = MagicMock(spec=BaseTool)
        tool.name = name
        return tool

    def create_mock_tool_context(state: Dict = None) -> ToolContext:
        """Helper to create mock tool context."""
        context = MagicMock(spec=ToolContext)
        context.state = state or {"user_id": "test_user", "session_id": "session_123"}
        return context

    print("=" * 60)
    print("Testing Audit Logging")
    print("=" * 60)

    # Reset metrics for testing
    reset_metrics()

    # Test 1: Log model call
    print("\n1. Testing model call logging:")
    callback_context = create_mock_callback_context()
    response = create_mock_response("Hello! How can I help you today?")
    result = log_model_calls(callback_context, response)
    print(f"   Logged model call")
    print(f"   Metrics: {_metrics.total_model_calls} model calls")
    assert _metrics.total_model_calls == 1

    # Test 2: Log tool execution
    print("\n2. Testing tool execution logging:")
    tool = create_mock_tool("get_weather")
    tool_context = create_mock_tool_context()
    args = {"city": "London"}
    tool_response = {"status": "success", "temperature": "15C"}

    # Simulate start time
    log_tool_start(tool, args, tool_context)
    time.sleep(0.01)  # Simulate execution time
    result = log_tool_executions(tool, args, tool_context, tool_response)
    print(f"   Logged tool call: get_weather")
    print(f"   Metrics: {_metrics.total_tool_calls} tool calls")
    assert _metrics.total_tool_calls == 1

    # Test 3: Track errors
    print("\n3. Testing error tracking:")
    error_response = {"status": "error", "message": "City not found"}
    result = log_tool_executions(tool, args, tool_context, error_response)
    print(f"   Logged error")
    print(f"   Metrics: {_metrics.total_errors} errors")
    assert _metrics.total_errors == 1

    # Test 4: Token usage
    print("\n4. Testing token usage tracking:")
    response = create_mock_response("This is a longer response " * 10)
    track_token_usage(callback_context, response)
    report = get_token_usage_report()
    print(f"   Input tokens: {report.total_input_tokens}")
    print(f"   Output tokens: {report.total_output_tokens}")
    print(f"   Estimated cost: ${report.estimated_cost_usd:.6f}")

    # Test 5: Argument redaction
    print("\n5. Testing sensitive argument redaction:")
    sensitive_args = {
        "username": "john",
        "password": "secret123",
        "api_key": "sk-abc123",
        "message": "Hello world"
    }
    redacted = _redact_sensitive_args(sensitive_args)
    print(f"   Original: password='secret123'")
    print(f"   Redacted: password='{redacted['password']}'")
    assert redacted["password"] == "[REDACTED]"
    assert redacted["api_key"] == "[REDACTED]"
    assert redacted["username"] == "john"

    # Test 6: Metrics summary
    print("\n6. Testing metrics summary:")
    summary = get_metrics_summary()
    print(f"   Total model calls: {summary['total_model_calls']}")
    print(f"   Total tool calls: {summary['total_tool_calls']}")
    print(f"   Error rate: {summary['error_rate_percent']:.1f}%")

    print("\n" + "=" * 60)
    print("All tests passed!")
    print("=" * 60)
