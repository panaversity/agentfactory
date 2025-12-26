---
sidebar_position: 7
title: "Error Handling & Recovery"
description: "Master production error patterns for MCP servers: JSON-RPC errors, graceful degradation, retry-safe design, and resilience without crashes"
keywords: [mcp, error-handling, json-rpc, resilience, graceful-degradation, production-servers, retry-safety]
chapter: 38
lesson: 7
duration_minutes: 28

skills:
  mcp-error-handling:
    proficiency_level: C2
    category: Technical
    bloom_level: Apply, Analyze
    digcomp_area: Problem-Solving
    measurable_at_this_level: "Students can implement structured error handling in MCP tools with proper JSON-RPC error responses"

  production-resilience:
    proficiency_level: C2
    category: Applied
    bloom_level: Analyze, Evaluate
    digcomp_area: Systems-Thinking
    measurable_at_this_level: "Students can design tools that gracefully degrade under failure and distinguish transient from permanent errors"

  retry-safe-design:
    proficiency_level: C2
    category: Technical
    bloom_level: Apply, Analyze
    digcomp_area: Problem-Solving
    measurable_at_this_level: "Students can identify and implement idempotent operations that safely handle retry scenarios"

learning_objectives:
  - objective: "Understand JSON-RPC error structure and when to use different error codes"
    proficiency_level: C2
    bloom_level: Understand, Apply
    assessment_method: "Error response construction in tool implementations"

  - objective: "Implement exception handling patterns that prevent server crashes"
    proficiency_level: C2
    bloom_level: Apply, Analyze
    assessment_method: "Tool code with try/except patterns and structured error responses"

  - objective: "Design tools with graceful degradation for partial failures"
    proficiency_level: C2
    bloom_level: Analyze, Evaluate
    assessment_method: "Multi-step operations that return partial results on error"

  - objective: "Build retry-safe (idempotent) tool operations"
    proficiency_level: C2
    bloom_level: Apply, Analyze
    assessment_method: "Operations that produce same result regardless of retry count"

cognitive_load:
  new_concepts: 5
  assessment: "Error handling structure, transient vs permanent errors, graceful degradation, idempotency, logging for debugging. All concepts naturally build from tool implementation knowledge."

differentiation:
  extension_for_advanced: "Implement exponential backoff retry strategies and design distributed transaction patterns for multi-tool operations"
  remedial_for_struggling: "Provide error handling templates and focus on exception handling basics through structured Try With AI walkthrough"
---

# Error Handling & Recovery

## The Production Reality Check

Imagine you've deployed an MCP server that helps users convert documents. Everything works perfectly in your local testing—you convert PDFs to Word, extract images, validate file formats.

Then production happens.

A user uploads a corrupted PDF. Your tool tries to parse it, raises an exception, and the server crashes. Claude's connection drops. The user sees "Unknown error." You get a support ticket you can't debug because the server isn't logging.

Later, another user has a flaky network connection. A file transfer timeout occurs mid-operation. Your tool retries automatically, processes the file twice, and creates duplicate conversions. The user pays twice.

These aren't hypothetical problems—they're the difference between hobby servers and production systems.

**Production servers have one non-negotiable requirement: They never crash on user errors.**

Instead, they:
1. Catch errors with structured try/except patterns
2. Return proper JSON-RPC error responses
3. Log what happened for debugging
4. Distinguish temporary failures from permanent ones
5. Design operations to be safely retryable

This lesson teaches you how.

## JSON-RPC Error Structure: Speaking the Protocol

When your MCP tool encounters an error, you don't just throw an exception. You return a structured error response that clients understand.

### The JSON-RPC Error Format

Here's the protocol specification. Every error response follows this structure:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "error": {
    "code": -32600,
    "message": "Invalid Request",
    "data": {
      "reason": "Additional context about what went wrong"
    }
  }
}
```

**Three required fields:**

1. **code**: An integer representing error type
   - `-32700` to `-32600`: Protocol-level errors (reserved)
   - `-32000` to `-32099`: Server-defined errors
   - Custom codes: Use negative integers in your range

2. **message**: Brief error description (one line)
   - Example: `"Document not found"`
   - Not: `"Unable to locate the requested document in the file system because the path resolution failed"`

3. **data**: Optional context object
   - Add details that help debugging
   - Include which parameters caused the error
   - Include what the server was attempting

### Standard Error Codes You'll Use

| Code | Name | When to Use |
|------|------|------------|
| `-32700` | Parse error | Client sent invalid JSON (handled by framework) |
| `-32600` | Invalid Request | Tool parameters don't match schema |
| `-32601` | Method not found | Client called non-existent tool |
| `-32602` | Invalid params | Parameters invalid for this tool |
| `-32603` | Internal error | Unhandled exception in server |
| `-32000` | Server error | Your custom error range |

### Practical Examples

**Example 1: Resource Not Found**

```python
# User asks to convert document "DOC-2024-001" but it doesn't exist

{
  "jsonrpc": "2.0",
  "id": 5,
  "error": {
    "code": -32000,
    "message": "Document not found",
    "data": {
      "document_id": "DOC-2024-001",
      "searched_locations": [
        "/documents/active",
        "/documents/archive"
      ],
      "suggestion": "Use list_documents to see available IDs"
    }
  }
}
```

**Example 2: Temporary Service Unavailable**

```python
# External API timeout—worth retrying

{
  "jsonrpc": "2.0",
  "id": 8,
  "error": {
    "code": -32000,
    "message": "External service unavailable",
    "data": {
      "service": "document-processing-api",
      "status_code": 503,
      "retry_after_seconds": 30,
      "is_transient": true
    }
  }
}
```

**Example 3: Invalid Parameters**

```python
# Tool expects file size between 1MB and 100MB, got 500MB

{
  "jsonrpc": "2.0",
  "id": 3,
  "error": {
    "code": -32602,
    "message": "Invalid params",
    "data": {
      "parameter": "file_size",
      "value": 524288000,
      "constraint": "Must be <= 104857600 (100MB)",
      "received_mb": 500,
      "max_mb": 100
    }
  }
}
```

## Exception Handling in Tools: The Production Pattern

Here's how to implement error handling that catches issues without crashing:

### Pattern 1: Basic Try/Except with Structured Response

```python
@mcp.tool()
async def convert_document(document_id: str, target_format: str) -> dict:
    """Convert document to target format."""

    try:
        # Attempt the operation
        await ctx.info(f"Converting {document_id} to {target_format}...")

        document = await fetch_document(document_id)
        result = await convert(document, target_format)

        await ctx.info(f"Conversion successful")
        return {
            "status": "success",
            "output_file": result.path,
            "format": target_format,
            "size_bytes": result.size
        }

    except DocumentNotFoundError as e:
        # Specific, recoverable error
        await ctx.error(f"Document {document_id} not found: {e}")
        raise ValueError(f"Document not found: {document_id}")

    except ConversionTimeoutError as e:
        # Temporary failure—client should retry
        await ctx.error(f"Conversion timeout after {e.elapsed_seconds}s")
        raise ValueError(f"Conversion timed out (transient). Please retry.")

    except Exception as e:
        # Catch-all for unexpected errors
        await ctx.error(f"Unexpected error: {type(e).__name__}: {e}")
        raise ValueError(f"Conversion failed unexpectedly. Support team notified.")
```

**Key principles:**

1. **Catch specific exceptions first** → Handle known error types specially
2. **Log context** → Use `ctx.info()`, `ctx.error()` for debugging
3. **Raise ValueError for client errors** → MCP framework converts to JSON-RPC errors
4. **Never let exceptions propagate uncaught** → That crashes the server

### Pattern 2: Graceful Degradation (Partial Results)

Some operations can partially succeed. Return what you can:

```python
@mcp.tool()
async def batch_convert_documents(document_ids: list[str], target_format: str) -> dict:
    """Convert multiple documents. Some may fail—that's okay."""

    successful = []
    failed = []

    for doc_id in document_ids:
        try:
            document = await fetch_document(doc_id)
            result = await convert(document, target_format)
            successful.append({
                "id": doc_id,
                "output": result.path
            })
        except DocumentNotFoundError:
            failed.append({
                "id": doc_id,
                "error": "Not found"
            })
        except ConversionError as e:
            failed.append({
                "id": doc_id,
                "error": str(e)
            })

    # Return partial results, not failure
    return {
        "status": "partial" if failed else "success",
        "successful_count": len(successful),
        "failed_count": len(failed),
        "successful": successful,
        "failed": failed,
        "note": f"Processed {len(successful)}/{len(document_ids)} documents successfully"
    }
```

**When to use graceful degradation:**

- Batch operations (process what you can)
- Operations with optional components (return what's available)
- Operations that fetch from multiple sources (return partial data)

### Pattern 3: Distinguishing Transient from Permanent Errors

Your error handling strategy depends on error type:

```python
@mcp.tool()
async def fetch_data_from_api(endpoint: str, retry_count: int = 0) -> dict:
    """Fetch data. Some errors are worth retrying, others aren't."""

    try:
        response = await http_client.get(endpoint, timeout=10)
        return response.json()

    except asyncio.TimeoutError as e:
        # TRANSIENT: Network timeout—retry makes sense
        if retry_count < 3:
            await ctx.info(f"Timeout, retrying ({retry_count}/3)...")
            await asyncio.sleep(2 ** retry_count)  # Exponential backoff
            return await fetch_data_from_api(endpoint, retry_count + 1)
        else:
            raise ValueError("API timeout after 3 retries. Service may be down.")

    except httpx.HTTPStatusError as e:
        if e.response.status_code == 404:
            # PERMANENT: Endpoint doesn't exist
            raise ValueError(f"Endpoint not found: {endpoint}")

        elif e.response.status_code == 429:
            # TRANSIENT: Rate limited—retry later
            retry_after = e.response.headers.get("Retry-After", "60")
            raise ValueError(f"Rate limited. Retry after {retry_after} seconds.")

        elif e.response.status_code >= 500:
            # TRANSIENT: Server error—worth retrying
            raise ValueError(f"Server error {e.response.status_code}. Please retry.")

        else:
            # PERMANENT: Client error (400-499 except 429)
            raise ValueError(f"API error {e.response.status_code}: {e.response.text}")

    except Exception as e:
        # UNKNOWN: Log and report
        await ctx.error(f"Unexpected API error: {type(e).__name__}: {e}")
        raise ValueError("API request failed unexpectedly.")
```

**Decision matrix:**

| Scenario | Type | Strategy |
|----------|------|----------|
| Connection timeout | Transient | Retry with backoff |
| 429 Rate limited | Transient | Retry after delay |
| 500 Server error | Transient | Retry |
| 404 Not found | Permanent | Fail—don't retry |
| 401 Unauthorized | Permanent | Fail—check credentials |
| Malformed response | Permanent | Fail—check API |

### Pattern 4: Resource Cleanup on Error

If your tool acquires resources, release them even on error:

```python
@mcp.tool()
async def process_large_file(file_path: str) -> dict:
    """Process file, ensuring cleanup even on error."""

    file_handle = None
    temp_resources = None

    try:
        # Acquire resources
        file_handle = open(file_path, 'rb')
        temp_resources = await create_temp_workspace()

        # Process
        await ctx.info(f"Processing {file_path}...")
        result = await process_with_resources(file_handle, temp_resources)

        return {
            "status": "success",
            "output": result
        }

    except IOError as e:
        raise ValueError(f"File access error: {e}")

    except ProcessingError as e:
        raise ValueError(f"Processing failed: {e}")

    finally:
        # ALWAYS cleanup, even on error
        if file_handle:
            file_handle.close()
        if temp_resources:
            await temp_resources.cleanup()
            await ctx.info("Resources cleaned up")
```

## Logging for Production Debugging

Your error handling is only as good as your logging. When production fails, logs are your only witness.

### What to Log

```python
@mcp.tool()
async def process_data(data_id: str, ctx: Context) -> dict:
    """Example with comprehensive logging."""

    try:
        # Log START—helps identify which invocation failed
        await ctx.info(f"START: Processing data_id={data_id}")

        # Log key milestones
        await ctx.info(f"Fetching data from storage...")
        data = await fetch_data(data_id)
        await ctx.info(f"Fetched {len(data)} records")

        await ctx.info(f"Validating data...")
        await validate_data(data)
        await ctx.info(f"Validation passed")

        # Log results
        await ctx.info(f"Processing complete: {len(data)} records processed")

        return {"status": "success", "count": len(data)}

    except DataNotFoundError as e:
        # Log error with context
        await ctx.error(f"ERROR: Data not found. data_id={data_id}, error={e}")
        raise ValueError(f"Data {data_id} not found")

    except ValidationError as e:
        # Log validation failure details
        await ctx.error(f"ERROR: Validation failed. data_id={data_id}, details={e.details}")
        raise ValueError(f"Data validation failed: {e.summary}")

    except Exception as e:
        # Log unexpected errors with full context
        await ctx.error(f"ERROR: Unexpected exception in process_data()")
        await ctx.error(f"  data_id={data_id}")
        await ctx.error(f"  exception_type={type(e).__name__}")
        await ctx.error(f"  exception_message={str(e)}")
        raise ValueError("Processing failed unexpectedly")
```

**Log format best practices:**

- Include operation name and parameters
- Use consistent level: `info()` for progress, `error()` for problems
- Add context identifiers (IDs, states, counts)
- Log before and after major operations
- Never log sensitive data (passwords, tokens, PII)

## Retry-Safe Design: Idempotency

Here's a subtle but critical problem:

Your tool converts a document successfully. It returns the result. But the network hiccups, so the response doesn't reach the client. The client sees timeout, retries the same request. Your tool runs again.

If your operation isn't idempotent (safe to repeat), you now have:
- Duplicate conversions
- Duplicate charges (if paid per operation)
- Incorrect state (document marked as converted twice)

### Pattern: Idempotent Operations

```python
# WRONG: Not idempotent
@mcp.tool()
async def charge_user(user_id: str, amount: float) -> dict:
    """Charge user. Retries will double-charge!"""

    # No de-duplication
    await deduct_from_account(user_id, amount)
    await create_transaction_record(user_id, amount)

    return {"status": "charged", "amount": amount}
```

**Right: Use idempotency key**

```python
@mcp.tool()
async def charge_user(user_id: str, amount: float, idempotency_key: str) -> dict:
    """Charge user safely. Same key = same result."""

    # Check if we've already processed this request
    existing = await check_for_duplicate(user_id, idempotency_key)
    if existing:
        await ctx.info(f"Duplicate request. Returning previous result.")
        return existing

    # First time—process normally
    transaction = await deduct_from_account(user_id, amount)
    await create_transaction_record(user_id, amount, idempotency_key)

    result = {
        "status": "charged",
        "amount": amount,
        "transaction_id": transaction.id
    }

    # Cache result for future duplicate attempts
    await cache_result(idempotency_key, result)

    return result
```

### When Idempotency Matters Most

| Operation | Idempotent? | Risk If Not |
|-----------|-------------|------------|
| Read operation | ✓ Yes | Just re-reads same data |
| Document conversion | ✓ Yes (if overwrite) | Duplicate outputs |
| Database write | ✗ No | Duplicate records |
| File deletion | ✗ No | Error on retry |
| Account charge | ✗ No | Double charge |
| Status update | ✓ Yes (if idempotent key used) | Duplicate operations |

**Making operations idempotent:**

1. **Reads are naturally idempotent** (no state change)
2. **Add idempotency keys** for state-changing operations
3. **Check for duplicates** before processing
4. **Store results** so retry returns same value
5. **Use database transactions** to ensure atomicity

## Complete Integration Example

Here's a production-grade tool with all error handling patterns:

```python
from mcp.server import Server, Request
from mcp.types import Tool
import asyncio
import json
from datetime import datetime
import logging

mcp = Server("document-processor")
logger = logging.getLogger(__name__)

# ===== ERROR DEFINITIONS =====

class DocumentError(Exception):
    """Base error for document operations."""
    pass

class DocumentNotFoundError(DocumentError):
    """Requested document doesn't exist."""
    pass

class ConversionError(DocumentError):
    """Conversion operation failed."""
    pass

# ===== IDEMPOTENCY TRACKING =====

idempotency_cache: dict[str, dict] = {}

async def get_cached_result(idempotency_key: str) -> dict | None:
    """Check if we've already processed this request."""
    return idempotency_cache.get(idempotency_key)

async def cache_result(idempotency_key: str, result: dict) -> None:
    """Store result for duplicate requests."""
    idempotency_cache[idempotency_key] = result

# ===== TOOL IMPLEMENTATION =====

@mcp.tool()
async def convert_document(
    document_id: str,
    target_format: str,
    idempotency_key: str,
    ctx
) -> dict:
    """
    Convert document to target format.

    Uses idempotency_key to safely handle retries.
    Returns partial results on error.
    """

    try:
        # Check for duplicate
        cached = await get_cached_result(idempotency_key)
        if cached:
            await ctx.info(f"Returning cached result for {idempotency_key}")
            return cached

        await ctx.info(f"Converting document_id={document_id} to {target_format}")

        # FETCH
        try:
            await ctx.info("Fetching document...")
            document = await fetch_document_from_db(document_id)
            await ctx.info(f"Fetched: {len(document.data)} bytes")
        except KeyError:
            raise DocumentNotFoundError(f"Document {document_id} not found in database")

        # CONVERT
        try:
            await ctx.info(f"Converting to {target_format}...")
            result = await convert_format(document, target_format)
            await ctx.info(f"Conversion successful")
        except asyncio.TimeoutError:
            # Transient—worth retrying
            await ctx.error(f"Conversion timeout after 30s")
            raise ValueError("Conversion timed out. Please retry.")
        except Exception as e:
            # Conversion failed
            await ctx.error(f"Conversion error: {type(e).__name__}: {e}")
            raise ValueError(f"Conversion failed: {str(e)}")

        # STORE RESULT
        try:
            await ctx.info("Storing result...")
            output_path = await store_conversion_result(
                document_id,
                target_format,
                result
            )
            await ctx.info(f"Stored at {output_path}")
        except Exception as e:
            await ctx.error(f"Storage error: {e}")
            # Return partial success
            return {
                "status": "partial",
                "message": "Conversion succeeded but storage failed",
                "conversion_data": result,
                "error": str(e)
            }

        success_result = {
            "status": "success",
            "document_id": document_id,
            "original_format": document.format,
            "target_format": target_format,
            "output_path": output_path,
            "output_size_bytes": len(result),
            "timestamp": datetime.utcnow().isoformat()
        }

        # Cache for retries
        await cache_result(idempotency_key, success_result)

        return success_result

    except DocumentNotFoundError as e:
        await ctx.error(f"Document not found: document_id={document_id}")
        raise ValueError(f"Document {document_id} not found")

    except ValueError as e:
        # Already formatted—re-raise
        raise

    except Exception as e:
        # Unexpected error
        await ctx.error(
            f"Unexpected error in convert_document: "
            f"type={type(e).__name__}, message={e}"
        )
        raise ValueError("Conversion failed unexpectedly. Support team notified.")

@mcp.tool()
async def batch_convert_documents(
    document_ids: list[str],
    target_format: str,
    ctx
) -> dict:
    """
    Convert multiple documents. Returns partial results.
    """

    await ctx.info(f"Batch converting {len(document_ids)} documents to {target_format}")

    successful = []
    failed = []

    for doc_id in document_ids:
        try:
            # Generate unique idempotency key for each
            idempotency_key = f"batch_{doc_id}_{target_format}_{int(datetime.utcnow().timestamp())}"

            result = await convert_document(doc_id, target_format, idempotency_key, ctx)
            successful.append(result)

        except ValueError as e:
            # Expected error
            failed.append({
                "document_id": doc_id,
                "error": str(e)
            })
        except Exception as e:
            # Unexpected error
            await ctx.error(f"Unexpected error for {doc_id}: {e}")
            failed.append({
                "document_id": doc_id,
                "error": "Unexpected error"
            })

    await ctx.info(
        f"Batch complete: {len(successful)}/{len(document_ids)} successful"
    )

    return {
        "status": "partial" if failed else "success",
        "total": len(document_ids),
        "successful_count": len(successful),
        "failed_count": len(failed),
        "successful": successful,
        "failed": failed
    }
```

## Try With AI

### Prompt 1: Understanding Error Response Design

Ask Claude:

```
I'm building an MCP server that processes user files. Files are stored in a database, but sometimes:
1. The file doesn't exist (permanent error)
2. The database connection times out (transient error)
3. The file is corrupted (permanent error)

Explain how I should handle these three cases differently in my error response. Which should include "is_transient: true"? How would my error response structure differ for each?

Also explain: why shouldn't I just throw an exception and let the framework handle it?
```

**What you're learning**: How to distinguish error types and structure appropriate responses that guide client behavior.

### Prompt 2: Building Exception Handling Pattern

Ask Claude:

```
Write a tool called `process_csv_file` that:

1. Takes a file_path and operation (validate, transform, analyze)
2. Reads and processes a CSV file
3. Handles these errors gracefully:
   - File not found → permanent error
   - CSV is malformed → permanent error
   - Processing timeout after 30s → transient error (suggest retry)
   - Memory exhausted processing large file → suggest pagination

Requirements:
- Use try/except to catch specific errors first, then general exceptions
- Log progress with ctx.info()
- Log errors with ctx.error() including relevant context
- Return structured error responses with helpful details
- Never crash the server
- Include comments explaining why each catch block is ordered that way

Provide the complete implementation.
```

**What you're learning**: How to layer exception handling so specific errors are caught before general ones, and why order matters for correct error handling.

Compare your implementation to the patterns in this lesson. Ask Claude: "What edge cases does this implementation handle that a naive try/except wouldn't?"

### Prompt 3: Designing Idempotent Operations

Ask Claude:

```
I'm building an MCP server that helps users organize documents. One tool assigns documents to categories:

```python
@mcp.tool()
async def assign_to_category(document_id: str, category: str) -> dict:
    # User assigns doc123 to "Important"
    # Network fails mid-response
    # Client retries the same request
    # Now doc123 is assigned twice? Or error?
```

The problem: If this tool isn't idempotent, retries could cause issues:
- Duplicate assignments in the database
- Duplicate notifications sent
- Incorrect audit logs

Design a solution that makes this tool idempotent. What do you need to:
1. Track to detect duplicate requests?
2. Store to return consistent results?
3. Validate to ensure safety?

Write the tool implementation that handles all of this.
```

**What you're learning**: How idempotency keys prevent duplicate operations and why they're critical for network-based systems.

Once Claude generates the code, test it mentally: What happens if you call the tool three times with the same idempotency_key? What if you call it three times with different keys? Does the behavior match what you expect?
