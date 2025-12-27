### Core Concept
Streaming sends data as it becomes available rather than waiting for completion—crucial for agent responses that take time. Server-Sent Events (SSE) is the simplest streaming protocol: server yields events with `event` and `data` fields, client receives them in real-time as a stream.

### Key Mental Models
- **Async generators produce data over time**: `async def` with `yield` creates a generator that produces values progressively. Each `yield` sends one SSE event. `await asyncio.sleep()` simulates work (real agents generate tokens).
- **Connection stays open**: Unlike request-response (send request, wait for full response), streaming opens a connection and keeps sending events until complete. Client sees responses forming immediately.
- **Error events mid-stream**: Once streaming starts, HTTP status is already 200. Can't change it. Instead, yield error as an event (SSE event with type "error"). Client parses the event type.

### Critical Patterns
- Create async generator that yields `{"event": "...", "data": json.dumps(...)}`
- Always serialize data to JSON string—SSE data must be strings
- Use `EventSourceResponse(generator())` to return streaming response
- Browser uses `EventSource` API for GET endpoints, `fetch` with response.body.getReader() for POST
- Close client connection with `source.close()` when done—prevents resource leaks
- Use `await asyncio.sleep()` not `time.sleep()`—blocking sleep freezes the event loop

### Common Mistakes
- Forgetting to import json for serialization—passing dict instead of string causes errors
- Leaving client connections open forever—they accumulate and exhaust server resources
- Using synchronous `time.sleep()` which blocks the entire event loop—other requests can't be processed
- Using `return` instead of `yield`—return ends the function immediately, yield makes it a generator
- Trying to change HTTP status mid-stream—status already sent, only events can carry error info

### Connections
- **Builds on**: Lesson 5's async endpoints and dependency injection
- **Leads to**: Lesson 7's agent integration that streams responses token by token
