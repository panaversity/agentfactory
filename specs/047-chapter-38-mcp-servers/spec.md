# Feature Specification: Chapter 38 - Advanced MCP Server Development

**Feature Branch**: `047-chapter-38-mcp-servers`
**Created**: 2025-12-26
**Status**: Draft (Revised v2 - Advanced Focus)
**Input**: User description: "Chapter 38: Advanced MCP Server Development - Teaching students ADVANCED server patterns beyond the basics covered in Chapter 37."

**Chapter Position**: Part 6, Chapter 38 (follows Chapter 37: MCP Fundamentals)
**Proficiency Level**: B2 (students have completed Chapter 37 with full FastMCP basics)

## Scope Clarification: Chapter 37 vs Chapter 38

| Topic | Chapter 37 (Fundamentals) | Chapter 38 (Advanced) |
|-------|--------------------------|----------------------|
| MCP Architecture | ✓ Host → Client → Server | ✗ Skip |
| Transport Theory | ✓ stdio vs HTTP basics | ✗ Skip |
| @mcp.tool Basics | ✓ Decorator, Field, schemas | ✗ Skip |
| @mcp.resource Basics | ✓ URIs, templates, MIME | ✗ Skip |
| @mcp.prompt Basics | ✓ Arguments, messages | ✗ Skip |
| Client Configuration | ✓ Claude Desktop/Code | ✗ Skip |
| MCP Inspector Basics | ✓ Debugging connections | ✗ Skip |
| **Context Object** | ✗ | ✓ Injection, lifespan |
| **Sampling** | ✗ | ✓ Server → Client LLM calls |
| **Progress Notifications** | ✗ | ✓ Real-time feedback |
| **Logging Notifications** | ✗ | ✓ context.info(), stderr |
| **Roots** | ✗ | ✓ File system permissions |
| **StreamableHTTP** | ✗ | ✓ SSE, sessions, production |
| **Stateful vs Stateless** | ✗ | ✓ Horizontal scaling |
| **Error Handling** | ✗ | ✓ Structured errors, recovery |
| **Packaging** | ✗ | ✓ pyproject.toml, distribution |

**Key Distinction**: Chapter 37 = Basic primitives (decorators, schemas). Chapter 38 = Production patterns (context, sampling, streaming, scaling).

## Assumed Knowledge

**What students know BEFORE this chapter (from Chapter 37)**:
- MCP architecture: Host → Client → Server flow
- Three primitives: Tools, Resources, Prompts with decorators
- @mcp.tool with Pydantic Field and type hints
- @mcp.resource with URI patterns and MIME types
- @mcp.prompt with arguments and message structures
- Transport options: stdio for local, HTTP for remote
- MCP Inspector for basic debugging
- Client configuration (Claude Desktop, Claude Code)

**What students know from Part 5**:
- Python async/await, decorators, type hints
- Pydantic for data validation
- uv package manager

**What this chapter teaches (ADVANCED PATTERNS ONLY)**:
- Context object injection and server lifespan management
- Sampling: Servers requesting LLM calls through clients
- Progress and logging notifications for long-running operations
- Roots for file system permission control
- StreamableHTTP transport for production deployments
- Stateful vs stateless server configurations for scaling
- Structured error handling and recovery patterns
- Packaging and distribution for production

## User Scenarios & Testing

### User Story 1 - Use Context Object in Tools (Priority: P1)

A student building a production server needs to access the Context object within tool functions for logging, progress reporting, and sampling.

**Why this priority**: Context is the gateway to all advanced features.

**Independent Test**: Student can inject Context into tool functions and access its methods.

**Acceptance Scenarios**:

1. **Given** a tool function, **When** student adds `ctx: Context` parameter, **Then** FastMCP injects the context automatically
2. **Given** context access, **When** student calls `await ctx.info("message")`, **Then** log appears in client
3. **Given** context access, **When** student accesses `ctx.session`, **Then** they can make sampling requests

---

### User Story 2 - Implement Sampling (Priority: P1)

A student wants their MCP server to call Claude (or another LLM) through the connected client, shifting cost and complexity to the client.

**Why this priority**: Sampling is the key differentiator for intelligent servers that need AI capabilities.

**Independent Test**: Student can implement a tool that uses sampling to generate text via the client.

**Acceptance Scenarios**:

1. **Given** a tool with Context, **When** student calls `ctx.session.create_message()`, **Then** client receives sampling request
2. **Given** sampling request, **When** client has sampling callback configured, **Then** Claude generates response
3. **Given** sampling result, **When** tool receives response, **Then** it can use the generated text in its return value

---

### User Story 3 - Send Progress Notifications (Priority: P1)

A student building a long-running tool (research, data processing) wants to show progress to users instead of appearing frozen.

**Why this priority**: UX critical for tools that take more than a few seconds.

**Independent Test**: Student can send progress updates that appear in the client during tool execution.

**Acceptance Scenarios**:

1. **Given** a long-running tool, **When** student calls `await ctx.report_progress(50, 100)`, **Then** client displays 50% progress
2. **Given** multiple progress calls, **When** tool progresses, **Then** client shows updated progress bar
3. **Given** tool completion, **When** tool returns, **Then** progress shows 100%

---

### User Story 4 - Send Logging Notifications (Priority: P2)

A student wants to send status messages to the client during tool execution for debugging and user feedback.

**Why this priority**: Essential for transparency in complex operations.

**Independent Test**: Student can send log messages that appear in the client.

**Acceptance Scenarios**:

1. **Given** a tool with Context, **When** student calls `await ctx.info("Starting research...")`, **Then** message appears in client
2. **Given** different log levels, **When** student uses info/warning/error, **Then** client displays appropriately
3. **Given** logging during execution, **When** tool runs, **Then** user sees real-time status updates

---

### User Story 5 - Implement Roots for File Access (Priority: P2)

A student building a file-processing server needs to define which directories the server can access, enabling path discovery without full filesystem exposure.

**Why this priority**: Security and UX for file-based servers.

**Independent Test**: Student can implement roots that limit and expose file access paths.

**Acceptance Scenarios**:

1. **Given** roots configuration, **When** client calls list_roots, **Then** approved directories are returned
2. **Given** a file path request, **When** path is within roots, **Then** access is granted
3. **Given** a file path request, **When** path is outside roots, **Then** access is denied with clear error

---

### User Story 6 - Configure StreamableHTTP Transport (Priority: P2)

A student wants to deploy their server remotely (not just local stdio) for production use cases.

**Why this priority**: Production servers need HTTP transport.

**Independent Test**: Student can run their server with StreamableHTTP and connect from remote clients.

**Acceptance Scenarios**:

1. **Given** FastMCP server, **When** configured for HTTP, **Then** server listens on specified port
2. **Given** HTTP transport, **When** client connects, **Then** SSE connection is established
3. **Given** session ID, **When** client makes requests, **Then** server tracks session state

---

### User Story 7 - Choose Stateful vs Stateless Configuration (Priority: P2)

A student needs to scale their server horizontally behind a load balancer, understanding the tradeoffs of stateless mode.

**Why this priority**: Production scaling requires understanding these flags.

**Independent Test**: Student can configure stateless_http and json_response flags appropriately.

**Acceptance Scenarios**:

1. **Given** `stateless_http=True`, **When** server runs, **Then** no session IDs are issued
2. **Given** stateless mode, **When** sampling is attempted, **Then** error indicates feature unavailable
3. **Given** `json_response=True`, **When** tool is called, **Then** response is plain JSON (no streaming)

---

### User Story 8 - Handle Errors Gracefully (Priority: P2)

A student needs their server to handle errors without crashing, returning structured error responses to clients.

**Why this priority**: Production servers must be resilient.

**Independent Test**: Student can implement error handling that returns proper JSON-RPC errors.

**Acceptance Scenarios**:

1. **Given** a tool that raises exception, **When** error occurs, **Then** server returns JSON-RPC error (not crash)
2. **Given** structured error, **When** client receives it, **Then** error code and message are clear
3. **Given** recoverable error, **When** client retries, **Then** server handles retry correctly

---

### User Story 9 - Package Server for Distribution (Priority: P3)

A student wants to share their production-ready server with others via pip/uv install.

**Why this priority**: Distribution is the end goal for sellable agents.

**Independent Test**: Student can package and distribute their server.

**Acceptance Scenarios**:

1. **Given** pyproject.toml configured, **When** user runs `uv pip install`, **Then** server installs
2. **Given** entry points defined, **When** user runs command, **Then** server starts
3. **Given** dependencies specified, **When** installed in fresh environment, **Then** all dependencies resolve

---

### User Story 10 - Build Production Server (Capstone) (Priority: P3)

A student applies all advanced patterns to build a production-ready server with sampling, progress, roots, and proper error handling.

**Why this priority**: Capstone integrates all advanced learning.

**Independent Test**: Student can build a complete server demonstrating all advanced features.

**Acceptance Scenarios**:

1. **Given** capstone requirements, **When** student designs server, **Then** they use Context, sampling, progress appropriately
2. **Given** implementation, **When** server runs with StreamableHTTP, **Then** all features work
3. **Given** completed server, **When** deployed, **Then** it handles real workloads reliably

---

### Edge Cases

- What happens when sampling callback is not configured? (Error returned to server)
- What happens when progress is reported after tool returns? (Ignored, no error)
- What happens when roots list is empty? (No file access allowed)
- What happens when StreamableHTTP client disconnects? (Session cleanup)
- What happens when stateless server receives session-dependent request? (Clear error)

## Requirements

### Functional Requirements

- **FR-001**: Chapter MUST teach Context object injection in tool functions
- **FR-002**: Chapter MUST teach server lifespan management
- **FR-003**: Chapter MUST teach sampling via `ctx.session.create_message()`
- **FR-004**: Chapter MUST teach client-side sampling callback implementation
- **FR-005**: Chapter MUST teach progress notifications via `ctx.report_progress()`
- **FR-006**: Chapter MUST teach logging notifications via `ctx.info()`, `ctx.warning()`, `ctx.error()`
- **FR-007**: Chapter MUST teach roots for file system permission control
- **FR-008**: Chapter MUST teach `is_path_allowed()` pattern for path validation
- **FR-009**: Chapter MUST teach StreamableHTTP transport configuration
- **FR-010**: Chapter MUST teach SSE connection model (primary + tool-specific)
- **FR-011**: Chapter MUST teach `stateless_http` flag and its implications
- **FR-012**: Chapter MUST teach `json_response` flag and its implications
- **FR-013**: Chapter MUST teach structured error handling patterns
- **FR-014**: Chapter MUST teach pyproject.toml configuration for distribution
- **FR-015**: Chapter MUST include capstone with all advanced features
- **FR-016**: Chapter MUST NOT re-teach @mcp.tool/@mcp.resource/@mcp.prompt basics (Chapter 37)
- **FR-017**: Chapter MUST NOT re-teach MCP architecture concepts (Chapter 37)

### Key Entities

- **Context Object**: Injected parameter providing access to logging, progress, and session
- **Sampling**: Server-to-client LLM request mechanism via `create_message()`
- **Progress Notification**: Real-time progress updates via `report_progress()`
- **Logging Notification**: Real-time log messages via `info()`, `warning()`, `error()`
- **Roots**: File system permission boundaries for secure path access
- **StreamableHTTP**: HTTP transport with SSE for bidirectional communication
- **Session State**: Stateful vs stateless server configuration for scaling

## Success Criteria

### Measurable Outcomes

- **SC-001**: Students can inject and use Context object in tools
- **SC-002**: Students can implement sampling to call LLMs through clients
- **SC-003**: Students can send progress notifications for long-running operations
- **SC-004**: Students can send logging notifications for status updates
- **SC-005**: Students can implement roots for file access control
- **SC-006**: Students can configure StreamableHTTP for production deployment
- **SC-007**: Students can choose appropriate stateful/stateless configuration
- **SC-008**: Students can handle errors gracefully with structured responses
- **SC-009**: Students can package servers for distribution
- **SC-010**: NO overlap with Chapter 37 content (no re-teaching of basics)

### Educational Outcomes

- **EO-001**: Students understand Context as the gateway to advanced features
- **EO-002**: Students can design servers that leverage client LLM capabilities
- **EO-003**: Students can build production-ready servers with proper UX
- **EO-004**: Students can scale servers horizontally with appropriate tradeoffs

## Lesson Structure

### Lesson 1: Context Object & Server Lifespan (L2 - AI Collaboration)
- Context injection in tool/resource/prompt functions
- Accessing session, logging, and progress methods
- Server lifespan management
- Async context managers for resources

### Lesson 2: Sampling - Servers Calling LLMs (L2 - AI Collaboration)
- The problem sampling solves (cost, complexity)
- `ctx.session.create_message()` API
- SamplingMessage and TextContent structures
- Client-side sampling callback implementation
- When to use sampling vs direct API calls

### Lesson 3: Progress & Logging Notifications (L2 - AI Collaboration)
- `ctx.report_progress(current, total)` for progress bars
- `ctx.info()`, `ctx.warning()`, `ctx.error()` for logging
- Client-side notification handling
- UX patterns for long-running operations

### Lesson 4: Roots - File System Permissions (L2 - AI Collaboration)
- What roots solve (security + path discovery)
- Implementing `list_roots` tool
- `is_path_allowed()` validation pattern
- Integration with file-processing tools

### Lesson 5: StreamableHTTP Transport (L2 - AI Collaboration)
- stdio vs StreamableHTTP tradeoffs
- Configuring HTTP transport
- SSE connection model (primary + tool-specific)
- Session ID management
- Production deployment patterns

### Lesson 6: Stateful vs Stateless Servers (L2 - AI Collaboration)
- Horizontal scaling challenges
- `stateless_http=True` configuration
- `json_response=True` configuration
- Feature limitations in stateless mode
- When to use each configuration

### Lesson 7: Error Handling & Recovery (L2 - AI Collaboration)
- Structured JSON-RPC error responses
- Exception handling in tools
- Graceful degradation patterns
- Retry-safe tool design
- Logging for debugging

### Lesson 8: Packaging & Distribution (L2 - AI Collaboration)
- pyproject.toml configuration
- Entry points for CLI commands
- Dependencies specification
- Installation testing
- Documentation requirements

### Lesson 9: Capstone - Production MCP Server (L4 - Spec-Driven)
- Requirements analysis
- Feature selection (sampling, progress, roots)
- StreamableHTTP deployment
- Error handling implementation
- Integration testing
- Documentation

### Lesson 10: Chapter Quiz (Assessment)
- Context object usage
- Sampling patterns
- Notification types
- Transport configuration
- Scaling tradeoffs
- Error handling

## Dependencies

- **Chapter 37**: MCP Fundamentals (PREREQUISITE - basics already taught)
- **Part 5**: Python Fundamentals (async/await, decorators, type hints, Pydantic)
- **Tools**: uv, Python 3.11+, Node.js (for Inspector)

## Assumptions

- Students have completed Chapter 37 (understand basic decorators)
- Students have completed Part 5 (Python skills)
- Students have MCP Inspector available
- MCP SDK version 1.6+ (December 2025)

## Out of Scope

- Basic @mcp.tool/@mcp.resource/@mcp.prompt patterns (Chapter 37)
- MCP architecture explanation (Chapter 37)
- Transport layer theory (Chapter 37)
- Client configuration (Chapter 37)
- OAuth 2.1 implementation (mentioned only)
- Database integration (Chapters 46-48)
- Cloud deployment specifics (Part 7)
