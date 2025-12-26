# Feature Specification: Chapter 38 - Building Custom MCP Servers

**Feature Branch**: `047-chapter-38-mcp-servers`
**Created**: 2025-12-26
**Status**: Draft (Revised)
**Input**: User description: "Chapter 38: Building Custom MCP Servers - Teaching students to BUILD MCP servers using Python's FastMCP SDK. Assumes Chapter 37 already taught MCP concepts."

**Chapter Position**: Part 6, Chapter 38 (follows Chapter 37: MCP Fundamentals)
**Proficiency Level**: B1-B2 (students have Python fundamentals from Part 5)

## Scope Clarification: Chapter 37 vs Chapter 38

| Topic | Chapter 37 (Fundamentals) | Chapter 38 (Building) |
|-------|--------------------------|----------------------|
| MCP Architecture | ✓ Conceptual overview | ✗ Skip (already covered) |
| Transport Layers | ✓ Theory of stdio/HTTP | ✗ Skip (already covered) |
| What Tools ARE | ✓ Model-controlled primitive | ✗ Skip (already covered) |
| What Resources ARE | ✓ App-controlled primitive | ✗ Skip (already covered) |
| What Prompts ARE | ✓ User-controlled primitive | ✗ Skip (already covered) |
| Client Configuration | ✓ Claude Code, Cursor setup | ✗ Skip (already covered) |
| Using Community Servers | ✓ Finding and using servers | ✗ Skip (already covered) |
| Debugging Basics | ✓ Troubleshooting connections | ✗ Skip (already covered) |
| **Server Project Setup** | ✗ | ✓ FastMCP, uv, dependencies |
| **@mcp.tool Code Patterns** | ✗ | ✓ Decorators, Pydantic, schemas |
| **@mcp.resource Code Patterns** | ✗ | ✓ URI handling, MIME types |
| **@mcp.prompt Code Patterns** | ✗ | ✓ Message structures |
| **Server Authentication** | ✗ | ✓ Env vars, API keys |
| **Testing YOUR Server** | ✗ | ✓ Inspector for server authors |
| **Packaging & Publishing** | ✗ | ✓ Distribution, PyPI |
| **Capstone Project** | ✗ | ✓ Domain-specific server |

**Key Distinction**: Chapter 37 = CONSUMER perspective (use servers). Chapter 38 = PRODUCER perspective (build servers).

## Assumed Knowledge

**What students know BEFORE this chapter (from Chapter 37)**:
- MCP architecture: Host → Client → Server flow
- Three primitives conceptually: Tools (model-controlled), Resources (app-controlled), Prompts (user-controlled)
- Transport options: stdio for local, HTTP for remote
- How to configure clients (Claude Desktop, Claude Code)
- How to use existing community MCP servers
- Basic debugging of MCP connections

**What students know from Part 5**:
- Python async/await, decorators, type hints
- Pydantic for data validation
- uv package manager

**What this chapter teaches (SERVER IMPLEMENTATION ONLY)**:
- FastMCP SDK project structure and initialization
- @mcp.tool decorator patterns with Pydantic Field
- @mcp.resource decorator patterns with URI templates
- @mcp.prompt decorator patterns with message structures
- Server-side authentication (environment variables, startup validation)
- Testing from the server author's perspective
- Packaging servers for distribution
- Building a complete domain-specific server (capstone)

## User Scenarios & Testing

### User Story 1 - Create First MCP Server Project (Priority: P1)

A student who understands MCP concepts from Chapter 37 wants to create their first server project. They need the correct project structure, dependencies, and minimal code to get a working server.

**Why this priority**: This is the entry point—students need a working project before implementing primitives.

**Independent Test**: Student can create a new project with `uv init`, add correct dependencies, write minimal FastMCP code, and verify it runs.

**Acceptance Scenarios**:

1. **Given** uv and Python installed, **When** student runs `uv init my-mcp-server && cd my-mcp-server && uv add mcp uvicorn`, **Then** dependencies install successfully
2. **Given** the project setup, **When** student writes FastMCP initialization code, **Then** the server starts without errors
3. **Given** a running server, **When** student accesses it via MCP Inspector, **Then** the connection succeeds (even with no primitives yet)

---

### User Story 2 - Implement Tools with @mcp.tool (Priority: P1)

A student wants to implement tools that Claude can call. They need to understand the decorator pattern, how type hints become JSON schemas, and how to use Pydantic Field for descriptions.

**Why this priority**: Tools are the most common primitive—most servers exist primarily to provide tools.

**Independent Test**: Student can implement a tool with parameters, descriptions, and return values that Claude can successfully call.

**Acceptance Scenarios**:

1. **Given** FastMCP server code, **When** student adds @mcp.tool decorator to a function, **Then** the tool appears in Inspector's tool list
2. **Given** a tool with typed parameters, **When** student views the tool schema, **Then** types are correctly converted to JSON schema
3. **Given** Pydantic Field with description, **When** student views tool in Inspector, **Then** parameter descriptions appear
4. **Given** a tool implementation, **When** Claude calls it, **Then** the function executes and returns results

---

### User Story 3 - Implement Resources with @mcp.resource (Priority: P2)

A student wants to expose read-only data as resources. They need to understand URI patterns (direct vs templated) and MIME type handling.

**Why this priority**: Resources are the second most common primitive, essential for data-centric servers.

**Independent Test**: Student can implement both direct and templated resources with correct URI patterns and MIME types.

**Acceptance Scenarios**:

1. **Given** @mcp.resource with static URI, **When** client reads the resource, **Then** data is returned correctly
2. **Given** @mcp.resource with `{parameter}` in URI, **When** client reads with parameter value, **Then** the function receives the parameter
3. **Given** mime_type specification, **When** client reads resource, **Then** response includes correct content type

---

### User Story 4 - Implement Prompts with @mcp.prompt (Priority: P2)

A student wants to create prompt templates that encode domain expertise. They need to understand message structures and parameter handling.

**Why this priority**: Prompts package expertise—essential for specialized servers.

**Independent Test**: Student can implement a parameterized prompt that returns structured messages.

**Acceptance Scenarios**:

1. **Given** @mcp.prompt decorator, **When** student lists prompts in Inspector, **Then** the prompt appears with description
2. **Given** prompt with parameters, **When** user requests it with arguments, **Then** function receives parameters correctly
3. **Given** prompt returning messages, **When** applied to conversation, **Then** AI receives structured instructions

---

### User Story 5 - Handle Server Authentication (Priority: P2)

A student needs their server to connect to external APIs securely. They need to handle API keys via environment variables and validate at startup.

**Why this priority**: Production servers typically connect to authenticated APIs.

**Independent Test**: Student can implement environment variable validation at startup and use credentials in tool implementations.

**Acceptance Scenarios**:

1. **Given** required environment variable, **When** variable is missing, **Then** server refuses to start with clear error
2. **Given** variable is set, **When** tool makes API call, **Then** key is used without appearing in logs
3. **Given** httpx/requests call, **When** API returns error, **Then** error message doesn't expose credentials

---

### User Story 6 - Test Server During Development (Priority: P2)

A student needs to test their server as they build it. They need to use MCP Inspector effectively from the server author's perspective.

**Why this priority**: Rapid iteration requires effective testing.

**Independent Test**: Student can use Inspector to test all primitives and diagnose issues.

**Acceptance Scenarios**:

1. **Given** a server with tools, **When** student uses Inspector, **Then** they can list, call, and verify tools
2. **Given** a failing tool, **When** student checks stderr output, **Then** they can identify the error
3. **Given** a change to server code, **When** student restarts server, **Then** changes are reflected in Inspector

---

### User Story 7 - Package Server for Distribution (Priority: P3)

A student wants to share their server with others. They need to package it correctly for distribution.

**Why this priority**: The goal is sellable/shareable agents—packaging is essential.

**Independent Test**: Student can package their server so others can install and run it.

**Acceptance Scenarios**:

1. **Given** a working server, **When** student adds pyproject.toml metadata, **Then** package is installable via pip/uv
2. **Given** packaged server, **When** another user installs it, **Then** they can configure it in Claude Desktop
3. **Given** server with dependencies, **When** packaged, **Then** all dependencies are correctly specified

---

### User Story 8 - Build Domain-Specific Server (Capstone) (Priority: P3)

A student applies all skills to build a complete, useful MCP server for a specific domain (e.g., project management, CRM, analytics).

**Why this priority**: Capstone integrates all learning into practical deliverable.

**Independent Test**: Student can build a server with multiple tools, resources, and prompts for a real use case.

**Acceptance Scenarios**:

1. **Given** capstone requirements, **When** student designs server, **Then** they choose appropriate primitives for each capability
2. **Given** implementation, **When** server is tested, **Then** all primitives work together cohesively
3. **Given** completed server, **When** connected to Claude Desktop, **Then** it provides meaningful domain functionality

---

### Edge Cases

- What happens when a tool is called with missing required parameters? (Pydantic validation error)
- What happens when @mcp.resource URI doesn't match template? (Resource not found)
- What happens when server uses print() with stdio transport? (Corrupts JSON-RPC—use logging to stderr)
- What happens when API key has special characters? (Should work with proper env var handling)
- What happens when tool raises exception? (Server returns error to client, doesn't crash)

## Requirements

### Functional Requirements

- **FR-001**: Chapter MUST teach FastMCP project setup with uv (uv init, uv add mcp uvicorn)
- **FR-002**: Chapter MUST teach @mcp.tool decorator including name, description parameters
- **FR-003**: Chapter MUST teach how Python type hints generate JSON schemas automatically
- **FR-004**: Chapter MUST teach Pydantic Field for parameter descriptions and validation
- **FR-005**: Chapter MUST teach @mcp.resource for both direct (static URI) and templated resources
- **FR-006**: Chapter MUST teach MIME type specification in @mcp.resource
- **FR-007**: Chapter MUST teach @mcp.prompt for creating prompt templates with parameters
- **FR-008**: Chapter MUST teach message structure returned by prompts (user/assistant messages)
- **FR-009**: Chapter MUST teach environment variable handling with os.environ
- **FR-010**: Chapter MUST teach startup validation pattern (check env vars before server starts)
- **FR-011**: Chapter MUST teach logging to stderr (not stdout) for stdio transport
- **FR-012**: Chapter MUST teach using MCP Inspector for server development testing
- **FR-013**: Chapter MUST teach pyproject.toml configuration for distribution
- **FR-014**: Chapter MUST include capstone project: building a complete domain-specific server
- **FR-015**: Chapter MUST NOT re-explain what primitives ARE (covered in Chapter 37)
- **FR-016**: Chapter MUST NOT re-explain transport theory (covered in Chapter 37)
- **FR-017**: Chapter MUST NOT re-explain client configuration (covered in Chapter 37)

### Key Entities

- **FastMCP Instance**: The server object created with `FastMCP("server-name")`. Entry point for all decorators.
- **Tool Function**: Python function decorated with @mcp.tool. Parameters become JSON schema, return value goes to client.
- **Resource Function**: Python function decorated with @mcp.resource. Returns data at URI pattern.
- **Prompt Function**: Python function decorated with @mcp.prompt. Returns list of messages for AI.
- **Server Package**: Distributable package with pyproject.toml, installable via pip/uv.

## Success Criteria

### Measurable Outcomes

- **SC-001**: Students can create a working MCP server project in under 10 minutes
- **SC-002**: Students can implement a tool with proper type hints and descriptions
- **SC-003**: Students can implement both direct and templated resources
- **SC-004**: Students can implement a parameterized prompt template
- **SC-005**: Students can configure environment variable validation
- **SC-006**: Students can package their server for distribution
- **SC-007**: Students complete capstone with multiple working primitives
- **SC-008**: NO overlap with Chapter 37 content (no re-explanation of concepts)

### Educational Outcomes

- **EO-001**: Students can translate domain requirements into MCP primitives
- **EO-002**: Students understand decorator-based development pattern
- **EO-003**: Students can debug their own server independently
- **EO-004**: Students can distribute servers to others

## Lesson Structure

### Lesson 1: Project Setup with FastMCP (L2 - AI Collaboration)
- Creating project with uv
- Adding dependencies (mcp, uvicorn, httpx)
- FastMCP initialization code
- Running with `mcp dev` or uvicorn
- First connection via Inspector

### Lesson 2: Implementing Tools (@mcp.tool) (L2 - AI Collaboration)
- The @mcp.tool decorator
- Type hints → JSON schema conversion
- Pydantic Field for descriptions
- Return values and error handling
- Multi-tool servers

### Lesson 3: Implementing Resources (@mcp.resource) (L2 - AI Collaboration)
- Direct resources (static URIs)
- Templated resources ({parameter} in URI)
- MIME type specification
- When to use resources vs tools

### Lesson 4: Implementing Prompts (@mcp.prompt) (L2 - AI Collaboration)
- The @mcp.prompt decorator
- Parameter handling
- Message structures (user/assistant)
- Use cases for prompt templates

### Lesson 5: Server Authentication & Security (L2 - AI Collaboration)
- Environment variables with os.environ
- Startup validation pattern
- Secure API call patterns
- Logging to stderr (not stdout)

### Lesson 6: Testing & Debugging Your Server (L2 - AI Collaboration)
- MCP Inspector for development
- Reading server logs (stderr)
- Common errors and fixes
- Iterative development workflow

### Lesson 7: Packaging & Publishing (L2 - AI Collaboration)
- pyproject.toml configuration
- Entry points for CLI
- Dependencies specification
- Installation testing

### Lesson 8: Capstone - Domain-Specific Server (L4 - Spec-Driven)
- Requirements analysis
- Primitive selection (tools vs resources vs prompts)
- Implementation with AI collaboration
- Testing and validation
- Documentation

### Lesson 9: Chapter Quiz (Assessment)
- Implementation patterns (decorators, type hints)
- Debugging scenarios
- Security best practices
- Distribution requirements

## Dependencies

- **Chapter 37**: MCP Fundamentals (PREREQUISITE - concepts already taught)
- **Part 5**: Python Fundamentals (async/await, decorators, type hints, Pydantic)
- **Tools**: uv, Python 3.11+, Node.js (for Inspector)

## Assumptions

- Students have completed Chapter 37 (understand MCP concepts)
- Students have completed Part 5 (Python skills)
- Students have MCP Inspector available (npx @modelcontextprotocol/inspector)
- MCP SDK version 1.6+ (December 2025)

## Out of Scope

- MCP architecture explanation (Chapter 37)
- Transport layer theory (Chapter 37)
- What primitives ARE conceptually (Chapter 37)
- Client configuration (Chapter 37)
- Using community servers (Chapter 37)
- OAuth 2.1 (advanced, mentioned only)
- Database integration (Chapters 46-48)
- Cloud deployment (Part 7)
