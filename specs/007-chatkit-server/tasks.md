# ChatKit Server Implementation Tasks

**Version**: 1.0 (Reverse Engineered)
**Date**: 2025-01-XX

## Overview

This task breakdown represents how to rebuild the ChatKit server from scratch using the specification and plan.

**Estimated Timeline**: 2-3 weeks (single developer)
**Team Size**: 1 backend engineer

---

## Phase 1: Core Infrastructure Setup

**Timeline**: Days 1-2
**Dependencies**: None

### Task 1.1: Project Setup

- [ ] Initialize Python project with `pyproject.toml`
- [ ] Configure dependencies: FastAPI, ChatKit SDK, Agents SDK, SQLAlchemy
- [ ] Setup virtual environment (uv or venv)
- [ ] Configure linting (ruff, mypy)
- [ ] Setup pre-commit hooks
- [ ] Create project structure: `chatkit_server.py`, `app.py`, `lifespan.py`

### Task 1.2: Configuration System

- [ ] Implement `core/config.py` with Pydantic Settings
- [ ] Support environment variables: `DATABASE_URL`, `CHATKIT_STORE_DATABASE_URL`
- [ ] Add validation for required settings
- [ ] Support `.env` file for local development
- [ ] Document configuration options

### Task 1.3: Logging Infrastructure

- [ ] Setup structured logging (JSON format)
- [ ] Configure log levels: DEBUG, INFO, WARN, ERROR
- [ ] Add request correlation IDs
- [ ] Integrate with logging destination (stdout, file, etc.)

---

## Phase 2: Database Layer

**Timeline**: Days 3-5
**Dependencies**: Phase 1 complete

### Task 2.1: PostgreSQL Store Implementation

- [ ] Create `chatkit_store/postgres_store.py`
- [ ] Implement `PostgresStore` class extending `Store[RequestContext]`
- [ ] Setup async SQLAlchemy engine with connection pooling
- [ ] Implement schema initialization (`initialize_schema()`)
- [ ] Create tables: `threads`, `items`, `attachments`
- [ ] Add indexes for performance (user_id, thread_id, created_at)

### Task 2.2: Request Context

- [ ] Create `chatkit_store/context.py`
- [ ] Implement `RequestContext` Pydantic model
- [ ] Support user_id, organization_id, request_id, metadata
- [ ] Add validation for required fields

### Task 2.3: Store Configuration

- [ ] Create `chatkit_store/config.py`
- [ ] Implement `StoreConfig` with database URL, pool settings
- [ ] Support environment variable prefixes
- [ ] Add validation and defaults

### Task 2.4: Connection Pool Warmup

- [ ] Implement connection warmup in `lifespan.py`
- [ ] Pre-create connections to avoid first-request delay
- [ ] Add logging for warmup status
- [ ] Handle warmup failures gracefully

---

## Phase 3: ChatKit Server Integration

**Timeline**: Days 6-8
**Dependencies**: Phase 2 complete

### Task 3.1: ChatKit Server Class

- [ ] Create `chatkit_server.py`
- [ ] Implement `RoboLearnChatKitServer` extending `ChatKitServer[RequestContext]`
- [ ] Override `respond()` method
- [ ] Handle read-only operations (let base class handle)
- [ ] Handle user message operations (custom agent logic)

### Task 3.2: Thread Item Converter

- [ ] Implement `RoboLearnThreadItemConverter` extending `ThreadItemConverter`
- [ ] Handle image attachments (convert to `ResponseInputImageContentParam`)
- [ ] Handle file attachments (convert to `ResponseInputFileContentParam`)
- [ ] Download files from attachment store URLs
- [ ] Convert to base64 data URLs
- [ ] Add error handling for download failures

### Task 3.3: Message History Loading

- [ ] Load last 10 messages from thread
- [ ] Reverse chronological order
- [ ] Extract user and assistant messages
- [ ] Format as role:content string
- [ ] Include in agent system prompt

### Task 3.4: Context Extraction

- [ ] Extract user info from `RequestContext.metadata`
- [ ] Extract page context from `RequestContext.metadata`
- [ ] Build user context string
- [ ] Build page context string
- [ ] Include both in agent system prompt

---

## Phase 4: Agent Integration

**Timeline**: Days 9-11
**Dependencies**: Phase 3 complete

### Task 4.1: Agent Context

- [ ] Create `state.py`
- [ ] Implement `RoboLearnAgentContext` extending `AgentContext[RequestContext]`
- [ ] Include required fields: thread, store, request_context

### Task 4.2: System Prompt

- [ ] Create `prompts.py`
- [ ] Define `ROBOLEARN_SYSTEM` prompt
- [ ] Include instructions for:
  - Using search tool
  - Citing sources
  - Remembering user information
  - Using conversation history

### Task 4.3: RAG Tool Integration

- [ ] Ensure `rag/tools.py` has `search_tool` function
- [ ] Verify `@function_tool` decorator
- [ ] Verify `ToolContext[RoboLearnAgentContext]` parameter
- [ ] Test tool execution

### Task 4.4: Agent Execution

- [ ] Create agent with `search_tool` in tools
- [ ] Build enhanced system prompt (history + user context + page context + system)
- [ ] Convert user message to agent input format
- [ ] Run agent with `Runner.run_streamed()`
- [ ] Stream results using `stream_agent_response()`
- [ ] Handle streaming errors gracefully

---

## Phase 5: HTTP Endpoint

**Timeline**: Days 12-13
**Dependencies**: Phase 4 complete

### Task 5.1: FastAPI Application

- [ ] Create `app.py` with FastAPI app
- [ ] Configure CORS middleware
- [ ] Add lifespan context manager
- [ ] Register health check endpoint

### Task 5.2: ChatKit Endpoint

- [ ] Implement `POST /chatkit` endpoint
- [ ] Extract user_id from `X-User-ID` header
- [ ] Extract request_id from `X-Request-ID` header (optional)
- [ ] Parse ChatKit protocol payload
- [ ] Extract metadata from payload
- [ ] Create `RequestContext` with user_id, request_id, metadata
- [ ] Call `chatkit_server.process()`
- [ ] Return StreamingResponse for streaming, JSON Response for read-only

### Task 5.3: Error Handling

- [ ] Handle missing user_id (401 error)
- [ ] Handle ChatKit server not initialized (503 error)
- [ ] Handle processing errors (500 error with message)
- [ ] Log all errors with context
- [ ] Return user-friendly error messages

---

## Phase 6: Application Lifecycle

**Timeline**: Day 14
**Dependencies**: Phase 5 complete

### Task 6.1: Lifespan Management

- [ ] Create `lifespan.py` with `asynccontextmanager`
- [ ] Startup: Initialize PostgreSQL store
- [ ] Startup: Create ChatKit server
- [ ] Startup: Warm up connection pool
- [ ] Startup: Store server in app.state
- [ ] Shutdown: Close database connections
- [ ] Handle initialization failures gracefully

### Task 6.2: Graceful Degradation

- [ ] Check for database URL
- [ ] Log warning if unavailable
- [ ] Continue startup (ChatKit disabled)
- [ ] Return 503 if ChatKit not available
- [ ] Don't crash entire application

---

## Phase 7: Testing & Quality

**Timeline**: Days 15-17
**Dependencies**: All phases complete

### Task 7.1: Unit Tests

- [ ] Test `RoboLearnChatKitServer.respond()`
- [ ] Test `RoboLearnThreadItemConverter`
- [ ] Test context extraction (user info, page context)
- [ ] Test message history loading
- [ ] Test agent prompt building
- [ ] Mock external dependencies (database, agent SDK)

### Task 7.2: Integration Tests

- [ ] Test `/chatkit` endpoint with real database
- [ ] Test thread creation and retrieval
- [ ] Test message persistence
- [ ] Test agent execution with RAG search
- [ ] Test streaming responses
- [ ] Test error handling

### Task 7.3: End-to-End Tests

- [ ] Test full conversation flow:
  - Create thread
  - Send message
  - Receive response
  - Send follow-up
  - Verify history included
- [ ] Test with file attachments
- [ ] Test with user context
- [ ] Test with page context

### Task 7.4: Performance Testing

- [ ] Test connection pool performance
- [ ] Test concurrent requests
- [ ] Test streaming response latency
- [ ] Test database query performance
- [ ] Document performance baselines

---

## Phase 8: Improvements & Production Readiness

**Timeline**: Days 18-21
**Dependencies**: Phase 7 complete

### Task 8.1: Rate Limiting

- [ ] Add rate limiting middleware
- [ ] Configure per-user limits
- [ ] Add rate limit headers to responses
- [ ] Test rate limiting behavior

### Task 8.2: Authentication Validation

- [ ] Integrate with auth system
- [ ] Validate user_id against auth service or JWT
- [ ] Reject invalid user_ids
- [ ] Add authentication error handling

### Task 8.3: Metrics & Monitoring

- [ ] Add Prometheus metrics:
  - Request rate
  - Latency (p50, p95, p99)
  - Error rate
  - Agent execution time
- [ ] Add health check endpoint (`/health`)
- [ ] Add readiness check (database connectivity)
- [ ] Setup alerting for errors

### Task 8.4: Error Handling Improvements

- [ ] Add retry logic for transient failures
- [ ] Improve error messages (more specific)
- [ ] Add error recovery mechanisms
- [ ] Test error scenarios

### Task 8.5: Documentation

- [ ] Write API documentation
- [ ] Document configuration options
- [ ] Document deployment process
- [ ] Create troubleshooting guide
- [ ] Add code comments for complex logic

---

## Phase 9: Deployment

**Timeline**: Days 22-23
**Dependencies**: Phase 8 complete

### Task 9.1: Containerization

- [ ] Write production Dockerfile
- [ ] Multi-stage build for optimization
- [ ] Non-root user for security
- [ ] Health check in container
- [ ] Optimize image size

### Task 9.2: Environment Configuration

- [ ] Document required environment variables
- [ ] Create `.env.example` file
- [ ] Document production configuration
- [ ] Setup secrets management

### Task 9.3: CI/CD Pipeline

- [ ] Setup GitHub Actions workflow
- [ ] Run tests on PR
- [ ] Build Docker image
- [ ] Deploy to staging
- [ ] Manual approval for production

---

## Post-Launch

**Timeline**: Ongoing

### Task 10.1: Monitoring & Incident Response

- [ ] Monitor production metrics
- [ ] Respond to alerts
- [ ] Conduct post-mortems
- [ ] Iterate on improvements

### Task 10.2: Feature Iterations

- [ ] Prioritize improvements from gaps
- [ ] Implement high-priority features
- [ ] Gather user feedback
- [ ] A/B test new features

