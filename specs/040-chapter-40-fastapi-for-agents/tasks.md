# Chapter 40: FastAPI for Agents — Tasks

**Version**: 1.0.0
**Status**: Active
**Created**: 2025-12-22
**Plan Reference**: specs/040-chapter-40-fastapi-for-agents/plan.md

---

## Task Overview

| Task ID | Description | Dependencies | Status |
|---------|-------------|--------------|--------|
| T01 | Create _category_.json | None | Pending |
| T02 | Implement Lesson 1: Hello FastAPI | T01 | Pending |
| T03 | Implement Lesson 2: POST and Pydantic Models | T02 | Pending |
| T04 | Implement Lesson 3: Full CRUD Operations | T03 | Pending |
| T05 | Implement Lesson 4: Error Handling | T04 | Pending |
| T06 | Implement Lesson 5: Dependency Injection | T05 | Pending |
| T07 | Implement Lesson 6: Streaming with SSE | T06 | Pending |
| T08 | Implement Lesson 7: Agent Integration | T07 | Pending |
| T09 | Implement Lesson 8: Capstone | T08 | Pending |
| T10 | Validate all lessons | T09 | Pending |

---

## Task Details

### T01: Create _category_.json

**Path**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/_category_.json`

**Content**:
```json
{
  "label": "Chapter 40: FastAPI for Agents",
  "position": 40,
  "link": {
    "type": "generated-index",
    "description": "Expose agent capabilities as production-ready REST APIs with FastAPI"
  }
}
```

**Acceptance Criteria**:
- [ ] File created in correct location
- [ ] Position matches chapter number
- [ ] Description matches README

---

### T02: Implement Lesson 1 — Hello FastAPI

**Path**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md`

**YAML Frontmatter**:
```yaml
---
title: "Hello FastAPI"
chapter: 40
lesson: 1
duration_minutes: 45

skills:
  - name: "Creating FastAPI Applications"
    proficiency_level: "A2"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student creates working FastAPI app with uvicorn"

  - name: "Path and Query Parameters"
    proficiency_level: "A2"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student implements endpoints with path and query params"

learning_objectives:
  - objective: "Create a FastAPI application and run it with uvicorn"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Application runs and responds at http://localhost:8000"

  - objective: "Implement endpoints with path and query parameters"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Endpoints accept parameters and return correct JSON"

cognitive_load:
  new_concepts: 5
  assessment: "FastAPI instance, decorators, path params, query params, Swagger UI"

generated_by: "content-implementer"
source_spec: "specs/040-chapter-40-fastapi-for-agents/spec.md"
---
```

**Content Sections**:
1. Introduction (connect to MCP HTTP concepts from Ch 38)
2. Creating the FastAPI App
3. Your First Endpoint
4. Path Parameters
5. Query Parameters
6. Swagger UI as Playground
7. Hands-On Exercise
8. Common Mistakes
9. Try With AI

**Acceptance Criteria**:
- [ ] All YAML frontmatter complete
- [ ] Code examples run correctly
- [ ] Swagger UI screenshot or description
- [ ] Hands-on exercise clear
- [ ] No framework labels in Try With AI

---

### T03: Implement Lesson 2 — POST and Pydantic Models

**Path**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/02-post-and-pydantic-models.md`

**YAML Frontmatter Skills**:
```yaml
skills:
  - name: "Pydantic Model Definition"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"

  - name: "Request Body Handling"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
```

**Content Sections**:
1. Why Pydantic?
2. Defining TaskCreate Model
3. Defining TaskResponse Model
4. POST Endpoint Implementation
5. Response Models
6. Validation Errors (422)
7. Hands-On: Create Tasks
8. Common Mistakes
9. Try With AI

**Acceptance Criteria**:
- [ ] TaskCreate and TaskResponse models complete
- [ ] In-memory storage pattern explained
- [ ] 422 validation errors demonstrated
- [ ] Swagger UI used for testing

---

### T04: Implement Lesson 3 — Full CRUD Operations

**Path**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/03-full-crud-operations.md`

**Content Sections**:
1. CRUD Overview
2. List All Tasks (GET /tasks)
3. Get Single Task (GET /tasks/{id})
4. Update Task (PUT)
5. Delete Task (DELETE)
6. Filtering with Query Params
7. Hands-On: Complete CRUD Cycle
8. Common Mistakes
9. Try With AI

**Acceptance Criteria**:
- [ ] All four CRUD operations implemented
- [ ] HTTPException for 404 introduced
- [ ] Status filter on list endpoint
- [ ] Full test workflow documented

---

### T05: Implement Lesson 4 — Error Handling

**Path**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/04-error-handling.md`

**Content Sections**:
1. HTTP Status Codes Reference
2. HTTPException Class
3. Using status Constants
4. 404 Not Found Pattern
5. 400 Bad Request (business rules)
6. 201 Created for POST
7. Hands-On: Error Cases
8. Common Mistakes
9. Try With AI

**Acceptance Criteria**:
- [ ] Status codes explained clearly
- [ ] HTTPException patterns shown
- [ ] status module from fastapi used
- [ ] Test cases for each error type

---

### T06: Implement Lesson 5 — Dependency Injection

**Path**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/05-dependency-injection.md`

**Layer Transition**: L1 → L2

**Content Sections**:
1. Why Dependency Injection?
2. The Depends() Function
3. Creating TaskRepository Class
4. Dependency Function Pattern
5. Injecting Repository
6. Hands-On: Refactor to DI
7. AI Collaboration: Repository Design
8. Common Mistakes
9. Try With AI

**Acceptance Criteria**:
- [ ] Repository pattern implemented
- [ ] Depends() usage clear
- [ ] Refactoring process documented
- [ ] AI collaboration introduced (no framework labels)

---

### T07: Implement Lesson 6 — Streaming with SSE

**Path**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/06-streaming-with-sse.md`

**Content Sections**:
1. Why Streaming?
2. Server-Sent Events Basics
3. Installing sse-starlette
4. Async Generators
5. EventSourceResponse
6. Task Updates Stream
7. Hands-On: Real-time Updates
8. Browser EventSource API
9. Try With AI

**Acceptance Criteria**:
- [ ] sse-starlette dependency added
- [ ] Async generator pattern clear
- [ ] Browser testing explained
- [ ] Connection handling discussed

---

### T08: Implement Lesson 7 — Agent Integration

**Path**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/07-agent-integration.md`

**Layer**: L2 (Full AI Collaboration)

**Content Sections**:
1. Agent as Service
2. OpenAI Client Setup
3. Task Context in Prompts
4. Non-Streaming Agent Endpoint
5. Streaming Agent Responses
6. Error Handling in Agent Calls
7. Hands-On: Task AI Assistant
8. AI Collaboration: Prompt Design
9. Try With AI

**Acceptance Criteria**:
- [ ] OpenAI integration complete
- [ ] Non-streaming and streaming versions
- [ ] Task context passed to agent
- [ ] Error handling for API failures
- [ ] AI collaboration for prompt design

---

### T09: Implement Lesson 8 — Capstone

**Path**: `apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/08-capstone-complete-task-api.md`

**Layer**: L4 (Spec-Driven)

**Content Sections**:
1. Capstone Overview
2. Specification Review
3. Project Structure
4. Implementation Checklist
5. Documentation Requirements
6. Testing Checklist
7. Submission Criteria

**Acceptance Criteria**:
- [ ] Full specification provided
- [ ] Clear implementation steps
- [ ] Testing criteria defined
- [ ] Documentation requirements clear

---

### T10: Validate All Lessons

**Validation Checklist**:

#### Content Validation
- [ ] All 8 lessons created
- [ ] YAML frontmatter complete
- [ ] Learning objectives measurable
- [ ] Code examples run correctly
- [ ] Domain consistent (Task Management)

#### Pedagogical Validation
- [ ] Manual-first progression (L1→L2→L4)
- [ ] Agent integration only in L7-8
- [ ] No framework labels in content
- [ ] Try With AI sections follow correct pattern
- [ ] Cognitive load within B1 limits

#### Technical Validation
- [ ] All imports correct
- [ ] Dependencies listed
- [ ] Code blocks syntax-highlighted
- [ ] No broken links

---

## Execution Notes

### Parallel Execution Opportunities
- T01 must complete first
- T02-T05 are sequential (each builds on previous)
- T06-T08 are sequential (each builds on previous)
- T09 depends on all previous lessons
- T10 runs after T09

### Quality Gates
Before moving to next lesson:
1. Run code examples locally
2. Verify Swagger UI works
3. Check YAML frontmatter complete
4. Confirm no framework labels

---

*Tasks generated by sp.orchestrator following SDD-RI workflow*
*Plan reference: specs/040-chapter-40-fastapi-for-agents/plan.md*
