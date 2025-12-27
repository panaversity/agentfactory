# Feature Specification: Chapter 49 Docker for AI Services Restructure

**Feature Branch**: `001-ch49-docker-restructure`
**Created**: 2025-12-27
**Status**: Draft
**Type**: Educational Content (Chapter Restructure)
**Proficiency Level**: B1 (Intermediate)

## Overview

Restructure Chapter 49 from 14 lessons to 8 focused lessons. The chapter teaches Docker containerization for AI services, using the In-Memory Task API from Chapter 40 as the running example throughout.

### Assumed Knowledge

**What students know BEFORE this chapter**:
- FastAPI fundamentals (Chapter 40 Lessons 1-5): routing, endpoints, Pydantic models
- Python development with UV package manager (Part 5)
- Command line basics (Chapter 7)
- HTTP concepts (Part 6 MCP chapters)
- The In-Memory Task API code from Chapter 40

**What this chapter must explain from scratch**:
- Container concepts (images vs containers, layers)
- Dockerfile syntax and instructions
- Docker CLI commands
- Multi-stage build patterns
- Production hardening (env vars, health checks, non-root)
- Skill creation methodology (Layer 3)

### Layer Progression

| Lesson | Layer | Rationale |
|--------|-------|-----------|
| L01-L06 | L1 (Manual) | Build foundational Docker understanding before AI assistance |
| L07 | L3 (Intelligence) | Create reusable skill from accumulated knowledge |
| L08 | L4 (Spec-Driven) | Apply all skills in specification-first capstone |

## User Scenarios & Testing

### User Story 1 - Learn Docker Fundamentals (Priority: P1)

A student who has completed Part 6 (FastAPI) wants to containerize their API for deployment. They need to understand what containers are, how to write Dockerfiles, and how to run containers locally.

**Why this priority**: Core foundation - without this, no other Docker skills are possible.

**Independent Test**: Student can create a Dockerfile for a simple FastAPI app and run it locally, accessing the API at localhost:8000.

**Acceptance Scenarios**:

1. **Given** a student with Docker Desktop installed, **When** they complete Lessons 1-3, **Then** they can build and run the Task API container with `docker build` and `docker run`.
2. **Given** a running container, **When** the student makes HTTP requests to localhost:8000, **Then** the Task API responds correctly.

---

### User Story 2 - Debug and Optimize Containers (Priority: P2)

A student needs to troubleshoot container issues and optimize images for production (smaller size, faster builds).

**Why this priority**: Essential for real-world usage - containers often fail or are too large.

**Independent Test**: Student can reduce image size by 70%+ using multi-stage builds and diagnose common container failures.

**Acceptance Scenarios**:

1. **Given** a container that won't start, **When** the student uses `docker logs` and `docker exec`, **Then** they can identify and fix the issue.
2. **Given** a 1.2GB naive image, **When** the student applies multi-stage builds and slim base images, **Then** image size drops to under 200MB.

---

### User Story 3 - Production-Harden Containers (Priority: P3)

A student needs to prepare containers for production with proper configuration, health checks, and security.

**Why this priority**: Bridge between local development and production deployment.

**Independent Test**: Student can create a container with environment variable configuration, health checks, and non-root execution.

**Acceptance Scenarios**:

1. **Given** a Dockerfile without security features, **When** the student applies hardening patterns, **Then** the container runs as non-root with a working health check.
2. **Given** a containerized API, **When** deployed to any Docker host, **Then** it uses environment variables for configuration (not hardcoded values).

---

### User Story 4 - Create Reusable Docker Skill (Priority: P4)

A student wants to encode their Docker knowledge into a reusable AI skill for future containerization tasks.

**Why this priority**: Layer 3 learning objective - transform tacit knowledge into organizational capability.

**Independent Test**: Student creates a SKILL.md file that produces consistent, high-quality Dockerfiles when invoked.

**Acceptance Scenarios**:

1. **Given** accumulated Docker knowledge, **When** the student creates a production-dockerfile skill, **Then** it generates correct Dockerfiles for Python, Node.js, and Go projects.
2. **Given** the skill file, **When** invoked via Claude, **Then** it asks the right questions and applies all security principles.

---

### User Story 5 - Complete Capstone Containerization (Priority: P5)

A student containerizes a real database-backed application (SQLModel + Neon from Chapter 40) using specification-driven development.

**Why this priority**: Synthesis of all learning - demonstrates mastery.

**Independent Test**: Student produces a production-ready container image pushed to a registry.

**Acceptance Scenarios**:

1. **Given** the SQLModel Task API from Chapter 40 Lesson 7, **When** the student completes the capstone, **Then** they have a container image under 200MB with health checks and non-root user.
2. **Given** the container image, **When** pulled on a different machine, **Then** it runs correctly with database URL provided via environment variable.

---

### Edge Cases

- What happens when Docker Desktop is not installed correctly? (L01 covers installation verification)
- How does the student handle port conflicts? (L04 covers debugging)
- What if the base image is not available? (L02 covers Docker Hub and image pulling)
- How to handle secrets in containers? (L06 covers environment variables, explicitly excludes secrets management as out of scope)

## Requirements

### Functional Requirements

#### Chapter Structure

- **FR-001**: Chapter MUST contain exactly 8 lessons (reduced from 14)
- **FR-002**: Lessons 1-6 MUST be Layer 1 (Manual Foundation)
- **FR-003**: Lesson 7 MUST be Layer 3 (Intelligence/Skill Creation)
- **FR-004**: Lesson 8 MUST be Layer 4 (Spec-Driven Capstone)
- **FR-005**: Chapter MUST NOT include Docker Compose content (deferred to Kubernetes chapter)
- **FR-006**: Chapter MUST NOT include Docker networking deep-dive (deferred to Kubernetes chapter)
- **FR-007**: Chapter MUST NOT include Gordon/Docker AI content (availability risk)

#### Running Example

- **FR-008**: Lessons 3-7 MUST use the In-Memory Task API from Chapter 40 Lessons 1-5 as the running example
- **FR-009**: Lesson 8 (Capstone) MUST containerize the SQLModel + Neon Task API from Chapter 40 Lesson 7
- **FR-010**: Code examples MUST be consistent with Chapter 40 patterns (FastAPI, Pydantic, UV)

#### Content Requirements

- **FR-011**: Each lesson MUST include full YAML frontmatter with skills, learning_objectives, cognitive_load
- **FR-012**: Each lesson MUST include 3 "Try With AI" prompts with learning explanations
- **FR-013**: Each lesson MUST include Output blocks for all executable commands
- **FR-014**: Lesson 7 MUST produce a working SKILL.md file at `.claude/skills/production-dockerfile/`
- **FR-015**: All Dockerfiles MUST use UV package manager (not pip) for Python dependencies

#### File Operations

- **FR-016**: System MUST delete 9 obsolete lesson files (06-14) plus their .summary.md files
- **FR-017**: System MUST update 5 existing lesson files (01-05) to use Task API example
- **FR-018**: System MUST create 3 new lesson files (06-08)
- **FR-019**: System MUST update README.md to reflect new 8-lesson structure

### Lesson Breakdown

| # | File | Title | Duration | Status |
|---|------|-------|----------|--------|
| 01 | 01-docker-installation-and-setup.md | Docker Installation and Setup | 30 min | UPDATE (minor) |
| 02 | 02-container-fundamentals.md | Container Fundamentals | 40 min | UPDATE (minor) |
| 03 | 03-writing-your-first-dockerfile.md | Writing Your First Dockerfile | 45 min | UPDATE (use Task API) |
| 04 | 04-container-lifecycle-and-debugging.md | Container Lifecycle and Debugging | 40 min | UPDATE (Task API examples) |
| 05 | 05-multi-stage-builds-and-optimization.md | Multi-Stage Builds and Optimization | 50 min | UPDATE (Task API) |
| 06 | 06-production-hardening.md | Production Hardening | 45 min | CREATE (new) |
| 07 | 07-docker-image-builder-skill.md | Docker Image Builder Skill | 50 min | CREATE (new - L3) |
| 08 | 08-capstone-containerize-your-api.md | Capstone: Containerize Your API | 60 min | CREATE (new - L4) |

### Files to Delete

| File | Reason |
|------|--------|
| 06-docker-networking-fundamentals.md | Deferred to Kubernetes chapter |
| 07-container-to-container-communication.md | Deferred to Kubernetes chapter |
| 08-volumes-persistent-data.md | Not needed for stateless API containers |
| 09-docker-engine-architecture.md | Pure theory, no practical value |
| 10-docker-compose-for-development.md | Deferred to Kubernetes chapter |
| 11-security-and-best-practices.md | Merged into L06 (Production Hardening) |
| 12-ai-assisted-docker-with-gordon.md | Gordon availability risk |
| 13-capstone-production-ready-agent.md | Replaced by simpler L08 |
| 14-building-production-dockerfile-skill.md | Content moves to L07 |

### Key Entities

- **Task API (In-Memory)**: FastAPI application with CRUD endpoints for tasks, storing data in Python list (from Chapter 40 Lessons 1-5)
- **Task API (SQLModel)**: Same API with PostgreSQL persistence via Neon (from Chapter 40 Lesson 7)
- **Docker Image**: Immutable artifact containing application and dependencies
- **Container**: Running instance of an image
- **Dockerfile**: Build instructions for creating images
- **SKILL.md**: Reusable AI skill file with Persona + Questions + Principles pattern

## Success Criteria

### Measurable Outcomes

- **SC-001**: Chapter contains exactly 8 lessons (down from 14)
- **SC-002**: Total chapter duration is 6 hours (360 minutes) or less
- **SC-003**: Students can containerize the Task API within 45 minutes following Lesson 3
- **SC-004**: Multi-stage builds reduce image size by at least 70% (from ~1.2GB to under 200MB)
- **SC-005**: All 8 lessons pass educational-validator checks
- **SC-006**: All factual claims verified via factual-verifier
- **SC-007**: Layer progression is validated: L1 (6 lessons) → L3 (1 lesson) → L4 (1 lesson)
- **SC-008**: Skill created in L07 successfully generates Dockerfiles for 3+ project types when tested

### Quality Gates

- **QG-001**: Each lesson has complete YAML frontmatter (skills, learning_objectives, cognitive_load, differentiation)
- **QG-002**: Each lesson has 3 "Try With AI" prompts with "What you're learning" explanations
- **QG-003**: All code examples have Output blocks showing expected results
- **QG-004**: No lessons remain after L08 - chapter ends cleanly

## Assumptions

1. Students have Docker Desktop installed or can install it (Lesson 1 covers this)
2. Students have completed Chapter 40 Lessons 1-5 (In-Memory Task API exists)
3. Students have completed Chapter 40 Lesson 7 for capstone (SQLModel version exists)
4. UV package manager is the standard for Python projects in this curriculum
5. No GPU requirements for this chapter (cloud GPU addressed in later chapters)
6. Registry push uses Docker Hub or GitHub Container Registry (both free tiers available)

## Dependencies

- **Chapter 40**: FastAPI for Agents (provides Task API code)
- **Part 5**: Python Fundamentals (provides language foundation)
- **Part 6**: AI-Native Software Development (provides agent concepts)

## Out of Scope

- Docker Compose (deferred to Kubernetes chapter)
- Docker networking deep-dive (deferred to Kubernetes chapter)
- Docker Swarm or orchestration (covered in Kubernetes)
- GPU-accelerated containers
- Secrets management (complex topic for security chapter)
- Private registry authentication
- Multi-architecture builds (ARM64 + x86)
- Gordon/Docker Desktop AI features (availability risk)
