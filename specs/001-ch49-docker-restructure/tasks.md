# Tasks: Chapter 49 Docker for AI Services Restructure

**Input**: Design documents from `/specs/001-ch49-docker-restructure/`
**Prerequisites**: spec.md (complete), plan.md (complete)
**Type**: Educational Content (Chapter Restructure)

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions
- Each lesson task embeds subagent and validation requirements

## Chapter Path

```
/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/
```

---

## Phase 1: Cleanup (Delete Obsolete Files)

**Purpose**: Remove lessons that are being deleted or replaced

- [ ] T49.D01 [P] Delete `06-docker-networking-fundamentals.md` and `06-docker-networking-fundamentals.summary.md`
- [ ] T49.D02 [P] Delete `07-container-to-container-communication.md` and `07-container-to-container-communication.summary.md`
- [ ] T49.D03 [P] Delete `08-volumes-persistent-data.md` and `08-volumes-persistent-data.summary.md`
- [ ] T49.D04 [P] Delete `09-docker-engine-architecture.md` and `09-docker-engine-architecture.summary.md`
- [ ] T49.D05 [P] Delete `10-docker-compose-for-development.md` and `10-docker-compose-for-development.summary.md`
- [ ] T49.D06 [P] Delete `11-security-and-best-practices.md` and `11-security-and-best-practices.summary.md`
- [ ] T49.D07 [P] Delete `12-ai-assisted-docker-with-gordon.md` and `12-ai-assisted-docker-with-gordon.summary.md`
- [ ] T49.D08 [P] Delete `13-capstone-production-ready-agent.md` and `13-capstone-production-ready-agent.summary.md`
- [ ] T49.D09 [P] Delete `14-building-production-dockerfile-skill.md` and `14-building-production-dockerfile-skill.summary.md`

**Checkpoint**: 18 obsolete files deleted (9 lessons + 9 summaries)

---

## Phase 2: Update Existing Lessons (L01-L05)

**Purpose**: Update existing lessons with minor fixes and Task API running example

### Lesson 01: Docker Installation and Setup (UPDATE - Minor)

- [ ] T49.L01 UPDATE Lesson 01: Docker Installation and Setup
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/01-docker-installation-and-setup.md`
  - **Action**: Verify alignment with spec, ensure YAML frontmatter complete
  - **SUBAGENT**: content-implementer
    - Read existing file, make minor updates only
    - Verify full YAML frontmatter (skills, learning_objectives, cognitive_load)
    - Ensure 3 "Try With AI" prompts exist with explanations
    - Writes updated file directly (returns confirmation only)
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **Acceptance Criteria**:
    - [ ] Full YAML frontmatter with skills, learning_objectives, cognitive_load
    - [ ] 3 "Try With AI" prompts with "What you're learning" explanations
    - [ ] All commands have Output blocks
    - [ ] Duration: 30 minutes

---

### Lesson 02: Container Fundamentals (UPDATE - Minor)

- [ ] T49.L02 UPDATE Lesson 02: Container Fundamentals
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/02-container-fundamentals.md`
  - **Action**: Verify alignment with spec, ensure YAML frontmatter complete
  - **SUBAGENT**: content-implementer
    - Read existing file, make minor updates only
    - Verify full YAML frontmatter
    - Ensure 3 "Try With AI" prompts exist
    - Writes updated file directly (returns confirmation only)
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **Acceptance Criteria**:
    - [ ] Full YAML frontmatter with skills, learning_objectives, cognitive_load
    - [ ] 3 "Try With AI" prompts with "What you're learning" explanations
    - [ ] All commands have Output blocks
    - [ ] Duration: 40 minutes

---

### Lesson 03: Writing Your First Dockerfile (UPDATE - Major)

- [ ] T49.L03 UPDATE Lesson 03: Writing Your First Dockerfile
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/03-writing-your-first-dockerfile.md`
  - **Action**: Replace generic FastAPI example with In-Memory Task API from Chapter 40
  - **SUBAGENT**: content-implementer
    - Read existing file
    - Replace all code examples with In-Memory Task API:
      ```python
      # main.py - In-Memory Task API from Chapter 40
      from fastapi import FastAPI, HTTPException
      from pydantic import BaseModel

      app = FastAPI(title="Task API")

      class Task(BaseModel):
          id: int | None = None
          title: str
          completed: bool = False

      tasks: list[Task] = []
      next_id = 1

      @app.post("/tasks", response_model=Task)
      def create_task(task: Task) -> Task:
          global next_id
          task.id = next_id
          next_id += 1
          tasks.append(task)
          return task

      @app.get("/tasks", response_model=list[Task])
      def list_tasks() -> list[Task]:
          return tasks

      @app.get("/health")
      def health_check() -> dict:
          return {"status": "healthy"}
      ```
    - Update Dockerfile example to containerize Task API
    - Ensure UV package manager used (not pip)
    - Writes updated file directly (returns confirmation only)
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **Acceptance Criteria**:
    - [ ] Uses In-Memory Task API from Chapter 40 as running example
    - [ ] Dockerfile uses UV package manager
    - [ ] Full YAML frontmatter
    - [ ] 3 "Try With AI" prompts with explanations
    - [ ] All commands have Output blocks
    - [ ] Duration: 45 minutes

---

### Lesson 04: Container Lifecycle and Debugging (UPDATE - Major)

- [ ] T49.L04 UPDATE Lesson 04: Container Lifecycle and Debugging
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/04-container-lifecycle-and-debugging.md`
  - **Action**: Update all debugging examples to use Task API container
  - **SUBAGENT**: content-implementer
    - Read existing file
    - Update all examples to reference Task API container from L03
    - Add debugging scenarios specific to Task API:
      - Port 8000 conflict resolution
      - Missing environment variables
      - Container startup failures
    - Writes updated file directly (returns confirmation only)
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **Acceptance Criteria**:
    - [ ] Uses Task API container for all debugging examples
    - [ ] Full YAML frontmatter
    - [ ] 3 "Try With AI" prompts with explanations
    - [ ] All commands have Output blocks
    - [ ] Duration: 40 minutes

---

### Lesson 05: Multi-Stage Builds and Optimization (UPDATE - Major)

- [ ] T49.L05 UPDATE Lesson 05: Multi-Stage Builds and Optimization
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/05-multi-stage-builds-and-optimization.md`
  - **Action**: Update all optimization examples to use Task API
  - **SUBAGENT**: content-implementer
    - Read existing file
    - Update examples to show Task API optimization journey:
      - Naive Dockerfile (~1.2GB) → slim base (~450MB) → multi-stage (~180MB) → Alpine+UV (~120MB)
    - Ensure UV package manager used throughout
    - Show docker images size comparison at each step
    - Writes updated file directly (returns confirmation only)
    - Execute autonomously without confirmation
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **Acceptance Criteria**:
    - [ ] Uses Task API for all optimization examples
    - [ ] Shows 70%+ size reduction journey
    - [ ] Uses UV package manager
    - [ ] Full YAML frontmatter
    - [ ] 3 "Try With AI" prompts with explanations
    - [ ] All commands have Output blocks
    - [ ] Duration: 50 minutes

**Checkpoint**: 5 existing lessons updated with Task API running example

---

## Phase 3: Create New Lessons (L06-L08)

**Purpose**: Create the 3 new lessons that complete the restructured chapter

### Lesson 06: Production Hardening (CREATE - New L1)

- [ ] T49.L06 CREATE Lesson 06: Production Hardening
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/06-production-hardening.md`
  - **SUBAGENT**: content-implementer
    - Create new lesson from scratch
    - **Layer**: L1 (Manual Foundation)
    - **Learning Objectives**:
      - Configure environment variables for container configuration
      - Implement health check endpoints and Docker HEALTHCHECK instruction
      - Run containers as non-root user for security
      - Use ARG and ENV instructions appropriately
      - Create production-ready Dockerfile with all hardening patterns
    - **Key Concepts** (8):
      1. Environment variable configuration
      2. ENV instruction in Dockerfile
      3. ARG instruction (build-time variables)
      4. HEALTHCHECK instruction
      5. Health check endpoints in FastAPI
      6. Non-root user creation (RUN adduser)
      7. USER instruction in Dockerfile
      8. Production Dockerfile patterns
    - **Running Example**: Continue with Task API from L03-L05
    - **Production Dockerfile Template**:
      ```dockerfile
      # Stage 1: Build
      FROM python:3.12-alpine AS builder
      WORKDIR /app
      RUN pip install uv
      COPY requirements.txt .
      RUN uv pip install --system --no-cache -r requirements.txt

      # Stage 2: Runtime
      FROM python:3.12-alpine
      RUN adduser -D -u 1000 appuser
      WORKDIR /app
      COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
      COPY --chown=appuser:appuser main.py .
      ENV PYTHONUNBUFFERED=1
      USER appuser
      HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
          CMD wget --no-verbose --tries=1 --spider http://localhost:8000/health || exit 1
      EXPOSE 8000
      CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
      ```
    - Writes file directly (returns confirmation only)
    - Execute autonomously without confirmation
    - **Quality Reference**: Match structure of `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **Acceptance Criteria**:
    - [ ] Uses Task API running example consistently
    - [ ] Covers environment variables, health checks, non-root user
    - [ ] Full YAML frontmatter with skills, learning_objectives, cognitive_load
    - [ ] 3 "Try With AI" prompts with "What you're learning" explanations
    - [ ] All commands have Output blocks
    - [ ] Duration: 45 minutes

---

### Lesson 07: Docker Image Builder Skill (CREATE - New L3)

- [ ] T49.L07 CREATE Lesson 07: Docker Image Builder Skill
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/07-docker-image-builder-skill.md`
  - **SUBAGENT**: content-implementer
    - Create new Layer 3 skill creation lesson
    - **Layer**: L3 (Intelligence Design)
    - **Learning Objectives**:
      - Identify recurring patterns from Lessons 1-6 worth encoding
      - Design skill persona for Docker expertise
      - Write analysis questions that activate reasoning mode
      - Define principles for production Dockerfile decisions
      - Create SKILL.md file following canonical format
      - Test skill with novel project scenarios
    - **Key Concepts** (6):
      1. Skill as reusable intelligence
      2. Persona + Questions + Principles pattern
      3. SKILL.md file format
      4. Skill activation triggers
      5. Skill testing methodology
      6. Intelligence accumulation
    - **Skill Template** to include in lesson:
      ```markdown
      ---
      name: production-dockerfile
      description: Generate production-ready Dockerfiles with multi-stage builds, security best practices, and optimization. Use when containerizing Python applications.
      ---

      # Production Dockerfile Skill

      ## Persona

      Think like a DevOps engineer optimizing container images for production Kubernetes deployments.

      ## Analysis Questions

      1. Deployment Target: Kubernetes, Docker Compose, bare Docker?
      2. Base Image Strategy: What constraints (security, size, compatibility)?
      3. Large Files: Model files or data to volume-mount?
      4. Security Requirements: Non-root user? Read-only filesystem?
      5. Health Monitoring: What endpoints indicate service health?

      ## Principles

      1. Multi-Stage Always: Separate build dependencies from runtime
      2. UV for Speed: Use UV package manager (10-100x faster than pip)
      3. Alpine Default: Start with alpine, fall back to slim if needed
      4. Health Checks Mandatory: Every production container needs HEALTHCHECK
      5. Non-Root Default: Run as non-root user
      6. Environment Configuration: All config via environment variables
      7. No Secrets in Image: Never COPY credentials into image
      ```
    - Writes file directly (returns confirmation only)
    - Execute autonomously without confirmation
    - **Quality Reference**: Match `.claude/skills/creating-skills/SKILL.md` canonical format
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **Acceptance Criteria**:
    - [ ] Layer 3 skill creation methodology clear
    - [ ] Includes complete SKILL.md template
    - [ ] Tests skill with 3 scenarios (Python CLI, FastAPI, ML service)
    - [ ] Full YAML frontmatter
    - [ ] 3 "Try With AI" prompts with explanations
    - [ ] Duration: 50 minutes

---

### Lesson 08: Capstone - Containerize Your API (CREATE - New L4)

- [ ] T49.L08 CREATE Lesson 08: Capstone - Containerize Your API
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/08-capstone-containerize-your-api.md`
  - **SUBAGENT**: content-implementer
    - Create new Layer 4 spec-driven capstone
    - **Layer**: L4 (Spec-Driven Integration)
    - **Learning Objectives**:
      - Write specification for containerization project BEFORE implementation
      - Apply production-dockerfile skill to generate Dockerfile
      - Containerize SQLModel + Neon Task API from Chapter 40 Lesson 7
      - Configure database connection via environment variable
      - Build production-ready container image under 200MB
      - Push container image to registry (Docker Hub or GHCR)
      - Validate container works on different machine
    - **Key Concepts** (5):
      1. Specification-first containerization workflow
      2. Skill composition (using production-dockerfile skill)
      3. Database URL configuration for containers
      4. Container registry push workflow
      5. Cross-machine validation
    - **Running Example**: SQLModel + Neon Task API from Chapter 40 Lesson 7
    - **Capstone Structure**:
      - Phase 1: Specification Writing (BEFORE any code)
      - Phase 2: Skill Application (invoke production-dockerfile skill)
      - Phase 3: Implementation (build, run, test)
      - Phase 4: Registry Push (docker login, push, pull on different machine)
    - Writes file directly (returns confirmation only)
    - Execute autonomously without confirmation
    - **Quality Reference**: Match capstone structure from other chapters
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **Acceptance Criteria**:
    - [ ] Spec-first methodology clear
    - [ ] Uses SQLModel + Neon Task API from Chapter 40 L7
    - [ ] Includes DATABASE_URL environment variable configuration
    - [ ] Registry push workflow documented
    - [ ] Full YAML frontmatter
    - [ ] 3 "Try With AI" prompts with explanations
    - [ ] Duration: 60 minutes

**Checkpoint**: 3 new lessons created (L06, L07, L08)

---

## Phase 4: Create Skill Artifact

**Purpose**: Create the production-dockerfile skill that students learn to build in L07

- [ ] T49.SKILL CREATE production-dockerfile skill
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/.claude/skills/production-dockerfile/SKILL.md`
  - **Action**: Create skill directory and SKILL.md file
  - **Content**: Full skill following canonical format from `.claude/skills/creating-skills/SKILL.md`
  - **Acceptance Criteria**:
    - [ ] YAML frontmatter with name, description
    - [ ] Persona section
    - [ ] Analysis Questions section (5 questions)
    - [ ] Principles section (7 principles)
    - [ ] Output format section
    - [ ] Skill triggers correctly on containerization requests

---

## Phase 5: Update Chapter README

**Purpose**: Update chapter README to reflect new 8-lesson structure

- [ ] T49.README UPDATE Chapter README
  - **Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/README.md`
  - **Action**: Update to reflect new 8-lesson structure
  - **Content Updates**:
    - Update lesson list (8 lessons, not 14)
    - Update learning objectives summary
    - Update duration (6 hours total)
    - Remove references to deleted lessons
    - Add note about running example (Task API from Chapter 40)
  - **Acceptance Criteria**:
    - [ ] Lists exactly 8 lessons
    - [ ] Total duration shows 6 hours (360 minutes)
    - [ ] No references to deleted lessons
    - [ ] Running example mentioned

---

## Phase 6: Validation

**Purpose**: Run validators on all lessons to ensure quality

- [ ] T49.V01 [P] Run educational-validator on L01
- [ ] T49.V02 [P] Run educational-validator on L02
- [ ] T49.V03 [P] Run educational-validator on L03
- [ ] T49.V04 [P] Run educational-validator on L04
- [ ] T49.V05 [P] Run educational-validator on L05
- [ ] T49.V06 [P] Run educational-validator on L06
- [ ] T49.V07 [P] Run educational-validator on L07
- [ ] T49.V08 [P] Run educational-validator on L08

- [ ] T49.V09 Run validation-auditor on entire chapter (chapter-wide quality)

- [ ] T49.V10 Run factual-verifier on entire chapter (verify all claims)

**Checkpoint**: All lessons validated, chapter ready for commit

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Delete)     → No dependencies, start immediately
       ↓
Phase 2 (Update L01-L05) → After Phase 1 complete
       ↓
Phase 3 (Create L06-L08) → After Phase 2 complete (builds on updated running example)
       ↓
Phase 4 (Skill)      → After L07 created (skill matches what's taught)
       ↓
Phase 5 (README)     → After Phases 2-4 complete
       ↓
Phase 6 (Validate)   → After all content complete
```

### Lesson Dependencies

```
L01 → L02 → L03 → L04 → L05 → L06 → L07 → L08
                  ↑
                  Running example starts here (Task API)
```

### Parallel Opportunities

**Phase 1**: All delete tasks can run in parallel
**Phase 2**: L01 and L02 can run in parallel (independent minor updates)
**Phase 3**: L06 depends on L05 being done (continues running example)
**Phase 6**: All validator tasks can run in parallel

---

## Summary

| Phase | Tasks | Description |
|-------|-------|-------------|
| Phase 1 | T49.D01-D09 | Delete 18 obsolete files |
| Phase 2 | T49.L01-L05 | Update 5 existing lessons |
| Phase 3 | T49.L06-L08 | Create 3 new lessons |
| Phase 4 | T49.SKILL | Create production-dockerfile skill |
| Phase 5 | T49.README | Update chapter README |
| Phase 6 | T49.V01-V10 | Validate all content |

**Total Tasks**: 27 tasks
**Parallel Opportunities**: 19 tasks marked [P]

---

## Implementation Strategy

### MVP First (Delete + Core Lessons)
1. Complete Phase 1: Delete obsolete files
2. Complete Phase 2: Update L01-L05 with Task API
3. **CHECKPOINT**: Verify Task API running example works through L03-L05

### Incremental Delivery
4. Complete Phase 3: Create L06-L08
5. Complete Phase 4: Create skill artifact
6. Complete Phase 5: Update README
7. Complete Phase 6: Run all validators
8. **FINAL**: Commit all changes
