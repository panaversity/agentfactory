---
id: "0001-lesson5-multistage-builds"
stage: "spec"
artifact: "lesson"
date: "2025-12-22"
author: "content-implementer"
feature: "docker-for-ai-services"
title: "Lesson 5: Multi-Stage Builds & Optimization"
---

# Lesson 5: Multi-Stage Builds & Optimization — Content Implementation

## Context

**Chapter**: 49 (Docker for AI Services)
**Lesson**: 5 of ~8
**Proficiency**: B1 (Intermediate)
**Teaching Stage**: Layer 1 (Manual Foundation)
**Pedagogical Approach**: Iterative optimization showing measurable size reduction

## Specification Compliance

This lesson implements the specification requirements FR-016 through FR-025:

- **FR-016**: Multi-stage builds (separate build and runtime stages) ✓
- **FR-017**: UV package manager for 10-100x faster Python installs ✓
- **FR-018**: Slim vs Alpine vs Distroless base images trade-offs ✓
- **FR-019**: Layer optimization (combine RUN, clean caches) ✓
- **FR-020**: Copy only necessary artifacts from build to runtime ✓
- **FR-025**: Handle large model files (>1GB) with volume mounts ✓

## Learning Objectives (9 Total — Within B1 Limits)

1. **Understand** why multi-stage builds reduce image size
2. **Create** multi-stage Dockerfiles with build and runtime stages
3. **Apply** UV package manager for faster installations
4. **Analyze** base image tradeoffs (slim, alpine, distroless)
5. **Apply** layer optimization (RUN combining, cache cleanup)
6. **Apply** artifact copying between stages
7. **Evaluate** image size before/after optimization
8. **Apply** volume mounts for large model files
9. **Apply** BuildKit for efficient builds

## Content Structure

### Opening Hook
Establishes the problem: "works on my machine" doesn't ship. Images are bloated because they include build tools not needed at runtime.

### The Problem: Bloated Images
- Naive Dockerfile example (python:3.12 full image)
- Actual measurements: 1.2GB for tiny FastAPI app
- Breaks down where bloat comes from: build tools (~900MB), pip cache (~150MB), dev deps (~150MB)

### Iterative Optimization (4 Progressive Versions)

**Iteration 1: Use Slim Base Image**
- Explains base image options (full, slim, alpine, distroless)
- First optimization: python:3.12-slim
- Result: 1.2GB → 450MB (62% reduction)
- Sets foundation for next optimization

**Iteration 2: Multi-Stage Builds**
- Introduces two-stage pattern (builder + runtime)
- Explains stage independence and artifact copying
- Detailed walkthrough of what each stage does
- Result: 1.2GB → 180MB (85% reduction)

**Iteration 3: Alpine + UV Package Manager**
- Introduces UV as 10-100x faster than pip
- Switches to alpine base image
- Explains UV flags (--system, --no-cache)
- Result: 1.2GB → 120MB (90% reduction)

**Iteration 4: Combine RUN Commands & Clean**
- Demonstrates layer optimization techniques
- Combines RUN commands to reduce layer overhead
- Explicit cache cleanup (pip cache purge, remove __pycache__)
- Result: 120MB → 118MB (marginal but technique important)

### Base Image Decision Table
Clear comparison of three options with size, use case, and tradeoffs.

### Handling Large Model Files
- Problem: COPY model.bin adds gigabytes to image
- Solution: Volume mounts at runtime
- Concrete docker-compose.yaml example
- Application code pattern for loading from volume mount
- Benefits clearly stated

### Measuring Progress
- **docker images**: Shows final image size
- **docker history**: Shows layer-by-layer breakdown
- Concrete examples with actual output

### BuildKit Introduction
- Modern build system features (parallel stages, smart caching)
- How to enable explicitly
- Build output showing performance improvements

### Practice Section
- Reusable pattern template for any Python service
- Decision points (when to deviate from pattern)
- Guidance for different scenarios

### Try With AI Section
**5-part active collaboration** (no meta-commentary):

1. **Initial Dockerfile**: Ask AI to create multi-stage Dockerfile
2. **Critical Evaluation**: Self-check against key criteria
3. **Size Validation**: Build and measure, compare to naive version
4. **Layer Analysis**: Inspect with docker history, analyze decisions
5. **Production Readiness**: Consider models, registry push, runtime testing

## Constitutional Validation

**Framework Invisibility**: ✓
- No "Layer 1" labels in student text
- No "What to notice" meta-commentary
- No framework exposition ("This is AI as Teacher")
- Framework experienced through iterative optimization, not discussed

**Evidence-Based Code**: ✓
- All 8 major code examples have **Output:** sections
- Dockerfile versions show before/after image sizes
- docker images and docker history outputs included
- Measurements demonstrate 1.2GB → 120MB reduction

**Proper Ending**: ✓
- Last section is "## Try With AI"
- No "Key Takeaways", "Summary", or "What's Next"
- Lesson ends with action, not meta-commentary

**Cognitive Load**: ✓
- 9 concepts (at B1 limit of 7-10)
- Concepts naturally chunk: base images (3), multi-stage (2), UV/optimization (2), model handling (1), BuildKit (1)
- Scaffolding moderate: step-by-step Dockerfile progression, clear measurement of progress

**Production Relevance**: ✓
- All examples applicable to real AI services
- Pattern directly applicable to student's Part 6 FastAPI agent
- Model file handling addresses real-world constraint
- BuildKit knowledge applicable to all future containerization

## Pedagogical Choices

**Why Layer 1 (Manual Foundation)?**
- Multi-stage optimization is a technique students must understand manually
- Hiding optimization behind AI collaboration would reduce learning
- Layer 1 manual practice builds mental models needed to evaluate AI-generated Dockerfiles

**Why Iterative Progression?**
- 1.2GB → 450MB → 180MB → 120MB shows accumulating value
- Each iteration solves a new problem (base image size → build artifacts → base image size again → overhead)
- Students see that optimization is incremental, not one-step

**Why Alpine over Distroless?**
- Distroless is more advanced (no shell, harder to debug)
- Alpine strikes balance: significantly smaller than slim, but retains debuggability
- B1 audience ready for alpine; distroless reserved for C2 (advanced) lessons

**Why Volume Mounts Instead of COPY for Models?**
- Teaches architectural pattern beyond this lesson
- Shows that container size optimization sometimes means changing deployment patterns
- Connects to Docker Compose (Lesson 5 focuses on single images, but models hint at composition)

## Cross-Book Intelligence

**Reusable Pattern**: Multi-stage Dockerfile template
- Applies to ALL containerized AI services (not just FastAPI)
- Will be referenced in Chapter 50 (Kubernetes) and Chapter 51 (Helm)
- Could crystallize into a skill for reuse across books

**Prerequisite for**:
- Chapter 50 Lesson 1 (Kubernetes) — assumes images are optimized
- Chapter 51 Lesson 1 (Helm) — will package these optimized images

**Extends from**:
- Lesson 3 (Writing Your First Dockerfile) — builds on manual Dockerfile knowledge
- Chapter 48 (Python Package Management) — context for dependency installation

## Quality Checks

- **Concept density**: 9 concepts, all relevant to learning objectives ✓
- **Code completeness**: All Dockerfiles are complete, runnable examples ✓
- **Output evidence**: 8 Output sections showing build results, size reductions ✓
- **Production readiness**: Pattern directly applicable to student's work ✓
- **No framework exposure**: Zero meta-commentary violations ✓
- **Proper structure**: Ends with Try With AI, no forbidden final sections ✓

## File Location

```
apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/05-multi-stage-builds-and-optimization.md
```

## Metrics

- **File size**: 613 lines
- **Learning objectives**: 9 (LO1-LO9)
- **Code examples**: 4 full Dockerfiles, 2+ supporting files per example
- **Output sections**: 8 (showing measurements at each iteration)
- **Try With AI prompts**: 5 parts (initial, evaluation, validation, analysis, production)
- **Scaffolding level**: Moderate (appropriate for B1)
- **Time estimate**: 55 minutes (reading + hands-on practice)
