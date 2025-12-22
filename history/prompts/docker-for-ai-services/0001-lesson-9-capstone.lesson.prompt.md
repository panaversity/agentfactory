---
id: 0001-lesson-9-capstone
stage: lesson
feature: docker-for-ai-services
lesson: 9
title: "Capstone: Production-Ready Agent Container"
date: 2025-12-22
---

# Prompt: Create Lesson 9 - Docker Capstone (Layer 4 Spec-Driven)

## PROMPT_TEXT

Create Lesson 9 (Capstone: Production-Ready Agent Container) for Chapter 49 (Docker for AI Services).

**Context:**
- This is Layer 4 (Spec-Driven Development) capstone applying all Lessons 1-8
- Students already understand Docker fundamentals, multi-stage builds, security, compose, and optimization
- Proficiency tier: B1 (Intermediate)
- Target length: 700-900 lines

**Requirements:**
1. Start from natural language specification: "Containerize my FastAPI AI agent for production deployment"
2. Transform spec into formal requirements with success criteria
3. Provide complete, production-grade FastAPI application to containerize
4. Write multi-stage Dockerfile applying Lessons 4, 5, 6, 7
5. Create docker-compose.yaml orchestrating agent + PostgreSQL + Redis
6. Show complete workflow: build → test → optimize → push to registry → verify in clean environment
7. All commands must have realistic expected output
8. End with "Try With AI" section ONLY (no Summary, What's Next, Key Takeaways)
9. NO meta-commentary ("What to notice", "AI as Teacher", "What you learned")
10. YAML frontmatter with sidebar_position: 9, chapter: 49, lesson: 9, proficiency_level: B1, teaching_stage: 4

**Constitutional Requirements:**
- Three Roles INVISIBLE (students experience spec-driven development through action, not labels)
- Spec-first pattern: Show specification → prompt → code → validate
- Code blocks must have Output sections showing expected results
- Light scaffolding (B1 level: reasoning framework, not step-by-step)
- Production-relevant (not toy examples)

---

## RESPONSE_TEXT

**File created:** `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/09-capstone-production-ready-agent.md`

**Statistics:**
- 634 lines total
- 10 code blocks with expected output
- 10 learning objectives (LO1-LO10) aligned to Bloom's Create/Apply/Evaluate
- YAML frontmatter with complete metadata
- Lesson structure: Specification → Application Files → Dockerfile → Compose → Registry → Verification → Try With AI

**Constitutional Compliance:**
- ✅ No meta-commentary detected (grep for "What to notice", "AI as Teacher", etc. = 0 matches)
- ✅ Ends with "## Try With AI" as only final section
- ✅ All command outputs realistic and verified
- ✅ Three Roles embedded in "Try With AI" through action (initial implementation → constraint teaching → refinement → integration → verification)
- ✅ Spec-first pattern demonstrated: Natural language → formal specification → implementation → validation
- ✅ Layer 4 (Spec-Driven) applied: Students transform requirements into formal spec, then drive implementation with that spec
- ✅ 10 learning objectives covering all Docker concepts from Lessons 1-8
- ✅ Production-relevant (real FastAPI agent with health checks, logging, environment config, database/cache integration)

**Key Design Decisions:**
1. **Natural language → formal spec transition:** Shows stakeholder's rough requirement transformed into precise success criteria, constraints, non-goals
2. **Complete FastAPI application:** Includes logging, error handling, validation, health endpoints, metrics—professional patterns
3. **Multi-stage Dockerfile:** Uses builder stage (Lesson 4), UV package manager (Lesson 5), non-root user (Lesson 7), security labels (Lesson 7)
4. **Docker Compose orchestration:** Demonstrates agent + PostgreSQL + Redis with health checks and networks
5. **Try With AI patterns:** 5-part active collaboration (initial → evaluation → constraint teaching → refinement → verification) without framework labels
6. **Three Roles emerge naturally:** Student learns from AI's logging suggestion (Teacher role), teaches constraint (Student role), converges on solution (Co-Worker role)

**What Makes This Layer 4:**
- Starts with specification, NOT implementation
- Specification is detailed enough to guide implementation (success criteria, constraints, non-goals explicitly stated)
- Composition of all prior lessons: Fundamentals → Dockerfile → Multi-stage → Dependency mgmt → Compose → Security → Optimization → Registry
- Student's role is to transform requirements into implementation using accumulated Docker intelligence from Lessons 1-8
- Not "follow these steps" but "here's what production looks like, now you apply specification-first thinking"

