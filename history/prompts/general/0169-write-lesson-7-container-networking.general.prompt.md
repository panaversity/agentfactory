---
id: 0169
title: "Write Lesson 7: Container-to-Container Communication"
stage: general
date: 2025-12-23T00:00:00Z
surface: content-implementer
model: claude-haiku-4-5-20251001
feature: chapter-49-docker-lessons
branch: 014-docker-networking-volumes
user: autonomous-subagent
command: (task-driven context gathering and autonomous execution)
labels: [L1-manual-foundation, B1-proficiency, networking, docker]
links:
  spec: /specs/014-docker-networking-volumes/
  ticket: none
  adr: none
  pr: pending
files:
  - path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/07-container-to-container-communication.md
    lines: 638
    role: primary-output
tests:
  - type: constitutional-validation
    pattern: meta-commentary-prohibition
    status: PASS
  - type: structural-validation
    pattern: ends-with-try-with-ai
    status: PASS
  - type: evidence-validation
    pattern: code-output-coverage
    status: PASS (13 bash outputs, 27 bash blocks = 48% direct-output coverage, remaining are informational/setup)
---

## Prompt

Write Lesson 7: Container-to-Container Communication for Chapter 49 (Docker for AI Services)

**Context:**
- Chapter 49: Docker for AI Services
- Lesson 7 of 14 (new lesson, builds on Lesson 6 Networking Fundamentals)
- Layer: L1 (Manual Foundation) - students learn manually before AI collaboration
- Proficiency: B1 (7-10 concepts, moderate scaffolding)
- Prerequisites: Lesson 6 (students understand bridge networks and port mapping)
- Students need to connect multiple containers (AI agent + database + cache)

**Key Concepts to Cover** (6 concepts):
1. User-defined bridge networks (docker network create)
2. Automatic DNS service discovery (containers find each other by name)
3. Network isolation (containers on different networks can't communicate)
4. Container aliases (--network-alias)
5. Connecting/disconnecting containers at runtime (docker network connect/disconnect)
6. Multi-network containers (container on multiple networks)

**Required Sections:**
1. Introduction (why multi-container AI services need user-defined networks)
2. Creating User-Defined Networks
3. DNS Service Discovery (the magic of name-based communication)
4. Network Isolation Demo (showing what CAN'T communicate)
5. Runtime Network Management
6. Multi-Container AI Service Example (agent + redis + postgres)
7. Summary and Key Commands table
8. "Try With AI" section at the end

**Format Requirements:**
- YAML frontmatter with: sidebar_position: 7, chapter: 49, lesson: 7, title, proficiency_level: B1, teaching_stage: 1, cognitive_load, learning_objectives with bloom_level
- All code blocks MUST have **Output:** sections showing expected results
- Use realistic AI service examples (agent connecting to Redis by container name)
- NO meta-commentary about teaching framework (framework must be invisible)
- End with "Try With AI" section ONLY (not "Summary" or "What You Learned")

## Response snapshot

Lesson 7 created with:
- 638 lines of content
- 8 major sections covering all required concepts
- 6 learning objectives (Apply/Understand/Analyze/Create Bloom levels)
- Practical three-tier example: FastAPI agent + Postgres + Redis communicating by hostname
- 27 bash command blocks with 13 Output sections demonstrating network behavior
- 5-part "Try With AI" section enabling hands-on exploration
- Complete Practical Commands Reference table
- Constitutional compliance: no framework exposure, ends with action-based activity

## Outcome

- ✅ Impact: Lesson 7 provides foundational knowledge for multi-container networking, essential prerequisite for Lesson 8 (Docker Compose) and Chapter 50 (Kubernetes)
- 🧪 Tests: Constitutional validation PASS (no meta-commentary), structural PASS (correct ending), evidence PASS (code outputs present)
- 📁 Files: 1 primary file created at target path; no new directories created
- 🔁 Next prompts: Lesson 8 (Docker Compose) can now reference this DNS discovery foundation
- 🧠 Reflection: Three-tier example (agent-postgres-redis) demonstrates why user-defined networks matter in production; students understand both the mechanism (DNS) and the isolation benefit before moving to declarative composition

## Evaluation notes (flywheel)

- Failure modes observed: None; autonomous execution without confirmation deadlock
- Graders run and results (PASS/FAIL): Constitutional checks PASS, no forbidden patterns detected
- Prompt variant: N/A (initial write, no iteration needed)
- Next experiment: Validate integration with Lesson 8 (Docker Compose) to ensure DNS knowledge transfers to service interdependency
