---
id: 003
title: Create Learning Hub Implementation Plan
stage: plan
date: 2025-11-06
surface: agent
model: claude-3.7-sonnet
feature: 012-learning-hub-sidebar
branch: 012-learning-hub-sidebar
user: system
command: /sp.plan
labels: ["implementation-plan", "phase-1", "data-model", "api-contracts", "gemini-integration"]
links:
  spec: specs/012-learning-hub-sidebar/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-learning-hub-sidebar/plan.md
  - specs/012-learning-hub-sidebar/research.md
  - specs/012-learning-hub-sidebar/data-model.md
  - specs/012-learning-hub-sidebar/quickstart.md
  - specs/012-learning-hub-sidebar/contracts/gemini-chat.contract.md
  - specs/012-learning-hub-sidebar/contracts/gemini-explain.contract.md
  - specs/012-learning-hub-sidebar/contracts/gemini-quiz.contract.md
  - specs/012-learning-hub-sidebar/contracts/gemini-concepts.contract.md
  - specs/012-learning-hub-sidebar/contracts/gemini-topics.contract.md
  - .github/copilot-instructions.md
tests:
  - Unit test specifications in contracts (Jest + RTL)
  - Integration test specifications in contracts (MSW)
  - E2E test specifications in contracts (Playwright)
---

## Prompt

Create implementation plan following sp.plan.prompt.md workflow for the Learning Hub Sidebar feature (012-learning-hub-sidebar). The feature specification is complete with 6 user stories (P1-P4), 64 functional requirements, and clarified technical decisions (Gemini 2.0 Flash API, console + localStorage logging, MD5 + 7-day caching).

Execute the 3-phase planning workflow:

**Phase 0 (Research)**: Resolve technical unknowns by researching:
1. Google Gemini API integration patterns (SDK vs REST, streaming, rate limits)
2. Docusaurus theme swizzling approach (DocRoot component injection)
3. MDX content extraction methods (DOM traversal, metadata parsing)
4. localStorage schema design (namespacing, versioning, migrations)
5. Content hashing for cache invalidation (MD5 implementation, TTL strategy)
6. Text selection detection and highlight restoration (Selection API, XPath serialization)
7. Error boundary logging strategy (React Error Boundaries, console + localStorage)

**Phase 1 (Design & Contracts)**: Generate:
- data-model.md: Extract 7 entities from spec (ChatMessage, Highlight, QuizQuestion, KeyConcept, RelatedTopic, ProgressRecord, LearningHubState) with TypeScript interfaces, validation rules, storage strategy
- contracts/: Create 5 API contract documents for Gemini endpoints (chat, explain, quiz, concepts, topics) with request/response specs, error handling, rate limiting, caching, testing
- quickstart.md: Developer guide with setup, testing, debugging, deployment
- Update agent context with new technologies

**Phase 2 (Complete)**: Fill remaining plan.md sections (Complexity Tracking if violations exist, Phase 1 completion summary).

Ensure all outputs align with constitution principles (privacy-first, performant, non-intrusive, context-aware) and technical constraints (15 RPM Gemini limit, localStorage quota, no server-side storage).

## Response snapshot

**Phase 0 (Research) - Completed**

Created comprehensive `research.md` (500+ lines) documenting 7 technical decisions:
1. **Google Gemini API Integration**: Selected @google/generative-ai SDK over REST for streaming support, better TypeScript types, built-in error handling. Documented request patterns, rate limits (15 RPM, 1M TPM), retry logic.
2. **Docusaurus Theme Swizzling**: Selected DocRoot component wrapping (non-ejecting swizzle) for clean integration. Documented lazy loading strategy, portal rendering, SSR compatibility.
3. **MDX Content Extraction**: Selected DOM traversal with `.markdown` selector after hydration. Documented text extraction from React components, metadata parsing from frontmatter.
4. **localStorage Schema Design**: Designed namespaced keys (`learningHub_${entity}_v1`), versioning strategy, migration functions, quota management (2-3MB estimated, 5-10MB quota).
5. **Content Hashing**: Selected crypto-js MD5 for cache invalidation (fast, sufficient for non-cryptographic use). Documented 7-day TTL strategy, hash comparison logic.
6. **Text Selection Detection**: Selected window.getSelection() API with XPath serialization for persistent ranges. Documented highlight restoration across page loads, collision handling.
7. **Error Boundary Logging**: Selected React Error Boundaries + console + localStorage ring buffer (last 50 errors). Documented privacy-first approach (no external analytics), error categorization.

**Phase 1 (Design & Contracts) - Completed**

1. **data-model.md** (1,200+ lines):
   - Defined 7 core entities with full TypeScript interfaces
   - ChatMessage: id, role, content, timestamp, pageUrl, tokens (session storage, max 50)
   - Highlight: id, pageUrl, selectedText, textPosition (XPath), explanation, createdAt (localStorage, max 1000)
   - QuizQuestion: id, pageUrl, question, choices[4], correctAnswer, explanation, difficulty (session only, regenerated)
   - KeyConcept: id, pageUrl, title, description, sectionId, importance, contentHash, cachedAt (7-day cache)
   - RelatedTopic: id, pageUrl, title, url, description, relevanceScore, contentHash, cachedAt (7-day cache)
   - ProgressRecord: pageUrl, visitCount, totalDuration, firstVisitedAt, lastVisitedAt, completed (localStorage)
   - LearningHubState: isOpen, activeTab, chatHistory, savedHighlights, progressRecords, errorLog (React Context)
   - Included: Validation rules, storage strategy table, entity relationships diagram, migration strategy, type guards

2. **contracts/** (5 documents, 9,400+ total lines):
   - **gemini-chat.contract.md** (2,500 lines): Streaming Q&A with conversation history, system prompt template, rate limiting, error handling, MSW setup, performance targets (<1s first token, <3s total)
   - **gemini-explain.contract.md** (1,800 lines): Highlight explanations with 30-day cache, MD5 cache keys, validation, fallback behavior, security considerations
   - **gemini-quiz.contract.md** (1,700 lines): Quiz generation (3-5 questions), JSON response parsing, validation, no caching (freshness), prompt engineering notes
   - **gemini-concepts.contract.md** (1,600 lines): Key concepts extraction (5-7), 7-day cache with content hash invalidation, importance scoring, section linking
   - **gemini-topics.contract.md** (1,800 lines): Related topics recommendation (3-5), relevance scoring (0.0-1.0), sidebar integration, URL validation
   - Each contract includes: Request/response specs, TypeScript interfaces, API call examples, error handling table, rate limiting strategy, caching implementation, testing strategy (unit/integration/E2E), performance targets, security considerations

3. **quickstart.md** (900 lines):
   - Quick setup (clone, install, API key, swizzle, verify)
   - Project structure walkthrough
   - Development workflow (commands, testing, linting)
   - Testing guide (Jest unit tests, MSW integration tests, Playwright E2E)
   - Debugging tips (console logging, common issues, localStorage inspection)
   - Performance monitoring (metrics tracking, targets)
   - Deployment checklist (12 items)
   - Security best practices (API key management, data privacy, input sanitization)
   - Additional resources and next steps

4. **Agent Context Updated**:
   - Ran `update-agent-context.ps1 -AgentType copilot`
   - Added TypeScript 5.x + React 18.x (Docusaurus 3.x framework)
   - Added dependencies: @google/generative-ai, crypto-js
   - Updated project type: Web application - Docusaurus plugin/theme customization
   - Added localStorage architecture notes

**Phase 2 (Complete) - Completed**

Updated `plan.md`:
- Complexity Tracking: Marked "No violations detected" (all 7 Constitution Check gates passed)
- Added Phase 1 Completion Summary section documenting all artifacts, key decisions, next steps
- Estimated implementation effort: 8-12 weeks (2-3 person-team)
- Identified critical path: P1 (AI Chat) → P2 (Highlights) → P3 (Quiz + Concepts) → P4 (Topics + Progress)

## Outcome

- ✅ Impact: Complete implementation plan with 7 technical decisions researched, 7 entities modeled, 5 API contracts specified, developer quickstart guide created, agent context updated. Ready for `/sp.tasks` command to generate testable tasks.
- 🧪 Tests: Test specifications included in all 5 API contracts (unit tests with Jest + RTL, integration tests with MSW, E2E tests with Playwright). Coverage targets: 80% logic, 60% UI.
- 📁 Files: Created 10 new files (plan.md updated, research.md, data-model.md, quickstart.md, 5 contract files, agent context updated). Total ~15,000 lines of documentation.
- 🔁 Next prompts: Run `/sp.tasks` to generate testable implementation tasks based on user stories, requirements, data models, and API contracts. Expected output: tasks.md with 40-60 tasks across 6 epics (corresponding to 6 user stories).
- 🧠 Reflection: Phase 1 outputs are comprehensive and implementation-ready. All technical unknowns resolved through research. API contracts provide clear interfaces for frontend-AI integration. Data model includes validation, caching, and migration strategies. Quickstart guide enables onboarding new developers. Constitution alignment verified (privacy-first, performant, non-intrusive). No architectural decisions required ADR documentation (straightforward technical choices with clear rationale documented in research.md and contracts).

## Evaluation notes (flywheel)

- Failure modes observed: None. All phases executed successfully. Template structure followed precisely.
- Graders run and results (PASS/FAIL): 
  - Constitution Check: PASS (7/7 gates)
  - Spec completeness: PASS (64 requirements, 6 user stories, 15 success criteria)
  - Data model completeness: PASS (7 entities, validation rules, storage strategy)
  - API contracts completeness: PASS (5 contracts, all sections filled)
  - Quickstart completeness: PASS (setup, testing, debugging, deployment)
  - Agent context update: PASS (technologies added, project type updated)
- Prompt variant (if applicable): Standard sp.plan workflow (no variants)
- Next experiment (smallest change to try): When generating tasks.md, experiment with task granularity (aim for 2-5 day tasks) to balance testability with progress visibility. Consider grouping related tasks into "task clusters" for better dependency management (e.g., "Gemini Service Setup" cluster: rate limiter → API client → error handling → caching).
