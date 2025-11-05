# Implementation Plan: Learning Hub Sidebar

**Branch**: `012-learning-hub-sidebar` | **Date**: 2025-11-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/012-learning-hub-sidebar/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The Learning Hub sidebar is a collapsible right-side panel integrated into Docusaurus book pages that provides AI-powered learning assistance. The feature delivers six progressively enhanced capabilities: (P1) AI chat with contextual Q&A, (P2) smart highlights with AI explanations, (P3) quick quiz generation, (P3) key concepts extraction, (P4) related topics navigation, and (P4) progress tracking.

Technical approach leverages Google Gemini 2.0 Flash API for AI-powered features, React components integrated via Docusaurus theme swizzling, localStorage for client-side data persistence with MD5 content hashing + 7-day cache expiry, and console + localStorage logging for observability. The sidebar is responsive (hidden <768px), non-intrusive (collapsible with smooth animations), and privacy-first (no server-side data collection).

## Technical Context

**Language/Version**: TypeScript 5.x with React 18.x (Docusaurus 3.x framework)  
**Primary Dependencies**:
- `@google/generative-ai` - Google Gemini API SDK for AI features
- `@docusaurus/theme-classic` - Base theme for swizzling
- `react` & `react-dom` - UI components
- `@docusaurus/router` - Navigation and routing
- `crypto-js` - MD5 hashing for content cache invalidation
- `date-fns` - Date/time utilities for progress tracking

**Storage**: Browser localStorage (structured JSON) for:
- Chat history (session-only, cleared on tab close)
- Saved highlights with explanations (persistent)
- Progress records (persistent)
- Key concepts cache (7-day TTL)
- Related topics cache (7-day TTL)
- Error log (last 50 errors)

**Testing**:
- Jest + React Testing Library for component unit tests
- Playwright for E2E testing (sidebar interactions, AI responses)
- MSW (Mock Service Worker) for Gemini API mocking
- Target coverage: 80% business logic, 60% UI components

**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge latest versions)
- Desktop: ≥1024px width (sidebar at 400px)
- Tablet: 768px-1024px (sidebar at 320px)
- Mobile: <768px (sidebar hidden)

**Project Type**: Web application - Docusaurus plugin/theme customization

**Performance Goals**:
- Initial sidebar load: <200ms (lazy loaded, no blocking)
- AI chat response: <3s for 90% of queries (Gemini streaming)
- Quiz generation: <5s for 3-5 questions
- Key concepts extraction: <3s for 5-7 concepts
- Text highlight popup: <100ms after selection
- Smooth animations: 60fps (300ms CSS transitions)

**Constraints**:
- No server-side data storage (privacy-first, localStorage only)
- No external analytics/tracking (console + localStorage logging only)
- Must not break existing Docusaurus navigation/layout
- Must work with MDX content (text extraction from React components)
- Gemini API rate limits (respecting free tier: 15 RPM, 1M TPM)
- localStorage quota (typically 5-10MB per domain)
- Maximum 2 concurrent AI API requests to prevent rate limiting

**Scale/Scope**:
- Expected users: 1,000-10,000 concurrent readers
- Content scope: ~56 chapters across multiple parts
- Average page size: 2,000-5,000 words
- Expected highlights per user: 10-50 across sessions
- Chat messages per session: 5-20 questions
- Cache size per page: ~10-50KB (concepts + topics + quiz)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Spec-driven**: Feature has complete specification with 6 prioritized user stories (P1-P4), 64 functional requirements, 15 success criteria, and clarified technical decisions
- [x] **User story independence**: Each of 6 stories is independently implementable and testable as MVP (P1: chat, P2: highlights, P3: quiz/concepts, P4: topics/progress)
- [x] **Privacy-first**: All data stored in browser localStorage only, no server-side collection, console + localStorage logging (last 50 errors), no external analytics
- [x] **Performance budget**: <200ms sidebar load, <3s AI response (90%), <5s quiz generation, <100ms UI interactions, lazy loading prevents page load blocking
- [x] **Non-intrusive**: Collapsible sidebar (300ms animation), hidden on mobile (<768px), toggle button always visible, doesn't interfere with reading experience
- [x] **Context-aware**: AI features extract current page URL, title, headings, content for context; highlights reference specific text; progress tracks per-page engagement
- [x] **Technology stack**: TypeScript + React (Docusaurus framework), Google Gemini API for AI, aligns with book's frontend technology stack

**✅ ALL GATES PASSED** - No violations. Feature fully complies with constitution principles.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book-source/
├── src/
│   ├── theme/
│   │   └── LearningHub/                 # Main feature directory
│   │       ├── index.tsx                # Main sidebar component (lazy loaded)
│   │       ├── LearningHubToggle.tsx    # Toggle button component
│   │       ├── components/
│   │       │   ├── AIChat/
│   │       │   │   ├── ChatInterface.tsx
│   │       │   │   ├── ChatMessage.tsx
│   │       │   │   └── ChatInput.tsx
│   │       │   ├── SmartHighlights/
│   │       │   │   ├── HighlightPopup.tsx
│   │       │   │   ├── HighlightMarker.tsx
│   │       │   │   └── HighlightsList.tsx
│   │       │   ├── QuickQuiz/
│   │       │   │   ├── QuizInterface.tsx
│   │       │   │   ├── QuizQuestion.tsx
│   │       │   │   └── QuizResults.tsx
│   │       │   ├── KeyConcepts/
│   │       │   │   ├── ConceptsList.tsx
│   │       │   │   └── ConceptItem.tsx
│   │       │   ├── RelatedTopics/
│   │       │   │   ├── TopicsList.tsx
│   │       │   │   └── TopicCard.tsx
│   │       │   └── ProgressTracker/
│   │       │       ├── ProgressDashboard.tsx
│   │       │       └── ProgressChart.tsx
│   │       ├── hooks/
│   │       │   ├── useGeminiAPI.ts          # Gemini API integration
│   │       │   ├── usePageContent.ts        # Extract page context
│   │       │   ├── useLocalStorage.ts       # Storage management
│   │       │   ├── useTextSelection.ts      # Highlight detection
│   │       │   ├── useCache.ts              # Cache with MD5 + TTL
│   │       │   └── useErrorLog.ts           # Console + localStorage logging
│   │       ├── services/
│   │       │   ├── geminiService.ts         # Gemini API client
│   │       │   ├── cacheService.ts          # Cache management (MD5 + 7-day)
│   │       │   ├── storageService.ts        # localStorage abstraction
│   │       │   ├── contentExtractor.ts      # MDX content parsing
│   │       │   └── errorLogger.ts           # Error logging service
│   │       ├── types/
│   │       │   ├── index.ts                 # Exported types
│   │       │   ├── entities.ts              # Data models
│   │       │   └── api.ts                   # API types
│   │       ├── utils/
│   │       │   ├── hash.ts                  # MD5 hashing utility
│   │       │   ├── debounce.ts              # Debounce helper
│   │       │   └── formatting.ts            # Text utilities
│   │       └── styles/
│   │           └── LearningHub.module.css   # Component styles
│   │
│   └── css/
│       └── learning-hub.css                 # Global styles for feature
│
├── tests/
│   ├── unit/
│   │   └── LearningHub/
│   │       ├── components/                  # Component tests
│   │       ├── hooks/                       # Hook tests
│   │       └── services/                    # Service tests
│   └── e2e/
│       └── learning-hub.spec.ts             # Playwright E2E tests
│
└── package.json                             # Add dependencies
```

**Structure Decision**: Selected web application structure integrated within existing Docusaurus project at `book-source/`. The LearningHub feature is implemented as a theme component that can be swizzled into the Docusaurus theme. This approach:
- Leverages Docusaurus theming system for clean integration
- Allows lazy loading of the sidebar to prevent page load blocking
- Maintains separation of concerns with clear directory structure
- Enables independent testing of components, hooks, and services
- Follows React best practices with component composition

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected.** All 7 Constitution Check gates passed. No complexity justifications required.

---

## Phase 1 Completion Summary

**Date**: 2025-11-06  
**Status**: ✅ Complete

### Artifacts Created

1. **data-model.md** (1,200+ lines)
   - Defined 7 core entities: ChatMessage, Highlight, QuizQuestion, KeyConcept, RelatedTopic, ProgressRecord, LearningHubState
   - TypeScript interfaces with full validation rules
   - Storage strategy with caching and TTL specifications
   - Entity relationship diagram
   - Migration strategy and type guards

2. **contracts/** (5 contract documents)
   - `gemini-chat.contract.md` - Interactive Q&A with streaming (2,500+ lines)
   - `gemini-explain.contract.md` - Highlight explanations with 30-day cache (1,800+ lines)
   - `gemini-quiz.contract.md` - Quiz generation (3-5 questions) (1,700+ lines)
   - `gemini-concepts.contract.md` - Key concepts extraction with 7-day cache (1,600+ lines)
   - `gemini-topics.contract.md` - Related topics recommendation with 7-day cache (1,800+ lines)
   - All contracts include: request/response specs, error handling, rate limiting, caching, testing strategy, security considerations

3. **quickstart.md** (900+ lines)
   - Setup instructions (API key, dependencies, swizzling)
   - Development workflow (commands, testing, linting)
   - Testing guide (unit, integration, E2E)
   - Debugging tips and common issues
   - Performance monitoring guidelines
   - Deployment checklist
   - Security best practices

4. **Agent Context Updated**
   - Added TypeScript 5.x + React 18.x (Docusaurus 3.x)
   - Added dependencies: @google/generative-ai, crypto-js
   - Updated project type: Web application - Docusaurus plugin/theme customization
   - Added localStorage architecture

### Key Decisions Documented

**AI Provider**: Google Gemini 2.0 Flash (fast, cost-effective, streaming support)  
**Integration Method**: Docusaurus DocRoot swizzling (non-invasive, upgrade-safe)  
**Content Extraction**: DOM traversal with `.markdown` selector (handles MDX)  
**Storage Schema**: Namespaced localStorage keys with versioning + migrations  
**Cache Invalidation**: MD5 content hashing + 7-day TTL (balances freshness & performance)  
**Text Selection**: window.getSelection() API + XPath serialization (persistent ranges)  
**Error Logging**: Console + localStorage ring buffer (last 50 errors, privacy-first)  
**Rate Limiting**: Shared 15 RPM limiter across all AI features (prevents quota exhaustion)

### Next Steps

Ready to proceed to `/sp.tasks` command to generate testable tasks based on:
- 6 user stories (P1-P4 priorities)
- 64 functional requirements
- 7 entities with full data models
- 5 API contracts with detailed specifications
- Project structure and component hierarchy
- Technical constraints and performance targets

**Estimated Implementation Effort**: 8-12 weeks (2-3 person-team)  
**Critical Path**: P1 (AI Chat) → P2 (Highlights) → P3 (Quiz + Concepts) → P4 (Topics + Progress)
