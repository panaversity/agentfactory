# Implementation Tasks: Learning Hub Sidebar

**Feature**: 012-learning-hub-sidebar  
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)  
**Generated**: 2025-11-06

## Task Summary

- **Total Tasks**: 78
- **Parallelizable Tasks**: 45 (57.7%)
- **User Stories**: 6 (P1: Chat, P2: Highlights, P3: Quiz + Concepts, P4: Topics + Progress)
- **Estimated Duration**: 8-12 weeks (2-3 person team)

## Dependencies Graph

```
Phase 1: Setup (foundational)
    ↓
Phase 2: Foundational (shared infrastructure)
    ↓
    ├─→ Phase 3: US1 - AI Chat (P1) ← MVP
    │       ↓
    ├─→ Phase 4: US2 - Smart Highlights (P2)
    │       ↓
    ├─→ Phase 5: US3 - Quick Quiz (P3) ────┐
    │       ↓                                ├─→ Can run in parallel
    └─→ Phase 6: US4 - Key Concepts (P3) ───┘
            ↓
    ├─→ Phase 7: US5 - Related Topics (P4) ──┐
    │       ↓                                  ├─→ Can run in parallel
    └─→ Phase 8: US6 - Progress Tracker (P4) ─┘
            ↓
Phase 9: Polish & Cross-Cutting
```

**Critical Path**: Setup → Foundational → US1 (Chat) → US2 (Highlights) → US3 (Quiz) → Polish

## Parallel Execution Opportunities

### Phase 3 (US1 - AI Chat)
- **Parallel**: T015, T016, T017 (React components - different files)
- **Sequential**: T010 → T011 → T012 → T013 → T014 (services depend on each other)

### Phase 4 (US2 - Highlights)
- **Parallel**: T025, T026, T027 (React components)
- **Parallel**: T029, T030 (independent utilities)

### Phase 5 (US3 - Quiz) + Phase 6 (US4 - Concepts)
- **FULL PARALLELIZATION**: These user stories are fully independent, teams can work simultaneously

### Phase 7 (US5 - Topics) + Phase 8 (US6 - Progress)
- **FULL PARALLELIZATION**: These user stories are fully independent

---

## Phase 1: Setup (Foundation)

**Goal**: Initialize project structure and dependencies

**Duration**: 1-2 days

### Tasks

- [x] T001 Install dependencies: `pnpm add @google/generative-ai crypto-js date-fns` in book-source/
- [x] T002 Install dev dependencies: `pnpm add -D @types/crypto-js msw @playwright/test` in book-source/
- [x] T003 Create environment template .env.example with GEMINI_API_KEY placeholder in book-source/
- [x] T004 [P] Create TypeScript types directory book-source/src/theme/LearningHub/types/ with index.ts
- [x] T005 [P] Create services directory book-source/src/theme/LearningHub/services/
- [x] T006 [P] Create hooks directory book-source/src/theme/LearningHub/hooks/
- [x] T007 [P] Create components directory book-source/src/theme/LearningHub/components/
- [x] T008 [P] Create utils directory book-source/src/theme/LearningHub/utils/
- [x] T009 [P] Create styles directory book-source/src/theme/LearningHub/styles/

---

## Phase 2: Foundational (Shared Infrastructure)

**Goal**: Build shared services that all user stories depend on

**Duration**: 3-5 days

**Independent Test**: Verify rate limiter enforces 15 RPM, storage service handles localStorage quota errors, error logger captures exceptions without breaking UI

### Tasks

- [x] T010 Define all TypeScript interfaces in book-source/src/theme/LearningHub/types/index.ts per data-model.md (ChatMessage, Highlight, QuizQuestion, KeyConcept, RelatedTopic, ProgressRecord, LearningHubState)
- [x] T011 Implement RateLimiter class in book-source/src/theme/LearningHub/services/rateLimiter.ts with 15 RPM limit, shared across all AI features
- [x] T012 Implement StorageService in book-source/src/theme/LearningHub/services/storageService.ts with namespaced keys, quota management, schema versioning
- [x] T013 Implement ErrorLogger in book-source/src/theme/LearningHub/services/errorLogger.ts with console + localStorage ring buffer (last 50 errors)
- [x] T014 Implement CacheService in book-source/src/theme/LearningHub/services/cacheService.ts with MD5 hashing via crypto-js, 7-day TTL, content hash invalidation
- [x] T015 [P] Implement ContentExtractor utility in book-source/src/theme/LearningHub/utils/contentExtractor.ts for MDX DOM traversal with .markdown selector
- [x] T016 [P] Implement hash utility in book-source/src/theme/LearningHub/utils/hash.ts for MD5 content hashing
- [x] T017 [P] Implement debounce utility in book-source/src/theme/LearningHub/utils/debounce.ts for 300ms toggle actions
- [x] T018 [P] Create LearningHub.module.css in book-source/src/theme/LearningHub/styles/ with sidebar base styles (400px width, 300ms transition, z-index layering)
- [x] T019 [P] Create global styles book-source/src/css/learning-hub.css for highlight markers and animations
- [x] T020 Swizzle DocRoot component: `pnpm swizzle @docusaurus/theme-classic DocRoot --wrap` in book-source/
- [x] T021 Create LearningHubContext in book-source/src/theme/LearningHub/context/LearningHubContext.tsx with React Context + useReducer for global state
- [x] T022 Implement GeminiService base client in book-source/src/theme/LearningHub/services/geminiService.ts with GoogleGenerativeAI initialization, error handling, rate limiting integration

---

## Phase 3: User Story 1 - AI Chat (P1) 🎯 MVP

**Goal**: Enable readers to ask questions and get AI-powered answers about page content

**Duration**: 5-7 days

**User Story**: A reader is on a chapter page and wants to ask a question about content they just read. They click the Learning Hub toggle button on the right side, the sidebar smoothly slides in, and they type their question in the AI chat interface. The AI responds with an answer that references the current chapter content.

**Independent Test**: Open any chapter page, toggle sidebar, ask "What is the main topic of this chapter?", verify AI response references visible content within 3 seconds

### Tasks

- [x] T023 [US1] Create main LearningHub component in book-source/src/theme/LearningHub/index.tsx with lazy loading, toggle state, tab navigation
- [x] T024 [US1] Create LearningHubToggle button component in book-source/src/theme/LearningHub/LearningHubToggle.tsx with fixed positioning (bottom-right), smooth animation
- [x] T025 [P] [US1] Implement ChatInterface component in book-source/src/theme/LearningHub/components/AIChat/ChatInterface.tsx with message history, input field, loading state
- [x] T026 [P] [US1] Implement ChatMessage component in book-source/src/theme/LearningHub/components/AIChat/ChatMessage.tsx with role-based styling (user vs assistant)
- [x] T027 [P] [US1] Implement ChatInput component in book-source/src/theme/LearningHub/components/AIChat/ChatInput.tsx with Enter/Shift+Enter handling, character limit (10,000), disabled state
- [x] T028 [US1] Implement sendChatMessage function in book-source/src/theme/LearningHub/services/geminiChatService.ts per gemini-chat.contract.md with streaming, system prompt template, conversation history (last 10 messages)
- [x] T029 [US1] Implement useGeminiChat hook in book-source/src/theme/LearningHub/hooks/useGeminiChat.ts with message state, streaming handling, error recovery
- [x] T030 [US1] Implement usePageContent hook in book-source/src/theme/LearningHub/hooks/usePageContent.ts to extract current page URL, title, MDX content, section metadata
- [x] T031 [US1] Integrate ChatInterface into LearningHub sidebar with "AI Chat" tab, wire up useGeminiChat and usePageContent hooks
- [x] T032 [US1] Update DocRoot wrapper in book-source/src/theme/DocRoot/index.tsx to include LearningHub component with portal rendering
- [x] T033 [US1] Add mobile responsiveness: hide LearningHub on viewports <768px via CSS media query
- [x] T034 [US1] Implement sidebar open/closed state persistence in localStorage with key learningHub_sidebarState_v1
- [x] T035 [US1] Add error handling for AI API failures: display user-friendly message "AI assistant temporarily unavailable" with retry button

**Acceptance Criteria**:
- ✅ Sidebar toggles smoothly (300ms animation)
- ✅ Chat accepts input and displays user message immediately
- ✅ AI response streams within 3 seconds for 90% of queries
- ✅ Conversation history maintained within session
- ✅ Hidden on mobile (<768px)
- ✅ Works on any chapter page in Docusaurus site

---

## Phase 4: User Story 2 - Smart Highlights (P2)

**Goal**: Enable readers to highlight text and get AI explanations of complex concepts

**Duration**: 4-6 days

**User Story**: A reader encounters a complex concept in the chapter. They highlight the text passage with their cursor, and a small popup appears with a "Explain" button. When clicked, the sidebar opens (if closed) and displays an AI-generated explanation of the highlighted concept in simpler terms.

**Independent Test**: Select any text (>10 characters) on a chapter page, click "Explain" button in popup, verify sidebar opens with explanation, save highlight, reload page, verify highlight persists with yellow background

### Tasks

- [x] T036 [US2] Implement useTextSelection hook in book-source/src/theme/LearningHub/hooks/useTextSelection.ts with window.getSelection() API, selection change detection, XPath serialization
- [x] T037 [US2] Implement explainText function in book-source/src/theme/LearningHub/services/geminiService.ts per gemini-explain.contract.md with 30-day cache, validation
- [x] T038 [US2] Implement useHighlights hook in book-source/src/theme/LearningHub/hooks/useHighlights.ts with localStorage persistence, CRUD operations (create, read, delete)
- [x] T039 [P] [US2] Implement HighlightPopup component in book-source/src/theme/LearningHub/components/SmartHighlights/HighlightPopup.tsx with "Explain" and "Save Highlight" buttons, positioned near selection
- [x] T040 [P] [US2] Implement HighlightMarker component in book-source/src/theme/LearningHub/components/SmartHighlights/HighlightMarker.tsx to render saved highlights with yellow background, clickable
- [x] T041 [P] [US2] Implement HighlightsList component in book-source/src/theme/LearningHub/components/SmartHighlights/HighlightsList.tsx in sidebar "Smart Highlights" tab with delete buttons
- [x] T042 [US2] Integrate HighlightPopup into page: detect text selection, show popup if >10 characters, position dynamically
- [x] T043 [US2] Implement highlight save flow: on "Save Highlight" click, persist to localStorage with explanation via useHighlights hook
- [x] T044 [US2] Implement highlight restore on page load: read from localStorage, apply yellow background via HighlightMarker, make clickable
- [x] T045 [US2] Implement highlight click handler: open sidebar, navigate to "Smart Highlights" tab, display stored explanation
- [x] T046 [US2] Add highlight length validation: limit to 1000 characters, show truncation message if exceeded
- [x] T047 [US2] Handle edge case: dismiss popup when clicking outside selection or pressing Escape key
- [x] T048 [US2] Implement highlight deletion: add delete button in HighlightsList, remove from localStorage and DOM on click

**Acceptance Criteria**:
- ✅ Popup appears within 100ms of text selection
- ✅ Explanation generated within 2 seconds
- ✅ Highlights persist across page reloads
- ✅ Yellow background applied to saved highlights
- ✅ Maximum 1000 highlights stored (FIFO cleanup)
- ✅ Works with MDX-rendered content

---

## Phase 5: User Story 3 - Quick Quiz (P3)

**Goal**: Generate AI-powered quizzes to test reader comprehension

**Duration**: 4-5 days

**User Story**: A reader finishes reading a section and wants to test their understanding. They open the Learning Hub sidebar and navigate to the "Quick Quiz" tab. The system automatically generates 3-5 questions based on the current page content. The reader answers the questions and receives immediate feedback on their understanding.

**Independent Test**: Navigate to "Quick Quiz" tab on any chapter page, verify 3-5 questions generated from page content within 5 seconds, answer all questions, verify score displayed as "X/Y - Z%"

### Tasks

- [x] T049 [US3] Implement generateQuiz function in book-source/src/theme/LearningHub/services/geminiService.ts per gemini-quiz.contract.md with JSON parsing, validation (3-5 questions, 4 choices each)
- [x] T050 [US3] Implement useQuiz hook in book-source/src/theme/LearningHub/hooks/useQuiz.ts with quiz session state, answer tracking, score calculation
- [x] T051 [P] [US3] Implement QuizInterface component in book-source/src/theme/LearningHub/components/QuickQuiz/QuizInterface.tsx with question navigation, "Generate Quiz" button, score display
- [x] T052 [P] [US3] Implement QuizQuestion component in book-source/src/theme/LearningHub/components/QuickQuiz/QuizQuestion.tsx with 4 radio button choices, "Check Answer" button, feedback (correct/incorrect with explanation)
- [x] T053 [P] [US3] Implement QuizResults component in book-source/src/theme/LearningHub/components/QuickQuiz/QuizResults.tsx with score summary, "Retake Quiz" button
- [x] T054 [US3] Integrate QuizInterface into sidebar "Quick Quiz" tab
- [x] T055 [US3] Implement quiz generation on tab click: extract page content via usePageContent, call generateQuiz with 4 questions, display in QuizInterface
- [x] T056 [US3] Implement answer validation: on "Check Answer" click, mark correct/incorrect, show explanation, track score
- [x] T057 [US3] Implement quiz completion: after all questions answered, display QuizResults with score as count and percentage
- [x] T058 [US3] Implement "Retake Quiz" flow: clear session state, regenerate new questions from same page content
- [x] T059 [US3] Handle short content edge case: if page <200 words, show message "Content too brief for quiz - try a longer chapter"
- [x] T060 [US3] Clear quiz state on page navigation to ensure fresh quiz per page

**Acceptance Criteria**:
- ✅ Quiz generated within 5 seconds
- ✅ 3-5 questions with exactly 4 choices each
- ✅ Immediate feedback on answer selection
- ✅ Score displayed as "X/5 - Y%"
- ✅ New questions generated on retake
- ✅ Quiz state cleared on navigation

---

## Phase 6: User Story 4 - Key Concepts (P3)

**Goal**: Extract and display key concepts from page content for quick orientation

**Duration**: 3-4 days

**User Story**: A reader opens a chapter and wants a quick overview of the main ideas before diving into details. They open the Learning Hub sidebar and navigate to the "Key Concepts" tab. The system displays 5-7 bullet points extracted from the page, highlighting the most important ideas with brief explanations.

**Independent Test**: Navigate to "Key Concepts" tab on any chapter page, verify 5-7 concepts extracted and cached, click a concept, verify page scrolls to relevant section

### Tasks

- [x] T061 [US4] Implement extractConcepts function in book-source/src/theme/LearningHub/services/geminiService.ts per gemini-concepts.contract.md with 7-day cache, content hash invalidation
- [x] T062 [P] [US4] Implement ConceptsList component in book-source/src/theme/LearningHub/components/KeyConcepts/ConceptsList.tsx with bullet points, loading state
- [x] T063 [P] [US4] Implement ConceptItem component in book-source/src/theme/LearningHub/components/KeyConcepts/ConceptItem.tsx with title, description, importance indicator, clickable
- [x] T064 [US4] Integrate ConceptsList into sidebar "Key Concepts" tab
- [x] T065 [US4] Implement concept extraction with cache: check localStorage cache first (learningHub_concepts_v1), if miss or stale, call extractConcepts, store result
- [x] T066 [US4] Implement concept click handler: extract sectionId from concept, use document.getElementById() to scroll to section
- [x] T067 [US4] Handle short content edge case: if page <500 words, show 2-3 concepts minimum or message "Explore longer chapters for detailed analysis"
- [x] T068 [US4] Implement cache invalidation: compare MD5 content hash, regenerate if mismatch or age >7 days

**Acceptance Criteria**:
- ✅ Concepts extracted within 3 seconds on first visit
- ✅ Cache hit loads instantly (<100ms)
- ✅ 5-7 concepts displayed for typical chapters
- ✅ Concepts clickable and scroll to section
- ✅ Cache invalidated on content change
- ✅ Consistent concepts across users (cached)

---

## Phase 7: User Story 5 - Related Topics (P4)

**Goal**: Recommend related chapters to help readers discover connected content

**Duration**: 3-4 days

**User Story**: A reader is exploring a chapter on Python fundamentals and wants to discover related content. They open the Learning Hub sidebar and navigate to the "Related Topics" tab. The system shows links to other chapters and sections that cover related concepts, helping the reader build connections across the book.

**Independent Test**: Navigate to "Related Topics" tab, verify 3-5 related chapters displayed with titles and descriptions, click a topic link, verify navigation to that page

### Tasks

- [ ] T069 [US5] Implement recommendTopics function in book-source/src/theme/LearningHub/services/geminiService.ts per gemini-topics.contract.md with relevance scoring, URL validation, 7-day cache
- [ ] T070 [US5] Create extractAvailableTopics utility in book-source/src/theme/LearningHub/utils/sidebarParser.ts to parse Docusaurus sidebars.ts and extract all chapter URLs + titles
- [ ] T071 [P] [US5] Implement TopicsList component in book-source/src/theme/LearningHub/components/RelatedTopics/TopicsList.tsx with topic cards, relevance scores
- [ ] T072 [P] [US5] Implement TopicCard component in book-source/src/theme/LearningHub/components/RelatedTopics/TopicCard.tsx with title, description, link
- [ ] T073 [US5] Integrate TopicsList into sidebar "Related Topics" tab
- [ ] T074 [US5] Implement topic recommendation with cache: extract available topics via extractAvailableTopics, check cache (learningHub_topics_v1), if miss call recommendTopics with current page content
- [ ] T075 [US5] Implement topic link navigation: use Docusaurus router Link component, update sidebar context on navigation
- [ ] T076 [US5] Handle no relationships edge case: if <3 topics found or relevance scores <0.5, show message "No related topics found - browse table of contents"

**Acceptance Criteria**:
- ✅ Topics recommended within 3 seconds on first visit
- ✅ Cache hit loads instantly
- ✅ 3-5 related topics displayed
- ✅ Topics clickable and navigate correctly
- ✅ Sidebar context updates on navigation
- ✅ Fallback message for sparse relationships

---

## Phase 8: User Story 6 - Progress Tracker (P4)

**Goal**: Track reader engagement and display learning progress

**Duration**: 3-4 days

**User Story**: A reader wants to track their learning journey through the book. They open the Learning Hub sidebar and navigate to the "Progress" tab. The system shows which chapters they've visited, how much time they've spent reading, and highlights they've created, giving them a sense of accomplishment and momentum.

**Independent Test**: View multiple chapter pages for >30 seconds each, create highlights, navigate to "Progress" tab, verify visited chapters listed with timestamps and read duration, total highlight count displayed

### Tasks

- [ ] T077 [US6] Implement useProgress hook in book-source/src/theme/LearningHub/hooks/useProgress.ts with page visit tracking, duration calculation (time between load and navigation), localStorage persistence (learningHub_progress_v1)
- [ ] T078 [P] [US6] Implement ProgressDashboard component in book-source/src/theme/LearningHub/components/ProgressTracker/ProgressDashboard.tsx with visited chapters list, total time, highlight count, completion percentage
- [ ] T079 [P] [US6] Implement ProgressChart component in book-source/src/theme/LearningHub/components/ProgressTracker/ProgressChart.tsx with visual progress indicators (CSS progress bars)
- [ ] T080 [US6] Integrate ProgressDashboard into sidebar "Progress" tab
- [ ] T081 [US6] Implement page visit tracking: on page load, record URL + timestamp in useProgress, mark as "visited" if duration >30 seconds
- [ ] T082 [US6] Implement read duration calculation: use useEffect with cleanup to track time between mount and unmount, store in ProgressRecord
- [ ] T083 [US6] Calculate aggregated statistics: total chapters visited, total highlights (from useHighlights), total time spent, completion percentage (visited / total chapters from sidebar)
- [ ] T084 [US6] Implement "Clear Progress" button with confirmation dialog: on confirm, clear learningHub_progress_v1 from localStorage

**Acceptance Criteria**:
- ✅ Page marked "visited" after 30 seconds
- ✅ Read duration accurately tracked
- ✅ Progress persists across sessions
- ✅ Statistics calculated correctly
- ✅ Clear progress works with confirmation
- ✅ Completion percentage based on total chapters

---

## Phase 9: Polish & Cross-Cutting Concerns

**Goal**: Finalize UI/UX, performance optimizations, error handling, accessibility

**Duration**: 3-5 days

### Tasks

- [ ] T085 [P] Implement React Error Boundaries in book-source/src/theme/LearningHub/components/ErrorBoundary.tsx to catch component errors without breaking entire UI
- [ ] T086 [P] Add loading states and skeletons for all AI operations (chat, highlights, quiz, concepts, topics)
- [ ] T087 [P] Implement offline detection: show connectivity status in sidebar header, queue failed requests for retry
- [ ] T088 [P] Add keyboard shortcuts: Ctrl+Shift+L to toggle sidebar, Escape to close sidebar
- [ ] T089 [P] Implement tab switching debounce (300ms) to cancel in-flight AI requests when user rapidly switches tabs
- [ ] T090 Optimize bundle size: lazy load LearningHub component, code-split by tab (dynamic imports for Quiz, Concepts, Topics)
- [ ] T091 Add ARIA labels and semantic HTML for accessibility (sidebar role, tab navigation, button labels)
- [ ] T092 Implement localStorage quota management: monitor usage, warn at 80% capacity, cleanup old cache entries
- [ ] T093 Add rate limit feedback in UI: show "X requests remaining this minute" in sidebar footer
- [ ] T094 Implement graceful degradation: if Gemini API key missing, show setup instructions instead of errors
- [ ] T095 Add analytics event hooks (optional): emit custom events for feature usage (sidebar toggle, chat sent, highlight created) for future analytics integration
- [ ] T096 [P] Write unit tests for services (rateLimiter, storageService, cacheService, geminiService) using Jest
- [ ] T097 [P] Write unit tests for hooks (useGeminiChat, useHighlights, useQuiz, useProgress) using React Testing Library
- [ ] T098 [P] Write integration tests for Gemini API calls using MSW (Mock Service Worker) per contract test scenarios
- [ ] T099 [P] Write E2E tests using Playwright: sidebar toggle, chat interaction, highlight creation, quiz completion, navigation flows
- [ ] T100 Performance audit: measure and optimize Initial sidebar load (<200ms), AI response time (<3s p90), animation smoothness (60fps)
- [ ] T101 Cross-browser testing: verify functionality on Chrome, Firefox, Safari, Edge (latest versions)
- [ ] T102 Mobile responsiveness validation: test on iOS Safari and Android Chrome, ensure sidebar hidden <768px
- [ ] T103 Create deployment documentation in quickstart.md: environment variables, build steps, troubleshooting
- [ ] T104 Security review: validate input sanitization, API key protection, CSP compatibility, XSS prevention

---

## Implementation Strategy

### MVP Scope (Recommended for Initial Release)

**Target**: User Story 1 (AI Chat) ONLY

**Rationale**: Delivers core value proposition (contextual AI assistance) with minimal scope. All other features build on this foundation.

**MVP Tasks**: T001-T022 (Setup + Foundational) + T023-T035 (US1 - AI Chat) = **35 tasks, ~2 weeks**

**MVP Acceptance**: Reader can toggle sidebar, ask questions, get AI responses referencing page content.

### Incremental Delivery Plan

1. **Week 1-2**: MVP (US1 - AI Chat)
2. **Week 3-4**: US2 (Smart Highlights) - extends chat with targeted help
3. **Week 5-6**: US3 (Quiz) + US4 (Concepts) in parallel - active learning features
4. **Week 7-8**: US5 (Topics) + US6 (Progress) in parallel - navigation and motivation
5. **Week 9-10**: Polish, testing, performance optimization

### Quality Gates

Each phase must pass before proceeding to next:

- ✅ All tasks completed and code reviewed
- ✅ Independent test criteria validated manually
- ✅ Unit tests passing (80% coverage for services/hooks)
- ✅ No console errors or warnings
- ✅ Performance targets met (sidebar load <200ms, AI response <3s p90)
- ✅ Accessibility audit passed (ARIA labels, keyboard navigation)

### Risk Mitigation

- **Gemini API Rate Limits**: Implement shared rate limiter early (T011), monitor usage, show user feedback
- **localStorage Quota**: Implement quota management (T092), cleanup old cache (T014), warn users at 80%
- **Content Extraction Failures**: Graceful fallback for non-standard MDX, test with various chapter structures
- **AI Response Quality**: Human-evaluate sample responses per feature, refine prompts in contracts
- **Performance Regression**: Continuous monitoring with Lighthouse, lazy loading, code splitting

---

## Validation Checklist

Before marking feature complete:

- [ ] All 104 tasks completed
- [ ] All 6 user stories independently testable and passing
- [ ] Constitution Check gates still passing (privacy, performance, non-intrusive, context-aware)
- [ ] Unit test coverage ≥80% for services, ≥60% for components
- [ ] Integration tests passing for all 5 Gemini API contracts
- [ ] E2E tests passing for all 6 user story flows
- [ ] Performance targets met (sidebar <200ms, AI <3s, UI <100ms)
- [ ] Cross-browser compatibility verified (Chrome, Firefox, Safari, Edge)
- [ ] Mobile responsiveness validated (sidebar hidden <768px)
- [ ] Accessibility audit passed (WCAG 2.1 AA compliance)
- [ ] Security review completed (input sanitization, API key protection)
- [ ] Documentation updated (README, quickstart.md, deployment guide)
- [ ] Stakeholder demo completed and approved

---

**Next Steps**: Begin with Phase 1 (Setup) tasks T001-T009 to initialize project structure and dependencies. Once complete, proceed to Phase 2 (Foundational) to build shared services that all user stories depend on. After foundational infrastructure is solid, start with MVP (US1 - AI Chat) for quickest value delivery.
