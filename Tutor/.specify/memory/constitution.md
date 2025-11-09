<!--
Sync Impact Report:
Version Change: [initial] → 1.0.0
Modified Principles: N/A (initial creation)
Added Sections:
  - Core Principles (6 principles)
  - MVP Strategy
  - Learning Philosophy
  - Development Philosophy
  - Quality Commitments
  - MVP Success Criteria
  - Launch Plan
  - Post-MVP Roadmap
  - Team Approach
  - Decision Framework
  - Governance
Templates Requiring Updates:
  ✅ plan-template.md — Constitution Check section aligned
  ✅ spec-template.md — User stories and acceptance criteria align with learning philosophy
  ✅ tasks-template.md — Task organization reflects development philosophy
Follow-up TODOs: None — all sections complete
-->

# TutorGPT Constitution

**Project**: TutorGPT - AI-Native Learning Companion for "AI-Native Software Development" Book
**Focus**: MVP 1.0 (ChatKit + Docusaurus Only)
**Mission**: Build an AI tutor that sits beside students as they read, ready to explain anything instantly

---

## Core Principles

### I. Ship Fast, Ship Well

**This means:**
- Focus on ONE thing: Book + ChatKit sidebar
- ChatKit appears on ALL Docusaurus pages (107 pages)
- Multi-level RAG (4 levels) works perfectly from day one
- Agent gives accurate, contextual answers
- Launch in 4 weeks maximum

**Not:**
- Building multiple interfaces at once
- Perfect features before launch
- Custom UI from scratch
- Over-engineering the MVP

**Rationale:** Focus delivers speed. One interface done excellently beats three interfaces done poorly. ChatKit provides production-ready UI, letting us concentrate on intelligence (multi-level RAG, autonomous agent). The 4-week deadline forces ruthless prioritization—only features that directly serve "student reading book + getting instant help" make the cut.

### II. Context is Everything

**The Agent MUST know:**
- Which page student is reading
- Which section they're viewing
- What text they highlighted
- Their learning history
- Their confused topics

**This means:**
- Multi-level RAG (4 levels) from day one: page → section → chapter → book
- Page context captured automatically
- Highlight detection working
- Session persistence implemented

**Not:**
- Generic answers without context
- Treating every question the same
- Forgetting previous conversations

**Rationale:** Generic chatbots fail students because they lack situational awareness. A student asking "What does this mean?" while reading Chapter 5, Section 2 needs an answer grounded in that specific content, not a general definition. Multi-level RAG ensures answers are precise, relevant, and connected to the student's current learning position.

### III. The Agent is Autonomous

**This is NOT a chatbot. This is an intelligent agent:**
- Agent understands student's mental model
- Agent decides best teaching approach
- Agent adapts explanations in real-time
- Agent makes autonomous decisions
- Agent greets warmly when student arrives

**Using:**
- OpenAI Agents SDK (latest: https://openai.github.io/openai-agents-python/)
- LangChain for RAG coordination
- GPT-4 for intelligence

**Not:**
- Simple pattern matching
- Static Q&A responses
- One-size-fits-all answers

**Rationale:** Students need a teacher, not a search engine. An autonomous agent detects confusion, adjusts explanation depth, provides examples without being asked, and encourages progress. This requires the OpenAI Agents SDK's decision-making capabilities and LangChain's orchestration, not simple prompt templates.

### IV. Students Learn Better with Help

**Reading alone is hard:**
- Get stuck on confusing concepts
- Don't know what to ask
- No feedback on understanding
- Easy to quit

**With our Agent:**
- Highlight text → instant explanation
- Ask questions → contextual answers
- Multi-level comprehension checks
- Encouragement and guidance

**Rationale:** Research shows students learning technical material alone have 60%+ dropout rates. Real-time guidance reduces frustration, maintains motivation, and provides the feedback loop that traditional books lack. Our agent removes the "I'm stuck and don't know what to do" barrier that causes most learning failures.

### V. Personalization Through Observation

**Agent learns about student by:**
- Tracking questions asked
- Noticing repeated confusion
- Observing reading pace
- Identifying strong topics

**Then adapts:**
- Simpler explanations for confused topics
- Faster pace for strong students
- More examples when needed
- References previous learning

**Not:**
- Annoying surveys
- Manual preference settings
- Forcing students to configure

**Rationale:** The best teachers observe and adapt without explicit feedback. Our agent does the same. If a student asks three questions about "multi-level RAG," the agent recognizes this as a struggle area and proactively simplifies future explanations. Personalization must be invisible and automatic.

### VI. Production Quality from Day One

**MVP means Minimum VIABLE:**
- Fast responses (< 3 seconds)
- Works on all Docusaurus pages
- Accurate answers from book content
- Sessions persist across visits
- No critical bugs

**Not:**
- "We'll fix it later" attitude
- Shipping broken features
- Slow or unreliable system

**Rationale:** Students will tolerate missing features but will abandon a slow, buggy, or inaccurate system instantly. Production quality is non-negotiable. Testing, performance optimization, and reliability engineering happen during development, not after launch. Every component must be deployment-ready before integration.

### VII. Test-Driven Development (TDD) from the Start

**Every feature is built test-first:**
- Write test BEFORE implementation
- Test defines expected behavior
- Implementation makes test pass
- Refactor with confidence

**This means:**
- Agent behavior is testable: "Does agent teach correctly?"
- Tools are testable: "Does search_book_content return relevant results?"
- All code has ≥80% test coverage
- Tests verify BEHAVIOR, not just code

**TDD for Agent-First System:**
- **Unit tests**: Individual functions (RAG search, embeddings, database queries)
- **Integration tests**: Agent + tools working together
- **Behavior tests**: Agent teaching quality (Socratic method, adaptation, encouragement)
- **Scenario tests**: Full user journeys (student asks → agent teaches → student understands)

**Not:**
- Writing tests after code is done
- Skipping tests for "simple" features
- Testing only happy paths
- Achieving coverage without testing behavior

**Rationale:** TDD ensures the agent truly teaches well, not just "works." When we test "agent simplifies explanation for confused student," we verify autonomous teaching behavior. Tests become living documentation of how TutorGPT should behave. Every task follows: Write test → Implement → Verify → Refactor.

---

## MVP Strategy

### What We're Building

**ONE Interface: Book + ChatKit Sidebar**
- Student reads Docusaurus book
- ChatKit widget in corner of every page
- Click to open chat
- Ask questions, get help
- Highlight text for explanations

**Technical Stack:**
- **Backend**: FastAPI (Python 3.11+)
- **AI Agent**: OpenAI Agents SDK + GPT-4
- **RAG**: LangChain + 4-level retrieval (page/section/chapter/book)
- **Database**: PostgreSQL (sessions, history)
- **Frontend**: ChatKit (pre-built UI) + Docusaurus integration
- **Deployment**: Docker + production hosting

### What We're NOT Building (Yet)

**Save for Phase 2:**
- Standalone chat interface
- Pure conversational learning mode (no book)
- Custom frontend
- Mobile app
- User accounts/authentication

**Rationale:** These are valuable but not critical for MVP. Focus = speed. We prove the concept (AI help while reading) with one interface first, then expand based on user feedback.

### Why This Strategy

**Focus = Speed:**
- 4 weeks to production
- One thing done well
- Proven concept
- Real user feedback

**Then Expand:**
- Add conversational mode later (Phase 2)
- Build on solid foundation
- Based on user needs

---

## Learning Philosophy

### Students Need Different Help at Different Times

**Sometimes they need:**
- Quick definition of a term
- Detailed explanation of concept
- Example to clarify
- Connection to previous learning
- Encouragement to continue

**Agent provides all of these:**
- Autonomously decides what type of help
- Adapts based on context
- Personalizes to student's level

**Implementation:**
- Agent uses OpenAI Agents SDK to reason about student need
- Multi-level RAG retrieves appropriate content depth
- Session history informs personalization

### Reading + Guidance = Best Learning

**Independent reading:**
- Student controls pace
- Can revisit sections
- Builds comprehension skills

**Plus AI guidance:**
- Removes frustration blocks
- Provides instant feedback
- Ensures understanding
- Maintains motivation

**Pedagogical Model:**
- Book remains primary learning source (respect author's structure)
- Agent supplements, never replaces
- Student retains agency (they decide when to ask for help)
- Learning is active, not passive

---

## Development Philosophy

### Build Backend First (Weeks 1-2)

**Backend is the intelligence:**
- FastAPI foundation
- Multi-level RAG system (4 levels: page → section → chapter → book)
- OpenAI Agents SDK integration
- Session management
- Database setup (PostgreSQL)

**Why first:**
- Backend is the intelligence—must work perfectly
- RAG must retrieve accurately (most critical component)
- Agent needs proper setup
- Can test independently before frontend integration

**Acceptance Criteria:**
- API endpoints respond < 500ms
- RAG retrieves correct content at all 4 levels
- Agent generates contextual answers
- Sessions persist and restore correctly
- All unit and integration tests pass

### Then Frontend (Week 3)

**ChatKit integration in Docusaurus:**
- ChatKit integration in Docusaurus theme
- Context capture (page, section, highlighted text)
- Highlight detection
- UI polish

**Why second:**
- ChatKit is ready-made UI (fast integration)
- Quick integration with Docusaurus
- Backend already tested and stable

**Acceptance Criteria:**
- ChatKit appears on all 107 book pages
- Highlight detection triggers correct API calls
- Context (page, section) sent with every message
- UI responsive and accessible

### Finally Polish (Week 4)

**Make it production-ready:**
- End-to-end testing (user journeys)
- Performance optimization (caching, query optimization)
- Bug fixes
- Documentation (API docs, deployment guide)
- Deploy to production

**Acceptance Criteria:**
- All critical user journeys complete successfully
- Response time < 3 seconds (95th percentile)
- Zero critical bugs
- Deployment automated and documented

### Test Everything (TDD Approach)

**TDD Workflow - ALWAYS:**
1. **Write test FIRST** - Define expected behavior
2. **Run test** - Watch it fail (red)
3. **Write minimal code** - Make test pass (green)
4. **Refactor** - Improve code while tests pass
5. **Repeat** - Next feature

**Every component tested:**
- **Unit tests**: Individual functions (RAG, embeddings, DB queries)
- **Integration tests**: Agent + tools + services working together
- **Behavior tests**: Agent teaching quality (does it teach well?)
- **Scenario tests**: Full user journeys (student confusion → agent adapts)
- **Manual testing**: UX and edge cases

**Before shipping:**
- All tests pass (green ✅)
- No critical bugs
- Performance acceptable (< 3 seconds)
- Agent behavior verified (teaches correctly)
- Ready for real users

**Testing Standards (TDD-First):**
- Minimum 80% code coverage (but behavior-focused, not line-focused)
- Contract tests for API endpoints (test-first)
- Integration tests for RAG pipeline (verify 4-level search works)
- Agent behavior tests (verify teaching strategies work)
- Load testing for 100 concurrent users
- **Every task**: Write test → Implement → Verify → Refactor

---

## Quality Commitments

### To Students

**We promise:**
- Agent answers accurately (uses book content, cites sources)
- Responses come quickly (< 3 seconds)
- Your progress is saved (sessions persist)
- The system is reliable (99% uptime target)
- Help is always available (no downtime during peak hours)

**We will NOT:**
- Give wrong information
- Let the system be slow
- Lose your conversation history
- Ship with major bugs

**Measurement:**
- Answer accuracy audited by reviewing 100 random responses/week
- Response time monitored via APM (Application Performance Monitoring)
- Session persistence tested in automated suite
- Uptime tracked via monitoring dashboard

### To the Book

**We promise:**
- Respect author's content
- Keep explanations aligned with book
- Cite sources properly (chapter, section, page)
- Direct students to original material

**We will NOT:**
- Misrepresent book content
- Replace the book experience
- Give contradictory information

**Implementation:**
- All RAG responses include source citations
- Agent instructed to defer to book for authoritative content
- Contradictions flagged and reviewed

### To Ourselves

**We promise:**
- Ship in 4 weeks
- Build quality foundation (clean code, documented)
- Write maintainable code (typed, tested, reviewed)
- Document everything (code, API, deployment)
- Learn from this project

**We will NOT:**
- Miss the deadline
- Cut corners on quality
- Write messy code
- Skip documentation

**Standards:**
- Code reviews required for all changes
- Documentation written alongside code
- Architecture decisions recorded in ADRs
- Weekly retrospectives to course-correct

---

## MVP Success Criteria

### We'll Know We Succeeded When

**Technical Success:**
- ✅ ChatKit appears on all 107 book pages
- ✅ Agent gives accurate answers from book content
- ✅ Multi-level RAG (4 levels) retrieves correct content
- ✅ Responses come in < 3 seconds (95th percentile)
- ✅ Sessions persist across visits (tested)
- ✅ Highlighting text triggers correct explanation
- ✅ Zero critical bugs (P0 issues)

**User Success:**
- ✅ Students say "This helped me understand"
- ✅ Students return to continue learning (>50% return rate)
- ✅ Students ask multiple questions (avg 5+ per session)
- ✅ Students complete more lessons (measured via page progression)
- ✅ Positive feedback from real users (Net Promoter Score > 40)

**Business Success:**
- ✅ Shipped in 4 weeks
- ✅ Production-ready quality
- ✅ Can demo confidently (stable, fast, accurate)
- ✅ Ready to scale (architecture supports growth)
- ✅ Foundation for Phase 2 (extensible codebase)

### We'll Know We Failed If

- ❌ Agent gives wrong information
- ❌ System is too slow (> 5 seconds)
- ❌ Students get frustrated (high bounce rate)
- ❌ Critical bugs in production
- ❌ Can't ship in 4 weeks
- ❌ Code is unmaintainable

---

## Launch Plan

### Week 1-2: Backend Core

**Build the intelligence:**
- FastAPI setup and project structure
- Multi-level RAG (4 levels) working and tested
- OpenAI Agents SDK integrated
- Database (PostgreSQL) and session management
- API endpoints tested (unit + integration)

**Deliverables:**
- Working API with documented endpoints
- RAG pipeline retrieving at all 4 levels
- Agent responding to test queries
- Session persistence verified
- All tests passing

### Week 3: ChatKit Integration

**Build the interface:**
- ChatKit widget integrated in Docusaurus
- Context capture (page, section, highlight) working
- Highlight detection implemented
- Full integration tested (frontend ↔ backend)

**Deliverables:**
- ChatKit visible on all pages
- Highlighting text sends correct context to API
- Messages display agent responses
- Session continuity across page navigation

### Week 4: Polish & Deploy

**Make it production-ready:**
- End-to-end testing (full user journeys)
- Performance optimization (caching, query tuning)
- Bug fixes (prioritize critical, high, medium)
- Documentation (user guide, API docs, deployment)
- Deploy to production (Docker + hosting)

**Deliverables:**
- Production deployment live
- All critical bugs resolved
- Documentation complete
- Monitoring dashboards active

### Post-Launch

**Iterate based on feedback:**
- Monitor usage (analytics, logs)
- Collect feedback (surveys, support tickets)
- Fix issues quickly (hotfix process)
- Plan Phase 2 (conversational mode)

---

## Post-MVP Roadmap

### Phase 2 (After MVP Success)

**Add Conversational Learning Mode:**
- Standalone chat interface (no book required)
- Pure agentic teaching (agent guides full journey)
- Agent guides full journey
- Student learns without reading

**Why wait:**
- Prove MVP first (validate core value)
- Learn from users (what do they actually want?)
- Build on solid foundation (reuse backend, agent)
- Based on real needs (not assumptions)

### Phase 3 (Future)

**Scale and expand:**
- User accounts/authentication
- Progress analytics (dashboards for students and educators)
- Multiple books (expand beyond AI-Native Software Development)
- Community features (student discussion, peer help)
- Mobile app (native iOS/Android)

---

## Team Approach

### How We Work

**Clear Focus:**
- Everyone knows: Ship Book + Sidebar in 4 weeks
- No scope creep (new features go to Phase 2 backlog)
- No distractions
- Finish what we start (complete phases before moving on)

**Communication:**
- Daily progress updates (async standup)
- Weekly team meetings (planning, retrospective)
- Quick problem solving (unblock immediately)
- Celebrate wins (recognize progress)

**Quality First:**
- Test before commit (CI runs tests)
- Review each other's code (all PRs reviewed)
- Document as we build (no deferred docs)
- No "quick hacks" (technical debt avoided)

**Fast Iteration:**
- Build → Test → Fix → Repeat
- Don't wait for perfect (ship when it works)
- Ship when it works
- Improve continuously (iterate based on feedback)

---

## Decision Framework

### When Making Decisions, Ask

**Does this help MVP?**
- ✅ Yes → Do it
- ❌ No → Save for Phase 2

**Does this improve learning?**
- ✅ Yes → Prioritize
- ❌ No → Reconsider

**Can we ship this in 4 weeks?**
- ✅ Yes → Include in MVP
- ❌ No → Move to Phase 2

**Will this work in production?**
- ✅ Yes → Build it properly
- ❌ No → Fix architecture

**Examples:**

- "Should we add user accounts?" → No (Phase 2). MVP uses anonymous sessions.
- "Should we support mobile?" → No (Phase 2). MVP is web-only.
- "Should we add voice interaction?" → No (Phase 2). MVP is text-only.
- "Should we implement multi-level RAG?" → Yes (critical for accurate answers).
- "Should we test the RAG pipeline?" → Yes (non-negotiable quality requirement).

---

## What Makes This Special

### Not Just Another Chatbot

**We're building:**
- Autonomous AI Agent (real intelligence, not scripted responses)
- Multi-level context awareness (4-level RAG)
- Deep personalization (learns student's patterns)
- Production-quality system (fast, reliable, tested)
- Foundation for expansion (extensible architecture)

**Not building:**
- Simple Q&A bot
- Generic responses
- Prototype/demo
- One-off project

### Smart MVP Scope

**Including:**
- Core value: AI help while reading
- Technical excellence: Multi-level RAG
- Quality: Production-ready (tested, fast, reliable)
- Foundation: Expandable architecture (Phase 2 ready)

**Excluding (for now):**
- Additional interfaces (standalone chat)
- Advanced features (voice, mobile, analytics)
- Nice-to-haves (user accounts, social features)
- Unproven concepts (experimental AI techniques)

---

## Governance

### Amendment Process

**This constitution can be amended when:**
- Core principles need revision (e.g., timeline changes, scope adjustments)
- New constraints discovered (e.g., technical limitations)
- Team agrees change is necessary (consensus required)

**Amendment requires:**
1. Document proposed change and rationale
2. Team review and discussion
3. Approval from majority of team
4. Update this constitution with new version
5. Migration plan if changes affect existing work

### Compliance

**All PRs and reviews MUST verify:**
- Adheres to core principles (especially "Ship Fast, Ship Well")
- Complexity is justified (see Decision Framework)
- Tests are included (see "Test Everything")
- Documentation is updated (see "Quality First")

**Constitution supersedes:**
- Individual preferences
- Undocumented practices
- Verbal agreements

**When in doubt:**
- Refer to this constitution
- Ask the team
- Prioritize learning quality and production readiness

### Versioning

**Version increments:**
- **MAJOR** (X.0.0): Backward incompatible changes (e.g., principle removal)
- **MINOR** (1.X.0): New principle/section added
- **PATCH** (1.0.X): Clarifications, typo fixes, non-semantic refinements

**Current version reflects:**
- Initial creation (1.0.0)
- All 6 core principles defined
- Complete governance structure

---

## Our Commitment

**We will:**
- Ship production-ready MVP in 4 weeks
- Build autonomous AI Agent properly (OpenAI Agents SDK + ChromaDB)
- Implement multi-level RAG correctly (4 levels, tested)
- **Follow TDD religiously** (test-first for EVERY feature)
- Test everything thoroughly (unit, integration, behavior, scenario)
- Document the system well (code, API, deployment)
- Make students learn better (measure success via user feedback)
- **Verify agent teaches well** (not just "works")

**We will not:**
- Rush and ship broken code
- Skip testing or write tests after code
- Write code without tests first
- Ignore user needs
- Over-engineer the MVP
- Miss the deadline
- Ship agent without verifying teaching quality

---

## Success Vision

### 4 Weeks from Now

**We'll have:**
- ChatKit on all book pages ✅
- Autonomous Agent working ✅
- Multi-level RAG accurate ✅
- Sessions persisting ✅
- Students using it ✅
- Positive feedback ✅
- Ready for Phase 2 ✅

**Students will say:**
- "This helped me understand the book"
- "I could ask questions anytime"
- "The explanations were clear"
- "I completed more lessons"
- "I'm recommending this to others"

**Then we'll add:**
- Conversational learning mode (Phase 2)
- More features (Phase 3)
- More books (expand content)
- Scale up (support 1000s of students)

---

## Living Constitution

**This document guides us through MVP.**

**Core principles stay:**
- Ship fast, ship well
- Context is everything
- Agent is autonomous
- Students learn better with help
- Personalization through observation
- Production quality from day one

**Review before every sprint.**
**Update as we learn.**
**Always serve the student.**

---

**Version**: 1.0.0 | **Ratified**: 2025-11-07 | **Last Amended**: 2025-11-07

---

*"The best teacher is always available. We're building that teacher. Starting with the sidebar."*
