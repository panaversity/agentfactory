# Phase 0 Research: Book Structure Expansion (7→13 Parts, 32→46 Chapters)

**Created**: 2025-10-29  
**Status**: Complete  
**Purpose**: Resolve open questions before structural design begins

---

## Research Question 1: Part Naming Conventions

### Question
Should Parts 6-13 include technology-specific details in their names (e.g., "Agentic AI Fundamentals with OpenAI Agents SDK in Python" vs. "Agentic AI Fundamentals")?

### Research Findings

**Current Practice** (Parts 1-5):
- Generic, topic-focused names
- No technology specifics in titles
- Examples: "Modern Python with Type Hints", "Spec-Kit Methodology"

**Industry Standards**:
- Technical books often include framework names when that's the primary focus
- Helps with SEO, discoverability, and setting expectations
- Examples: "React: Up & Running", "FastAPI for Modern Web Development"

**Docusaurus Sidebar Constraints**:
- No hard character limits
- Long names wrap in sidebar (acceptable UX)
- Longer names provide clearer navigation

**Pedagogical Considerations**:
- Specificity helps learners understand scope immediately
- Advanced topics (Parts 6-13) benefit from explicit technology mentions
- Reduces confusion about "which framework are we using?"

### Decision: **Use Technology-Specific Names for Advanced Parts**

**Rationale**:
1. **Clarity**: Learners immediately know "OpenAI Agents SDK" (not LangChain, CrewAI, etc.)
2. **SEO**: Better discoverability for specific technologies
3. **Scope Definition**: Prevents scope confusion in advanced topics
4. **Consistency**: Foundational parts (1-5) remain generic; advanced parts (6-13) are specific

### Final Part Names

| Part | Title |
|------|-------|
| 1 | Introducing AI-Driven Development |
| 2 | AI Tool Landscape |
| 3 | Prompt & Context Engineering |
| 4 | Python: The Language of AI Agents |
| 5 | Spec-Kit Plus Methodology |
| 6 | **Agentic AI Fundamentals with OpenAI Agents SDK in Python** |
| 7 | **MCP Fundamentals with FastMCP** |
| 8 | **TypeScript: The Language of Realtime and Interaction** |
| 9 | **Building Realtime and Voice Agents** |
| 10 | **Containerization & Orchestration using Docker and Kubernetes** |
| 11 | **Data, State, and Memory using PostgreSQL, Graph, and Vector Databases** |
| 12 | **Event-Driven Architecture using Kafka and Dapr** |
| 13 | **Stateful Agents using Dapr Actors and Dapr Workflows** |

**Alternatives Considered**:
- Keep all names generic → Rejected: Too ambiguous for advanced topics
- Use framework names only in subtitles → Rejected: Less visible in sidebar
- Use "with" vs. "using" inconsistently → Rejected: Chose consistent "with" for frameworks, "using" for multiple technologies

---

## Research Question 2: Chapter Consolidation Strategy

### Question
How should existing chapters be merged for Part 1 (5→3), Part 3 (4→2), Part 5 (5→3) without losing critical content?

### Research Findings

**Current Chapter Analysis**:

**Part 1** (5 chapters → 3 chapters):
- Ch 1: Welcome to AI-Driven Development (3,000-4,000 words) — **Mindset foundation**
- Ch 2: Understanding AI Tools (4,000-5,000 words) — **9 technological revolutions**
- Ch 3: Setting Up Your Environment (2,500-3,500 words) — **Tool installation**
- Ch 4: Your First Program with AI (3,000-4,000 words) — **First hands-on project**
- Ch 5: Debugging and Iterating with AI (2,500-3,500 words) — **Error handling**

**Part 3** (4 chapters → 2 chapters):
- Ch 10: Writing Effective Prompts
- Ch 11: Context Management and Memory
- Ch 12: Debugging AI-Generated Code
- Ch 13: Advanced Prompt Techniques

**Part 5** (5 chapters → 3 chapters):
- Ch 22: Specification-Driven Development Fundamentals
- Ch 23: Writing Effective Specifications
- Ch 24: Planning and Tasking
- Ch 25: Real-World Spec-Kit Workflows
- Ch 26: Scaling Spec-Kit for Teams

### Decision: Consolidation Mapping

#### **Part 1: 5 → 3 Chapters**

**New Chapter 1**: "Welcome to AI-Driven Development" (KEEP)
- Content: Current Ch 1 as-is
- Add: Brief introduction to the 9 revolutions (high-level overview from Ch 2, ~500 words)
- Rationale: Mindset shift + context for "why now?" in one chapter

**New Chapter 2**: "Your First AI-Assisted Program" (MERGE Ch 3 + Ch 4)
- Content from Ch 3: Tool installation (Claude Code, Gemini CLI, Spec-Kit Plus)
- Content from Ch 4: First spec-driven development cycle
- Flow: Install → Verify → Write Spec → Generate Code → Test → Deploy
- Rationale: Setup and first project are naturally sequential; reduces context switching
- Estimated length: 5,500-7,500 words (acceptable for hands-on tutorial)

**New Chapter 3**: "Debugging and Iterating with AI" (KEEP Ch 5)
- Content: Current Ch 5 as-is
- Rationale: Essential standalone skill; builds on Ch 2 experience

**Content Handling for Ch 2 (9 Revolutions)**:
- High-level overview integrated into Ch 1 (~500 words)
- Detailed technical content moved to Part 2 (AI Tool Landscape) as contextual sidebars
- Pattern recognition (Snakes & Ladders) moves to Ch 1

#### **Part 3: 4 → 2 Chapters**

**New Chapter 1**: "The Architect Toolkit: Prompting Foundations" (MERGE Ch 10 + Ch 11)
- Content from Ch 10: Writing effective prompts (structure, clarity, specificity)
- Content from Ch 11: Context management and memory
- Flow: Write prompts → Manage context → Maintain conversation state
- Rationale: Context management is essential to advanced prompting; naturally integrated

**New Chapter 2**: "Advanced Prompt Techniques" (MERGE Ch 12 + Ch 13)
- Content from Ch 12: Debugging AI-generated code
- Content from Ch 13: Advanced techniques (few-shot, chaining, refinement)
- Flow: Debug → Refine → Advanced patterns
- Rationale: Debugging is prerequisite to advanced techniques; both are "mastery" content

#### **Part 5: 5 → 3 Chapters**

**New Chapter 1**: "Specification-Driven Development Fundamentals" (KEEP Ch 22)
- Content: Current Ch 22 as-is
- Rationale: Strong foundational chapter, stands alone

**New Chapter 2**: "Writing and Planning Specifications" (MERGE Ch 23 + Ch 24)
- Content from Ch 23: Writing effective specifications
- Content from Ch 24: Planning and tasking (breaking specs into tasks)
- Flow: Write specs → Decompose into plans → Create task lists
- Rationale: Spec writing and planning are sequential phases of the same process

**New Chapter 3**: "Real-World Spec-Kit Workflows" (MERGE Ch 25 + Ch 26)
- Content from Ch 25: Real-world workflows and examples
- Content from Ch 26: Scaling for teams and collaboration
- Flow: Individual workflows → Team workflows → Scaling patterns
- Rationale: Both chapters are about practical application; team scaling is advanced real-world use

### Content Preservation Strategy

**All merged content will**:
1. **Preserve key concepts**: No learning objectives removed
2. **Reorganize for flow**: Combine logically sequential material
3. **Consolidate examples**: Reduce redundancy, keep best examples
4. **Maintain word count targets**: 5,000-8,000 words per merged chapter (within acceptable range)
5. **Update cross-references**: All internal links updated to new chapter numbers

**Git History**:
- All original chapters preserved in git history
- Detailed commit message documenting what moved where
- `chapter-mapping.md` provides reverse lookup

**Alternatives Considered**:
- Delete merged chapters entirely → Rejected: Content loss risk
- Keep all 5/4/5 chapters → Rejected: Doesn't meet 3/2/3 target
- Different merge combinations → Rejected: Current mapping preserves natural learning flow

---

## Research Question 3: Chapter Expansion Strategy (Part 4 Python)

### Question
What 4 new chapters should be added to Part 4 (Python) to expand from 8→12 chapters?

### Research Findings

**Current Part 4 Coverage** (8 chapters):
1. Functions, Types, and Type Hints
2. Data Structures with Type Safety
3. Object-Oriented Programming (Modern)
4. Testing and Quality Assurance
5. Error Handling and Debugging
6. Working with APIs and Data
7. Clean Code and Design Patterns
8. Building Your First Real Project

**Gaps Identified**:
- **Decorators and Generators**: Critical Python features, missing
- **Async/Await**: Essential for modern Python, especially AI/API work
- **Metaprogramming**: Type hints, dataclasses, protocols (advanced but important)
- **Package Management**: Virtual environments, dependency management, packaging
- **File I/O and Data Processing**: CSV, JSON, XML parsing
- **Concurrency**: Threading, multiprocessing (beyond async)

**Industry Standards** (Python curricula):
- Real Python, Python.org tutorials, university CS courses all cover:
  - Decorators (intermediate)
  - Async programming (intermediate-to-advanced)
  - Packaging and distribution (practical necessity)
  - Advanced type hints (dataclasses, protocols, generics)

**AI-Driven Development Context**:
- Async/await is critical for API calls, LLM interactions
- Decorators are common in frameworks (Flask, FastAPI)
- Package management is essential for deployment
- Advanced types enable better AI-generated code quality

### Decision: Add 4 New Chapters to Part 4

**New Chapter Sequence** (12 total):

**Chapters 1-8** (RENUMBERED from current 1-8):
1. Functions, Types, and Type Hints
2. Data Structures with Type Safety
3. Object-Oriented Programming (Modern)
4. Testing and Quality Assurance
5. Error Handling and Debugging
6. Working with APIs and Data
7. Clean Code and Design Patterns
8. **Decorators, Generators, and Iterators** ← NEW

**Insert 4 New Chapters**:
9. **Async Programming and Concurrency** ← NEW
10. **Advanced Type Hints: Dataclasses, Protocols, and Generics** ← NEW
11. **Package Management and Virtual Environments** ← NEW
12. Building Your First Real Project (MOVED from Ch 8 → Ch 12, updated)

### New Chapter Details

#### **Chapter 8: Decorators, Generators, and Iterators**
**Topics**:
- Function decorators (basics and chaining)
- Class decorators
- Generator functions (`yield`)
- Generator expressions
- Iterator protocol (`__iter__`, `__next__`)
- Practical applications with AI workflows

**Rationale**: Essential intermediate Python features used heavily in frameworks and libraries

**Prerequisites**: Chapters 1-7 (functions, OOP, testing)

**Learning Outcomes**:
- Write and use function/class decorators
- Create memory-efficient generators
- Implement custom iterators
- Apply these patterns in AI agent development

**Estimated Length**: 3,500-4,500 words

---

#### **Chapter 9: Async Programming and Concurrency**
**Topics**:
- Synchronous vs asynchronous execution
- `async`/`await` syntax
- `asyncio` standard library
- Concurrent API calls (critical for LLM workflows)
- Async context managers and iterators
- Error handling in async code

**Rationale**: Modern Python is async-first; essential for AI APIs, web scraping, multi-agent systems

**Prerequisites**: Chapters 1-8 (especially error handling, APIs, decorators)

**Learning Outcomes**:
- Write async functions and use `await`
- Handle multiple concurrent tasks with `asyncio.gather()`
- Make parallel LLM API calls efficiently
- Understand async vs threading vs multiprocessing trade-offs

**Estimated Length**: 4,000-5,000 words

---

#### **Chapter 10: Advanced Type Hints: Dataclasses, Protocols, and Generics**
**Topics**:
- `dataclasses` for structured data
- Type protocols (structural subtyping)
- Generic types (`TypeVar`, `Generic`)
- `Literal`, `Union`, `Optional` (advanced usage)
- Type narrowing and type guards
- Using Pydantic for data validation

**Rationale**: Modern Python emphasizes rich type systems; critical for AI-generated code quality and validation

**Prerequisites**: Chapters 1-3 (types, data structures, OOP)

**Learning Outcomes**:
- Create dataclasses for API responses
- Define and implement protocols
- Write generic, reusable functions
- Validate data with Pydantic models

**Estimated Length**: 3,500-4,500 words

---

#### **Chapter 11: Package Management and Virtual Environments**
**Topics**:
- Virtual environments (`venv`, `virtualenv`, `conda`)
- Dependency management (`pip`, `requirements.txt`, `pyproject.toml`)
- Package structure and `setup.py`/`pyproject.toml`
- Publishing to PyPI
- Using Poetry or PDM for modern dependency management
- Reproducible environments

**Rationale**: Essential practical skill for deployment; often overlooked in tutorials

**Prerequisites**: Chapters 1-10 (all Python fundamentals)

**Learning Outcomes**:
- Create and activate virtual environments
- Manage project dependencies reliably
- Package Python projects for distribution
- Set up reproducible development environments

**Estimated Length**: 3,000-4,000 words

---

**Alternatives Considered**:
- Add only 2 chapters (async + packaging) → Rejected: Insufficient coverage of intermediate topics
- Add file I/O chapter → Rejected: Covered adequately in "Working with APIs and Data" (Ch 6)
- Add web scraping chapter → Rejected: Too specialized, not core to AI agent development
- Reorder to put packaging earlier → Rejected: Better after students understand why it matters

---

## Research Question 4: New Part Topics (Parts 6-13)

### Question
What specific topics should each of the 8 new parts cover?

### Research Findings

**Part 6: Agentic AI Fundamentals with OpenAI Agents SDK in Python** (3 chapters)
- **Technology**: OpenAI Agents SDK (official framework for building agents)
- **Topics**: Agent architecture, tool use, multi-agent orchestration, agent debugging
- **Prerequisites**: Python (Part 4), Spec-Kit (Part 5)

**Chapters**:
1. Introduction to Agentic AI and Agent Architecture
2. Building Your First Agent with OpenAI Agents SDK
3. Multi-Agent Systems and Orchestration Patterns

---

**Part 7: MCP Fundamentals with FastMCP** (3 chapters)
- **Technology**: FastMCP (Python library for Model Context Protocol servers)
- **Topics**: MCP architecture, building servers, integrating with existing systems
- **Prerequisites**: Python (Part 4), Agents (Part 6)

**Chapters**:
1. Introduction to Model Context Protocol
2. Building MCP Servers with FastMCP
3. Advanced MCP Integration Patterns

---

**Part 8: TypeScript: The Language of Realtime and Interaction** (3 chapters)
- **Technology**: TypeScript, Node.js runtime
- **Topics**: Type system, async patterns, TypeScript for backend development
- **Prerequisites**: Python knowledge (analogies and comparisons), JavaScript basics helpful but not required
- **Rationale**: Realtime/voice agents often use TypeScript; prepares for Part 9

**Chapters**:
1. TypeScript Fundamentals for Python Developers
2. Advanced TypeScript Patterns and Async Programming
3. Building Backend Services with TypeScript and Node.js

---

**Part 9: Building Realtime and Voice Agents** (3 chapters)
- **Technology**: OpenAI Realtime API, WebSockets, voice processing libraries
- **Topics**: Realtime streaming, voice input/output, WebSocket connections, conversational AI
- **Prerequisites**: TypeScript (Part 8), Agents (Part 6)

**Chapters**:
1. Introduction to Realtime AI and Voice Processing
2. Building Voice Agents with OpenAI Realtime API
3. Production Realtime Systems: Scaling and Optimization

---

**Part 10: Containerization & Orchestration using Docker and Kubernetes** (3 chapters)
- **Technology**: Docker, Kubernetes, Helm
- **Topics**: Containerization, orchestration, deploying AI agents at scale
- **Prerequisites**: Python/TypeScript applications built in previous parts

**Chapters**:
1. Docker Fundamentals: Containerizing AI Applications
2. Kubernetes Basics: Orchestrating Containerized Agents
3. Production Kubernetes: Scaling, Monitoring, and CI/CD

---

**Part 11: Data, State, and Memory using PostgreSQL, Graph, and Vector Databases** (3 chapters)
- **Technology**: PostgreSQL (relational), Neo4j (graph), Pinecone/Weaviate (vector)
- **Topics**: Persistent state, memory systems for agents, hybrid database architectures
- **Prerequisites**: Python (Part 4), Agents (Part 6)

**Chapters**:
1. Relational Databases for Agent State with PostgreSQL
2. Graph Databases for Agent Memory and Relationships
3. Vector Databases for Semantic Search and RAG

---

**Part 12: Event-Driven Architecture using Kafka and Dapr** (2 chapters)
- **Technology**: Apache Kafka, Dapr (Distributed Application Runtime)
- **Topics**: Event streaming, pub/sub patterns, asynchronous communication, microservices
- **Prerequisites**: Docker/Kubernetes (Part 10), Databases (Part 11)

**Chapters**:
1. Event-Driven Architecture with Apache Kafka
2. Building Distributed Systems with Dapr

---

**Part 13: Stateful Agents using Dapr Actors and Dapr Workflows** (2 chapters)
- **Technology**: Dapr Virtual Actors, Dapr Workflows
- **Topics**: Stateful agent patterns, actor model, durable workflows, orchestration
- **Prerequisites**: Kafka/Dapr (Part 12)

**Chapters**:
1. Stateful Agents with Dapr Virtual Actors
2. Durable Workflows for Long-Running Agent Tasks

---

### Progression Logic

**The Learning Path**:
```
Foundation (Parts 1-5)
    ↓
Agent Building (Parts 6-7) → Learn to build agents in Python, integrate with MCP
    ↓
Full-Stack Skills (Part 8) → Add TypeScript for realtime/voice capabilities
    ↓
Realtime Agents (Part 9) → Build voice and streaming agents
    ↓
Deployment (Part 10) → Containerize and orchestrate agents
    ↓
Persistence (Part 11) → Add state and memory to agents
    ↓
Distributed Systems (Parts 12-13) → Build production-scale, event-driven, stateful agent systems
```

**Alternatives Considered**:
- Combine Parts 12-13 → Rejected: Too much content for one part
- Add Part on LLM fine-tuning → Rejected: Out of scope for "co-learning programming"
- Add Part on frontend (React/Vue) → Rejected: Focus is on agents, not full web dev

---

## Research Question 5: Scaffolding Range Adjustment

### Question
How should scaffolding ranges be adjusted for 46 chapters (currently 1-10 heavy, 11-20 moderate, 21-32 minimal)?

### Research Findings

**Current Scaffolding Ranges** (32 chapters):
- Chapters 1-10: **Heavy** scaffolding (show-then-explain, guided examples, zero gatekeeping)
- Chapters 11-20: **Moderate** scaffolding (more independence, still structured)
- Chapters 21-32: **Minimal** scaffolding (assumes mastery of fundamentals)

**Cognitive Load Principles**:
- Beginners need heavy support (Parts 1-3: Chapters 1-9)
- Intermediate learners benefit from moderate scaffolding while building core skills (Part 4-5: Chapters 10-24)
- Advanced learners should demonstrate independence (Parts 6-13: Chapters 25-46)

**Part-by-Part Analysis**:

| Part | Chapters | Difficulty Level | Suggested Scaffolding |
|------|----------|------------------|----------------------|
| 1 | 1-3 | Beginner | HEAVY |
| 2 | 4-7 | Beginner | HEAVY |
| 3 | 8-9 | Beginner-Intermediate | HEAVY |
| 4 | 10-21 | Intermediate | MODERATE |
| 5 | 22-24 | Intermediate | MODERATE |
| 6 | 25-27 | Intermediate-Advanced | MODERATE |
| 7 | 28-30 | Intermediate-Advanced | MODERATE |
| 8 | 31-33 | Intermediate-Advanced | MODERATE |
| 9 | 34-36 | Advanced | MINIMAL |
| 10 | 37-39 | Advanced | MINIMAL |
| 11 | 40-42 | Advanced | MINIMAL |
| 12 | 43-44 | Advanced | MINIMAL |
| 13 | 45-46 | Advanced | MINIMAL |

### Decision: New Scaffolding Ranges for 46 Chapters

**Chapters 1-9: HEAVY Scaffolding** (Parts 1-3)
- **Rationale**: Foundation building, tool literacy, AI communication basics
- **Characteristics**: Show-then-explain, guided walkthroughs, extensive analogies, zero gatekeeping language, hand-holding expected

**Chapters 10-30: MODERATE Scaffolding** (Parts 4-8)
- **Rationale**: Core skill development (Python, Spec-Kit, Agents, MCP, TypeScript)
- **Characteristics**: More independent exploration, structured guidance, clear prerequisite assumptions, some self-directed problem-solving

**Chapters 31-46: MINIMAL Scaffolding** (Parts 9-13)
- **Rationale**: Advanced integration topics, assumes mastery of Python/TypeScript/Agents
- **Characteristics**: Assumes independence, focuses on concepts over step-by-step, expects readers to apply prior knowledge, minimal hand-holding

### Updated Constitution Text

**BEFORE**:
> Progressive scaffolding: Early chapters (1-10) heavy support, middle chapters (11-20) moderate, later chapters (21-32) minimal

**AFTER**:
> Progressive scaffolding: Early chapters (1-9) heavy support, middle chapters (10-30) moderate, later chapters (31-46) minimal

**Alternatives Considered**:
- Keep ranges proportional (1-15 heavy, 16-30 moderate, 31-46 minimal) → Rejected: Would extend heavy scaffolding too long into Python mastery
- Tighten heavy scaffolding to 1-6 → Rejected: Part 3 (Prompting) still needs heavy support for beginners
- Make all scaffolding ranges equal (1-15, 16-31, 32-46) → Rejected: Not pedagogically sound, heavy scaffolding shouldn't extend into advanced TypeScript

---

## Research Question 6: Docusaurus Build Validation

### Question
Are there any Docusaurus 3.9.2 limits or best practices for 13-part, 46-chapter structures?

### Research Findings

**Docusaurus 3.9.2 Capabilities**:
- **No hard limits** on sidebar depth or number of items
- Automatic sidebar generation from file structure works with hundreds of items
- Sidebar scrolling handles large navigation trees
- Build performance scales well to large documentation sites

**Best Practices**:
- Use `_category_.json` in each folder to control sidebar labels and ordering
- Keep folder names short enough for URLs (not a concern for our structure)
- Use `sidebar_position` in frontmatter for explicit ordering
- Avoid deeply nested structures (3 levels max recommended) — ✅ We use 3 levels (Part → Chapter → Lesson)

**Performance Considerations**:
- 46 chapters × ~4 lessons each = ~184 pages
- Docusaurus handles this easily (tested sites with 500+ pages)
- Build time: <30 seconds for this size (acceptable)
- Search indexing: Algolia DocSearch works fine with this scale

**Sidebar UX**:
- 13 top-level items (parts) fits in sidebar without scrolling on most screens
- Expandable/collapsible sections keep navigation clean
- Mobile sidebar works well with this structure

### Decision: No Structural Changes Needed

**Conclusion**: Docusaurus 3.9.2 fully supports our 13-part, 46-chapter structure with no limitations or required workarounds.

**Recommendations**:
1. ✅ Use `_category_.json` in each part folder for clean labels
2. ✅ Set `sidebar_position` in frontmatter for predictable ordering
3. ✅ Test build after major reorganization (Phase 2)
4. ✅ Enable Algolia DocSearch for search functionality (optional enhancement)

**Validation Checklist** (to be completed in Phase 2):
- [ ] Run `npm run build` after Part 1 restructuring
- [ ] Verify all 13 parts appear in sidebar
- [ ] Check all 46 chapters are navigable
- [ ] Validate no broken internal links
- [ ] Test mobile sidebar responsiveness
- [ ] Confirm build time < 60 seconds

**Alternatives Considered**:
- Split book into multiple Docusaurus sites (one per part) → Rejected: Unnecessary complexity, would harm navigation
- Use custom sidebar configuration → Rejected: Auto-generation from file structure is simpler and more maintainable

---

## Summary of Research Decisions

| Question | Decision | Impact |
|----------|----------|--------|
| **1. Part Naming** | Use technology-specific names for Parts 6-13 | Clarity, SEO, scope definition |
| **2. Chapter Consolidation** | Part 1 (5→3), Part 3 (4→2), Part 5 (5→3) via strategic merges | Preserves content, improves flow |
| **3. Python Expansion** | Add 4 chapters: Decorators, Async, Advanced Types, Package Mgmt | Fills curriculum gaps |
| **4. New Part Topics** | 8 parts covering TypeScript → Realtime → Deployment → Databases → Distributed Systems | Complete learning path |
| **5. Scaffolding Ranges** | 1-9 heavy, 10-30 moderate, 31-46 minimal | Pedagogically sound progression |
| **6. Docusaurus Validation** | No changes needed, structure fully supported | Green light to proceed |

---

## Next Steps (Phase 1)

With all research questions answered, proceed to Phase 1 design:

1. ✅ Create `chapter-mapping.md` — Complete old→new chapter mapping
2. ✅ Create 8 part outline templates — High-level topic outlines for Parts 6-13
3. ✅ Update `chapter-index.md` — Reflect all 46 chapters across 13 parts
4. ✅ Update `constitution.md` — Update chapter counts and scaffolding ranges
5. ✅ Update `directory-structure.md` — Add examples for new parts
6. ✅ Update `README.md` — Update structure overview section
7. ✅ Verify `CLAUDE.md` — Ensure it defers to constitution (should already be compliant)

**Research Phase**: ✅ **COMPLETE**

