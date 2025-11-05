<!--
Sync Impact Report (Constitution Update):
- Version Change: [Template] → 1.0.0 (Initial constitution establishment)
- Modified Principles: N/A (Initial creation)
- Added Sections: 
  * Core Principles (7 principles established)
  * Technical Standards
  * Development Workflow
  * AI Assistant Feature Requirements
  * Governance
- Removed Sections: N/A
- Templates Requiring Updates:
  ✅ plan-template.md - Constitution Check section aligns with principles
  ✅ spec-template.md - Prioritized user stories align with Independent Deliverables
  ✅ tasks-template.md - Phase organization aligns with User Story Focus
- Follow-up TODOs: None
-->

# AI Native Software Development Book Constitution

## Core Principles

### I. Spec-Driven Development (NON-NEGOTIABLE)
Every feature MUST start with a specification in `/specs/<feature-name>/`. Specifications define:
- User scenarios with prioritized, independently testable stories (P1, P2, P3...)
- Clear acceptance criteria using Given-When-Then format
- Technical requirements and constraints
- Success metrics

**Rationale**: Specifications serve as executable contracts between humans and AI agents, ensuring shared understanding before implementation and enabling AI-driven code generation.

### II. AI-First Architecture
Features and components MUST be designed with AI agents as first-class consumers. This means:
- Clear, parseable APIs and interfaces
- Comprehensive documentation in markdown format
- Structured data formats (JSON, YAML) over unstructured
- Context-aware design that supports agent reasoning

**Rationale**: As an AI-native development book and framework, all systems must demonstrate best practices for human-AI collaboration.

### III. User Story Focus (Independent Deliverables)
Each user story MUST be:
- Independently implementable without requiring other stories
- Independently testable with clear success criteria
- Deliverable as a viable MVP increment
- Prioritized (P1 = critical, P2 = important, P3 = nice-to-have)

**Rationale**: Independent stories enable parallel development, incremental delivery, and clear demonstration of value at each stage.

### IV. Non-Intrusive User Experience
UI components and features MUST:
- Not interfere with primary content consumption
- Be collapsible/dismissible when not in use
- Load asynchronously without blocking main content
- Respect user preferences and state

**Rationale**: For educational content, the learning experience is paramount. Tools and assistants are enhancers, not replacements.

### V. Privacy-First & Performant
All features MUST:
- Minimize data collection (collect only what's necessary)
- Clearly document what data is processed and how
- Process sensitive data client-side when possible
- Optimize for performance (lazy loading, code splitting, caching)
- Target <100ms interaction response time for UI components

**Rationale**: Privacy and performance are non-negotiable for educational platforms. Users must trust the platform and have smooth experiences regardless of device capabilities.

### VI. Context-Aware Intelligence
AI-powered features MUST:
- Know the current page/chapter context
- Maintain conversation history within sessions
- Provide relevant suggestions based on user behavior
- Support text selection and highlighting for contextual help

**Rationale**: Generic AI responses provide limited value. Context-aware assistance dramatically improves learning effectiveness.

### VII. Technology Stack Consistency
The project uses a bilingual stack:
- **Python 3.13+** for reasoning, AI agents, and backend logic
- **TypeScript** for interactive frontend and UI components
- **Docusaurus** for documentation and book publishing
- **OpenAI Agents SDK / FastMCP** for agent integration
- **React** for UI components

All contributions MUST align with this stack unless explicitly justified.

**Rationale**: Consistency reduces cognitive load, improves maintainability, and demonstrates the bilingual approach taught in the book.

## Technical Standards

### Code Quality
- All code MUST pass linting (ESLint for TypeScript, Ruff for Python)
- Type safety enforced (TypeScript strict mode, Python type hints)
- Code coverage target: 80% for business logic, 60% for UI components
- Maximum function complexity: cyclomatic complexity ≤ 10

### Documentation Requirements
- Public APIs MUST have JSDoc/docstring comments
- README files required for all feature directories
- Architecture Decision Records (ADRs) for significant technical choices
- Inline comments for non-obvious logic

### Performance Budgets
- Initial page load: <3s on 3G connection
- Time to Interactive (TTI): <5s
- First Contentful Paint (FCP): <1.5s
- JavaScript bundle: <200KB (gzipped)
- API response time: p95 <200ms

### Security Standards
- No secrets in source code (use environment variables)
- Input validation on all user inputs
- Output sanitization for user-generated content
- HTTPS only in production
- Content Security Policy (CSP) headers configured

## Development Workflow

### Execution Flow
1. **Specification** (`/sp.spec`): Define user stories and requirements
2. **Planning** (`/sp.plan`): Research, architecture, and technical design
3. **Tasks** (`/sp.tasks`): Break down into testable implementation tasks
4. **Implementation**: Red-Green-Refactor cycle
5. **Documentation**: Update guides, ADRs, and PHRs

### Constitution Check Gates
All features MUST pass Constitution Check before implementation:
- ✅ Spec-driven: Feature has complete specification
- ✅ User story independence: Stories can be implemented separately
- ✅ Privacy-first: Data handling documented and minimized
- ✅ Performance budget: Resource usage estimated and acceptable
- ✅ Non-intrusive: UI design respects primary content
- ✅ Context-aware: AI features leverage page/chapter context

### Prompt History Records (PHRs)
MUST create PHR after every significant user interaction:
- Implementation work (code changes, new features)
- Planning and architecture discussions
- Debugging sessions
- Multi-step workflows

PHRs are routed automatically:
- `constitution` → `history/prompts/constitution/`
- Feature work → `history/prompts/<feature-name>/`
- General → `history/prompts/general/`

### Architecture Decision Records (ADRs)
Create ADR when decision meets ALL criteria:
- **Impact**: Long-term consequences (framework, data model, API, security)
- **Alternatives**: Multiple viable options considered
- **Scope**: Cross-cutting and influences system design

Suggest ADR creation but require user consent before creating.

## AI Assistant Feature Requirements

### Functional Requirements
The collapsible sidebar AI assistant MUST:
1. **Context Awareness**
   - Detect current page/chapter from URL and metadata
   - Access page content for relevant Q&A
   - Maintain conversation history within session

2. **Interaction Modes**
   - Answer questions about current page content
   - Explain highlighted text passages
   - Suggest related topics and chapters
   - Provide code explanations for examples

3. **User Experience**
   - Collapsible sidebar on right side (default collapsed)
   - Toggle button visible but non-intrusive
   - Persist open/closed state in localStorage
   - Responsive design (hide on mobile, show on tablet+)

4. **Performance**
   - Lazy load assistant component
   - Stream responses for better perceived performance
   - Cache common queries per page
   - Debounce user input (300ms)

### Non-Functional Requirements
- **Privacy**: No user data sent to servers without explicit consent
- **Accessibility**: Keyboard navigable, screen reader compatible
- **Compatibility**: Works with existing Docusaurus setup
- **Maintainability**: Clean separation from book content

## Governance

### Amendment Process
1. Identify proposed change and rationale
2. Determine version bump (MAJOR/MINOR/PATCH):
   - **MAJOR**: Backward incompatible principle changes or removals
   - **MINOR**: New principles or material expansions
   - **PATCH**: Clarifications, wording fixes, non-semantic changes
3. Update constitution with Sync Impact Report
4. Validate dependent templates and documentation
5. Commit with message: `docs: amend constitution to vX.Y.Z (<change summary>)`

### Compliance and Review
- All PRs MUST reference constitution principles in description
- Code reviews MUST verify Constitution Check compliance
- Quarterly constitution review for relevance and clarity
- Violations require explicit justification in ADR

### Versioning Policy
Version format: `MAJOR.MINOR.PATCH`
- Track changes in git history
- Link to ADRs for MAJOR/MINOR changes
- Maintain changelog in Sync Impact Report

**Version**: 1.0.0 | **Ratified**: 2025-11-05 | **Last Amended**: 2025-11-05
