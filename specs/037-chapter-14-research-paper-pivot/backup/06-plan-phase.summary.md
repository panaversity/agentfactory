### Core Concept
/sp.plan combines RESEARCH (discovering actual Gemini capabilities, Playwright MCP patterns, failure modes) with ARCHITECTURE (designing solution strategy around discovered constraints). Plans grounded in research are realistic; plans based on assumptions fail. Architecturally significant decisions (session persistence, timeout strategy, retry logic) warrant ADR documentation for future reference.

### Key Mental Models
- **Research First, Design Second**: Don't assume tool capabilities—discover what Gemini actually does in 2025, what Playwright MCP actually supports. Design architecture around discovered reality
- **Cascade Quality Effect**: Clear spec + thorough research → detailed realistic plan. Vague spec or skipped research → vague or unrealistic plan
- **Decision Impact = ADR Worthy**: Document decisions that have long-term consequences (affects error handling, reliability, all future features), not tactical details (retry delay milliseconds)

### Critical Patterns
- **Research Gemini Reality**: Maximum duration limits, output format options, free tier rate limits, session timeout behavior, error modes
- **Identify Architectural Decisions**: Session persistence (one connection vs fresh each time?), timeout strategy (30/90/180 seconds?), retry logic (how many attempts?)
- **Tradeoff Analysis**: Each decision has competing concerns—persistent sessions are faster but fragile; fresh sessions are reliable but slow

### Common Mistakes
- Planning without research—designing for theoretical capabilities instead of actual constraints leads to implementation surprises
- Over-engineering before constraints discovered—designing for 1000s of concurrent generations before learning Gemini free tier can't handle that
- Ignoring discovered error modes—research finds failure scenarios that implementation must handle; ignoring them creates rework

### Connections
- **Builds on**: Clarified video specification that identifies all requirements (Lesson 5)
- **Leads to**: Breaking implementation plan into atomic tasks with /sp.tasks (Lesson 7)
