### Core Concept
Write peer-reviewable specification (not implementation) for Prompt Template Library tool that orchestrates prompt engineering principles into reusable, discoverable system—demonstrating specification as primary AI-native skill.

### Key Mental Models
- **Specification-First Thinking**: Define success criteria and features BEFORE describing implementation (prevents scope creep)
- **Measurable Success**: Every success criterion quantifiable (time, accuracy %, adoption %, quality metrics) not vague
- **User Workflows**: Describe HOW people use tool in concrete scenarios (template discovery → filling → validation → usage feedback)
- **Peer-Reviewable Spec**: Document complete enough that developer could implement without clarification questions
- **Framework Integration**: Specification demonstrates Anthropic (step-by-step), Google (structured prompts), OpenAI (constraint-based), Zia (8-Element) frameworks

### Critical Patterns
- **Success Criteria**: 5-7 measurable outcomes (discovery speed <30sec, selection accuracy >85%, completeness >90%, adoption >80%)
- **Core Features**: 5-7 capabilities (storage, discovery, interactive filling, validation, analytics, evolution)
- **User Workflows**: 3-4 scenarios showing concrete interactions (find template, fill template, refine template, browse templates)
- **Validation Tests**: 8-10 acceptance tests covering all features with specific setup→action→result
- **Open Questions**: Areas needing decisions (algorithm approach, validation strictness, sharing mechanism, privacy level)

### AI Collaboration Keys
- Tool automates question-driven development (captures QDD workflows into guided interaction)
- Template recommendations use AI to match task description to appropriate template
- Interactive filling provides real-time validation and guidance (prevents incomplete/vague templates)
- Usage analytics feed back into template evolution (AI learns which templates effective)

### Common Mistakes
- Success criteria too vague ("users are happy") instead of measurable (discovery <30sec, accuracy >85%)
- Missing feature details (listed without examples/workflows/commands)
- Workflows incomplete or missing scenarios (don't cover key user journeys)
- Validation tests too generic (not specific enough to actually verify behavior)
- No open questions (indicates incomplete thinking, not realistic decision-making)

### Connections
- **Builds on**: Question-Driven Development (tool implements QDD), reusable templates (tool manages templates), template selection (tool encodes decision frameworks)
- **Integration**: Demonstrates Layer 4 (Spec-Driven Integration) by writing spec first, then would implement later with accumulated intelligence
- **Capstone Role**: Orchestrates ALL Chapter 11 concepts into single specification, showing mastery of prompt engineering as specification skill
