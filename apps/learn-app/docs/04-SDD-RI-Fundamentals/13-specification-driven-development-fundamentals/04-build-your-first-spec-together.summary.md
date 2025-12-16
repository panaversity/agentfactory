### Core Concept
Specifications improve through collaborative iteration: you bring domain knowledge/intent, AI brings systematic thinking/edge cases, together you create specs neither could write independently.

### Key Mental Models
- **User Stories First (WHY)**: "As a [user], I want [action], so that [benefit]" format forces user-centric thinking before technical specs
- **Acceptance Criteria (WHAT)**: "GIVEN [context] WHEN [action] THEN [outcome]" format makes success testable with concrete examples
- **Edge Cases ARE Specification**: Happy path (2+3=5) is obvious; real spec is in boundary conditions (0.1+0.2 floating point, division by zero, type preservation)
- **Quality Iteration Loop**: Draft spec → Identify gaps with AI → Refine constraints → Validate AI understanding → Code generation → Validation reveals spec gaps → Refine and regenerate

### Critical Patterns
- **Spec-First Workflow**: User stories → acceptance criteria → edge cases → complete spec → code generation → validation → refinement (not code-first with specs written later)
- **Progressive Complexity**: Addition (simple) → subtraction (order matters) → multiplication (zero special) → division (error handling) reveals different specification decisions
- **Three Implementation Outcomes**: (1) Clear spec → clean code; (2) Spec gaps → AI asks questions (good! Fix spec before code); (3) Ambiguous spec → AI makes wrong assumptions (bad! Reveals ambiguity)
- **Convergence at Spec Level**: You don't iterate code wildly; you iterate specification until clear, THEN code implementation becomes straightforward

### AI Collaboration Keys
- AI asks clarifying questions during spec-writing (type preservation rules? division by zero behavior?) revealing gaps when they're cheap to fix
- AI identifies vagueness: "I would assume X, but I'm not sure if the spec intends that"
- AI suggests test scenarios covering edge cases you might not think of independently

### Common Mistakes
- Skipping user story phase (jumping to technical constraints)
- Leaving edge cases ambiguous ("handle division by zero" without specifying exception vs return value)
- Not validating spec against AI understanding before generating code
- Iterating code when problem is ambiguous specification

### Connections
- **Builds on**: Lessons 1-3 (Why specs matter, anatomy of specs), understanding spec structure and quality principles
- **Leads to**: Advanced SDD work (multi-module systems, integration specifications, architecture specifications)
