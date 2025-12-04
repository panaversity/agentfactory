### Core Concept
Specifications have four core sections (Intent, Success Criteria, Constraints, Non-Goals) and specifications come FIRST in SDD, with code as the output (not input) of clear intent.

### Key Mental Models
- **Evals-First Principle**: Define success criteria BEFORE implementing, creating a contract between intent and validation (forces clarity, enables independent verification, survives iteration)
- **Four-Section Anatomy**: Intent (problem solved) → Success Criteria (testable outcomes) → Constraints (non-negotiables) → Non-Goals (intentionally deferred) = complete specification
- **Specification Quality**: Not about length or formality, about clarity (testable language), completeness (all edge cases), and testability (can write automated tests from criteria)
- **Memory Banks vs Specs**: Memory banks are permanent organizational knowledge (security rules, tech stack); specs are temporary feature blueprints (password reset system, CSV exporter)

### Critical Patterns
- **Good vs Bad Language**: "System should handle errors gracefully" (bad) vs "Raises FileNotFoundError if file missing" (good); "Make it fast" (bad) vs "Response time <200ms for 95th percentile at 1000 req/sec" (good)
- **Quality Checklist**: Clear Intent? (answers "why") → Testable Criteria? (could write tests) → Explicit Constraints? (edge cases answered) → Prevents Scope Creep? (non-goals defer features) → No Ambiguity? (two developers implement identically)
- **Common Pitfalls**: Vague language, missing edge cases, over-specification (premature optimization), conflating spec with implementation (WHAT vs HOW), forgetting rationale

### AI Collaboration Keys
- Specs remove ambiguity about success, letting AI verify if implementation matches intent independently
- AI helps refine specs by asking "what about [edge case]?" questions, but spec quality gate happens BEFORE implementation

### Common Mistakes
- Assuming "obvious" edge cases (division by zero, empty input, unicode) don't need explicit specification
- Over-constraining implementation details (specifying algorithm instead of intent)
- Writing success criteria that are subjective ("looks good") instead of testable ("44x44px touch targets")

### Connections
- **Builds on**: Lesson 1 (Why vague specs fail), Lesson 2 (Why specs matter now)
- **Leads to**: Lesson 4 (Building first spec collaboratively with AI)
