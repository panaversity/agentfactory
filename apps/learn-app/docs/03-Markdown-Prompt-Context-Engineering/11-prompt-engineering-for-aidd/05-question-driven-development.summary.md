### Core Concept
Transform AI from making assumptions to asking clarifying questions upfront, preventing 10-15 iterations of corrections by capturing requirements explicitly.

### Key Mental Models
- **Assumption Discovery**: Every underspecified prompt forces AI to guess (60% per assumption × 5 assumptions = 7.8% all correct)
- **QDD Workflow**: Questions → Answers → Specification → Validation → Implementation (prevents implementation errors)
- **Specification Checkpoint**: Validate AI's understanding BEFORE wasted implementation effort
- **Question Categories**: Structure questions around audience, constraints, format, success criteria, non-goals (prevents trivial questions)

### Critical Patterns
- **Meta-Prompt Structure**: "Before implementing, ask 5-8 clarifying questions about [categories]" (activates QDD)
- **Specification Generation**: Convert answers into structured spec document that AI generates FROM
- **Three Roles in QDD**: AI teaches (suggests questions you didn't consider), AI learns (adapts to your answers), AI co-creates (converges on complete spec)
- **Task-Specific QDD**: Different task types (bugs, refactoring, features) use same QDD framework with different question categories

### AI Collaboration Keys
- Prompt AI to ask YOU questions FIRST (prevents wrong assumptions)
- AI learns your project-specific constraints through your answers
- Convergence through iteration (your clarifications → AI's refined spec → validation → implementation)
- AI acts as requirements analyst extracting hidden constraints you didn't articulate

### Common Mistakes
- Asking AI for too many questions (20+ → exhaustion, shallow answers)
- Not providing question categories (AI asks obvious/trivial questions)
- Skipping specification validation (misinterpretations caught after implementation, not before)
- Answering vaguely (broad audience → broad documentation instead of targeted)

### Connections
- **Builds on**: Specification primacy (prompts are specs), prompt anatomy (intent/constraints/success criteria)
- **Leads to**: Reusable templates (encode QDD workflows), template selection (decision frameworks apply QDD logic)
