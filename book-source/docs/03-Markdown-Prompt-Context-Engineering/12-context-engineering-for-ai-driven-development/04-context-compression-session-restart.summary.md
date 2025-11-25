## Core Concept
Context compression extracts key architectural decisions and progress into checkpoints under 600 tokens, enabling session restart with clean context while preserving essential session intelligence.

## Key Mental Models
- **Checkpoint as compression filter**: Preserves decisions and constraints (permanent influence), discards dialogue and exploration (conversational noise)
- **Session restart as context reclamation**: Restoring ~500 tokens of checkpoint frees ~170K tokens of conversation history for new work
- **Checkpoint density**: Under 600 tokens requires ruthless elimination of redundancy; every line must be actionable for work resumption
- **Decisions with reasoning**: Store WHY choices were made (constraints, alternatives rejected), not just WHAT was decided

## Critical Patterns
- **What to keep**: Architectural decisions, non-obvious constraints, current working state, deferred items, discovered edge cases
- **What to discard**: Conversational timestamps, rejected alternatives, obvious implementation details, standard workflow steps
- **Compression iteration**: Initial draft comprehensive (720 tokens), refined through feedback to identify essentials (500 tokens)
- **Checkpoint trigger**: Create at 80%+ utilization OR when degradation symptoms detected, not after they become severe

## AI Collaboration Keys
- AI generates comprehensive checkpoint draft; you identify compression targets through feedback
- AI analyzes what's redundant (obvious from code, standard practice) versus essential (non-obvious constraints)
- AI validates compressed checkpoint preserves enough context to resume work in fresh session
- AI restores context in new session to verify checkpoint success

## Common Mistakes
- Creating oversized checkpoints (wasteful, defeats compression purpose)
- Removing context that seems obvious but isn't in code (design reasoning, constraints that shouldn't be forgotten)
- Waiting too long to checkpoint (degradation makes extraction harder and decisions harder to identify)
- Creating checkpoint for wrong session boundary (checkpoint should capture completed decision milestones, not mid-task)

## Connections
- **Builds on**: Lessons 1-3 (recognizing degradation, preventing it through loading, identifying when prevention insufficient)
- **Leads to**: Lesson 5 (isolating unrelated tasks; checkpoints enable managing multiple parallel sessions)
