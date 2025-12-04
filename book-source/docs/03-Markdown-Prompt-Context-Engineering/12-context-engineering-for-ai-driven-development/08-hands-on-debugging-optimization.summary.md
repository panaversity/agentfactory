### Core Concept
Context engineering is a diagnostic discipline: recognize degradation symptoms → identify root cause → select appropriate remediation strategy (progressive loading, compression, isolation, persistence) → measure results. Real-world application requires systematic debugging under pressure, not just theoretical knowledge.

### Key Mental Models
- **Symptom-to-cause mapping**: Each degradation type has characteristic symptoms (Lesson 2) that map to specific root causes
- **Utilization-driven decision logic**: Different strategies apply at different utilization levels (70% checkpoint threshold, 85% crisis threshold)
- **Strategy layering**: Single technique solves single problem; complex scenarios require combining strategies (compression + restart, isolation + progressive loading)
- **Validation through metrics**: Before/after utilization, response quality, symptom resolution confirm remediation effectiveness
- **Integration exercise patterns**: 5-day multi-feature sprints require choreographing all 7 lessons (tool selection Day 1, persistence Day 2, isolation Days 3-4, compression as needed)

### Critical Patterns
- **Four scenario archetypes**: High utilization crisis (85%+) → checkpoint, Context pollution (mixed domains) → isolate, Lost intelligence (new session) → load memory files, Saturation (exhausted budget early) → progressive loading
- **Diagnostic framework execution**: Observe symptoms (Lesson 2) → identify root cause → map to lesson → apply strategy → validate with metrics
- **Checkpoint protocol**: Extract decisions + progress summary + next steps < 600 tokens, save as CHECKPOINT.md, restart session with checkpoint loaded
- **Isolation trigger**: Semantic similarity < 50% indicates task domains sufficiently different to warrant separate sessions
- **Multi-day orchestration**: Day 1 tool selection + memory setup, Days 2-4 feature work with persistence + isolation, Day 5 integration with full context

### AI Collaboration Keys
- Checkpoints help AI recover decision context when session memory fades (compression addresses AI's working memory limitation)
- Isolation prevents AI from confusing patterns across domains (reduces pattern cross-contamination)
- Memory files allow AI to reconstruct project history across multi-day sprints
- Degradation is observable (shorter responses, generic suggestions, forgotten constraints) → can be diagnosed collaboratively

### Common Mistakes
- Recognizing symptoms but not identifying root cause (treating symptom rather than disease)
- Applying one strategy everywhere (compression solves high utilization, but isolation solves pollution—different problems)
- Not measuring remediation effectiveness (assume strategy worked without validating before/after metrics)
- Skipping memory file loading in new sessions (persistence exists but isn't used)
- Over-loading context early (saturation) rather than progressive discovery

### Connections
- **Builds on**: All Lessons 1-7 (diagnosis requires recognizing degradation, understanding all remediation strategies)
- **Leads to**: Lesson 9 (capstone orchestrates these diagnostic and remediation strategies into automated system)
- **Integration point**: Practical validation of all context engineering techniques in realistic development scenarios
- **Skill application**: Each scenario trains diagnostic reasoning for production use (emergency bugs, multi-day features, large codebases)
