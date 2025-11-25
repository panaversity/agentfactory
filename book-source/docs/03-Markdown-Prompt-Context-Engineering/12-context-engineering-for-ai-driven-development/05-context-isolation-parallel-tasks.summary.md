## Core Concept
Context isolation prevents pattern contamination by measuring task similarity (0-100 point scoring) and isolating unrelated tasks (under 50 points) into separate sessions while grouping similar tasks together.

## Key Mental Models
- **Task similarity scoring**: Quantifies shared characteristics (domain, data models, service/component, file routes, test patterns) to predict whether mixed context adds value or causes pollution
- **Context pollution as recommendation confusion**: Mixed unrelated patterns force AI to disambiguate which context applies to questions
- **Isolation threshold at 50 points**: Below threshold, pattern interference costs more than context pollution risk
- **Multi-session workflows**: Sequential isolation (do A then B), parallel isolation with checkpoints (work on A, checkpoint, switch to B), convergent isolation (complete A and B separately, then integrate)

## Critical Patterns
- **High similarity work together** (≥50 points): Adding password reset + email verification to same auth API benefits from shared authentication context
- **Low similarity isolate** (under 50 points): REST API authentication + CLI CSV import have zero shared patterns; mixing confuses both
- **Project constraints refine scores**: If tasks touch same configuration file or require integrated testing, similarity may increase despite different domains
- **Workflow pattern selection**: Sequential if tasks independent, parallel if both need multiple sittings, convergent if eventual integration planned

## AI Collaboration Keys
- AI applies similarity framework to task pairs and scores each characteristic
- AI identifies when project-specific constraints should override general similarity scores
- AI validates pollution symptoms in mixed sessions and recommends restructuring
- AI helps design multi-session workflow that matches project dependencies

## Common Mistakes
- Assuming "in same project" means "should be in same session" (domain matters more than proximity)
- Ignoring project-specific constraints that increase similarity (files touched, shared configuration)
- Starting isolated tasks in wrong order (creating dependency problems)
- Not creating checkpoints between isolated task sessions (prevents efficient session switching)

## Connections
- **Builds on**: Lessons 1-4 (understanding context limits, preventing degradation, compressing when limits approached)
- **Leads to**: Chapter capstone (orchestrating multi-session projects with context isolation, checkpoints, and progressive loading)
