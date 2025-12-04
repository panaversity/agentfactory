### Core Concept
Write specifications BEFORE prompting AI—define success criteria, constraints, non-goals, output format, and validation tests upfront to eliminate wasted iteration and get 90%+ accuracy on first implementation instead of 60%.

### Key Mental Models
- **Specification-first workflow**: Write WHAT before prompting (specification document) → validate completeness → prompt with specification → AI implements correctly first/second try
- **WHAT vs HOW distinction**: Specification defines outcomes (success criteria), not implementation details; prompt can suggest HOW but specification defines target
- **Jake Heller's principle**: "Define 'what good looks like' for every step. Create objective tests." Built CoCounsel to $650M accuracy through specification discipline
- **5-component template**: WHAT (goal) → SUCCESS CRITERIA (measurable) → CONSTRAINTS (rules) → NON-GOALS (exclusions) → OUTPUT FORMAT + VALIDATION (tests)

### Critical Patterns
- **Measurable vs vague criteria**: "Script should be reliable" (unmeasurable) vs "Returns 0 on success, 1 on failure" (testable)
- **Specific vs assumed constraints**: "Handle errors well" (assumption) vs "Permission errors: log + continue; disk full: log + exit 1" (explicit behavior)
- **Explicit non-goals prevent scope creep**: "No compression, no cloud backup, no notifications" keeps AI focused (training data suggests common backup features)
- **Actionable validation tests**: "Test: Create file → Run script → Verify backed up" (specific steps) vs "Verify backup works correctly" (unmeasurable)
- **Rich context improves decisions** (Google best practice): Specify environment (Ubuntu 22.04, 2GB RAM), deployment (EC2 t3.medium), schedule (cron daily), security (non-root user)

### AI Collaboration Keys
- Complete specifications eliminate AI guessing, reducing iterations from 5-7 down to 1-2
- Specification defines measurable targets; iteration then focuses on refinement not requirements discovery
- Non-goals prevent AI from adding "common features" (compression, notifications) based on training data patterns
- Output format specificity (log format, timestamp structure, exit codes) eliminates ambiguous interpretation

### Common Mistakes
- **Success criteria are implementation details** ("Use rsync" HOW, not outcome WHAT)
- **Constraints are too vague** ("Handle errors appropriately" undefined)
- **Non-goals assumed not stated** (AI adds compression/cloud/notifications—common backup features)
- **Validation tests not actionable** ("Verify works correctly" vs "Test: Stage file → Verify shown in green")

### Connections
- **Builds on**: Lesson 1-2 foundations (prompts as specs, anatomy); Lesson 3 iteration; Lesson 4 systematizes all into specification-first discipline
- **Reduces iteration**: Specification-first cuts typical iteration from 5-7 cycles to 1-2 by eliminating requirements discovery phase
- **Next step**: Lesson 5 (Question-Driven Development) adds "ask AI clarifying questions" to ensure no specification gaps
- **Production methodology**: Jake Heller's approach to building $650M CoCounsel product through specification discipline
