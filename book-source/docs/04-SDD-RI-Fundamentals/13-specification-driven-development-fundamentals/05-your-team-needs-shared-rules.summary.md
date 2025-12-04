### Core Concept
A Constitution (or Memory Bank) is a set of system-wide rules that apply to ALL specifications, ensuring consistency across teams without constant meetings or debates.

### Key Mental Models
- **System-wide vs Feature-specific**: Specifications define how individual features work; Constitutions define universal rules applying to everything
- **Consistency at scale**: As team size grows from 5 to 50 to 500 developers, shared rules prevent security/architecture/quality chaos
- **Organizational learning**: Production bugs → root cause analysis → constitutional rule → future specs automatically prevent recurrence
- **Triple documentation layer**: Constitution (immutable global rules) + ADRs (why we chose this decision) + PHRs (how we learned it)

### Critical Patterns
- **Constitution structure**: Product vision, architecture patterns, technology stack, security rules, quality standards, common patterns
- **Spec compliance to Constitution**: Every spec references constitutional rules as "per Constitution" rather than redefining basics
- **Bug-to-rule encoding**: When security/quality gaps appear in production, convert to constitutional rule preventing same gap across 50 features
- **Code review simplification**: When Constitution is enforced in specs, code review focuses on logic, not security/architecture basics

### AI Collaboration Keys
- AI agents read Constitution before generating code (ensures consistent implementations)
- Team members reference Constitution when writing specs (prevents ad-hoc decisions)
- Constitutions encode expert knowledge that humans might forget or vary

### Common Mistakes
- Writing Constitution too specific (should apply broadly, not lock teams into single pattern)
- Treating Constitution as static (must evolve as team learns from production incidents)
- Forgetting that Constitution is enforcement mechanism, not documentation (specs must reference rules explicitly)

### Connections
- **Builds on**: SDD fundamentals (clear specifications as foundation)
- **Leads to**: Reusable Intelligence (patterns that repeat across specs become Skills/Subagents), ADRs (documenting why constitutional decisions were made), PHRs (tracking how team learned each constitutional lesson)
