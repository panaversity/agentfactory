### Core Concept
Knowing when NOT to automate separates responsible practitioners from reckless ones. Six common pitfalls—autonomous legal/medical/financial decisions, biased hiring, untracked data access, missing audit trails—create unacceptable liability that no automation benefit justifies.

### Key Mental Models
- **Three-Tier Security Model**: Data access controls (minimum necessary) → Audit and logging (immutable, comprehensive) → Governance and oversight (human accountability, escalation procedures).
- **Shadow Mode Deployment**: Phase 1 (agent suggests, human decides, log agreement rate) → Phase 2 (human uses agent input, measure override rate) → Phase 3 (selective automation for high-confidence scenarios).
- **Red Flag Framework**: Insufficient audit feasibility, irreplaceable human judgment, regulatory uncertainty, high-consequence errors, adversarial pressure, biased data. Two+ signals = reconsider; three+ = stop.

### Critical Patterns
- Human reviews ALL agent outputs before external contact (legal opinions, medical recommendations, financial transactions)
- Log all decisions with reasoning, not just outcomes—"Why was this decision made?" must be answerable
- Industry-specific guardrails: HIPAA (healthcare), SOC 2 (enterprise), PCI DSS (payments), GDPR (EU data)
- Shadow mode checkpoints: 80%+ agreement with humans (Phase 1), under 20% override rate (Phase 2), bias audits across demographic groups (Phase 3)

### Common Mistakes
- Automating fully without human oversight because "it's faster"—creates liability you cannot defend
- Skipping audit trails because logging is "expensive"—makes incident investigation impossible
- Ignoring bias in training data—agent perpetuates discrimination at scale

### Connections
- **Builds on**: Three requirements for success (Lesson 8)
- **Leads to**: Strategic synthesis (Lesson 10) where you integrate guardrails into your personal Digital FTE plan
