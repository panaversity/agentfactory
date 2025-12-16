### Core Concept
Agents need to work together (A2A Protocol, Agent Cards) and stay secure (defense in depth: deterministic guardrails + AI-powered guard models). Every capability is a risk—security design manages the trust trade-off.

### Key Mental Models
- **Agent Cards**: JSON descriptions of capabilities, inputs, limits. Enables agents to discover what other agents can do.
- **New Principal Class**: Agents aren't users or services—they're autonomous actors with their own identity and permissions
- **Trust Trade-Off**: Every capability introduces risk. Balance utility against exposure; design for compromise.

### Critical Patterns
- **A2A Protocol**: Task delegation with progress updates. Agents publish cards, request tasks, stream status.
- **Defense in Depth**: Two layers working together
  - *Deterministic Guardrails*: Hard limits enforced by infrastructure (rate limits, caps, scope). Can't be bypassed by prompting.
  - *Guard Models*: Separate LLM evaluates actions for context-dependent risks (social engineering, unusual patterns)
- **Agent Identity**: Credentials (prove who), permissions (what can do), audit log (what has done). When compromised, revoke credentials; blast radius limited to permissions.

### Common Mistakes
- Giving agents unlimited capabilities without guardrails (unlimited blast radius)
- Relying only on guardrails without guard models (misses context-dependent risks)
- Relying only on guard models without guardrails (can be bypassed by clever prompting)
- Not planning for compromise—no revocation path, no audit trail

### Connections
- **Builds on**: Agent Ops (Lesson 5)—security integrates with operational discipline
- **Leads to**: SDK Landscape (Lesson 7)—frameworks implement these security patterns differently
