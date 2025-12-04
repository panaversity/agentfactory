### Core Concept
Multiple SDD frameworks emerged (Kiro, Spec-Kit, Spec-Kit Plus, Tesel), each optimizing for different contexts; Spec-Kit Plus adds ADRs (document reasoning), PHRs (log AI interactions), and Intelligence Templates (domain expertise) specifically for AI-native teams learning SDD-RI.

### Key Mental Models
- **Context determines framework fit**: Team size, problem complexity, and compliance constraints drive framework choice (not one-size-fits-all)
- **Spec-Kit Plus evolution**: Builds on GitHub's Spec-Kit (strong governance) + adds intelligence tracking (ADRs), interaction logging (PHRs), and domain templates
- **Compliance-driven design**: Regulated domains (healthcare, finance, aerospace) need auditable decisions (PHRs track "why did AI suggest this?")
- **Knowledge persistence**: Intelligence Templates encode domain expertise so new teams don't rebuild (healthcare template has HIPAA rules pre-loaded)

### Critical Patterns
- **Kiro**: Simple (requirements → design → tasks → code), best for solo developers, learns SDD for first time
- **Spec-Kit**: Strong governance (Constitution enforces consistency), best for 5-50+ teams, enterprise environments
- **Spec-Kit Plus**: Spec-Kit + ADRs (why decisions) + PHRs (AI interaction logs) + Templates (domain expertise), designed for AI-native teams
- **Tesel**: Specs are ONLY source of truth (code generated, never hand-edited), for safety-critical systems (aerospace, medical)

### AI Collaboration Keys
- ADRs capture reasoning you've learned in P+Q+P design (why Skill X instead of Subagent Y?)
- PHRs log co-learning pattern throughout book (AI suggests → you refine → convergence)
- Intelligence Templates provide pre-built domain expertise so teams start with knowledge baseline

### Common Mistakes
- Choosing framework before understanding context (team size, compliance needs)
- Assuming Spec-Kit Plus overhead is unnecessary (ADRs/PHRs are not overhead; they're how AI-native teams learn)
- Treating framework choice as permanent (can migrate between frameworks as team grows)

### Connections
- **Builds on**: All SDD-RI concepts (Specifications, Constitutions, Skills, Subagents, P+Q+P design)
- **Leads to**: Chapter 35+ implementation using Spec-Kit Plus in practice, real feature development, SDD-RI workflow execution
