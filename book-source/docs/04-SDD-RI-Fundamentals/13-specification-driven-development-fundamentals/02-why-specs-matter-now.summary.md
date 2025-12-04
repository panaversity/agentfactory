### Core Concept
SDD (Specification-Driven Development) emerged in 2025 because three forces converged: AI got good enough to generate production code, developers discovered specs save iteration time, and AI's literal-mindedness made specs mandatory (not optional).

### Key Mental Models
- **AI Literal-Mindedness vs Human Improvisation**: Humans infer intent from vague requirements; AI implements exactly what's specified (no inferring)
- **Cost-Benefit Shift**: Pre-2025, specs weren't worth the effort (AI-generated code was broken anyway); 2025+, specs save 8 hours per feature (66% faster)
- **SDD > MDD**: Model-Driven Development (2000s) failed due to abstraction mismatch, tool lock-in, divergence; SDD succeeds because natural language specs are flexible, LLMs need no custom tools, no lock-in
- **Specifications as Interface**: Specs became the boundary between humans (intent) and AI (implementation), like command-line mastery defined Unix, specification-writing defines AI-native mastery

### Critical Patterns
- **Historical Timeline**: Formal Methods (1970s: too mathematical) → Design by Contract (1980s: language-locked) → MDD (2000s: tool lock-in) → Agile backlash (2010s: no specs) → SDD (2025+: natural language specs with powerful LLMs)
- **Three Convergent Forces**: (1) AI code quality jumped from broken to production-ready, (2) vague-vs-clear time costs became measurable (12h vs 4h), (3) AI literal-mindedness made specs essential, not optional

### AI Collaboration Keys
- AI benefits from specs more than human colleagues because it cannot improvise or ask clarifying questions during development
- AI asks questions during spec-writing phase (when corrections are cheap) rather than during code-generation phase (when fixes are expensive)
- Specs became AI's communication language the way English became human communication

### Common Mistakes
- Assuming specs are overhead like they were pre-AI
- Thinking "experienced developers" don't need specs (AI isn't experienced, needs explicit clarity)
- Skipping spec phase to "iterate faster" (actually costs more time in AI iterations)

### Connections
- **Builds on**: Lesson 1 (Vague code problems), understanding that AI works differently than humans
- **Leads to**: Lesson 3 (Specification anatomy and structure), understanding WHAT makes specs work
