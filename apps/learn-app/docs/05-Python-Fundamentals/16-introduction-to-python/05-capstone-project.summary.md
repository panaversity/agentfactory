### Core Concept
The capstone integrates all Chapter 17 concepts into one working program, demonstrating specification-first development in miniature. Students design intent (what program does), plan approach (which pieces to use), implement (write code), and validate (does output match design?)—the exact cycle professional AI-Driven Development follows.

### Key Mental Models
- **Specification as Guardrail**: Clear requirements (collect 5 fields, use type hints, display formatted summary) prevent random coding; design before implementing saves iteration
- **Integration as Proof**: Building one complete program proves mastery of variables, type hints, syntax, input/output, and formatting—not isolated concepts but working system
- **Validation as Non-Negotiable**: Checking program against requirements (does it collect 5 fields? do all variables have type hints? does output use f-strings?) teaches professional quality assurance
- **Input/Output as Interactivity**: `input()` function makes programs interactive; prompting users, storing responses, displaying results transforms programming from abstract to tangible

### Critical Patterns
- **Specification-First Workflow**: Write plain English design → Ask AI if design makes sense → Implement from design → Validate against design → Extensions from validated base
- **Layered Build Approach**: Collect information, display summary, test incrementally; building in stages prevents overwhelming complexity and isolates where bugs occur
- **Type Hints Everywhere**: All variables declare types (`name: str`, `age: str`); consistency enables AI review and demonstrates that type hints aren't optional feature but foundational practice

### AI Collaboration Keys
- Asking AI "Does my design make sense?" before coding reveals intention clarity; good specs enable AI to implement without ambiguity
- Code review with AI ("Check my type hints, naming, f-strings, comments") teaches professional quality gates; validation by external perspective (human or AI) prevents blind spots
- Explaining capstone demonstrates specification-first value: design intent drives implementation; implementation never predates clear requirements

### Common Mistakes
- Forgetting type hints on variables (violates specification; all should be `str` in capstone despite being numbers)
- Using string concatenation instead of f-strings (old pattern; clear pattern preference in modern code)
- Missing comments explaining sections (reduces readability; comments should frame purpose of each block)
- Skipping validation step (running program but not checking against requirements; validation proves specification understanding, not just code execution)

### Connections
- **Builds on**: Variables, type hints, syntax, input() function, f-strings, comments, print(), file execution
- **Leads to**: Control flow (if/loops) enabling conditional input validation, functions encapsulating reusable logic, data structures handling multiple entries, professional code review practices, real-world program complexity
