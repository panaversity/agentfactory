### Core Concept
Effective prompts have three structural components—Intent (WHAT outcome), Constraints (MUST/MUST NOT rules), and Success Criteria (measurable validation)—and this anatomy transforms vague requests into precise specifications that guide AI implementation.

### Key Mental Models
- **Three-part structure** (Intent → Constraints → Success) prevents ambiguity by forcing specification clarity before prompting
- **Technical action verbs** (CREATE, DEBUG, REFACTOR, ANALYZE, OPTIMIZE, GENERATE, EXPLAIN, VALIDATE) communicate implementation intention precisely
- **Constraints define boundaries**: Technical (language/platform), Scope (what's included/excluded), Quality (standards to meet), Format (output structure)
- **Success criteria are testable**: Replace vague outcomes ("should work well") with measurable conditions ("processes 1000 files in <10 seconds")

### Critical Patterns
- **Vague intent** ("Help me with Git") vs **strong intent** ("Explain difference between git merge and git rebase for feature branch integration")
- **Anti-patterns that kill prompts**: Ambiguous language ("make robust"), missing context ("optimize this" what?), no validation criteria, over-specifying HOW instead of WHAT
- **Zia Kaukab's 8-element framework** maps to anatomy: Command→Intent, Context→Constraints, Examples→Success Criteria (systematic approach)
- **OpenAI's best practice**: First prompts are drafts; iterate 2-3 times adding constraints AI violated in previous attempts

### AI Collaboration Keys
- Precise action verbs (CREATE, DEBUG, etc.) tell AI to BUILD, not explain—directing implementation partnership
- Constraints filter AI's infinite possibilities into project-specific solutions; without constraints, AI makes generic assumptions
- Success criteria enable objective validation instead of subjective "does this feel right?" judgment
- Framework anatomy incorporates all 8 elements (Zia's) systematically—structure prevents omission of critical specification pieces

### Common Mistakes
- **Anti-Pattern 1: Ambiguous language** ("Make the script more robust" = robust how? error handling? input validation? all of these?)
- **Anti-Pattern 2: Missing context** ("Optimize this" = optimize for what? speed? memory? readability? code provided?)
- **Anti-Pattern 3: No validation criteria** ("Create tests for the backup script" = what tests? how many? which scenarios?)
- **Anti-Pattern 4: Prescribing HOW** (over-specifying implementation removes AI's ability to suggest better approaches; specify WHAT not HOW)
- **Anti-Pattern 5: Not expecting iteration** (assuming detailed prompt = perfect first output; even excellent prompts improve through 2-3 refinement cycles)

### Connections
- **Builds on**: Lesson 1's concept that prompts are specifications; deepens understanding of what makes them effective
- **Leads to**: Lesson 3 (iteration to refine these components), Lesson 4 (applying anatomy systematically in specification-first documents)
- **Applies**: Zia Kaukab's Google research + Anthropic's progressive refinement + OpenAI's iteration guidance
