### Core Concept
Prompt refinement is bidirectional collaboration where you and AI improve solutions through iteration: you provide domain context AI doesn't have, AI suggests patterns you don't know, and convergence through dialogue produces better results than either could achieve alone.

### Key Mental Models
- **First prompts yield 60%**: Professional results require 10-20 iterations, not expecting perfection on first try
- **Iteration loop pattern**: Initial Prompt → AI Output → Analyze → Refine → Improved Output → Repeat until success criteria met
- **Bidirectional learning**: AI teaches you patterns (conventions, best practices), you teach AI constraints (project requirements, team preferences)
- **Progressive layering** (Anthropic): Simple → Structured → Examples → Reasoning—each layer adds precision moving from 60% to 95%+

### Critical Patterns
- **Discovering knowledge**: When AI suggests patterns (Conventional Commits format), investigate instead of rejecting; this is AI teaching you
- **Providing context**: Share project-specific requirements (Jira tickets, team conventions, business focus) so AI adapts to YOUR needs
- **Convergence through dialogue**: Neither you nor AI had perfect solution initially; iteration combined AI's conventional format + your project constraints
- **Incremental refinement** (OpenAI best practice): Add 1-2 constraints per iteration so you see which change fixed which problem
- **Diminishing returns signal**: When improvement drops below 5% per iteration, convergence is reached

### AI Collaboration Keys
- AI has pattern knowledge (conventions, frameworks, best practices) from millions of examples; you have domain knowledge (project constraints, team preferences)
- Iteration creates partnership: you correct AI when it misses your context, AI suggests improvements you hadn't considered
- Collaborative dialogue > back-and-forth fighting: When AI suggests something unexpected, ask "Why?" instead of forcing your original approach
- Multiple iteration strategies: Incremental constraints, discover-then-apply, converge-through-dialogue depending on task complexity

### Common Mistakes
- **Quitting too early**: Concluding "AI can't do this" after 1-2 iterations when 60% → 97% requires persistent refinement
- **Ignoring AI's suggestions**: Forcing AI back to your original inferior approach when AI suggests better patterns (missed learning opportunity)
- **Not providing context**: Re-iterating "try again" 5 times without explaining project-specific requirements (AI has no basis to improve)
- **Changing too much at once**: Rewriting entire prompt with 10 new constraints when you can't tell which change helped (make iteration debuggable with 1-2 changes per round)

### Connections
- **Builds on**: Lesson 2's anatomy (Intent/Constraints/Success structure); Lesson 3 applies this structure iteratively
- **Leads to**: Lesson 4 (Specification-First) which prevents some iteration by defining success criteria upfront
- **Real-world parallel**: Jake Heller's CoCounsel 60%→97% methodology showing weeks of iteration for production systems
