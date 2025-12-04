### Core Concept
Vague specifications cause 5-10x iteration cycles compared to clear ones because AI agents literally implement exactly what you specify, not what you meant.

### Key Mental Models
- **Vibe Coding**: Developing by intuition/conversation instead of precise specification (generates code that looks right but misses 30% of requirements)
- **AI Literal-Mindedness**: AI implements EXACTLY what specified, cannot infer unstated intent (unlike human colleagues)
- **Cost-Benefit Math**: Vague prompts cost 10-20 hours (initial 5 min + 10-15 iterations); clear specs cost 4-6 hours upfront (1 hour spec + 3-5 hours implementation)
- **Collaborative Specification**: AI asks clarifying questions you didn't consider, you provide constraints, together you discover edge cases before coding

### Critical Patterns
- **Vague → Iterate Pattern**: 5-min vague prompt → incomplete code → discover missing password reset → add it → discover missing rate limiting → add it... (cycle repeats 10+ times)
- **Collaborative Dialogue**: AI suggests questions (email verification? rate limiting? session management?) → you provide constraints (healthcare app needs HIPAA compliance) → together you refine spec
- **Specification Before Code**: Write clear spec (intent + constraints + success criteria) BEFORE asking for code → AI implements correctly on first iteration

### AI Collaboration Keys
- AI helps you think through edge cases by asking "what about [scenario]?" questions you didn't consider independently
- AI adapts recommendations when you provide project constraints (e.g., MVP timeline, healthcare context)
- Convergence happens through iteration: AI suggests → you refine → AI incorporates → spec improves (neither perfect alone, both better together)

### Common Mistakes
- Assuming "obvious" features (password reset) don't need to be specified
- Treating AI like a search engine ("get results") instead of a pair programmer (needs clarity)
- Not specifying security/edge case requirements upfront
- Iterating solution when problem was ambiguous specification

### Connections
- **Builds on**: Previous understanding of AI capabilities and communication patterns
- **Leads to**: Lesson 2 (Why SDD emerged NOW), Lesson 3 (Specification anatomy), Lesson 4 (Building first spec with AI collaboration)
