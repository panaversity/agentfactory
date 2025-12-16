# Specify Phase — Writing Complete Specifications

This lesson teaches students how to transform vague project ideas into clear, measurable specifications through a two-part workflow: (1) Pre-specification conversation where YOU drive exploration through questions with AI, then (2) Formalize into specification using `/sp.specify`. Students learn that specification answers "What are we building?" with Intent, Constraints, Success Evals, and Non-Goals—not "How" (that's planning). The key paradigm shift: your ability to write a clear specification is more valuable than your ability to write code.

### Mental Models

- **Pre-Specification Conversation**: Before writing formal specs, have an informal conversation with AI to clarify intent. YOU drive the exploration through questions ("What aspects should I focus on? Who is my audience? What does success look like?"). The AI doesn't interview you; you use AI to think through requirements.

- **SMART Criteria Filter**: Success Evals must be Specific, Measurable, Achievable, Relevant, Time-bound. "Paper is high-quality" fails (subjective). "Paper cites 8+ peer-reviewed sources within past 10 years" passes (verifiable).

- **What vs How Boundary**: Specifications describe WHAT (outcomes). Plans describe HOW (process). If you're writing about steps, sequence, or tools—you're in planning, not specification.

- **Cascade Effect**: Clear spec → clear plan → clear tasks → quality output. Vague spec breaks everything downstream. Test your spec by asking: "Will this produce a good plan?"

### Key Patterns

- **Two-Part Workflow**: (1) Pre-specification conversation to explore requirements, then (2) `/sp.specify` to formalize into structured document. Never skip the conversation—it surfaces constraints and edge cases.

- **Four-Section Structure**: Intent (what problem, why, audience) → Constraints (limits, format, scope) → Success Evals (measurable done-ness) → Non-Goals (explicit out-of-scope). This structure works for any project type.

- **Specification Checklist**: After generating spec, verify: Intent clear? Constraints specific? Success Evals SMART? Non-Goals explicit? No HOW leaked? Could someone else build from this?

- **Ask AI to Review**: After `/sp.specify`, prompt: "For each success criterion, check if it's SMART. Identify vague criteria and suggest specific alternatives."

### Common Mistakes

- **Leaking Implementation**: Writing "Use Claude AI to research, outline, generate sections" in spec. This is HOW, not WHAT. Keep spec focused on outcomes: "5000-word paper with 8+ sources."

- **Vague Success Criteria**: "Paper is high-quality" or "sources are credible" can't be verified objectively. Use testable criteria: "8+ peer-reviewed sources; APA formatting; 3000-5000 words."

- **Missing Non-Goals**: Not specifying what you're NOT building causes scope creep. Explicitly state: "Not a comprehensive literature review; not a product comparison; no ethical discussion."

- **Skipping Pre-Specification Conversation**: Jumping straight to `/sp.specify` without clarifying intent, audience, and success criteria. The conversation surfaces requirements you'd otherwise miss.

### Progression Context

- **Builds on**: Lesson 3 (Constitution Phase) established global rules. Lesson 4 applies those rules to a specific feature specification.

- **Leads to**: Lesson 5 (Clarify Phase) uses `/sp.clarify` to identify gaps and ambiguities in the specification. Lesson 6 (Plan Phase) takes the clarified specification and designs HOW to build it.
