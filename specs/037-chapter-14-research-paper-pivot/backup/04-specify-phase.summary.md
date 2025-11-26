### Core Concept
Specification quality comes from evals-first dialogue—a real conversation with AI exploring what success means before formal specification writing. Clear specs (intent, constraints, measurable evals) produce AI video outputs that meet expectations on first try; vague specs produce mediocre videos requiring iteration.

### Key Mental Models
- **Evals-First = Dialogue Before Documentation**: Have a conversation exploring success criteria, THEN formalize it—not the reverse
- **Business Success > Technical Validation**: Success evals measure marketing impact ("viewers understand sign-up") not just technical correctness ("file downloads successfully")
- **Intent→Constraints→Evals→Non-Goals Structure**: This sequence ensures NO implementation details leak in, keeping spec focused on WHAT not HOW

### Critical Patterns
- **Evals-First Conversation**: Ask clarifying questions about workflow, duration, target audience, platform, call-to-action—answers become measurable success criteria
- **Testable Success Evals**: "Video is high quality" fails; "Video downloads within 60 seconds, plays without error, text is readable at 1080p, CTA visible 3+ seconds" passes
- **Explicit Non-Goals**: State what you're NOT building (no post-editing, no voiceover, no custom animations) to prevent scope creep

### Common Mistakes
- Mixing specification and implementation—spec says "what" (45-60 second product demo), implementation says "how" (use Gemini with this prompt)
- Missing video-specific constraints—forgetting file size limits, Gemini timeout, playback validation requirements
- Vague success criteria that can't be objectively verified—subjective judgments ("looks professional") don't work

### Connections
- **Builds on**: Constitution establishing project-wide standards (Lesson 3)
- **Leads to**: Clarifying ambiguities in your specification with /sp.clarify (Lesson 5)
