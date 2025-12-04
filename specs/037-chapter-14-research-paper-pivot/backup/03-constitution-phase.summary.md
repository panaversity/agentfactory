### Core Concept
Constitution is the project's rulebook for ALL video-related work—written once, applied to every video feature. It defines non-negotiable standards (output format, Gemini constraints, validation requirements) that cascade through every downstream specification, plan, and implementation. Weak Constitution produces vague work; strong Constitution produces precise, standards-aligned work.

### Key Mental Models
- **Cascade Starting Point**: Constitution quality determines all downstream quality. Clear Constitution → clear specs → clear plans → working code. Vague Constitution → vague specs → confused plans → broken code
- **Global Rules vs Feature Requirements**: Constitution applies to ALL videos (format, codec, validation); Specification applies to ONE video (this demo's script, duration, CTA)
- **One-Time Investment, Repetitive Payoff**: Write Constitution once in Phase 1; reuse it across dozens of video features without modification

### Critical Patterns
- **Testable Standards Only**: "Professional quality" is subjective; "MP4 H.264, 1920x1080, 30 FPS, validates in VLC" is testable
- **Document Constraints Explicitly**: Gemini free tier has limits (90-second timeout, session persistence required, rate limits)—Constitution makes these visible to every downstream phase
- **Quality Gates Define Validation**: Constitution specifies HOW video quality will be verified (format check, playability test, duration validation)

### Common Mistakes
- Copying Constitution from unrelated projects—calculator Constitution is irrelevant for video generation; write one specific to your domain
- Vague quality standards that can't be verified—"good video quality" has no measurable criteria
- Ignoring Gemini constraints during Constitution—free tier limitations discovered too late, in implementation, causing rework

### Connections
- **Builds on**: Spec-Kit Plus framework installation (Lesson 2)
- **Leads to**: Writing specific video feature specifications that inherit Constitution standards (Lesson 4)
