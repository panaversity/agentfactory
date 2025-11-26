### Core Concept
/sp.clarify identifies gaps in video specifications that you missed—ambiguous terms, unspoken assumptions about Gemini, edge cases not documented, and conflicts between your spec and Gemini's actual capabilities. Two clarification rounds typically produce specs ready for planning; one round usually insufficient for AI-generated video work.

### Key Mental Models
- **Ambiguity is Expensive**: "Professional quality video" unclear now = argument about video quality later during implementation
- **Assumptions Surface Through Questions**: You assumed Gemini generates MP4 until /sp.clarify asks what format fallbacks exist
- **Technology Constraints are Non-Negotiable**: Gemini free tier has real limits; specs that ignore them cause implementation failure, not clever workarounds

### Critical Patterns
- **Clarify Video-Specific Gaps**: Resolution, frame rate, format compatibility, Gemini timeout behavior, download failure recovery
- **Identify Critical vs Nice-to-Have**: Not all clarification suggestions are equally important—distinguish blocking gaps from nice-to-have improvements
- **Cascade Quality Test**: Clear specs enable clear plans. If /sp.clarify reveals major gaps, planning will reveal even more

### Common Mistakes
- Skipping /sp.clarify because "my spec looks good"—video generation edge cases are subtle; almost all specs need clarification
- Accepting all AI suggestions uncritically—evaluate each: is this likely in my use case? Does it improve clarity or add noise?
- Ignoring Gemini constraints documented during clarification—unaddressed constraints become implementation surprises

### Connections
- **Builds on**: Written video specification (Lesson 4)
- **Leads to**: Research-based planning that discovers tool capabilities and designs architecture around them (Lesson 6)
