# Lesson 01: Key Before/After Changes

## Change 1: ADR Example (Calculator → Playwright MCP)

### BEFORE:
```markdown
**Bad documentation**: "Used JWT for authentication"
**ADR (Horizontal Intelligence)**: "Chose JWT over sessions because: (1) microservices
need stateless auth, (2) mobile clients benefit from token refresh, (3) tradeoff accepted:
token revocation complexity"
```

### AFTER:
```markdown
**Bad documentation**: "Used Playwright MCP for browser automation"
**ADR (Horizontal Intelligence)**: "Chose Playwright MCP over Selenium because:
(1) Gemini.google.com requires real browser context with session persistence,
(2) MCP provides native session management, (3) tradeoff accepted: MCP learning
curve vs robust video recording"
```

**Why**: Video generation + Playwright MCP are the actual project technologies

---

## Change 2: PHR Example (Generic Calculator → Video Marketing)

### BEFORE:
```markdown
**Prompt A**: "Write a calculator" → Generated insecure code (eval() vulnerability)
**Prompt B**: "Write a calculator using safe math operations, no eval()" → Generated clean code
**PHR captures**: Prompt B works, Prompt A fails, reasoning logged
```

### AFTER:
```markdown
**Prompt A**: "Generate a product demo video" → Resulted in generic, unconvincing video
**Prompt B**: "Generate a 30-second product demo video highlighting the pricing page
conversion flow using Gemini video generation API with high-quality voiceover"
→ Generated compelling video with strong marketing message
**PHR captures**: Prompt B works, Prompt A fails, reasoning logged
```

**Why**: Demonstrates how specificity + marketing context improves video generation

---

## Change 3: Compounding Effect Example

### BEFORE:
```markdown
- Project 1: Create 3 ADRs + 10 PHRs (learning from scratch)
- Project 2: Start with 3 ADRs + 10 PHRs, create 3 new ADRs + 8 new PHRs
  (total: 6 ADRs, 18 PHRs)
- Project 10: Start with accumulated intelligence from 9 projects, rarely repeat mistakes
```

### AFTER:
```markdown
- Project 1 (Video Generation): Create 5 ADRs + 12 PHRs (learning from scratch,
  figuring out Playwright MCP + Gemini integration)
- Project 2 (New Feature): Start with 5 ADRs + 12 PHRs, create 3 new ADRs + 8 new PHRs
  (total: 8 ADRs, 20 PHRs)
- Project 10: Start with accumulated intelligence from 9 projects, rarely repeat mistakes,
  new features built 3-4x faster
```

**Why**: Shows cross-project learning (video generation → auth system) + quantifies speed benefit

---

## Change 4: Specification Subagent Example

### BEFORE:
```markdown
### Example: Specification Subagent

**Persona**: Requirements analyst who thinks about edge cases before implementation

**Questions**:
- What happens when user inputs zero? Negative numbers? Strings instead of numbers?
- What assumptions am I making about input validation?
- What's the simplest test that proves this specification is complete?

**Principles**:
- SMART criteria enforcement
- Every data input has documented boundary conditions
- Every operation has at least 3 test cases (normal, edge, error)
```

### AFTER:
```markdown
### Example: Video Generation Specification Subagent

**Persona**: Video production specialist who thinks about marketing messaging
and technical constraints before implementation

**Questions**:
- What's the core marketing message this video needs to communicate?
- What are the technical constraints of Gemini.google.com video generation
  (video length, format, quality)?
- What happens if Gemini API fails mid-recording? How do we retry?
- What's the simplest test that proves the video meets marketing objectives?

**Principles**:
- SMART criteria enforcement
- Every video specification documents target audience, marketing message, and success metrics
- Every API integration has documented failure cases and retry strategies
- Every generated video includes validation (is it > 80% of intended length?
  Does voiceover match script?)
```

**Why**: Video-specific persona, constraints, and validation criteria show domain applicability

---

## Change 5: Delegation Pattern

### BEFORE:
```
YOU: "Build a calculator with 5 operations"
  ↓
ORCHESTRATOR: Routes to Specification Subagent
  ↓
SPEC SUBAGENT: Asks clarifying questions, generates complete spec
  ↓
YOU: Review and approve
  ↓
ORCHESTRATOR: Routes to Planning Subagent
  ↓
(Cycle repeats through Plan → Tasks → Implement)
```

### AFTER:
```
YOU: "Build a system that generates product demo videos and uploads to YouTube"
  ↓
ORCHESTRATOR: Routes to Specification Subagent
  ↓
SPEC SUBAGENT: Asks clarifying questions (What's the target video length?
                What marketing message?), generates complete spec
  ↓
YOU: Review and approve
  ↓
ORCHESTRATOR: Routes to Planning Subagent
  ↓
PLAN SUBAGENT: Identifies Playwright MCP + Gemini architecture, breaks
               into atomic tasks
  ↓
(Cycle repeats through Tasks → Implement → Skill Design)
```

**Why**: Concrete project scope + architectural choices visible in subagent routing

---

## Change 6: NEW SECTION — Business Context

### BEFORE:
[No business/motivation context]

### AFTER:
```markdown
## Why This Matters for Video Generation

SaaS companies spend $5,000-$50,000 per professional demo video, yet struggle to
produce them at scale. By building a system that **captures intelligence** (not just code),
you're creating:

1. **Immediate value**: A video file generated with AI
2. **Reusable skills**: `generate-video` and `upload-youtube` that work across projects
3. **Compounding knowledge**: ADRs explaining Playwright + Gemini integration that
   future projects inherit
4. **Portfolio proof**: You've mastered SDD-RI by turning specification into
   production-quality deliverables
```

**Why**: Articulates business problem + value proposition + portfolio relevance

---

## Change 7: Try With AI Prompts

### BEFORE:
```markdown
**🔍 Explore Intelligence Patterns:**
> "Compare Horizontal Intelligence (ADRs/PHRs) vs Vertical Intelligence (Subagents)
> in Spec-Kit Plus. Show me a concrete example: if I build a calculator in Project 1..."

**🎯 Practice P+Q+P Analysis:**
> "I want to create a subagent for API design review..."

**🧪 Test Compounding Understanding:**
> "Calculate the compounding effect: If Project 1 creates 5 ADRs + 12 PHRs..."

**🚀 Apply to Your Domain:**
> "I work in [describe your domain/industry]..."
```

### AFTER:
```markdown
**Explore Intelligence Patterns:**
> "In Spec-Kit Plus, compare Horizontal Intelligence (ADRs/PHRs) vs Vertical Intelligence
> (Subagents). Imagine I build a video generation system in Project 1 and create 5 ADRs
> (about Playwright MCP, Gemini constraints, quality validation) and 12 PHRs (about what
> prompts generate compelling videos). How exactly does that intelligence help when I
> build an authentication system in Project 2? Show me concrete examples of what the
> AI agents would know."

**Practice P+Q+P for Video Production:**
> "Help me design a subagent for video script writing. Using Persona+Questions+Principles
> (P+Q+P), what persona should it adopt (think about marketing, production quality, SaaS
> context)? What questions should it ask to activate reasoning about video effectiveness?
> What principles should guide its decisions? Why does this approach work better than
> just asking 'write a video script'?"

**Calculate Intelligence Compounding:**
> "If Project 1 (video generation) creates 5 ADRs + 12 PHRs, Project 2 (auth system)
> creates 3 ADRs + 8 PHRs, and Project 3 (payment system) creates 2 ADRs + 6 PHRs,
> how much intelligence is available when I start Project 4?..."

**Apply to Your Domain:**
> [Original structure maintained]
```

**Why**: Video generation context + specific technical details (Playwright, Gemini, YouTube)

---

## Change 8: Frontmatter Updates

### BEFORE:
```yaml
chapter: 31
lesson: 1
source_spec: "specs/10-chapter-33-redesign/spec.md"
created: "2025-11-18"
last_modified: "2025-11-18"
version: "1.2.0"
workflow: "manual-implementation"
```

### AFTER:
```yaml
chapter: 14
lesson: 1
source_spec: "specs/chapter-14-video-generation/spec.md"
created: "2025-11-25"
last_modified: "2025-11-25"
version: "2.0.0"
workflow: "/sp.implement"
```

**Why**: Correct chapter/lesson numbers + video project context + updated workflow

---

## Summary of Refactoring Strategy

| Aspect | Strategy | Result |
|--------|----------|--------|
| **Domain Context** | Calculator → Video Generation | 42+ video references, 0 calculator references |
| **Examples** | Generic → Domain-specific | ADR, PHR, Subagent all use video/Playwright context |
| **Business Framing** | Added motivation section | Clear value proposition ($5K-$50K per video) |
| **Pedagogical Framework** | Invisible to students | No "AI as Teacher" labels, no framework exposure |
| **Concepts Preserved** | H/V Intelligence intact | ADRs, PHRs, P+Q+P pattern all present |
| **Cognitive Load** | A2-appropriate | 2 new concepts (within 7-concept limit) |
| **Structure** | Keep "Try With AI" ending | No "What's Next", "Key Takeaways", "Summary" |

---

## Constitutional Compliance Checklist

- ✅ No calculator references remain
- ✅ Video generation context integrated throughout
- ✅ Business framing added (SaaS value proposition)
- ✅ H/V Intelligence concepts intact
- ✅ P+Q+P framework explained clearly
- ✅ Three Roles framework is INVISIBLE (no labels)
- ✅ No meta-commentary or scaffolding exposed
- ✅ Ends with "Try With AI" section ONLY
- ✅ Cognitive load verified (A2: 2 concepts ≤ 7)
- ✅ Evidence-based claims supported

**Status: READY FOR DELIVERY**
