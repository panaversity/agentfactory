---
id: "0008"
date: 2025-12-22
stage: "misc"
feature: "docker-chapter"
chapter: 49
lesson: 8
title: "AI-Assisted Docker with Gordon Lesson"
status: "completed"
---

# Prompt History Record: AI-Assisted Docker with Gordon

## Summary

Created Lesson 8 for Chapter 49 (Docker for AI Services) following Layer 2 AI Collaboration framework. Lesson teaches interactive workflow with Gordon (Docker's AI agent), focusing on bidirectional learning where students guide AI suggestions toward their specific requirements.

## Key Decisions

### Layer 2 (AI Collaboration) - Three Roles INVISIBLE

- **AI as Teacher**: Gordon suggests optimization patterns (multi-stage builds, Alpine vs slim tradeoff)
- **Student as Teacher**: Student constraints force Gordon to adapt (distroless health check failure)
- **Co-Worker iteration**: Dialogue between student requirements and Gordon suggestions produces better Dockerfile than either could alone

**Critical**: Framework labels remain completely invisible to students. Learning emerges through natural narrative of "your request → Gordon generates → you evaluate → you refine → Gordon adapts."

### Constitutional Requirements Met

1. **Three Roles INVISIBLE**: Zero mentions of "Layer 2", "AI as Teacher", "Three Roles", "What to notice", "AI learned from you"
2. **Show roles through ACTION**:
   - "Your request to Gordon:" + "Gordon generates:" (AI as Teacher role emerges naturally)
   - "Your refinement request:" + "Gordon adapts:" (Student as Teacher role emerges naturally)
   - "What emerged from this dialogue" (Co-Worker convergence without labeling)
3. **Ends with Try With AI**: Only final section, no Summary/Key Takeaways
4. **Evidence-based**: All code examples show expected outputs, prompts are copyable

### Content Progression

1. **Availability note** (FR-086a): Gordon requires Docker Desktop 4.50+ with fallback guidance
2. **Access**: Clear setup steps for enabling Gordon in Docker Desktop
3. **Generation**: Start with generating Dockerfiles from descriptions (simple collaborative case)
4. **Debugging**: Using Gordon for diagnostic expertise (container exit codes, resource issues)
5. **Optimization**: Collaborative refinement of existing Dockerfiles (multi-stage builds, size reduction)
6. **Dialogue iterations**: Two rounds of refinement showing adaptation (distroless → slim health check)
7. **Override patterns**: Four scenarios when to reject or adapt Gordon's suggestions
8. **Try With AI**: Structured 5-part activity with reflection

### Teaching Modality Variation

- Previous lesson (03): Direct teaching, Layer 1 (Manual Foundation) - students write Dockerfile by hand
- This lesson (08): Collaborative dialogue, Layer 2 (AI Collaboration) - students work WITH Gordon
- Creates pedagogical arc: Foundation → Application

## Content Architecture

### Three Roles in Action (Without Labels)

**Round 1: AI as Teacher**
- Student asks for Dockerfile with specific constraints
- Gordon suggests optimized version (Alpine, non-root, multi-stage)
- Student evaluates: Does this match requirements?

**Round 1: Student as Teacher**
- Student identifies issue: Manual file copying is error-prone
- Requests constraint: "Include all project files"
- Gordon adapts: Switches to `COPY . .` pattern

**Round 2: Student Teaches AI (Correction)**
- Student discovers: Distroless lacks curl for health checks
- Tells Gordon: This breaks my requirements
- Gordon adapts: Uses slim + Python-based health check

**Convergence**: Final Dockerfile balances all constraints:
- Size optimization (slim vs full)
- Health checks (working, not broken)
- Maintainability (team can understand)
- Security (non-root)

### Cognitive Load Management (B1 Tier)

- 7 concepts: Gordon access, Dockerfile generation, debugging, optimization, refinement, evaluation, override patterns
- B1-appropriate: Moderate scaffolding, guided exploration, decision frameworks
- Examples: 3 main iteration patterns (generation, debugging, optimization)
- Not exhaustive: Could explore 10+ Gordon scenarios; focused on core workflow instead

### Production-Relevant Patterns

- Real issue: Multi-stage Dockerfile size reduction (55% savings is meaningful)
- Real constraint: AI health checks using unavailable tools in distroless
- Real context: Team maintainability vs optimization tradeoff
- Real problem: Gordon prioritizes simplicity but student needs security/size/reliability

## Constitutional Alignment

**Principle 1 (Specification Primacy)**: ✓ Student specifies requirements BEFORE asking Gordon
**Principle 2 (Progressive Complexity)**: ✓ B1 cognitive load (7 concepts, 3-4 options)
**Principle 3 (Factual Accuracy)**: ✓ Gordon information from official Docker docs (v4.50+, health checks)
**Principle 4 (Coherent Structure)**: ✓ Foundation → Application progression across chapter
**Principle 5 (Intelligence Accumulation)**: ✓ Builds on Layer 1 manual foundation from Lesson 3
**Principle 6 (Anti-Convergence)**: ✓ Layer 1 + Layer 2 variation (manual vs collaborative)
**Principle 7 (Minimal Content)**: ✓ Only Try With AI ending (no Summary/Key Takeaways)

## Validation Checklist

- [x] Zero role labels ("AI as Teacher", "Layer 2", "Three Roles", "What to notice")
- [x] Framework invisible: Students EXPERIENCE bidirectional learning, don't STUDY it
- [x] Every code block has **Output:** section showing expected result
- [x] Ends only with "## Try With AI" (no Summary, Key Takeaways, etc.)
- [x] 7 concepts within B1 limits (7-10 concepts)
- [x] 3-4 main examples/options (B1-appropriate)
- [x] Gordon information from official Docker docs
- [x] No meta-commentary breaking immersion
- [x] Narrative focuses on "What emerged" not "What AI learned"

## Implementation Notes

### Three Roles Narrative (Critical for Constitutional Compliance)

**NOT this** (exposes framework):
```
"This is AI as Teacher: Gordon suggests patterns..."
"What to notice: AI is teaching you optimization..."
"What you learned: AI showed you multi-stage builds"
```

**BUT this** (natural narrative):
```
"Your request to Gordon: [prompt]"
"Gordon generates: [output]"
"What emerged from this dialogue: [outcome]"
```

Every section uses natural dialogue narrative, never framework exposition.

### Iterative Refinement Pattern

Showed two rounds of refinement:
1. Gordon suggests multi-stage → Student evaluates → Student concerns about manual copies → Gordon adapts
2. Gordon suggests distroless → Student tests → Health check fails → Gordon explains tradeoff

This demonstrates:
- AI can't predict all your constraints (distroless health check)
- You teach AI through constraints
- Better solution emerges from dialogue than initial suggestion

### Safety Note Placement

Included safety note INSIDE Try With AI (context-specific: don't hardcode API keys). Did NOT include standalone "Safety" section after activity (would violate Principle 7 - Minimal Content).

## Sources Consulted

- Docker Gordon (Ask Gordon) official documentation
- Docker Desktop 4.50 release notes (security update from November 2025)
- Constitution v6.0.1 (especially Section IIa: Meta-Commentary Prohibition)
- Chapter 49 existing lessons (Lesson 3: Manual Foundation)
- Layer 2 AI Collaboration framework (Section IIa)

## Files Created

- `/apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/08-ai-assisted-docker-with-gordon.md`
  - 470 lines
  - YAML frontmatter with learning objectives
  - 5 main sections + Try With AI activity
  - Production-ready for publication

## Next Steps

- Technical review: Verify Gordon information accuracy against Docker official docs
- Validation audit: Check for any exposed framework labels or meta-commentary
- Context consistency: Verify previous/next lessons create coherent pedagogical arc
