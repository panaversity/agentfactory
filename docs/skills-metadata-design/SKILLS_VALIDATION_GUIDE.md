# Skills Metadata Validation Guide

## Quick Answer to Your Question

**"How do I know if the skills are right and getting connected?"**

You now have a **validation framework** consisting of three key documents:

### 1. **Master Skill Registry** (`specs/part-1-skills-registry.md`)
Use this to verify:
- ✓ Is this skill unique or does it already exist?
- ✓ What's the proper name for this skill?
- ✓ What are the dependencies?
- ✓ What proficiency progression should it follow?

**Key Structure**: Each skill entry includes:
- Unique canonical name
- Definition of what proficiency means
- CEFR path (A1 → A2 → B1 → B2)
- First appearance (where introduced)
- Dependencies (what must be learned first)
- Validation criteria (how to assess mastery at each level)

### 2. **Skill Progression Map** (`specs/part-1-skill-progression-map.md`)
Use this to verify:
- ✓ How does this skill build across chapters?
- ✓ Is there proper A1 → A2 → B1 progression?
- ✓ Are prerequisites taught before advanced skills?
- ✓ Is the skill isolated or does it connect to other skills?

**Key Structure**: Four progression tracks showing how skills build:
1. AI Capability & Development Impact (Ch1 → Ch2 → Ch4)
2. Business Models & Strategy (Ch1 → Ch3 → Ch4)
3. Professional Growth (Ch1 → Ch4)
4. Systems Thinking & AIDD (Ch2 → Ch4)

Each track shows:
- When skill is introduced (A1 level)
- How it deepens (A2 level)
- How it's applied (B1 level)
- Any gaps or issues in progression

### 3. **Validation Checklist** (this document)
Use this before creating/modifying skills in Part 2+.

---

## How to Know if Skills Are "Right"

### Test 1: Uniqueness
**Question**: Is this a new skill, or am I duplicating an existing skill?

**How to Check**:
1. Open `specs/part-1-skills-registry.md`
2. Look for skills in the same domain
3. Check if the concept is already defined

**Examples of Issues We Found**:
- ❌ "Evaluating Career Relevance" vs. "Assessing Personal Role Transition" — Same thing, different names
- ❌ "Understanding Opportunity Window" (Ch1) vs. "Recognizing Opportunity Window" (Ch3) — Essentially same skill

**Fix**: Use consistent names and reference existing skills if extending them

---

### Test 2: Naming Convention
**Question**: Is the skill name clear and distinct from similar skills?

**How to Check**:
1. Does the skill use one of these verbs consistently?
   - **Recognizing** = Identifying patterns/signals (A1)
   - **Understanding** = Grasping concepts/frameworks (A1-A2)
   - **Assessing** = Determining current state (A2)
   - **Evaluating** = Judging quality/fitness (A2)
   - **Personalizing** = Adapting to individual context (A2)
   - **Prioritizing** = Making strategic choices (A2)

2. Is the skill name specific enough to distinguish it from others?
   - ❌ "Understanding X" is too generic
   - ✓ "Understanding AI Stack Architecture" is specific

**Examples**:
- ✓ "Recognizing Competitive Layer Structures" (specific, clear)
- ✓ "Understanding Vertical Intelligence Reuse" (distinct concept)
- ❌ "Evaluating Things" (too vague)
- ❌ "Understanding X" (if X is broad concept, be more specific)

---

### Test 3: Proficiency Progression
**Question**: Does the skill show proper A1 → A2 → B1 progression?

**How to Check**:
1. Find the skill in the registry
2. Trace where it appears across chapters
3. Verify proficiency only stays same or increases (never regresses)

**Example - Correct Progression**:
```
Ch1, L1: "Recognizing AI Impact" (A1)
  ↓ builds on
Ch2, L1: "Understanding Capability Breakthroughs" (A1) — same level, different example
  ↓ builds on
Ch4, L5: "Understanding M-Shaped Developer" (A2) — increased proficiency ✓
```

**Example - Incorrect Progression**:
```
Ch2, L2: "Understanding Vibe Coding Tradeoffs" (A2)
  ↓ later in same chapter
Ch2, L3: "Understanding AI as Amplifier" (A1) — REGRESSED ❌
```

**Rule**: If skill appears multiple times, proficiency levels should be:
- Same level = fine (showing application in new context)
- Higher level = good (showing progression)
- Lower level = WRONG (regression indicates confusion)

---

### Test 4: Prerequisites
**Question**: Have all prerequisite skills been taught before this skill?

**How to Check**:
1. Find the skill in the registry
2. Look at "Dependencies" field
3. Check if prerequisite skills appear in earlier chapters

**Example - Correct Prerequisites**:
```
Skill: "Understanding Vertical Intelligence Reuse" (A2)
Prerequisites:
  - "Understanding Economic Scaling (90-10 Rule)" (A1) — taught in Ch3, L3 ✓
  - "Understanding Competitive Layer Strategy" (A1) — taught in Ch3, L2 ✓
Position: Ch3, L4 (after both prerequisites) ✓
```

**Example - Missing Prerequisites**:
```
Skill: "Evaluating Market Risk & Timing" (A2)
Position: Ch3, L5
Prerequisites mentioned in registry:
  - "Recognizing Market Dynamics" (A1) — NOT taught anywhere ❌
```

**Rule**: A2 skills must have A1 prerequisites taught earlier

---

### Test 5: Skill Connectivity
**Question**: Does this skill connect to other skills, or is it isolated?

**How to Check**:
1. Open `specs/part-1-skill-progression-map.md`
2. Find the progression track for this skill's domain
3. Look for your skill in the visual map
4. Is it connected via ↓ (builds on) relationships?

**Example - Well Connected**:
```
Progression Track 2: Business Models & Strategy

"Recognizing AI Opportunity Timing" (A1)
  ↓ (builds on)
"Recognizing Competitive Layer Structures" (A1)
  ↓ (builds on)
"Understanding Economic Scaling" (A2)
  ↓ (builds on)
"Understanding Vertical Market Strategy" (A1)
```

Each skill clearly supports the next. ✓

**Example - Isolated Skill**:
```
"Recognizing CS Education Gaps" (A1)
[appears only in Ch1, L8]
[not mentioned again in any later chapter]
[no skills build on it]
```

Isolated skills work for context-setting, but important concepts should reappear. ⚠️

---

## Validation Checklist for New Skills

**Before adding a new skill to Part 2+, verify**:

### Naming & Definition
- [ ] Skill name is unique (not duplicate of existing skill)
- [ ] Skill name uses appropriate verb (Recognizing/Understanding/Assessing/Evaluating/etc.)
- [ ] Skill name is specific enough (e.g., not just "Understanding X")
- [ ] Definition clearly states what proficiency means at each level

### Proficiency & Prerequisites
- [ ] First appearance is at A1 or A2 proficiency level
- [ ] All prerequisite skills are listed in registry
- [ ] All prerequisites are taught before this skill (in earlier chapters)
- [ ] If skill appears multiple times, proficiency is monotonically increasing (no regressions)
- [ ] Bloom level matches CEFR level (A1=Understand, A2=Apply, B1=Analyze)

### Cognitive Load & Coherence
- [ ] Lesson has ≤5 concepts (if A1), ≤7 (if A2), ≤10 (if B1)
- [ ] Skill fits into one of four domains (Development, Business, Career, Systems)
- [ ] Skill builds on previous knowledge (not isolated)
- [ ] Skill prepares for future learning (has clear progression path)

### Assessment & Measurability
- [ ] Proficiency is measurable at each level (A1/A2/B1/B2)
- [ ] Assessment criteria are clear and objective
- [ ] Criteria distinguish between proficiency levels

### Cross-Chapter Coherence
- [ ] If skill appears in multiple chapters, each appearance shows progression
- [ ] Skill dependencies are documented in learning objectives
- [ ] Skill contributes to one of the four progression tracks

---

## Common Issues We Found (Learn From These)

### Issue 1: Skill Fragmentation
**Problem**: Same concept named differently in different chapters
**Example**:
- Ch1, L1: "Recognizing AI's Impact on Development"
- Ch4, L1: "Recognizing Paradigm Shifts in Development"
- → These are essentially the same skill!

**Fix**: Choose one canonical name and use consistently

---

### Issue 2: Missing A1 Prerequisites
**Problem**: A2 skill with no A1 prerequisite taught
**Example**:
- Ch3, L5: "Evaluating Market Risk & Timing" (A2)
- But no A1 skill like "Recognizing Market Risk Factors"

**Fix**: Add the A1 prerequisite skill earlier, or explicitly note in prerequisite field

---

### Issue 3: Proficiency Regression
**Problem**: Skill appears at higher proficiency, then reappears at lower
**Example**:
- Ch2, L2: "Understanding Vibe Coding Tradeoffs" (A2)
- Ch2, L3: "Understanding AI as Amplifier" (A1)
- → Why would we go backwards?

**Fix**: Reorder lessons or change proficiency level

---

### Issue 4: Isolated Skills
**Problem**: Skill appears once and never again; doesn't build to anything
**Example**:
- Ch1, L8: "Recognizing CS Education Gaps" (A1)
- [never appears again]
- [no other skills build on it]

**Fix**: Either integrate into progression track or acknowledge as context-setting

---

### Issue 5: Overused Verbs
**Problem**: 20+ skills all use "Evaluating" without distinguishing cognitive action
**Examples**:
- Evaluating Career Relevance
- Evaluating Evidence Quality
- Evaluating Personal Opportunity
- Evaluating Layer Selection
- → All mean different things but use same verb

**Fix**: Use distinct verbs:
- **Assessing** = determining current state (readiness, position)
- **Evaluating** = judging quality/fitness for purpose
- **Prioritizing** = making strategic choices
- **Personalizing** = adapting to context

---

## How to Resolve Issues Before Part 2

### Phase 1: Cleanup (3-4 hours)
1. **Identify duplicates**: Search for skills with similar names/definitions
2. **Consolidate**: Merge duplicates, choose canonical name
3. **Add missing prerequisites**: Create A1 skills for A2 skills that lack them
4. **Rename for clarity**: Distinguish verbs (assess vs. evaluate vs. prioritize)
5. **Document gaps**: Note skills that are isolated (context-setting only)

### Phase 2: Validation (2-3 hours)
1. **Cross-check against registry**: Ensure all Part 1 skills are documented
2. **Verify progression**: Trace each skill across chapters
3. **Check prerequisites**: Ensure all A2 have A1, all B1 have A2
4. **Audit coherence**: Map each skill to progression tracks
5. **Create visual maps**: Generate skill dependency graphs by domain

### Phase 3: Documentation (1-2 hours)
1. **Update YAML frontmatter**: Add prerequisite_lessons field to lesson.md
2. **Create assessment rubrics**: Define how proficiency is assessed at each level
3. **Build learning pathways**: Show students progression routes
4. **Generate skill badges**: Prepare student-facing skill documentation

---

## Tools for Ongoing Validation

### For Chapter-Planner Agent
**Before approving new skills**, check:
```yaml
new_skill:
  name: "Skill Name"
  check_against_registry: "Is this unique or duplicate?"
  check_prerequisites: "Are all prerequisites taught earlier?"
  check_proficiency: "Does progression make sense?"
  check_coherence: "Does skill fit into one progression track?"
```

### For Lesson-Writer Agent
**In Step 2: Validate Skills Proficiency Alignment**, ensure:
```yaml
skill_validation:
  proficiency_match: "Does content match stated CEFR level?"
  cognitive_load: "Count new concepts against CEFR limits"
  bloom_alignment: "Verify cognitive level matches proficiency"
  prerequisite_coverage: "Are prerequisites taught?"
```

### For Instructors/Reviewers
**When reviewing chapters**, ask:
- [ ] Are all skills in the registry?
- [ ] Do skills show progression across chapters?
- [ ] Are prerequisites taught before advanced skills?
- [ ] Does this chapter add new skills or deepen existing ones?
- [ ] Is cognitive load appropriate?

---

## Next Steps for You

### Immediate
1. **Review the two validation documents**:
   - `specs/part-1-skills-registry.md` — Master reference
   - `specs/part-1-skill-progression-map.md` — Coherence analysis

2. **Understand the four domains**:
   - Development (tools, patterns, stack, methodology)
   - Business (opportunity, strategy, requirements)
   - Career (roles, learning, M-shaped)
   - Systems (pillars, completeness, interdependencies)

3. **Identify any Part 1 skills you want to fix** (before starting Part 2)

### Before Part 2
1. **Run cleanup** if you found issues in Part 1
2. **Apply validation checklist** to first Part 2 lesson
3. **Update chapter-planner** to enforce prerequisite checking

### Ongoing
1. **Use Master Skill Registry** as reference when planning chapters
2. **Check progression tracks** when sequencing lessons
3. **Validate using checklist** before finalizing skills metadata

---

## Key Takeaways

✓ **Uniqueness**: Each skill should have one canonical name used consistently
✓ **Progression**: A1 → A2 → B1 → B2 (never regress)
✓ **Prerequisites**: A2 must have A1; B1 must have A2
✓ **Connectivity**: Skills should build on each other across chapters
✓ **Naming**: Use distinct verbs; be specific (not just "Understanding X")
✓ **Coherence**: Skills should fit into one of four progression tracks
✓ **Assessment**: Proficiency must be measurable at each level

Use the **Master Skill Registry** and **Progression Map** as your reference. They're the source of truth for Part 1 and the template for Part 2+.
