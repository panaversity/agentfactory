---
sidebar_position: 6
title: "Skill Creation + Polish"
proficiency_level: "B1"
estimated_time: "90 minutes"
learning_objectives:
  - "Identify recurring patterns across completed features"
  - "Create Persona + Questions + Principles skill files"
  - "Apply skills to new problems (Feature 5)"
  - "Validate skill completeness and reusability"
created: "2025-11-25"
version: "1.0.0"
---

# Skill Creation + Polish

You've built four features in less time than Feature 1 alone consumed. That acceleration wasn't luck. You solved the same problem patterns multiple times: JSON transformations appeared in Features 1, 2, 3, and 4. The specification structure you refined for Lead Profiler shaped every feature that followed. Error handling approaches you developed once got reused four times without rethinking.

This is intelligence accumulation. Patterns that repeat 2+ times with 5+ decision points become skills. Skills encode knowledge so future projects benefit immediately—no re-solving, no re-discovering, no re-deciding.

Now you'll formalize those patterns into actual reusable skills using the **P+Q+P Framework: Persona + Questions + Principles**. You'll create real skill files you can invoke in future projects.

## Identify Recurring Patterns

Open your project folder. Review each feature's spec AND implementation.

**Go through each feature file and find moments where you solved the same problem**:

**Feature 1: Lead Profiler**
- How did you structure the output JSON?
- How did you handle invalid/missing website data?
- What validation happened before returning results?

**Feature 2: ICP Scorer**
- What input format did it expect from Feature 1's output?
- How did you calculate the score?
- How did you document the scoring breakdown?

**Feature 3: Outreach Generator**
- How did you structure the outreach message?
- What data from Features 1-2 did you use?
- How did you handle cases where ICP score was too low?

**Feature 4: Campaign Dashboard**
- How did you aggregate data from Features 1-3?
- What formatting rules did you apply for display?
- How did you handle missing or incomplete records?

**Now list your top 2-3 patterns** (things you solved multiple times):

1. _________________ (appeared in features: _____)
2. _________________ (appeared in features: _____)
3. _________________ (appeared in features: _____) [optional]

**Examples of patterns to look for**:
- **JSON data transformer**: Input JSON → processing → structured JSON output (likely Features 1, 2, 3, 4)
- **Specification structure**: Input/Output/Success Criteria/Error Cases template (Features 1, 2, 3, 4)
- **Pipeline connector**: Output of Feature N feeds input of Feature N+1 (Features 1→2, 2→3, 3→4)
- **Error handler**: Validation, missing fields, graceful failure (probably all features)
- **Data validator**: Schema validation before transformation (likely multiple features)
- **CLI/Output formatter**: Terminal output, JSON display, summaries (Feature 4 and earlier)

## Create Your First Skill File

Choose your most valuable pattern—the one you want every future project to inherit. This should be a pattern you found in step 1 above (ideally appearing in 3+ features).

### Step 1: Create the File

```bash
mkdir -p .claude/skills
touch .claude/skills/[your-pattern-name].md
```

Replace `[your-pattern-name]` with your pattern name in kebab-case (example: `json-data-transformer`).

### Step 2: Write the Persona Section

Your persona should describe:
- Your role in building this pattern
- Your values (what matters to you)
- What you prioritize when making decisions

**Template**:
```markdown
## Persona
You are a [specific role]. You [value system]. You prioritize [decision criteria].
```

**Example for JSON Data Transformer**:
```markdown
## Persona
You are a data pipeline architect who transforms JSON inputs into structured JSON outputs.
You value schema consistency and clear error messages. You prioritize graceful failure and explicit validation.
```

### Step 3: Write 5 Key Questions

These are the design decisions you ask BEFORE implementing the pattern. They should:
- Force you to think about requirements explicitly
- Address edge cases and constraints
- Clarify inputs and outputs

**Template**:
```markdown
## Questions (Ask Before Implementing)
1. [Question about inputs or requirements]
2. [Question about outputs or constraints]
3. [Question about error cases]
4. [Question about validation or edge cases]
5. [Question about testing or verification]
```

**Example for JSON Data Transformer**:
```markdown
## Questions (Ask Before Implementing)
1. What is the input JSON schema? (required fields, optional fields, nested structures?)
2. What is the output JSON schema? (exact field names, types, nested structure?)
3. What happens when required input fields are missing or invalid?
4. Should transformation preserve original fields or only output derived fields?
5. What validation should occur before transformation? (type checking, range validation?)
```

### Step 4: Write 5 Principles

These are the implementation patterns you apply while building. They should:
- Guide implementation choices
- Explain why each choice matters
- Be testable (you can verify you followed them)

**Template**:
```markdown
## Principles (Apply During Implementation)
- **[Name]**: [Why this matters and when to apply it]
- **[Name]**: [Why this matters and when to apply it]
...
```

**Example for JSON Data Transformer**:
```markdown
## Principles (Apply During Implementation)
- **Schema First**: Define output schema before writing transformation logic (prevents rework)
- **Fail Explicitly**: Missing required fields → error with clear message, not silent default (easier debugging)
- **Preserve Context**: Include source reference or metadata in output when relevant (helps trace data lineage)
- **Type Safety**: Validate input types match expected schema before processing (catches errors early)
- **Test Edge Cases**: Empty arrays, null values, missing optional fields, boundary values (ensures robustness)
```

### Step 5: Write the Example Application

This should show how you ACTUALLY used this pattern in one of Features 1-4. Be specific:
- Which feature did you use this pattern in?
- What was the concrete input and output?
- Which questions did you answer?
- Which principles did you apply?

**Example for JSON Data Transformer**:
```markdown
## Example Application
When building ICP Scorer (Feature 2), you transformed Lead Profile JSON into Score JSON:

**Input**: Lead Profile from Feature 1
```json
{
  "company_name": "Acme Corp",
  "industry": "manufacturing",
  "size": "enterprise",
  "tech_indicators": ["AWS", "Python"],
  "confidence_score": 85
}
```

**Output**: ICP Score JSON
```json
{
  "lead_id": "acme-corp",
  "score": 78,
  "category": "high_fit",
  "breakdown": {
    "industry_match": 90,
    "size_match": 85,
    "tech_match": 60
  },
  "reasoning": "Enterprise + matching industry, slight tech gap"
}
```

**Questions you answered**:
- Input schema: Lead Profile with 5 fields (answered Q1)
- Output schema: Score + category + breakdown (answered Q2)
- Error handling: Missing fields return score 0 with "incomplete profile" message (answered Q3)
- Preserve or transform: Transformed only derived fields (answered Q4)
- Validation: Type check all fields before processing (answered Q5)

**Principles you applied**:
- Schema First: Wrote the Score JSON schema before scoring logic
- Fail Explicitly: Missing company_name returns error, not null score
- Preserve Context: Included breakdown so others can understand score
- Type Safety: Validated industry was string, size was enum before scoring
- Test Edge Cases: Tested empty tech_indicators, null confidence_score, missing fields
```

### Step 6: Create Your Actual Skill File

Now create `.claude/skills/[your-pattern-name].md` with all five sections. Use the structure above as a template.

**Your skill file checklist**:
- [ ] Persona describes your role and values (1-2 sentences)
- [ ] 5 Questions are specific and testable (not generic)
- [ ] 5 Principles explain WHY they matter (not just WHAT to do)
- [ ] Example Application shows real code/data from one of your features (not hypothetical)
- [ ] Example answers each of the 5 Questions
- [ ] Example shows which Principles you applied and where

## Create Your Second Skill File

Choose another pattern—ideally different from your first. If your first skill is data-focused (JSON transformer), your second might be structural (spec writing) or operational (error handling).

Review the pattern candidates below. Which one did you use in 2+ features?

**Example candidates** (with indicators of where they appeared):

**Pattern 1: Feature Specification Writer**
- Appears in: Every feature (F1, F2, F3, F4)
- Signs you used it: Spec.md for each feature, consistent Input/Output/Success Criteria structure
- Questions it answers: How to structure intent clearly? What belongs in each section?

**Pattern 2: Error Handler**
- Appears in: Any feature with validation (likely all 4)
- Signs you used it: Error messages, validation logic, graceful failure handling
- Questions it answers: When to fail vs. gracefully degrade? What error info to return?

**Pattern 3: Data Validator**
- Appears in: Any feature accepting input (likely F1, F2, F3)
- Signs you used it: Schema checking, field validation, type validation
- Questions it answers: What makes data valid? When to reject vs. transform?

**Pattern 4: CLI Output Formatter**
- Appears in: Feature 4, possibly others
- Signs you used it: Table formatting, summary display, list organization
- Questions it answers: How to display data clearly? What detail to show/hide?

### Step 1: Choose Your Second Pattern

**Decision**: Which pattern appeared in your features and would be most valuable to reuse?

Pattern name: _______________________

Features where it appeared: _________, _________, _________

### Step 2: Use the Template Structure

Follow the same five-section template from your first skill:

1. **Persona** (1-2 sentences describing your role and values)
2. **Questions** (5 design decisions to ask before implementing)
3. **Principles** (5 implementation patterns and why they matter)
4. **Example Application** (real code/data from one of your features showing how you applied it)

### Step 3: Be Specific and Concrete

Your second skill should be SPECIFIC to your actual work, not generic. Examples:

**Too generic** (bad):
- Persona: "You write good error messages"
- Principle: "Be clear about errors"

**Specific and testable** (good):
- Persona: "You are a data validation expert who returns structured errors with field-level diagnostics"
- Principle: "Validation Errors Are Data": Return field names, error types, and suggested values (not vague "error" messages)

### Step 4: Create the File

```bash
touch .claude/skills/[your-second-pattern-name].md
```

Write the complete skill with all five sections. Use your first skill as a template for format and depth.

**Your second skill file checklist**:
- [ ] Different focus than first skill (not another data transformer)
- [ ] Persona is specific to how YOU solved this problem
- [ ] 5 Questions are testable (can you verify you asked them?)
- [ ] 5 Principles are grounded in your actual experience (not theory)
- [ ] Example shows real code or structure from one of your features
- [ ] Example proves you can apply the skill again (to another feature or project)

## Test Skills Against Feature 5

Now test whether your skills are actually reusable. Build a Feature 5 specification using ONLY your skills' Questions and Principles. Don't look at Features 1-4 implementations—instead, let your skills guide the design.

**Feature 5: Lead Nurture Sequencer**

**Scenario**: After outreach sent, no response in 3 days → recommend next action with timing and message variant.

**High-level requirement**: Takes combined lead data (lead profile + ICP score + outreach history) and recommends the next action with timing and variant message.

### Part 1: Apply Your First Skill (Answer the Questions)

Open your first skill file and read its 5 Questions. For Feature 5, answer each question:

**Question 1**: [Your first skill's Q1]
**Your answer for Feature 5**: _______________________________________________

**Question 2**: [Your first skill's Q2]
**Your answer for Feature 5**: _______________________________________________

**Question 3**: [Your first skill's Q3]
**Your answer for Feature 5**: _______________________________________________

**Question 4**: [Your first skill's Q4]
**Your answer for Feature 5**: _______________________________________________

**Question 5**: [Your first skill's Q5]
**Your answer for Feature 5**: _______________________________________________

**Example** (if your first skill is "JSON Data Transformer"):

**Q1: What is the input JSON schema?**
**Answer**: Lead Nurture input combines three data sources:
```json
{
  "lead_profile": { /* from Feature 1 */ },
  "icp_score": { /* from Feature 2 */ },
  "outreach_history": [
    { "sent_date": "2025-11-22", "status": "sent", "variant_id": 1 },
    { "sent_date": "2025-11-25", "status": "no_response", "variant_id": 1 }
  ]
}
```
Required fields: lead_profile, icp_score, outreach_history (non-empty)

**Q2: What is the output JSON schema?**
**Answer**: Nurture recommendation output:
```json
{
  "lead_id": "string",
  "next_action": "follow_up" | "escalate" | "pause",
  "recommended_timing": "immediate" | "1_day" | "3_days",
  "message_variant": 1 | 2 | 3,
  "reasoning": "string",
  "confidence": 0-100
}
```

**Q3: What happens when required fields are missing?**
**Answer**: If outreach_history is empty → error "Cannot recommend nurture action without prior outreach". If lead_profile incomplete → return confidence < 50 with note.

**Q4: Should output preserve original fields or only derived fields?**
**Answer**: Only derived recommendation fields (action, timing, variant, reasoning). Don't repeat lead data. If caller needs to correlate, they have lead_id.

**Q5: What validation before transformation?**
**Answer**: Validate outreach_history has at least one entry. Validate ICP score is 0-100. Validate lead_profile has company_name and industry.

### Part 2: Apply Your First Skill (Review the Principles)

Now read your first skill's 5 Principles. For each principle, identify how it applies to Feature 5:

**Principle 1**: ____________________
**How it applies to Feature 5**: _______________________________________________

**Principle 2**: ____________________
**How it applies to Feature 5**: _______________________________________________

**Principle 3**: ____________________
**How it applies to Feature 5**: _______________________________________________

**Principle 4**: ____________________
**How it applies to Feature 5**: _______________________________________________

**Principle 5**: ____________________
**How it applies to Feature 5**: _______________________________________________

**Example** (if your first skill has "Schema First" principle):

**Schema First**: Define output schema before writing recommendation logic
**How it applies**: Write the nurture recommendation JSON schema (action, timing, variant, reasoning, confidence) BEFORE writing the logic that decides which action to recommend. This prevents logic creep where recommendations grow unstructured.

### Part 3: Apply Your Second Skill

Using your second skill's Questions and Principles, answer the same way:

**Your second skill's 5 Questions:**

Q1: [Your skill's question 1 applied to Feature 5]
**Answer**: _____________________________________________________________________

Q2: [Your skill's question 2 applied to Feature 5]
**Answer**: _____________________________________________________________________

Q3: [Your skill's question 3 applied to Feature 5]
**Answer**: _____________________________________________________________________

Q4: [Your skill's question 4 applied to Feature 5]
**Answer**: _____________________________________________________________________

Q5: [Your skill's question 5 applied to Feature 5]
**Answer**: _____________________________________________________________________

**Your second skill's 5 Principles** (how they apply):

P1: ____________ → applies to Feature 5 by: _________________________________

P2: ____________ → applies to Feature 5 by: _________________________________

P3: ____________ → applies to Feature 5 by: _________________________________

P4: ____________ → applies to Feature 5 by: _________________________________

P5: ____________ → applies to Feature 5 by: _________________________________

### Part 4: Validate Your Design Decisions

After answering all questions and principles from both skills, you should have a complete Feature 5 design. Check:

**Input Completeness**:
- [ ] You defined exactly what data Feature 5 receives
- [ ] You validated what makes input acceptable vs. incomplete
- [ ] You know what happens if data is missing

**Output Completeness**:
- [ ] You defined exactly what Feature 5 returns
- [ ] The output schema is clear and machine-parseable (likely JSON)
- [ ] Downstream features could use this output

**Error Handling**:
- [ ] You identified 3+ error cases (missing history, low confidence, etc.)
- [ ] You decided whether each error fails or gracefully degrades
- [ ] Error messages would help someone debug the recommendation

**Reusability**:
- [ ] Someone else could read your skills and build Feature 5 independently
- [ ] Your skills answered 80%+ of design decisions
- [ ] The 20% remaining decisions are feature-specific (not skill gaps)

### Part 5: Identify Skill Gaps

If you found design decisions NOT answered by your skills:

**Decision missing from first skill**: _______________________________________________

**Why was it missing?** (too specific? wrong pattern? new question needed?)

**Decision missing from second skill**: ______________________________________________

**Why was it missing?**

**Should you refine your skills?**
- If 1-2 gaps: Minor refinement (add a Question or Principle)
- If 3+ gaps: Major gap (rethink whether this is the right pattern for a skill)

**Refine your skill files now** if needed.

## Create Your Third Skill (Optional)

Look back at Features 1-4. Did you solve any other patterns repeatedly?

**Candidates**:
- **Pipeline Connector**: How output of one feature becomes input for the next (appears in F1→F2, F2→F3, F3→F4)
- **Specification Template**: Consistent structure across all specs (appears in F1, F2, F3, F4)
- **API/CLI Interface**: How features expose inputs and outputs (appears across all features)
- **Logging and Debugging**: Error messages, diagnostics, traces (appears if you added instrumentation)

**Should you create a third skill?**

Ask yourself:
- Did this pattern appear in 2+ features? (Yes → create skill)
- Does it have 5+ decision points? (Yes → create skill)
- Would future projects benefit from encoding this? (Yes → create skill)

**If all three are YES, create your third skill file** using the same five-section template.

**If NO to any, stop here.** Two skills are often sufficient. Quality > quantity.

---

## Try With AI

Use AI to validate and improve your skills.

**Prompt 1: Get Feedback on Your First Skill**

"Review this skill definition for reusability:

[PASTE YOUR FIRST SKILL FILE]

Answer these questions:
1. Does the P+Q+P structure actually cover the design decisions needed to implement this pattern again?
2. What key Questions or Principles am I missing?
3. What parts are too specific and should be removed?
4. If someone used only this skill (no other context), could they build this pattern correctly?

Be direct about gaps."

**Expected response**: AI should identify:
- Questions that are too vague or domain-specific
- Missing principles (e.g., if you didn't mention testing)
- Principles that are redundant or overlap
- Whether your example application is convincing

**How to use feedback**:
- Add missing questions → refined skill is more reusable
- Consolidate overlapping principles → cleaner, clearer skill
- Remove too-specific details → transferable to new domains
- If AI says example isn't convincing → add concrete code or data

**Prompt 2: Test Transferability**

"I built a sales pipeline with these two skills:

[SKILL 1 NAME]: [BRIEF DESCRIPTION]
[SKILL 2 NAME]: [BRIEF DESCRIPTION]

Now I'm building a completely different pipeline for B2B hiring (not sales). The features are similar structure (1. Candidate Profiler, 2. Fit Scorer, 3. Outreach Generator, 4. Pipeline Dashboard).

Which of my two skills would transfer directly to the hiring pipeline? Which would need adaptation? What third skill pattern should I create that's specific to hiring?"

**Expected response**: AI should:
- Identify which skills transfer unchanged (likely JSON Data Transformer works for both)
- Identify which skills need adaptation (spec template might be slightly different)
- Suggest what pattern is unique to hiring (e.g., Skills Matcher, Interview Scheduler)

**How to use feedback**:
- Skills that transfer unchanged → prove high reusability (good sign)
- Skills that need adaptation → identify what changes and why
- New pattern suggestion → validate whether you should create a third skill
