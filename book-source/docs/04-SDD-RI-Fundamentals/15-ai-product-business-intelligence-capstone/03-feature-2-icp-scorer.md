---
sidebar_position: 3
title: "Feature 2: ICP Scorer"
proficiency_level: "B1"
estimated_time: "30-45 minutes"
cognitive_load:
  new_concepts: 4
  reused_concepts: 6
generated_by: content-implementer v1.0.0
created: "2025-11-25"
workflow: "/sp.implement"
---

# Feature 2: ICP Scorer

Your Feature 1 (Lead Profiler) outputs structured lead profiles. Feature 2 scores those profiles against your Ideal Customer Profile (ICP) criteria and outputs a priority category (hot/warm/cold).

This is where you experience **intelligence acceleration in action**: The specification→plan→tasks→implement workflow you executed for F1 directly transfers to F2. You're applying the same decision-making process, reusing the same SDD-RI rhythm, and leveraging patterns you've already proven work.

**This feature should build in 60-70% of your F1 time.**

**Start your timer now.** Check your F1 duration from your notes: _____ minutes. Your goal: complete F2 in _____ minutes (60-70% of F1).

## Step 1: Define ICP Criteria (5-10 minutes)

Before writing the specification, define what makes a customer ideal **for your business**. Create this configuration file:

```bash
touch icp_criteria.json
```

Add your scoring criteria with weights. This is YOUR decision—tailor it to your product vision:

```json
{
  "criteria": [
    {
      "name": "company_size",
      "description": "Prefer established companies (SMB and enterprise over startups)",
      "weights": {
        "startup": 20,
        "smb": 80,
        "enterprise": 100
      }
    },
    {
      "name": "industry_fit",
      "description": "Tech and finance have highest use case fit",
      "weights": {
        "technology": 100,
        "finance": 90,
        "healthcare": 70,
        "retail": 50,
        "other": 30
      }
    },
    {
      "name": "tech_sophistication",
      "description": "Higher tech adoption indicates readiness for AI tools",
      "calculation": "10 points per tech indicator, max 30 points"
    },
    {
      "name": "pain_point_alignment",
      "description": "Mentions of pain points your product solves",
      "keywords": ["efficiency", "automation", "scale", "integration"],
      "points_per_match": 15
    }
  ],
  "score_ranges": {
    "hot": {
      "min": 80,
      "max": 100,
      "label": "High priority lead"
    },
    "warm": {
      "min": 50,
      "max": 79,
      "label": "Worth pursuing"
    },
    "cold": {
      "min": 0,
      "max": 49,
      "label": "Low priority"
    }
  }
}
```

**Save this file now.** You've defined your first business rule—the criteria your ICP Scorer will use to evaluate every lead.

## Step 2: Understand Data Flow (Pipeline Architecture)

This is critical: **F2 consumes F1's output directly.** You're not building in isolation—you're building part of a pipeline.

**F1 Output** (what Lead Profiler generates):
```json
{
  "company_name": "Stripe",
  "industry": "technology",
  "estimated_size": "enterprise",
  "tech_indicators": ["API-driven architecture", "Developer-friendly", "Cloud infrastructure"],
  "pain_points": ["Scale management", "Integration complexity"],
  "confidence_score": 92
}
```

**What F2 Does** (takes F1 output, applies your ICP criteria):
- Reads `estimated_size` → matches against your size weights
- Reads `industry` → matches against your industry weights
- Counts `tech_indicators` → calculates sophistication score
- Searches `pain_points` for your keywords → calculates alignment score
- Sums scores → produces final priority

**F2 Output** (what ICP Scorer produces):
```json
{
  "score": 87,
  "category": "hot",
  "breakdown": {
    "company_size_score": 100,
    "industry_fit_score": 100,
    "tech_sophistication_score": 30,
    "pain_point_score": 15
  },
  "reasoning": "Enterprise technology company with strong tech indicators. Mentions 'scale' and 'integration'—both pain points we address. High priority lead."
}
```

**Connection**: F1's output JSON structure MUST match F2's input expectations. This is the contract between features.

## Step 3: Write the Specification (10-15 minutes)

**What you're reusing from F1**:
- The `/sp.specify` workflow itself (same command)
- The structure: define intent, input/output contracts, success criteria
- The validation approach: verify spec against implementation

**Run specification workflow**:
```bash
/sp.specify
```

**Provide this feature description**:
```
ICP Scorer: Takes a Lead Profile JSON (from Feature 1) and scores it against
Ideal Customer Profile criteria defined in icp_criteria.json. Outputs a numeric
score (0-100), priority category (hot/warm/cold), per-criterion breakdown, and
business reasoning.
```

**Your spec.md should define these sections**:

**Input Contract**:
- Source: Lead Profile JSON from Feature 1 (lead_profiler.py output)
- Schema: `{company_name, industry, estimated_size, tech_indicators[], pain_points[], confidence_score}`
- Example: Show a Stripe profile flowing into F2

**Output Contract**:
```json
{
  "score": "number (0-100)",
  "category": "string (hot|warm|cold)",
  "breakdown": {
    "company_size_score": "number",
    "industry_fit_score": "number",
    "tech_sophistication_score": "number",
    "pain_point_alignment_score": "number"
  },
  "reasoning": "string (1-2 sentences explaining the score)"
}
```

**Success Criteria** (these must be testable):
- ✓ Same input produces identical score every run (deterministic)
- ✓ Category assignment matches score_ranges from icp_criteria.json
- ✓ Breakdown totals approximately equal final score
- ✓ Reasoning references specific data from lead profile (company name, tech indicators, pain point matches)
- ✓ Handles missing fields (null pain_points, etc.) without crashing

**Constraints**:
- Load ICP criteria from icp_criteria.json (file-based, not hardcoded)
- Accept input as JSON file OR stdin (enables pipeline: `lead_profiler | icp_scorer`)
- Reasoning should be business-focused (why this lead matters for sales), not technical

## Step 4: Plan, Tasks, Implement (15-20 minutes)

**The acceleration happens here.** You've executed this sequence once (F1). This time, you're doing it faster because the workflow is familiar.

**Generate your implementation plan**:
```bash
/sp.plan
```

Review it:
```bash
cat .specify/specs/*/plan.md
```

**Key decision point**: How much of F1's pattern will you reuse? Think:
- Did F1 use environment variables for API keys? Use same approach.
- Did F1 have helper functions for JSON validation? Adapt them for F2.
- Did F1 handle errors with try/except blocks? Reuse that pattern.

**Generate tasks**:
```bash
/sp.tasks
```

**Critical tasks to watch for** (these appeared in F1 and should be faster now):
1. JSON input parsing (you've done this)
2. Criteria matching and weighting logic (new logic, but familiar structure)
3. Score aggregation (new calculation, similar flow to F1)
4. Category assignment (straightforward—lookup table)
5. Reasoning generation (new, but similar to F1's output formatting)

**Execute implementation**:
```bash
/sp.implement
```

**Work systematically with AI**. Ask specific questions:
- "How should I structure the scoring function to match my icp_criteria.json schema?"
- "For pain_point_alignment, should I do substring matching, regex, or exact matches? Trade-offs?"
- "How do I pipe Feature 1 output directly to Feature 2 without intermediate files?"

**Record decisions** (you'll explain these in retrospective):
- What patterns transferred directly from F1?
- What required new logic?
- Where did F2 deviate from F1's approach? Why?

## Step 5: Test Feature 2 + Pipeline (8-12 minutes)

### Test 1: F2 Standalone (Validate Input/Output Contract)

Test Feature 2 with a sample lead profile:

```bash
# Create test input (copy your earlier F1 output)
cat > test_lead.json << 'EOF'
{
  "company_name": "Stripe",
  "industry": "technology",
  "estimated_size": "enterprise",
  "tech_indicators": ["API-driven architecture", "Developer-friendly", "Cloud infrastructure"],
  "pain_points": ["Scale management", "Integration complexity"],
  "confidence_score": 92
}
EOF

# Run F2 with test input
python icp_scorer.py test_lead.json
```

**Verify output**:
- [ ] Score is between 0-100
- [ ] Category is hot/warm/cold
- [ ] Breakdown has all 4 scores
- [ ] Reasoning references the lead's specific data (company name, industry, pain points)
- [ ] Output is valid JSON

### Test 2: Full Pipeline (F1 → F2)

This is the real test—does F2 actually consume F1's output?

**Option A: Using saved F1 output**
```bash
# If you saved F1 output earlier
python icp_scorer.py lead_profile.json
```

**Option B: Real-time pipeline (no intermediate files)**
```bash
# Generate F1 output and pipe directly to F2
python lead_profiler.py https://stripe.com | python icp_scorer.py
```

This is acceleration in action: F1's JSON flows directly into F2. No manual steps, no file copying.

**Test multiple companies** to validate your ICP criteria:
```bash
# Test 3-5 different URLs
python lead_profiler.py https://openai.com | python icp_scorer.py
python lead_profiler.py https://github.com | python icp_scorer.py
python lead_profiler.py https://yourcompany.com | python icp_scorer.py
```

**Observe patterns**:
- Which companies score hot? (Match your ICP)
- Which score cold? (Different profile)
- Do the scores make sense for your business?

### Test 3: Edge Cases

Test scenarios that F1 might produce:

```bash
# Missing pain_points
cat > edge_case.json << 'EOF'
{
  "company_name": "NoData Inc",
  "industry": "unknown",
  "estimated_size": "startup",
  "tech_indicators": [],
  "pain_points": null,
  "confidence_score": 30
}
EOF

python icp_scorer.py edge_case.json
```

Does it handle gracefully? Score should be calculable even with missing data.

**Stop your timer now.** You've completed Feature 2 from specification through pipeline testing.

## Record Your Acceleration

Open `TIME_TRACKER.md` (or create it):

```markdown
# Intelligence Acceleration Tracking

## Feature 1: Lead Profiler
- Start time: [time]
- End time: [time]
- **Duration: _____ minutes** (BASELINE)

## Feature 2: ICP Scorer
- Start time: [time]
- End time: [time]
- **Duration: _____ minutes**
- **% of F1 time: _____%** (Goal: 60-70%)

## Pattern Reuse: What Transferred from F1?
- [ ] JSON input parsing structure
- [ ] Error handling approach
- [ ] Configuration file pattern (icp_criteria.json similar to other config)
- [ ] Output formatting (JSON structure)
- [ ] Command-line interface pattern
- [ ] Pipeline capability (stdin/stdout)

## What Was Different in F2?
(Describe any decisions that deviated from F1's approach)

---
```

**Analyze your acceleration**:
- If F2 took 60-70% of F1 time: Patterns transferred effectively
- If F2 took >70% of F1 time: Identify what wasn't reused
- If F2 took <60% of F1 time: You found shortcuts (document them for F3)

## Try With AI

Work with your AI companion to deepen understanding of acceleration patterns and scoring logic.

**Prompt 1: Analyze Your Acceleration**
```
I built Feature 1 (Lead Profiler) in [X] minutes and Feature 2 (ICP Scorer) in [Y] minutes.

Analyze which patterns I reused from F1 to F2:
1. What code structures transferred directly?
2. What required new logic?
3. Why did F2 build faster? (or slower?)
4. What patterns should I prepare to reuse for Feature 3?
```

**Expected outcome**: AI identifies specific patterns (JSON parsing, error handling, CLI structure) that accelerated F2. You capture this for Feature 3 planning.

---

**Prompt 2: Debug Your Scoring Logic**
```
My ICP criteria are [paste icp_criteria.json].

Here's a sample lead profile from Feature 1:
[paste your F1 output JSON]

Walk me through step-by-step:
1. How would you score this lead using my criteria?
2. Show the calculation for each criterion
3. What final score and category would this get?
4. If the score surprises you, what ICP weights should I adjust?
```

**Expected outcome**: AI demonstrates scoring logic transparently. You validate if your ICP criteria align with your actual business preferences.
