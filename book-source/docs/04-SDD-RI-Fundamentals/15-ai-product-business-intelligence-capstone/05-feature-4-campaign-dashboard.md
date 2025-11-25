---
sidebar_position: 5
title: "Feature 4: Campaign Dashboard"
---

# Feature 4: Campaign Dashboard

You've built three features. Now you aggregate them into a unified dashboard—your proof of intelligence acceleration. This is the test: **your target is to complete Feature 4 in less than 50% of your Feature 1 time**.

**Start your timer now.** Pull your Feature 1 duration from Lesson 02 and write it here:

```
F1 Time (from Lesson 02): _____ minutes
F4 Target (50% of F1):    _____ minutes
```

Calculate: F1 time ÷ 2. That's your ceiling. Can you build F4 underneath it?

## Hands-On: Dashboard Scope (3 minutes)

Stop reading. Open a terminal. You'll build a single Python script that reads the outputs from F1, F2, and F3 and combines them into one view.

**Your dashboard displays:**
- All processed leads in a table
- ICP scores with visual indicators (emoji or symbols)
- Outreach message previews (first 50 characters)
- Campaign summary statistics (total leads, category counts, average score)

**Output format**: CLI table (fastest path to 50% target).

Write down your scope now:

```
Dashboard Input: ./campaign_data/
  ├── profiles/ (from Feature 1)
  ├── scores/ (from Feature 2)
  └── outreach/ (from Feature 3)

Dashboard Output: Formatted CLI table with summary

Script Name: campaign_dashboard.py
```

## Hands-On: Write Your Specification (5 minutes)

Open `specs/feature-4-dashboard/spec.md`. This spec should be **much shorter** than F1 because you're reusing familiar structure.

**Your spec must include:**

```yaml
intent: |
  Aggregate processed leads from Feature 1-3 into a single CLI table.
  Display company info, ICP scores, and outreach previews.

inputs:
  - directory: ./campaign_data/profiles/ (F1 output)
  - directory: ./campaign_data/scores/ (F2 output)
  - directory: ./campaign_data/outreach/ (F3 output)

outputs:
  - CLI table with columns: company_name, industry, icp_score, category, outreach_preview
  - Summary section: total_leads, hot_count, warm_count, cold_count, average_score

success_criteria:
  - All leads appear in table
  - Sorted by ICP score (descending)
  - Categories show with emoji (hot=🔥, warm=🟡, cold=🔵)
  - Summary stats calculated correctly
  - Handles empty campaign gracefully

constraints:
  - Single Python script
  - Reuse JSON parsing from F1-F3
  - No new dependencies
  - Console output only
```

**Key reuse pattern:** You're not building a new JSON parser. You're importing the parsing logic you already wrote in F1-F3.

## Hands-On: Plan and Implement (10-15 minutes)

Execute the workflow:

```bash
/sp.plan
/sp.tasks
/sp.implement
```

**Speed signals (you should see these):**
- Plan is 60-70% shorter than F1 plan (you know the pattern)
- Tasks reference F1-F3 by name (e.g., "Reuse JSON parsing from Feature 1")
- Implementation pulls existing code, doesn't rewrite it

**If your implementation takes longer than expected:**
- Did you copy-paste F1-F3 code instead of importing it? (Wrong—adds lines)
- Did you add features beyond the spec? (Wrong—scope creep)
- Did you write a new JSON parser instead of reusing F1? (Wrong—ignores intelligence)

**Time check:** Are you still under your F1 time?

## Hands-On: Create Test Data (3 minutes)

Run all three companies through your Feature 1-3 pipeline:

```bash
mkdir -p campaign_data/{profiles,scores,outreach}

# Stripe
python lead_profiler.py https://stripe.com > campaign_data/profiles/stripe.json
python icp_scorer.py campaign_data/profiles/stripe.json > campaign_data/scores/stripe.json
python outreach_generator.py campaign_data/profiles/stripe.json campaign_data/scores/stripe.json > campaign_data/outreach/stripe.json

# Shopify
python lead_profiler.py https://shopify.com > campaign_data/profiles/shopify.json
python icp_scorer.py campaign_data/profiles/shopify.json > campaign_data/scores/shopify.json
python outreach_generator.py campaign_data/profiles/shopify.json campaign_data/scores/shopify.json > campaign_data/outreach/shopify.json

# Notion
python lead_profiler.py https://notion.so > campaign_data/profiles/notion.json
python icp_scorer.py campaign_data/profiles/notion.json > campaign_data/scores/notion.json
python outreach_generator.py campaign_data/profiles/notion.json campaign_data/scores/notion.json > campaign_data/outreach/notion.json
```

**Verify data created:**

```bash
ls campaign_data/profiles/
ls campaign_data/scores/
ls campaign_data/outreach/
```

Each should have 3 files: stripe.json, shopify.json, notion.json.

## Hands-On: Run Your Dashboard (2 minutes)

Execute your implementation:

```bash
python campaign_dashboard.py ./campaign_data/
```

**Expected output (your actual numbers may differ):**

```
╔════════════════════════════════════════════════════════════════════════════╗
║                   AI SALES ASSISTANT - CAMPAIGN DASHBOARD                  ║
╠════════════════════════════════════════════════════════════════════════════╣
║ Company   │ Industry     │ Score │ Category   │ Outreach Preview          ║
╠═══════════╪══════════════╪═══════╪════════════╪═══════════════════════════╣
║ Stripe    │ Fintech      │ 87    │ 🔥 HOT     │ "Hi there, I noticed..."  ║
║ Shopify   │ E-commerce   │ 72    │ 🟡 WARM    │ "Hi there, I've been..."  ║
║ Notion    │ Productivity │ 65    │ 🟡 WARM    │ "Hi there, I've been..."  ║
╠════════════════════════════════════════════════════════════════════════════╣
║ SUMMARY   │ 3 total │ 1 hot │ 2 warm │ 0 cold │ Average: 74.7             ║
╚════════════════════════════════════════════════════════════════════════════╝
```

**Verification checklist** (all must pass):

- ✓ All 3 leads appear in table
- ✓ Table sorted by score (highest first)
- ✓ Category shows with emoji
- ✓ Outreach preview truncated to ~50 chars
- ✓ Summary stats correct

If any fail, check:
- Is your F1 JSON output valid?
- Is your F2 scoring logic correct?
- Did you handle the JSON file paths correctly?

## Hands-On: Measure Your Acceleration

**Stop your timer now.**

Get your Feature 1 time from Lesson 02. Fill in your actual times:

| Feature | Your Time | % of F1 | Target | Status |
|---------|-----------|---------|--------|--------|
| F1: Lead Profiler | ___ min | 100% | 100% | Baseline |
| F2: ICP Scorer | ___ min | ___% | <100% | Should be faster |
| F3: Outreach Generator | ___ min | ___% | <75% | Should be faster |
| F4: Campaign Dashboard | ___ min | ___% | **<50%** | THE TEST |

**The test result:**

```
If F4 time < 50% of F1 time:
  ✓ Intelligence accumulation proven
  ✓ Reusable patterns work
  ✓ You've proven spec-driven development compounds

If F4 time ≥ 50% of F1 time:
  ? Blocker detected (Lesson 07 will diagnose)
  ? Scope creep happened
  ? Reuse didn't materialize as expected
```

Save this to `TIME_TRACKER.md`:

```markdown
# Intelligence Acceleration Results

## Actual Times
- Feature 1: ___ minutes (Lesson 02)
- Feature 2: ___ minutes
- Feature 3: ___ minutes
- Feature 4: ___ minutes

## Analysis
- F4 as % of F1: ___% (Target: <50%)
- Result: PASS / NEEDS REVIEW

## Observations
What accelerated F4?
What slowed it down?
```

## Troubleshooting Dashboard Issues

**Problem: "No such file or directory: campaign_data/profiles/..."**
- Solution: Did you run the test data creation commands? Run the 9 commands above.

**Problem: Table shows empty (no leads)**
- Check: `cat campaign_data/profiles/stripe.json` — is F1 JSON valid?
- Check: Did you run Feature 1, 2, and 3 successfully before F4?

**Problem: Scores show 0 or wrong values**
- Check: Is your ICP scorer logic correct? (Lesson 03)
- Check: Did you pass the right JSON files to F2?

**Problem: "Average: NaN" in summary**
- Check: Are all scores valid numbers? Did F2 output JSON correctly?

**Problem: Table doesn't sort by score**
- Check: Is your sort logic `sorted(..., key=lambda x: x['icp_score'], reverse=True)`?

**If you're stuck:**
Ask your AI tool: "My dashboard runs but shows [specific problem]. Here's my code: [paste campaign_dashboard.py]. What's wrong?"

## Try With AI

**Setup**: You've completed Feature 4 and calculated your F4 time. Now analyze what you built.

**Prompt 1 (Acceleration Analysis)**:
```
Here's my feature acceleration data:
- F1: [X] minutes
- F2: [Y] minutes
- F3: [Z] minutes
- F4: [W] minutes

Analyze: What drove the acceleration from F1 to F4?
What patterns from F1-F3 helped me build F4 faster?
```

**Prompt 2 (Extrapolation)**:
```
Based on this acceleration pattern, if I built Feature 5 (Lead Nurture Sequencer),
how long should it take?
What patterns from F1-F4 would directly transfer to F5?
What's new about F5 that wouldn't transfer?
```

**Expected outcomes**:
- Prompt 1 reveals your reuse patterns (JSON parsing, sorting logic, summary calculations)
- Prompt 2 lets you predict F5 complexity based on F1-F4 acceleration
