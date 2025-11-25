---
sidebar_position: 7
title: "Ship + Retrospective"
---

# Ship + Retrospective

Your four features are built. Your acceleration data is recorded. Now verify everything works together, document your learning, and ship.

Run the complete pipeline one more time with fresh data to ensure all features integrate correctly.

## Final Pipeline Test

Process a new company through all four features in sequence:

```bash
# Create a test company (if you don't have another URL ready)
URL="https://stripe.com"

# Run through full pipeline
python lead_profiler.py $URL > campaign_data/profiles/stripe.json
python icp_scorer.py campaign_data/profiles/stripe.json > campaign_data/scores/stripe.json
python outreach_generator.py campaign_data/profiles/stripe.json campaign_data/scores/stripe.json > campaign_data/outreach/stripe.json

# View unified dashboard
python campaign_dashboard.py ./campaign_data/
```

**Verification Checklist:**
- ✓ Lead Profiler extracts structured data from website
- ✓ ICP Scorer reads profiler output and produces scores
- ✓ Outreach Generator reads scores and produces personalized messages
- ✓ Campaign Dashboard displays all four features in unified view
- ✓ No errors or warnings in console output
- ✓ JSON files created in correct directories

Fix any issues before proceeding.

## Complete Time Tracker with All Data

Open `TIME_TRACKER.md` and fill in all timing data:

```markdown
# Feature Build Times - Final Data

| Feature | Duration (min) | % of F1 | Acceleration |
|---------|----------------|---------|--------------|
| F1: Lead Profiler | _____ | 100% | baseline |
| F2: ICP Scorer | _____ | ___% | ___% faster |
| F3: Outreach Generator | _____ | ___% | ___% faster |
| F4: Campaign Dashboard | _____ | ___% | ___% faster |

### Target Verification

- F1 baseline: _____ minutes
- F4 target (50% of F1): _____ minutes
- F4 actual: _____ minutes
- **Target achieved:** YES / NO

### Total Impact

- All 4 features combined: _____ minutes
- If no reuse (4x F1): _____ minutes
- **Time saved by intelligence accumulation:** _____ minutes
```

Fill in your actual numbers. If you didn't hit the 50% target, record what happened—that's part of learning.

## Calculate Acceleration Metrics

Create a summary table showing your acceleration ratio. If F1 took 120 minutes and F4 took 50 minutes, that's 58% acceleration (or F4 is 42% of F1 time).

Example:
```
F1: 120 min → F2: 95 min (21% faster) → F3: 70 min (42% faster) → F4: 50 min (58% faster)
```

Your data may differ. What matters is that you have concrete numbers.

## Write Your Retrospective

Create `RETROSPECTIVE.md` in your project root:

```bash
touch RETROSPECTIVE.md
```

Answer each question directly. This is not an essay—concise, honest answers:

```markdown
# AI Sales Assistant Retrospective

## What Caused Acceleration?

List specific skills, patterns, or reusable components that made F2, F3, F4 faster than F1:

1.
2.
3.

## What Slowed Me Down?

Obstacles or patterns that didn't transfer well:

1.
2.

## What Would I Do Differently?

If building this again:

1.
2.

## Skills That Transfer to Next Project

Patterns I can reuse in future work:

1.
2.
3.

## Key Insight

One sentence: What did intelligence accumulation teach me?

>
```

Be honest. If F4 wasn't 50% faster, analyze why. Missing the target is still valuable data.

## Ship: Commit and Update

Commit your completed project:

```bash
git add TIME_TRACKER.md RETROSPECTIVE.md
git add campaign_data/ (or specify your data directories)
git commit -m "Ship: AI Sales Assistant v1.0 - 4 features, measured acceleration"
```

Update your project `README.md` with final results:

```markdown
# AI Sales Assistant

A complete sales intelligence pipeline demonstrating SDD-RI workflow and intelligence accumulation.

## Features Built

1. **Lead Profiler** — Extracts structured company profiles from websites
2. **ICP Scorer** — Scores leads against Ideal Customer Profile criteria
3. **Outreach Generator** — Creates personalized outreach based on ICP match
4. **Campaign Dashboard** — Unified view of all leads, scores, and outreach

## Acceleration Results

| Feature | Build Time | Acceleration |
|---------|------------|--------------|
| F1: Lead Profiler | X min | baseline |
| F2: ICP Scorer | Y min | Z% faster |
| F3: Outreach Generator | A min | B% faster |
| F4: Campaign Dashboard | C min | D% faster |

**Key Learning**: [Your one-sentence insight from RETROSPECTIVE.md]

## Reusable Skills Created

Patterns formalized using P+Q+P framework:

- Skill 1: [name and brief description]
- Skill 2: [name and brief description]
- Skill 3: [name and brief description]

## Usage

[How to run the sales assistant pipeline]
```

Commit the updated README:

```bash
git add README.md
git commit -m "Docs: Final results and key learnings"
git log --oneline -5  # Verify commits are recorded
```

## Try With AI

**Prompt 1**: "Here's my TIME_TRACKER.md data: [paste your table]. Did I achieve meaningful acceleration from F1 to F4? What patterns should I analyze?"

**Prompt 2**: "I created these skills during the capstone: [list skills]. What future projects should I use these for? Give 3 specific ideas."
