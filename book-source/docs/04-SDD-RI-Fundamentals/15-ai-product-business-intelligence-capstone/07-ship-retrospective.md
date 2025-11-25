---
sidebar_position: 7
title: "Ship + Retrospective"
---

# Ship + Retrospective

Your four features are built. Your acceleration data is recorded. Now verify everything works together, document what you learned, and ship.

## Final Pipeline Test

Process a new company through all four features in sequence. Use fresh data to ensure all features integrate correctly and no intermediate errors propagate.

```bash
# Create a test company (different from earlier development data)
URL="https://github.com"

# Run through complete pipeline in order
echo "Testing Lead Profiler..."
python lead_profiler.py $URL > campaign_data/profiles/github.json

echo "Testing ICP Scorer..."
python icp_scorer.py campaign_data/profiles/github.json > campaign_data/scores/github.json

echo "Testing Outreach Generator..."
python outreach_generator.py campaign_data/profiles/github.json campaign_data/scores/github.json > campaign_data/outreach/github.json

echo "Testing Campaign Dashboard..."
python campaign_dashboard.py ./campaign_data/

echo "Pipeline complete!"
```

### Verify Complete Output

Before proceeding, check:

1. **Lead Profiler output** — View the JSON structure:
   ```bash
   cat campaign_data/profiles/github.json
   ```
   Should contain: company_name, industry, size, tech_stack, hiring_indicators (if detectable)

2. **ICP Scorer output** — Verify scores were calculated:
   ```bash
   cat campaign_data/scores/github.json
   ```
   Should contain: icp_score (0-100), category (high/medium/low), breakdown, reasoning

3. **Outreach Generator output** — Check personalized message:
   ```bash
   cat campaign_data/outreach/github.json
   ```
   Should contain: personalized_message, subject_line, follow_up_angle (based on company profile and ICP score)

4. **Dashboard output** — View unified summary:
   ```bash
   # Terminal will display table with all leads, their scores, and outreach status
   # No errors or warnings should appear
   ```

**Troubleshooting**: If any step fails:
- Check JSON syntax: `python -m json.tool campaign_data/profiles/github.json`
- Review error message carefully—it shows which field caused the issue
- Fix the field in the previous feature's output, then re-run that step
- Don't proceed to shipping until all four features run cleanly

## Complete Your TIME_TRACKER Data

Open or create `TIME_TRACKER.md` in your project root. Fill in your actual build times:

```bash
# Create file if it doesn't exist
touch TIME_TRACKER.md
```

```markdown
# Feature Build Times - Final Data

Record the actual minutes you spent building each feature (from start of spec to first working output):

| Feature | Minutes | % of F1 | Acceleration |
|---------|---------|---------|--------------|
| F1: Lead Profiler | _____ | 100% | baseline |
| F2: ICP Scorer | _____ | ___% | ___% faster |
| F3: Outreach Generator | _____ | ___% | ___% faster |
| F4: Campaign Dashboard | _____ | ___% | ___% faster |

### Calculate Acceleration Step-by-Step

Use F1 time as your baseline. Let's say F1 took 120 minutes.

**F2 Acceleration:**
- F1: 120 minutes
- F2: 95 minutes
- Calculation: ((120 - 95) / 120) × 100 = 21% faster
- As % of F1: (95 / 120) × 100 = 79% (F2 is 79% of F1's time)

**F3 Acceleration:**
- F1: 120 minutes (baseline)
- F3: 70 minutes
- Calculation: ((120 - 70) / 120) × 100 = 42% faster
- As % of F1: (70 / 120) × 100 = 58% (F3 is 58% of F1's time)

**F4 Acceleration (the target):**
- F1: 120 minutes (baseline)
- F4: 50 minutes
- Calculation: ((120 - 50) / 120) × 100 = 58% faster
- As % of F1: (50 / 120) × 100 = 42% (F4 is 42% of F1's time)

### Target Verification

Did you achieve the "50% of F1" goal for Feature 4?

- F1 baseline (actual minutes): _____
- F4 target (50% of F1): _____
- F4 actual (actual minutes): _____
- **Target achieved:** YES / NO

### Total Impact: Intelligence Accumulation

What if you'd rebuilt Feature 1 four times instead of leveraging intelligence?

- All 4 features (with reuse): _____ minutes total
- F1 × 4 (no reuse): (F1 time) × 4 = _____ minutes
- **Time saved by intelligence accumulation:** _____ minutes

This is the real ROI of SDD-RI—you proved it with your data.
```

**Fill in your actual numbers now.** If you didn't hit the 50% target for F4, that's still valuable data. Record what happened—understanding why teaches more than hitting a target you expected.

## Write Your Retrospective

Create `RETROSPECTIVE.md` in your project root. This is not an essay. Answer each question directly and concisely.

```bash
touch RETROSPECTIVE.md
```

Use this template. Answer in 1-3 sentences per question:

```markdown
# AI Sales Assistant Retrospective

## 1. What Specific Patterns Made Features 2-4 Faster?

List the concrete reusable patterns, not just "AI helped":

- Pattern: _____ (example: "JSON transformation for company profile data")
  - Where it appeared: Feature 1 (_____), Feature 2 (_____), Feature 3 (_____), Feature 4 (_____)
  - Why it was reusable: _____

- Pattern: _____
  - Where it appeared: Features _____, _____, _____
  - Why it was reusable: _____

## 2. What Slowed You Down or Didn't Transfer Well?

What patterns seemed like they'd be reusable but actually required rethinking?

- Bottleneck: _____
  - Why it didn't transfer: _____
  - What you did instead: _____

- Bottleneck: _____
  - Why it didn't transfer: _____

## 3. Your F1 Actual Build Time vs Your Expectations

- Did F1 take longer/shorter than you expected? Why?
- What surprised you most about Feature 1's complexity?

## 4. Features 2-4 Compared to Your Predictions

Did Features 2-4 actually accelerate as much as you hoped?

- F4 target was 50% of F1. You achieved: ____%
- If you missed the target: What would you change in F1's design to make F2-F4 even faster?
- If you exceeded the target: What surprised you about the acceleration?

## 5. Reusable Skills You Created

List the skills you formalized in Lesson 6:

- Skill 1: [name] — When would you use this again?
- Skill 2: [name] — What problem does it solve?
- Skill 3: [name] — How is it different from ad-hoc coding?

## 6. One Sentence: Intelligence Accumulation Lesson

What's the single biggest insight from this project?

Example: "Investing 2 hours in a clear specification for Feature 1 saved 4+ hours across Features 2-4 because each feature reused the pattern."

Your insight: _____

## 7. What Would You Do Differently Next Time?

If building a new project:

1. Would you start the same way (full spec for Feature 1)? Why or why not?
2. At what point would you have created skills instead of waiting until Lesson 6?
3. What would you time differently?
```

## Ship It: Commit and Push

Your project is complete. Document it with clear commit messages:

```bash
# Commit your timing and retrospective
git add TIME_TRACKER.md RETROSPECTIVE.md
git commit -m "Data: Final timing metrics - Feature 4 achieved X% acceleration"

# If you have campaign_data with results
git add campaign_data/
git commit -m "Data: Complete campaign pipeline results - 4 features tested and verified"

# Update your project README with final results
git add README.md
git commit -m "Docs: Final project summary and intelligence accumulation results"

# Verify all commits are recorded
git log --oneline -10
```

## Update Your README with Final Results

Open `README.md` and replace placeholders with your actual data:

```markdown
# AI Sales Assistant

A complete sales intelligence pipeline demonstrating SDD-RI workflow and intelligence accumulation in action.

## The Challenge

Build four interconnected features (Lead Profiler → ICP Scorer → Outreach Generator → Campaign Dashboard) and prove that Feature 4 takes less than 50% of Feature 1's build time.

## Features Built

1. **Lead Profiler** — Analyze company websites, extract structured profiles (company name, industry, size, tech stack, hiring indicators)
2. **ICP Scorer** — Score each lead against Ideal Customer Profile criteria (0-100 scale with reasoning)
3. **Outreach Generator** — Create personalized outreach messages based on company profile and ICP score
4. **Campaign Dashboard** — Unified dashboard showing all leads, scores, and personalized outreach

## Intelligence Acceleration Results

| Feature | Build Time | % of F1 | Acceleration |
|---------|-----------|---------|--------------|
| F1: Lead Profiler | X min | 100% | baseline |
| F2: ICP Scorer | Y min | __% | __% faster |
| F3: Outreach Generator | A min | __% | __% faster |
| F4: Campaign Dashboard | B min | __% | __% faster |

**Target achieved:** Feature 4 completed in __% of Feature 1 time (target: 50%)

## Key Learning

[Your one-sentence insight from RETROSPECTIVE.md]

## Reusable Skills Created

Patterns formalized using P+Q+P framework for future projects:

- **[Skill 1]**: [Brief description of when you'd use this]
- **[Skill 2]**: [Brief description of when you'd use this]
- **[Skill 3]**: [Brief description of when you'd use this]

## Running the Pipeline

Start with a fresh company URL:

```bash
URL="https://example.com"
python lead_profiler.py $URL > campaign_data/profiles/example.json
python icp_scorer.py campaign_data/profiles/example.json > campaign_data/scores/example.json
python outreach_generator.py campaign_data/profiles/example.json campaign_data/scores/example.json > campaign_data/outreach/example.json
python campaign_dashboard.py ./campaign_data/
```

## What This Proves

This project demonstrates that Specification-Driven Development with Reusable Intelligence (SDD-RI) isn't theoretical. When you invest in clarity (specs) and reusable patterns (skills), subsequent features accelerate measurably. Your timing data is proof.
```

Commit the updated README:

```bash
git add README.md
git commit -m "Docs: Final project results - intelligence accumulation measured and documented"

# Optional: Create a release tag marking project completion
git tag -a "v1.0-shipped" -m "AI Sales Assistant capstone complete - 4 features, measured acceleration"
git log --oneline -15  # View your completed work
```

## Try With AI

Open your AI tool (Claude, Gemini, or your preferred assistant) and use these prompts to deepen your retrospective analysis:

**Prompt 1: Analyze Your Acceleration Data**

```
Here's my TIME_TRACKER.md data from the AI Sales Assistant capstone:

[Paste your complete TIME_TRACKER table]

I achieved X% acceleration from Feature 1 to Feature 4.

Analyze:
1. Is this meaningful acceleration? (compare to industry benchmarks for similar projects)
2. What specific patterns in my work caused this acceleration?
3. If I missed the 50% target, what architecture choices would unlock faster Feature 4 build?
4. If I exceeded the target, what surprised you most about the acceleration?
```

**Prompt 2: Skills for Future Projects**

```
During the AI Sales Assistant capstone, I created these reusable skills:

[List your skills from Lesson 6]

For each skill, suggest:
1. Three new projects where this skill would accelerate development
2. How I would adapt this skill for a different domain (e.g., if "JSON Data Transformer" was a skill, how would it change in a data science vs. sales context?)
3. What additional principles should I add to make this skill more powerful?
```

**Expected Outcomes:**
- Prompt 1 deepens your understanding of what actually drove acceleration (was it specification clarity? reusable patterns? better AI prompts? all three?)
- Prompt 2 connects your learning to future projects and compounds the ROI of skills you created

Your capstone is complete. You've proven that intelligence accumulates. Ship it.
