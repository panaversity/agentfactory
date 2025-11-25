---
sidebar_position: 5
title: "Feature 4: Campaign Dashboard"
---

# Feature 4: Campaign Dashboard

You've built three features. Now you aggregate them into a unified dashboard—your proof of intelligence acceleration. This is the test: **your target is to complete Feature 4 in less than 50% of your Feature 1 time**.

**Start your timer now.** Write down your Feature 1 duration from Lesson 02. Calculate your target: F1 time ÷ 2. That's your ceiling. Can you build F4 underneath it?

## Hands-On: Define Dashboard Scope

Stop reading. Open a file. Define what the dashboard displays:

- All processed leads in a table
- ICP scores with visual indicators (emoji or symbols)
- Outreach message previews
- Campaign summary statistics

**Your choice**: Output format.
- Option A: CLI table (simpler, faster—recommended for hitting 50% target)
- Option B: HTML file (more visual, takes longer)

**For this lesson, use Option A.** You can enhance later.

Write this down. 2 minutes max.

## Hands-On: Write Your Specification

You know the pattern now. This should be **much faster** than Feature 1 because you're reusing familiar structure.

Open a new file: `specs/feature-4-dashboard/spec.md`

**Your specification template:**

```yaml
intent: |
  Aggregate all processed leads from Feature 1-3 outputs into a single
  formatted table view. Display lead profiles, ICP scores, and outreach
  message previews. Provide campaign-level summary statistics.

success_criteria:
  - All leads display in table format
  - Leads sorted by ICP score (descending)
  - Category shown with visual indicator (emoji)
  - Summary statistics: total leads, hot/warm/cold counts, average score
  - Handles empty campaigns gracefully

inputs:
  - type: directory
    path: ./campaign_data/
    description: Contains profiles/, scores/, outreach/ subdirectories
    format: JSON files from Features 1-3

outputs:
  - type: formatted_table
    columns:
      - company_name
      - industry
      - icp_score
      - category (hot/warm/cold)
      - outreach_preview (first 50 characters)
    summary_section:
      - total_leads
      - hot_leads
      - warm_leads
      - cold_leads
      - average_score

constraints:
  - Single Python script (campaign_dashboard.py)
  - Reuse JSON parsing from Feature 1-3
  - No external dependencies beyond what Feature 1-3 already use
  - Output to console only (no file writing)

non_goals:
  - Real-time data updates
  - Database persistence
  - HTML/web interface (defer to future iteration)
```

**Stop. Write YOUR spec. Don't copy this exactly.** Make it yours. 5 minutes max.

## Hands-On: Plan and Implement

Run the workflow:

```bash
/sp.plan
/sp.tasks
/sp.implement
```

**Notice the difference from Feature 1:**
- Your plan should be shorter (you've done this pattern 3 times)
- Your tasks should reference existing Features (reuse, don't rebuild)
- Your implementation should be faster (familiar patterns)

**Track time as you work.** You're measuring intelligence reuse.

**If you hit 50% of F1 time before fully implementing:**
This is the test. Stop and assess: what's missing? What could wait for Lesson 06?

## Hands-On: Create Test Data

Populate your dashboard with real data. Run the same companies through your Feature 1-3 pipeline:

```bash
mkdir -p campaign_data/{profiles,scores,outreach}

# Process leads through your existing features
python lead_profiler.py https://stripe.com > campaign_data/profiles/stripe.json
python icp_scorer.py campaign_data/profiles/stripe.json > campaign_data/scores/stripe.json
python outreach_generator.py campaign_data/profiles/stripe.json campaign_data/scores/stripe.json > campaign_data/outreach/stripe.json

python lead_profiler.py https://shopify.com > campaign_data/profiles/shopify.json
python icp_scorer.py campaign_data/profiles/shopify.json > campaign_data/scores/shopify.json
python outreach_generator.py campaign_data/profiles/shopify.json campaign_data/scores/shopify.json > campaign_data/outreach/shopify.json

python lead_profiler.py https://notion.so > campaign_data/profiles/notion.json
python icp_scorer.py campaign_data/profiles/notion.json > campaign_data/scores/notion.json
python outreach_generator.py campaign_data/profiles/notion.json campaign_data/scores/notion.json > campaign_data/outreach/notion.json
```

## Hands-On: Run Your Dashboard

Execute your implementation:

```bash
python campaign_dashboard.py ./campaign_data/
```

**Expected output (approximate):**

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

Verify: Does every lead appear? Are scores sorted descending? Do categories show?

**Stop your timer.**

## Hands-On: Calculate Your Acceleration

Pull your Feature 1 time from Lesson 02. Fill in your actual times:

| Feature | Your Time | % of F1 | Target |
|---------|-----------|---------|--------|
| F1: Lead Profiler | ___ min | 100% | 100% |
| F2: ICP Scorer | ___ min | ___% | <100% |
| F3: Outreach Generator | ___ min | ___% | <75% |
| F4: Campaign Dashboard | ___ min | ___% | **<50%** |

**The question:** Did F4 take less than 50% of F1?

- **Yes**: You've proven intelligence accumulation. Patterns and skills you created in F1 accelerated F2, F3, F4. Reusable intelligence works.
- **No**: Lesson 07 (Retrospective) will analyze why. Was there a blocker? An underestimated complexity? A skill gap? Track it.

Write this in a file: `TIME_TRACKER.md`

## Try With AI

**Prompt 1**: "Here's my feature acceleration data: F1=[X]min, F2=[Y]min, F3=[Z]min, F4=[W]min. Analyze this trajectory. What drove the acceleration? What would I need to do to push F4 even lower?"

**Prompt 2**: "Based on my acceleration pattern, if I built a Feature 5 (Lead Nurture Sequencer), how long should it take? What patterns from F1-F4 would transfer to F5?"
