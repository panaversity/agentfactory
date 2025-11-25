---
sidebar_position: 3
title: "Feature 2: ICP Scorer"
---

# Feature 2: ICP Scorer

Your Feature 1 (Lead Profiler) outputs structured lead profiles. Feature 2 scores those profiles against your Ideal Customer Profile (ICP) criteria and outputs a priority category (hot/warm/cold).

This is where you'll first experience acceleration: You're reusing the spec→plan→tasks→implement workflow from F1. This feature should build faster.

**Start your timer now.** Check your F1 duration: _____ minutes. Your goal: complete F2 in less time.

## Define ICP Criteria

Before specification, define what makes a customer ideal for your AI Sales Assistant. Create this file:

```bash
touch icp_criteria.json
```

Add your scoring criteria with weights:

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

## Data Flow: F1 → F2

Feature 2 receives Feature 1's output directly. This is the JSON your Lead Profiler generates:

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

Feature 2 ingests this profile and outputs a scored assessment:

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

## Write the Specification

Run the specification workflow:

```bash
/sp.specify
```

Provide this feature description:

```
ICP Scorer: Takes a Lead Profile JSON (from Feature 1) and scores it against
Ideal Customer Profile criteria. Outputs a numeric score (0-100), priority
category (hot/warm/cold), per-criterion breakdown, and reasoning.
```

Your specification should define:

**Input**: Lead Profile JSON (with structure: company_name, industry, estimated_size, tech_indicators, pain_points, confidence_score)

**Output**:
```yaml
{
  "score": number (0-100),
  "category": string (hot|warm|cold),
  "breakdown": {
    "company_size_score": number,
    "industry_fit_score": number,
    "tech_sophistication_score": number,
    "pain_point_alignment_score": number
  },
  "reasoning": string
}
```

**Success Criteria**:
- Score calculated consistently for same input
- Category correctly assigned based on score ranges
- Breakdown shows each criterion's contribution
- Reasoning includes references to specific profile data

**Constraints**:
- Load ICP criteria from icp_criteria.json
- Handle missing fields gracefully
- Reasoning should be 1-2 sentences

## Plan, Tasks, Implement

Generate your implementation plan:

```bash
/sp.plan
```

Review the plan, then generate tasks:

```bash
/sp.tasks
```

Execute the implementation:

```bash
/sp.implement
```

Work with AI systematically. You've already done this workflow in F1—reuse those patterns. You'll need:
1. JSON input parsing
2. Criteria matching and weighting logic
3. Score aggregation
4. Category assignment
5. Reasoning generation

## Test the Pipeline

Test Feature 2 independently first:

```bash
# Score a single lead profile
python icp_scorer.py lead_profile.json
```

Then test the full pipeline (F1 → F2):

```bash
# Generate lead profile from F1
python lead_profiler.py https://stripe.com > lead_profile.json

# Score that profile with F2
python icp_scorer.py lead_profile.json
```

Or pipe them directly:

```bash
python lead_profiler.py https://stripe.com | python icp_scorer.py
```

Verify the JSON output matches your specification schema and all success criteria are met.

**Stop your timer.** Record in TIME_TRACKER.md:

```
Feature 2 Duration: _____ minutes
Feature 1 Duration: _____ minutes (reference)
Acceleration: F2 took ___% of F1 time
Pattern Reuse Notes: [What patterns transferred from F1?]
```

## Try With AI

**Prompt 1**: "I built Feature 1 in [X] minutes and Feature 2 in [Y] minutes. What patterns did I reuse from F1? Why was F2 faster?"

**Prompt 2**: "I'm scoring leads with these criteria [show icp_criteria.json]. Given this sample lead profile [paste F1 output], walk me through your scoring logic step-by-step and explain if you'd adjust the criteria weights."
