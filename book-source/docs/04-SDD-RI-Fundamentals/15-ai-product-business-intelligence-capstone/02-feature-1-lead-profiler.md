---
sidebar_position: 2
title: "Feature 1: Lead Profiler"
---

# Feature 1: Lead Profiler

**This is your baseline.** You're building a Lead Profiler that takes a company URL and outputs a structured JSON profile. You'll measure how long this takes—then build Features 2-4 and compare acceleration.

**Start your timer now.** Record the exact start time.

## Specification

Run:
```bash
/sp.specify
```

Provide this description:
```
Lead Profiler: Given a company URL, analyze website and extract structured
profile (company name, industry, size estimate, technology indicators, pain points).
```

Your spec.md should define:

**Input**: URL string
**Output**: JSON with these fields:
- `company_name`: string
- `industry`: string
- `estimated_size`: "startup" | "smb" | "enterprise"
- `tech_indicators`: array of strings
- `pain_points`: array of strings
- `confidence_score`: 0-100 number

**Success Criteria**:
- Company name extracted correctly
- Industry classification present
- At least 2 tech indicators identified
- At least 1 pain point inferred

**Constraints**:
- Handle sites that block scrapers gracefully
- Complete in under 30 seconds
- Accept manual text input if website unavailable

Verify:
```bash
cat .specify/specs/lead-profiler/spec.md
```

## Plan

Generate implementation plan:
```bash
/sp.plan
```

Review the plan:
```bash
cat .specify/specs/lead-profiler/plan.md
```

## Tasks

Generate task breakdown:
```bash
/sp.tasks
```

View your tasks:
```bash
cat .specify/specs/lead-profiler/tasks.md
```

## Implement

Execute implementation:
```bash
/sp.implement
```

Work through each task. Your code will likely include:
1. Website content fetching (or text input option)
2. AI-powered profile extraction
3. JSON output formatting
4. Error handling

## Test

Run your implementation with a real company URL:

```bash
# Example: Stripe (tech/payments company)
python lead_profiler.py https://stripe.com
# OR your chosen test URL
```

Verify output against specification:
- [ ] Company name correct
- [ ] Industry identified
- [ ] Size estimated
- [ ] Tech indicators listed
- [ ] Pain points present
- [ ] Confidence score included
- [ ] Output is valid JSON

**Stop your timer now.** Record:
- Start time: _____
- End time: _____
- Total duration: _____ minutes

## Try With AI

**Prompt 1**: "I've built the Lead Profiler specification and implementation plan for analyzing company URLs. What are the 3-5 hardest technical decisions I'll make during implementation, and what trade-offs should I consider?"

**Prompt 2**: "Generate 5 test company URLs that would comprehensively validate my Lead Profiler: include different industries, company sizes, and website architectures. Explain why each is a useful test case."
