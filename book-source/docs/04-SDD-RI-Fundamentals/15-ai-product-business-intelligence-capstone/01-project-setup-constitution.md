---
sidebar_position: 1
title: "Project Setup + Constitution"
---

# Project Setup + Constitution

You're building an AI Sales Assistant with four features: Lead Profiler, ICP Scorer, Outreach Generator, and Campaign Dashboard. Before writing any code, you'll establish a project structure and a constitution that defines quality standards. This constitution becomes your source of truth for design decisions across all four features, preventing quality drift and keeping implementation consistent.

The work in this lesson is foundational. You're not building features yet—you're building the decision framework that will guide feature development.

## Create Your Project Directory

Start in your terminal. Create a dedicated directory for this capstone and navigate into it:

```bash
mkdir ai-sales-assistant
cd ai-sales-assistant
```

**Verify you're in the right location:**

```bash
pwd
```

You should see a path ending in `ai-sales-assistant`. Now initialize Spec-Kit Plus (the framework you learned in Chapter 14):

```bash
specifyplus init --here
```

This command creates the `.specify/` directory structure and configuration files. Verify the setup:

```bash
ls -la
```

You should see:
- `.specify/` directory (framework folder)
- `.specify/memory/` (stores decisions)
- `.specify/templates/` (reusable templates)
- Configuration files (`.specify.yaml` or similar)

If you don't see `.specify/`, check that you installed Spec-Kit Plus in Chapter 14 and try the `specifyplus init --here` command again.

## Write Your Constitution

The constitution is a single document that captures your project's quality standards, scope, and non-goals. It answers questions that will come up repeatedly during feature development: "Should we include this?" "How detailed should this be?" "What's outside scope?"

Create the constitution file:

```bash
touch .specify/memory/constitution.md
```

Open this file in your editor and write the following constitution for your AI Sales Assistant project:

```markdown
# AI Sales Assistant Constitution

## Project Vision

Build an intelligent system that helps sales teams identify, qualify, and engage high-potential leads through AI-powered analysis and personalized outreach recommendations.

The system processes company information, evaluates fit against ideal customer profile criteria, and generates targeted outreach messages—all structured for integration into sales workflows.

## Core Principles

1. **Structured over Freeform**: All feature outputs are JSON-structured. No feature generates plain text that requires manual parsing.

2. **Lead-Centric Design**: Every feature serves the lead evaluation workflow. Lead profiling → ICP scoring → Personalized outreach → Campaign tracking.

3. **Decisiveness**: Features provide clear verdicts (scores, recommendations, summaries). Ambiguous outputs are errors.

4. **Traceability**: Every output includes reasoning or source data so sales users can validate AI suggestions.

## Feature Specifications

### Lead Profiler (Feature 1)

**Input**: Company name, website URL or description

**Output**: Structured profile with:
- Company name
- Industry classification
- Estimated employee count
- Technology stack indicators (programming languages, frameworks detected)
- Identified pain points (inferred from company description, website focus)
- Confidence score (0-100 for data quality)

**Quality Gate**: Every profile must include minimum 3 pain points identified from available data.

### ICP Scorer (Feature 2)

**Input**: Ideal Customer Profile criteria (industry, company size, technology requirements), Lead profile from Feature 1

**Output**: Fitness score (0-100) with:
- Match breakdown (industry match, size match, tech alignment)
- Reasoning for score (e.g., "Industry match: 90 (exact match), Size match: 75 (candidate at higher end of range)")
- Recommendation (Strong Fit / Good Fit / Monitor / Not a Fit)

**Quality Gate**: Every score must be justified with at least 2 matching criteria explained.

### Outreach Generator (Feature 3)

**Input**: Lead profile (Feature 1), ICP score and reasoning (Feature 2)

**Output**: Personalized outreach message with:
- Subject line (specific to company, 50 char max)
- Email body (reference 2+ specific pain points from lead profile, 150-200 words)
- Call-to-action (clear next step)

**Quality Gate**: Outreach must cite specific data from lead profile (not generic). If lead profile is missing data, outreach must flag that limitation.

### Campaign Dashboard (Feature 4)

**Input**: Lead profiles, ICP scores, outreach messages

**Output**: Summary dashboard showing:
- Total leads processed
- Score distribution (how many fit each category)
- Top pain points across all leads
- Sample outreach messages
- Performance metrics (if available: open rate, response rate)

**Quality Gate**: Dashboard aggregates all lead data without redundancy. Metrics are accurate to underlying feature outputs.

## Non-Goals

- Production deployment (prototype quality acceptable)
- User authentication or login systems
- Persistent database (file-based storage acceptable)
- Visual UI polish (functional over beautiful)
- Real-time lead data ingestion (batch processing acceptable)
- Multi-language support

## Quality Standards

### Code Quality

- Python 3.8+ compatibility
- Type hints on all functions
- Docstrings on all feature functions
- Error handling for missing/invalid inputs
- Test coverage for core logic (70%+ target)

### Output Quality

- Structured JSON only (no plain text outputs)
- All strings UTF-8 encoded
- Numeric scores (0-100) with no fractional decimals
- Timestamps in ISO 8601 format if time data included

### Documentation Standards

- Every feature has a specification (intent, input/output contract, quality gates)
- Every feature has example inputs and expected outputs
- README describes how to run all four features in sequence

## Constraints

- Single developer building sequentially (Features 1 → 2 → 3 → 4)
- Features depend on prior feature outputs (must build in order)
- Each feature must be completable in under 2 hours (acceleration goal)
- All external API calls (if any) must be wrapped with fallback/mock data
```

**Save this file.** Read it through once. You now have a written standard that every feature must meet.

## Create Your Time Tracker

As you build each feature, you'll measure how long it takes. This helps you evaluate SDD-RI acceleration: does Feature 4 (which reuses decisions from Feature 1) build faster because you're not re-deciding quality standards?

Create the time tracker file:

```bash
touch TIME_TRACKER.md
```

Open this file and write:

```markdown
# Feature Build Times - AI Sales Assistant

| Feature | Start Time | End Time | Duration (min) | Notes |
|---------|------------|----------|----------------|-------|
| F1: Lead Profiler | | | | |
| F2: ICP Scorer | | | | |
| F3: Outreach Generator | | | | |
| F4: Campaign Dashboard | | | | |

## Acceleration Analysis

After completing all four features, fill in:

- F1 baseline: _____ minutes (total time to complete Feature 1)
- F2 time: _____ minutes
- F3 time: _____ minutes
- F4 target: _____ minutes (goal: 50% of F1 time, achieved through constitution reuse)
- F4 actual: _____ minutes
- Achieved 50% acceleration? Yes / No

## Notes

Document anything that sped up or slowed down development:
- Did constitution clarity reduce decision time?
- Did reusing quality standards from Feature 1 speed up Feature 4 decisions?
- What decisions took longest to make?

---
```

Save this file. You'll fill it in as you complete each feature over the next lessons.

## Verify Your Project Structure

You now have the foundation in place. Your project directory should contain:

```
ai-sales-assistant/
├── .specify/
│   ├── memory/
│   │   └── constitution.md          (just created)
│   ├── templates/
│   └── (other Spec-Kit Plus files)
├── TIME_TRACKER.md                  (just created)
└── (Spec-Kit Plus config files)
```

Verify this structure:

```bash
ls -la
cat .specify/memory/constitution.md
```

The first command shows your project contents. The second displays your constitution to confirm it was saved correctly. If either shows errors, troubleshoot before proceeding to Feature 1.

## Try With AI

Before starting Feature 1 (Lead Profiler), get AI feedback on your decision framework and setup:

**Prompt 1**: "I've written a constitution for an AI Sales Assistant. Review it for completeness: Are there quality constraints or specifications I've missed that would cause problems when building the four features (Lead Profiler, ICP Scorer, Outreach Generator, Campaign Dashboard)?"

**Prompt 2**: "Help me design the specification for Feature 1 (Lead Profiler). What should I specify before starting implementation? Cover: What inputs the feature accepts, what structure the output should have, how to validate that a lead profile is 'complete' enough."
