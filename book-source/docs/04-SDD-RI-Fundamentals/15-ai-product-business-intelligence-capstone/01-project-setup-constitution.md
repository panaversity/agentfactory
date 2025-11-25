---
sidebar_position: 1
title: "Project Setup + Constitution"
---

# Project Setup + Constitution

You're building an AI Sales Assistant with four features: Lead Profiler, ICP Scorer, Outreach Generator, and Campaign Dashboard. Before writing any code, you'll establish a project structure and a constitution that defines quality standards. This constitution becomes your source of truth—the decision framework you reference when building Feature 1, Feature 2, and Feature 3. If Feature 4 builds significantly faster, you'll know it's because you stopped re-deciding quality standards and reused the constitution from Feature 1.

## Create Your Project Directory

Start in your terminal:

```bash
mkdir ai-sales-assistant
cd ai-sales-assistant
pwd
```

**Expected output**: Path ending in `ai-sales-assistant`

Initialize Spec-Kit Plus (the framework you learned in Chapter 14):

```bash
specifyplus init --here
```

Verify the structure:

```bash
ls -la
```

**Expected output**: You see `.specify/` directory with subdirectories:
- `.specify/memory/` (stores constitution)
- `.specify/templates/` (reusable patterns)
- `.specify/scripts/` (utilities)

**Troubleshooting**: If `.specify/` is missing, check Chapter 14 installation. Run `specifyplus --version` to confirm it's installed, then retry `specifyplus init --here`.

## Write Your Constitution

The constitution answers questions that come up during Feature 1, 2, 3, and 4 builds: "Should we include this?" "How detailed?" "In scope or out?"

Create the file:

```bash
touch .specify/memory/constitution.md
```

Open in your editor and write:

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

**Save this file and verify:**

```bash
cat .specify/memory/constitution.md
```

You should see the full constitution. This is your quality gate document—every feature (F1, F2, F3, F4) must satisfy these standards.

**Quick practice**: Read through the constitution once. Pick ONE feature (Lead Profiler). What quality gates does it have? (Hint: See "Quality Gate:" line for each feature.)

## Create Your Time Tracker

The time tracker measures how intelligence compounds. When you build Feature 1, you'll spend time deciding quality standards. When you build Feature 4, you'll reuse those decisions—so Feature 4 should take less than 50% of Feature 1's time.

Create the file:

```bash
touch TIME_TRACKER.md
```

Add this content:

```markdown
# Feature Build Times - AI Sales Assistant

| Feature | Start Time | End Time | Duration (min) | Notes |
|---------|------------|----------|----------------|-------|
| F1: Lead Profiler | | | | |
| F2: ICP Scorer | | | | |
| F3: Outreach Generator | | | | |
| F4: Campaign Dashboard | | | | |

## Acceleration Analysis

After all four features, calculate:

- F1 baseline time: _____ minutes (Feature 1 total time)
- F2 actual time: _____ minutes
- F3 actual time: _____ minutes
- F4 actual time: _____ minutes
- F4 vs F1 ratio: _____ % (calculate: F4_time / F1_time × 100)
- **Target: F4 should be ≤50% of F1**

## Decision Time Breakdown (Optional)

For each feature, track WHERE time went:

- Designing specification: _____ min
- Making quality decisions: _____ min
- Writing code: _____ min
- Testing: _____ min

This shows if reusing the constitution actually reduces "quality decision" time in Features 2-4.

---
```

Verify the file:

```bash
cat TIME_TRACKER.md
```

You'll fill in start/end times as you complete each feature lesson.

## Verify Your Project Structure

Check your setup:

```bash
ls -la
```

**Expected output**: Shows `.specify/`, `TIME_TRACKER.md`, and config files.

Verify constitution is in place:

```bash
ls -la .specify/memory/
```

**Expected output**: `constitution.md` listed

Verify content is correct:

```bash
wc -l .specify/memory/constitution.md
wc -l TIME_TRACKER.md
```

You should see:
- `constitution.md`: ~160 lines (full feature specs + quality gates)
- `TIME_TRACKER.md`: ~30 lines (empty cells to fill during feature builds)

If files are missing or empty, troubleshoot before proceeding to Feature 1.

## Try With AI

**Prompt 1: Constitution Review**

Open your AI tool and copy your entire constitution. Ask:

"Review this constitution for missing constraints. Specifically: Are there quality gates that would prevent Feature 1 (Lead Profiler) or Feature 4 (Campaign Dashboard) from working together? What integration points between features did I miss?"

**Observe**: The AI will likely identify gaps in your Feature 1 output format that affect Feature 2 (which consumes Feature 1's output). This is exactly what you need to fix BEFORE building.

**Prompt 2: Feature 1 Specification Start**

Now ask:

"Using this constitution, outline the specification for Feature 1 (Lead Profiler). What should I specify? Cover: (1) What structure the input is (company name, URL, or both?), (2) What JSON structure the output has (show an example), (3) How I validate a profile is 'complete' according to the constitution's quality gate."

**Observe**: The AI will help you move from constitution (principles) to specification (executable requirements).

---

**You're ready for Feature 1.** Close this lesson. Start Lesson 2: Build Feature 1: Lead Profiler. When you start, open TIME_TRACKER.md and record your start time.
