---
sidebar_position: 2
title: "Feature 1: Lead Profiler"
proficiency_level: "B1"
estimated_time: "90 minutes"
cognitive_load: 9
---

# Feature 1: Lead Profiler

**This is your baseline.** You're building a Lead Profiler that takes a company URL and outputs a structured JSON profile. You'll measure how long this takes—then build Features 2-4 and compare acceleration.

The full SDD-RI cycle—from specification to verified implementation—happens here. Reuse this workflow pattern for Features 2-4.

## Timing: Your Baseline Measurement

This lesson is NOT timed internally. You control the timer.

**START YOUR TIMER NOW** (use your phone's clock):
- **Start Time**: ________________________
- **Current Time (write now)**: ____________

You'll stop the timer after verification (below) and record the total duration.

---

## Complete JSON Output Schema

Before you specify anything, understand the JSON structure your Lead Profiler must produce. Every output matches this schema exactly:

```json
{
  "company_name": "string (exact company name from website)",
  "industry": "string (technology, finance, healthcare, retail, manufacturing, etc.)",
  "estimated_size": "startup | smb | enterprise (based on employee count indicators)",
  "tech_indicators": [
    "string (programming language, framework, or tool detected)",
    "string (example: 'React', 'Node.js', 'Kubernetes')",
    "minimum 2 indicators, maximum 8"
  ],
  "pain_points": [
    "string (inferred pain point from company description or website focus)",
    "string (example: 'Scale management', 'Integration complexity')",
    "minimum 3 pain points required"
  ],
  "confidence_score": "number (0-100, integer only, reflects data quality)"
}
```

**What each field means**:
- `company_name`: Must match the legal company name on the website
- `industry`: One of 6 categories (tech, finance, healthcare, retail, manufacturing, other)
- `estimated_size`: Based on team size, funding, or company age indicators
- `tech_indicators`: Tools the company uses or builds (from website, pricing pages, careers pages, or API documentation)
- `pain_points`: Inferred needs based on company description (what problems do they describe solving?)
- `confidence_score`: 100 = high-quality data from multiple sources, 50 = incomplete/uncertain data, 0 = no data

**Example output** (Stripe):
```json
{
  "company_name": "Stripe",
  "industry": "technology",
  "estimated_size": "enterprise",
  "tech_indicators": [
    "API-driven architecture",
    "Webhooks",
    "GraphQL support",
    "JavaScript/Node.js",
    "Cloud infrastructure"
  ],
  "pain_points": [
    "Payment integration complexity",
    "Cross-border payments",
    "Developer experience in payment processing"
  ],
  "confidence_score": 95
}
```

Write this schema to a file so you reference it during implementation:

```bash
cat > LEAD_PROFILER_SCHEMA.json << 'EOF'
{
  "company_name": "string",
  "industry": "string",
  "estimated_size": "startup | smb | enterprise",
  "tech_indicators": ["string"],
  "pain_points": ["string"],
  "confidence_score": "number (0-100)"
}
EOF

# Verify the file was created:
cat LEAD_PROFILER_SCHEMA.json
```

---

## Step 1: Specification

Run the specification command:
```bash
/sp.specify
```

When prompted, provide this description:
```
Lead Profiler: Given a company URL, analyze the website and public information to extract a structured company profile.

INPUT: Company URL (string)
OUTPUT: JSON profile with company name, industry, estimated size, technology indicators, pain points, and confidence score

QUALITY GATES:
- Company name must match legal name on website
- Industry must be classified in one of 6 categories
- Minimum 2 tech indicators identified
- Minimum 3 pain points identified
- Confidence score reflects data quality (0-100)

CONSTRAINTS:
- Complete analysis in under 30 seconds
- Handle websites that block scrapers gracefully
- Accept manual company description text if website is unreachable
- No external database or API calls for company data (web-only)

NON-GOALS:
- Competitor analysis
- Financial data extraction
- Lead scoring (that's Feature 2)
```

Verify the specification was created:
```bash
cat .specify/specs/lead-profiler/spec.md
```

The file should describe intent, success criteria, and constraints. Read through it. If anything is missing or unclear, edit the spec directly in your editor and clarify it.

---

## Step 2: Plan

Generate your implementation plan:
```bash
/sp.plan
```

This creates a high-level architecture (how to fetch website content, how to extract data, how to format output).

Review the plan:
```bash
cat .specify/specs/lead-profiler/plan.md
```

**Key decisions the plan should address**:
1. How will you fetch website content? (HTTP library? AI content fetching?)
2. How will you extract company info? (Pattern matching? AI analysis? Both?)
3. How will you identify tech indicators? (Keyword search? Code analysis?)
4. How will you infer pain points? (Company description? Marketing messaging?)
5. How will you calculate confidence scores?

If the plan is incomplete, ask clarifying questions and refine it before moving to tasks.

---

## Step 3: Tasks

Generate your task breakdown:
```bash
/sp.tasks
```

This breaks the plan into executable, ordered tasks.

View your tasks:
```bash
cat .specify/specs/lead-profiler/tasks.md
```

You should see 5-8 tasks like:
- "Set up HTTP content fetching"
- "Build company name extraction"
- "Build industry classification"
- "Build tech indicator detection"
- "Build pain point inference"
- "Implement JSON output formatting"
- "Add error handling and fallbacks"
- "Write test cases"

Read through tasks carefully. This is your execution checklist.

---

## Step 4: Implementation

Execute the implementation command:
```bash
/sp.implement
```

This starts the active coding workflow. Work through each task in order.

**What you'll be building**:
Your code will likely have these components:

```python
# 1. Fetch website content
def fetch_company_page(url: str) -> str:
    """Retrieve website content, handle errors gracefully"""

# 2. Extract basic info
def extract_company_name(content: str) -> str:
    """Find exact company name from content"""

def classify_industry(content: str) -> str:
    """Categorize company into one of 6 industries"""

def estimate_company_size(content: str) -> str:
    """Determine startup/smb/enterprise based on indicators"""

# 3. Extract advanced features
def find_tech_indicators(content: str) -> list[str]:
    """Identify programming languages, frameworks, tools"""

def infer_pain_points(content: str) -> list[str]:
    """Extract or infer pain points company is solving"""

# 4. Calculate confidence
def calculate_confidence(extraction_data: dict) -> int:
    """Score 0-100 based on data completeness"""

# 5. Format output
def generate_lead_profile(url: str) -> dict:
    """Orchestrate all extraction, return valid JSON"""
```

**Implementation reality**: You may use AI to accelerate this work (asking Claude or Gemini to generate extraction logic, then testing it). This is expected. Your job is to verify the code works and the output matches the schema.

---

## Step 5: Test and Verify

Run your implementation against a real company URL:

```bash
# Test 1: Stripe (tech/payments, easy case)
python lead_profiler.py https://stripe.com

# Test 2: Nike (retail/footwear, harder case)
python lead_profiler.py https://nike.com

# Test 3: JPMorgan Chase (finance, complex case)
python lead_profiler.py https://jpmorganchase.com
```

For each test, verify the output matches the schema and quality gates.

### Verification Checklist

**For EACH test URL, verify**:

**Schema Compliance**:
- [ ] Output is valid JSON (no syntax errors)
- [ ] All required fields present: `company_name`, `industry`, `estimated_size`, `tech_indicators`, `pain_points`, `confidence_score`
- [ ] Data types correct: strings for name/industry, array for tech/pain points, integer for confidence

**Content Quality**:
- [ ] Company name matches legal name on website
- [ ] Industry in one of 6 categories: technology, finance, healthcare, retail, manufacturing, other
- [ ] Estimated size is one of: startup, smb, enterprise
- [ ] Tech indicators: minimum 2, maximum 8 items
- [ ] Pain points: minimum 3 items (required per constitution)
- [ ] Confidence score between 0-100, is integer (no decimals)

**Success Criteria Met**:
- [ ] Company name extracted correctly
- [ ] Industry classified
- [ ] At least 2 tech indicators identified
- [ ] At least 3 pain points identified
- [ ] Confidence score present and reflects data quality

**Example successful output** (for Stripe):
```json
{
  "company_name": "Stripe",
  "industry": "technology",
  "estimated_size": "enterprise",
  "tech_indicators": [
    "API architecture",
    "Webhooks",
    "Node.js",
    "React",
    "Cloud infrastructure"
  ],
  "pain_points": [
    "Payment integration complexity",
    "Global payment coordination",
    "Developer experience"
  ],
  "confidence_score": 92
}
```

**If any check fails**:
1. Note which check failed
2. Run the code with debug output to see what's happening
3. Fix the extraction logic
4. Re-test until all checks pass

---

## Step 6: Stop Timer and Record Duration

**STOP YOUR TIMER NOW**:

- **End Time**: ________________________
- **Start Time (from above)**: ____________
- **Total Duration**: ____________ minutes

This is your Feature 1 baseline. You'll use this number to measure acceleration:
- Feature 2 target: less than F1 time
- Feature 3 target: less than F2 time
- Feature 4 target: 50% of F1 time

Record this duration in your TIME_TRACKER.md:

```bash
# Edit your TIME_TRACKER.md file
# Find the F1 row and fill in: Start Time | End Time | Duration (min) | Any notes
```

---

## Try With AI

Now that you've completed the full SDD-RI cycle, strengthen your implementation through collaborative reflection.

**Setup**: Open your AI tool (Claude Code, Gemini CLI, or ChatGPT) and have your lead_profiler.py code available.

**Prompt 1: Validation & Edge Cases**
```
I've built a Lead Profiler that outputs JSON profiles for company URLs.
Test URL results:
- Stripe: [paste your Stripe output]
- Nike: [paste your Nike output]
- JPMorgan Chase: [paste your JPMorgan output]

Are there edge cases or validation errors I'm missing? What should I test next?
```

**Expected outcome**: AI identifies gaps (missing tech indicators, inaccurate pain points, confidence scores that don't match data quality) and suggests test cases.

**Prompt 2: Acceleration Reflection**
```
I just completed Feature 1 (Lead Profiler) using the full SDD-RI workflow
(/sp.specify → /sp.plan → /sp.tasks → /sp.implement).
Completion time: [your duration] minutes.

For Feature 2 (ICP Scorer), which will consume my Lead Profiler output,
what decisions from Feature 1 should I explicitly reuse to build faster?
What new decisions will Feature 2 introduce?
```

**Expected outcome**: AI outlines reusable decisions (output schema, data structures, error handling patterns) and new decisions (scoring logic, weighting criteria) so you know what to accelerate vs. rebuild.

---

**Next**: Proceed to Lesson 03 (Feature 2: ICP Scorer). Start your timer again. Your goal: build faster than Feature 1.
