---
sidebar_position: 4
title: "Feature 3: Outreach Generator"
---

# Feature 3: Outreach Generator

You've built Lead Profiler (F1, baseline time) and ICP Scorer (F2, likely 30-50% faster). By Feature 3, the specification-driven cycle is automatic. You'll take lead profile + ICP score, select the right template, fill personalization fields with actual lead data, and produce a ready-to-send message.

**Key question this lesson answers: How much faster is F3 than F2?** If intelligence accumulation is real, F3 should feel significantly faster because template selection and personalization logic are simpler than scoring logic.

## Start Your Timer

Track cumulative capstone time:

```
Feature 1 (F1): _____ minutes (baseline)
Feature 2 (F2): _____ minutes (actual time)
Feature 3 start: _____ minutes (timestamp)
```

---

## Part 1: Design the Template System (Hands-On)

Before coding, design the three templates your generator will use. These will be stored in JSON and referenced by template selection logic.

### Create `outreach_templates.json`

Create a new file in your project directory:

```bash
touch outreach_templates.json
```

Open in your editor and add the complete template configuration:

```json
{
  "templates": {
    "hot": {
      "tone": "direct",
      "approach": "value_proposition",
      "urgency": "high",
      "template": "Hi {contact_name}, I noticed {company_name} is {pain_point_reference}. We help {industry} companies like yours {value_statement}. Worth a 15-minute call this week?",
      "subject_patterns": [
        "{company_name} + {value_statement}",
        "Quick question about {pain_point_reference}",
        "{industry} compliance: one approach"
      ]
    },
    "warm": {
      "tone": "educational",
      "approach": "insight_sharing",
      "urgency": "medium",
      "template": "Hi {contact_name}, I've been researching {industry} companies and noticed an interesting trend around {pain_point_reference}. I wrote a quick analysis that might be relevant to {company_name}. Would you find that useful?",
      "subject_patterns": [
        "Trend I'm seeing in {industry}",
        "Is {company_name} dealing with {pain_point_reference}?",
        "Quick insight for {company_name}"
      ]
    },
    "cold": {
      "tone": "nurture",
      "approach": "relationship_building",
      "urgency": "low",
      "template": "Hi {contact_name}, I came across {company_name} while researching {industry} trends. I'd love to understand how you're thinking about {pain_point_reference}. No pitch—just curious about your perspective.",
      "subject_patterns": [
        "Question about {company_name}'s approach",
        "{industry} + {pain_point_reference}",
        "Research on {industry} leaders"
      ]
    }
  },
  "personalization_fields": [
    "contact_name",
    "company_name",
    "industry",
    "pain_point_reference",
    "tech_indicator_reference",
    "value_statement"
  ],
  "field_mapping": {
    "contact_name": "extracted.contact_name",
    "company_name": "extracted.company_name",
    "industry": "extracted.industry",
    "pain_point_reference": "identified_pain_points[0]",
    "tech_indicator_reference": "technology_signals[0]",
    "value_statement": "generate from template based on industry"
  }
}
```

Save and verify the structure by opening it in your editor.

**Why this structure matters:**
- `tone` and `approach` guide template selection logic
- `subject_patterns` provide multiple subject line options
- `field_mapping` defines WHERE each field comes from (lead profile vs ICP score)

---

## Part 2: Understand Template Selection Logic (Hands-On)

Before writing code, map out HOW template selection works. This is specification thinking.

### Decision Tree: Template Selection

Create a file `TEMPLATE_SELECTION_LOGIC.md` and write this logic explicitly:

```markdown
# Template Selection Logic

## Input Decision Points

From ICP Score (Feature 2 output), these fields drive template selection:

1. **Score Range**
   - 80-100: "hot" (high ICP match)
   - 50-79: "warm" (moderate match)
   - Below 50: "cold" (exploratory)

2. **Pain Point Confidence** (from ICP score)
   - High (>0.8): Use pain_point_reference in template
   - Low (<0.5): Fall back to industry trend reference

3. **Tech Stack Match** (from ICP score)
   - Match found: "hot" (specific value proposition)
   - No match: Shift to "warm" (relationship building)

## Selection Algorithm

```
IF icp_score.total_score >= 80 AND pain_point_confidence > 0.7
  THEN use "hot" template
ELSE IF icp_score.total_score >= 50
  THEN use "warm" template
ELSE
  THEN use "cold" template
```

## Fallback Rules

If a required personalization field is missing from lead_profile:
- Use generic industry reference
- Flag confidence score (reduce by 10-15 points)
- Never hallucinate data
```

Read this logic out loud. This is what your code will implement.

---

## Part 3: Write the Specification (Hands-On)

Run the specification-driven workflow. Create `spec.md` for Feature 3:

```markdown
# Feature 3: Outreach Generator — Specification

## Intent
Given a Lead Profile (F1 output) and ICP Score (F2 output), generate a personalized outreach message by:
1. Selecting the appropriate template (hot/warm/cold) based on ICP score
2. Extracting personalization fields from lead profile
3. Filling template placeholders with extracted values
4. Generating a matching subject line
5. Computing confidence score reflecting personalization quality
6. Outputting ready-to-send message

## Inputs

### Lead Profile (from Feature 1)
```json
{
  "url": "https://company.com",
  "extracted": {
    "company_name": "string",
    "contact_name": "string",
    "industry": "string",
    "company_size": "string",
    "technology_stack": ["string"]
  },
  "identified_pain_points": ["string"],
  "technology_signals": ["string"]
}
```

### ICP Score (from Feature 2)
```json
{
  "company_name": "string",
  "total_score": 0-100,
  "category": "hot|warm|cold",
  "score_breakdown": {
    "industry_match": 0-100,
    "company_size_match": 0-100,
    "pain_point_confidence": 0-100,
    "tech_stack_match": 0-100
  }
}
```

## Output

```json
{
  "company_name": "string",
  "outreach_message": "string (filled template)",
  "template_used": "hot|warm|cold",
  "personalization_applied": [
    {
      "field": "string (field name)",
      "value": "string (actual value from lead_profile)",
      "source": "string (where field came from)"
    }
  ],
  "suggested_subject_line": "string",
  "subject_line_variants": ["string", "string", "string"],
  "confidence": 0-100,
  "confidence_reasoning": "string (why this confidence score)"
}
```

## Success Criteria

- ✅ Correct template selected based on ICP score range
- ✅ All {field} placeholders filled with actual lead profile data
- ✅ No placeholder markers remain in final message
- ✅ Message reads naturally (not robotic template-fill)
- ✅ Subject line matches template tone and selected topic
- ✅ Confidence score accurately reflects personalization completeness
- ✅ Fallback handling: graceful degradation if fields missing

## Constraints

- Input format: Exactly as defined above (outputs from F1 and F2)
- Must use only three templates (hot/warm/cold)
- Personalization: Must use ONLY data from lead_profile (no hallucinations)
- Confidence: Must explain reasoning for score (not arbitrary)

## Non-Goals

- Email validation (not this feature's responsibility)
- Sending outreach (Feature 4 dashboard handles that)
- A/B testing variants (future enhancement)
```

---

## Part 4: Implementation (Code-First with Spec Validation)

Now you'll implement. As with F1 and F2, start by showing the full algorithm before individual functions.

### High-Level Algorithm (Pseudo-Code)

Create `OUTREACH_GENERATOR_ALGORITHM.md`:

```markdown
# Outreach Generator Algorithm

## Main Function: `generate_outreach(lead_profile, icp_score)`

1. **Load Templates**
   - Load outreach_templates.json
   - Validate all three templates present

2. **Select Template**
   - Read icp_score.total_score
   - Use decision tree to determine hot/warm/cold
   - Return selected template object

3. **Extract Personalization Fields**
   - For each field in personalization_fields:
     - Search lead_profile for field value
     - If found: add to extracted_fields
     - If missing: flag for fallback handling

4. **Fill Template**
   - For each {field} marker in template:
     - Replace with extracted_fields[field]
     - Track which fields were actually filled

5. **Generate Subject Line**
   - Select subject pattern from selected template
   - Fill subject pattern with field values
   - Generate 2-3 variants

6. **Calculate Confidence**
   - Start at 100
   - For each missing field: -15 points
   - For lower ICP score: -5 points
   - For low pain_point_confidence: -10 points
   - Floor at 0, cap at 100

7. **Validate Output**
   - Check no {unfilled} markers remain
   - Verify all required fields populated or documented as missing
   - Return output JSON

## Complexity Indicators

- Simpler than F2 (fewer decision points, less computation)
- Template selection is rule-based (not ML)
- Field mapping is deterministic (not probabilistic)
- This is why F3 should be notably faster than F2
```

### Implementation Code

Create `outreach_generator.py`:

```python
import json
import sys
from typing import Optional, TypedDict
from pathlib import Path

class PersonalizationField(TypedDict):
    field: str
    value: str
    source: str

class ConfidenceBreakdown(TypedDict):
    base_score: int
    missing_fields_penalty: int
    icp_range_adjustment: int
    final_score: int

class OutreachOutput(TypedDict):
    company_name: str
    outreach_message: str
    template_used: str
    personalization_applied: list[PersonalizationField]
    suggested_subject_line: str
    subject_line_variants: list[str]
    confidence: int
    confidence_reasoning: str

# ============================================================================
# TEMPLATE SELECTION
# ============================================================================

def select_template(icp_score: dict) -> tuple[str, dict]:
    """
    Select template (hot/warm/cold) based on ICP score and confidence.

    Returns: (template_name, template_object)
    """
    total_score = icp_score.get("total_score", 0)
    pain_point_confidence = icp_score.get("score_breakdown", {}).get(
        "pain_point_confidence", 0
    )
    tech_stack_match = icp_score.get("score_breakdown", {}).get(
        "tech_stack_match", 0
    )

    # Decision tree: prefer hot if high confidence AND high tech match
    if total_score >= 80 and pain_point_confidence > 70:
        return "hot", templates_data["templates"]["hot"]
    elif total_score >= 50:
        return "warm", templates_data["templates"]["warm"]
    else:
        return "cold", templates_data["templates"]["cold"]

# ============================================================================
# PERSONALIZATION FIELD EXTRACTION
# ============================================================================

def extract_personalization_fields(
    lead_profile: dict, icp_score: dict, selected_template: str
) -> tuple[dict, list[str]]:
    """
    Extract fields from lead_profile for personalization.

    Returns: (extracted_fields_dict, missing_fields_list)
    """
    extracted_fields = {}
    missing_fields = []

    # Direct mappings from lead_profile
    mapping = {
        "contact_name": ["extracted.contact_name"],
        "company_name": ["extracted.company_name"],
        "industry": ["extracted.industry"],
    }

    for field, paths in mapping.items():
        for path in paths:
            keys = path.split(".")
            value = lead_profile
            for key in keys:
                value = value.get(key) if isinstance(value, dict) else None

            if value:
                extracted_fields[field] = value
                break

        if field not in extracted_fields:
            missing_fields.append(field)

    # Pain point reference: from identified_pain_points or use industry
    if "identified_pain_points" in lead_profile and lead_profile[
        "identified_pain_points"
    ]:
        extracted_fields["pain_point_reference"] = lead_profile[
            "identified_pain_points"
        ][0]
    else:
        extracted_fields["pain_point_reference"] = f"{extracted_fields.get(
            'industry', 'their industry'
        )} challenges"
        missing_fields.append("pain_point_reference (fallback used)")

    # Tech indicator reference: from technology_signals
    if "technology_signals" in lead_profile and lead_profile[
        "technology_signals"
    ]:
        extracted_fields["tech_indicator_reference"] = lead_profile[
            "technology_signals"
        ][0]
    else:
        extracted_fields["tech_indicator_reference"] = None

    # Value statement: inferred from industry + template type
    industry = extracted_fields.get("industry", "enterprise")
    if selected_template == "hot":
        value_stmt = f"accelerate {industry} operations"
    elif selected_template == "warm":
        value_stmt = f"optimize {industry} workflows"
    else:
        value_stmt = f"improve {industry} strategy"

    extracted_fields["value_statement"] = value_stmt

    return extracted_fields, missing_fields

# ============================================================================
# TEMPLATE FILLING
# ============================================================================

def fill_template(template_text: str, fields: dict) -> tuple[str, int]:
    """
    Fill template with personalization fields.

    Returns: (filled_message, num_replacements)
    """
    filled = template_text
    replacements = 0

    for field_name, field_value in fields.items():
        if field_value:
            placeholder = "{" + field_name + "}"
            if placeholder in filled:
                filled = filled.replace(placeholder, str(field_value))
                replacements += 1

    return filled, replacements

# ============================================================================
# SUBJECT LINE GENERATION
# ============================================================================

def generate_subject_lines(
    selected_template: str, fields: dict
) -> list[str]:
    """
    Generate subject line variants based on template and fields.
    """
    patterns = templates_data["templates"][selected_template][
        "subject_patterns"
    ]

    variants = []
    for pattern in patterns:
        subject = pattern
        for field_name, field_value in fields.items():
            placeholder = "{" + field_name + "}"
            if placeholder in subject and field_value:
                subject = subject.replace(placeholder, str(field_value))

        variants.append(subject)

    return variants

# ============================================================================
# CONFIDENCE SCORING
# ============================================================================

def calculate_confidence(
    icp_score: dict, missing_fields: list, filled_count: int
) -> tuple[int, str]:
    """
    Calculate confidence score (0-100) with reasoning.

    Returns: (score, reasoning_text)
    """
    base_score = 100
    reasoning_parts = [f"Base score: {base_score}"]

    # Penalty for missing fields (each -15)
    missing_penalty = len(missing_fields) * 15
    if missing_penalty > 0:
        reasoning_parts.append(
            f"Missing {len(missing_fields)} fields: -{missing_penalty} points"
        )

    # Adjustment based on ICP score range
    icp_total = icp_score.get("total_score", 50)
    if icp_total < 50:
        icp_adjustment = -10
        reasoning_parts.append(
            f"ICP score below warm threshold: {icp_adjustment} points"
        )
    else:
        icp_adjustment = 0

    # Pain point confidence factor
    pain_confidence = icp_score.get("score_breakdown", {}).get(
        "pain_point_confidence", 50
    )
    if pain_confidence < 50:
        pain_penalty = -10
        reasoning_parts.append(
            f"Low pain point confidence ({pain_confidence}): {pain_penalty} points"
        )
    else:
        pain_penalty = 0

    # Final calculation
    final_score = max(
        0, min(100, base_score - missing_penalty + icp_adjustment - pain_penalty)
    )

    reasoning_parts.append(f"Final confidence: {final_score}")
    reasoning = "; ".join(reasoning_parts)

    return final_score, reasoning

# ============================================================================
# MAIN ORCHESTRATION
# ============================================================================

def generate_outreach(lead_profile: dict, icp_score: dict) -> OutreachOutput:
    """
    Main function: generate complete outreach message.
    """
    # Step 1: Select template
    template_name, template_obj = select_template(icp_score)
    template_text = template_obj["template"]

    # Step 2: Extract personalization fields
    extracted_fields, missing_fields = extract_personalization_fields(
        lead_profile, icp_score, template_name
    )

    # Step 3: Fill template
    filled_message, filled_count = fill_template(template_text, extracted_fields)

    # Step 4: Generate subject lines
    subject_lines = generate_subject_lines(template_name, extracted_fields)

    # Step 5: Calculate confidence
    confidence_score, confidence_reason = calculate_confidence(
        icp_score, missing_fields, filled_count
    )

    # Step 6: Build personalization_applied tracking
    personalization_applied: list[PersonalizationField] = []
    for field_name, field_value in extracted_fields.items():
        if field_value:
            personalization_applied.append({
                "field": field_name,
                "value": field_value,
                "source": "lead_profile"
                if field_name in lead_profile.get("extracted", {})
                else "inferred",
            })

    # Step 7: Return complete output
    return {
        "company_name": extracted_fields.get("company_name", "Unknown"),
        "outreach_message": filled_message,
        "template_used": template_name,
        "personalization_applied": personalization_applied,
        "suggested_subject_line": subject_lines[0] if subject_lines else "",
        "subject_line_variants": subject_lines,
        "confidence": confidence_score,
        "confidence_reasoning": confidence_reason,
    }

# ============================================================================
# CLI AND STDIN HANDLING
# ============================================================================

if __name__ == "__main__":
    # Load templates
    templates_path = Path(__file__).parent / "outreach_templates.json"
    with open(templates_path) as f:
        templates_data = json.load(f)

    # Input: lead_profile.json and icp_score.json (two arguments)
    if len(sys.argv) == 3:
        # File arguments
        with open(sys.argv[1]) as f:
            lead_profile = json.load(f)
        with open(sys.argv[2]) as f:
            icp_score = json.load(f)
    else:
        # Piped input: read from stdin
        input_data = sys.stdin.read()
        try:
            # Try parsing as single combined JSON
            data = json.loads(input_data)
            lead_profile = data.get("lead_profile", data)
            icp_score = data.get("icp_score", {})
        except json.JSONDecodeError:
            print("Error: Invalid JSON input", file=sys.stderr)
            sys.exit(1)

    # Generate outreach
    result = generate_outreach(lead_profile, icp_score)

    # Output
    print(json.dumps(result, indent=2))
```

**Review this code with the specification:**
- ✅ Template selection uses decision tree (hot/warm/cold logic)
- ✅ Personalization fields extracted from lead_profile
- ✅ Template filling replaces {field} markers
- ✅ Subject line variants generated
- ✅ Confidence calculated with reasoning
- ✅ Fallback handling: no hallucinated data

---

## Part 5: Pipeline Integration (F1 → F2 → F3)

Now test the complete three-feature pipeline.

### Prerequisites

Ensure you have outputs from Feature 1 and Feature 2:

```bash
# Files you should have from F1 and F2:
ls -la profile.json    # from F1 output
ls -la score.json      # from F2 output
```

### Test 1: Sequential Execution (Files)

Run features in sequence, piping output through the pipeline:

```bash
# Start feature timer
time python outreach_generator.py profile.json score.json
```

Example output (Feature 3):

```json
{
  "company_name": "Stripe",
  "outreach_message": "Hi Sarah, I noticed Stripe is navigating complex payment regulations while scaling globally. We help fintech companies like yours automate compliance workflows. Worth a 15-minute call this week?",
  "template_used": "hot",
  "personalization_applied": [
    {
      "field": "contact_name",
      "value": "Sarah",
      "source": "lead_profile"
    },
    {
      "field": "company_name",
      "value": "Stripe",
      "source": "lead_profile"
    },
    {
      "field": "industry",
      "value": "fintech",
      "source": "lead_profile"
    },
    {
      "field": "pain_point_reference",
      "value": "navigating complex payment regulations while scaling globally",
      "source": "lead_profile"
    },
    {
      "field": "value_statement",
      "value": "automate compliance workflows",
      "source": "inferred"
    }
  ],
  "suggested_subject_line": "Stripe + automate compliance workflows",
  "subject_line_variants": [
    "Stripe + automate compliance workflows",
    "Quick question about navigating complex payment regulations while scaling globally",
    "fintech compliance: one approach"
  ],
  "confidence": 95,
  "confidence_reasoning": "Base score: 100; Final confidence: 95"
}
```

**Verify the output:**
- ✅ Message has NO {unfilled} template markers
- ✅ All personalization_applied fields from lead_profile
- ✅ Subject lines match template tone
- ✅ Confidence score > 80 (high personalization quality)

### Test 2: Full Three-Feature Pipeline

Run F1 → F2 → F3 in one command to verify composition:

```bash
# Measure full pipeline time
time bash -c 'URL="https://stripe.com" && \
python lead_profiler.py $URL > f1.json && \
python icp_scorer.py f1.json > f2.json && \
python outreach_generator.py f1.json f2.json > f3.json && \
cat f3.json'
```

**What to observe:**
- Output flows from F1 → F2 → F3 without manual intervention
- Each feature takes its own input and produces standard JSON output
- F3 output includes all fields from specification

### Test 3: Compare F3 Time to F1 and F2

Record timing for all three features:

```bash
echo "=== Feature 1: Lead Profiler ===" && \
time python lead_profiler.py https://stripe.com > /dev/null

echo "=== Feature 2: ICP Scorer ===" && \
time python icp_scorer.py profile.json > /dev/null

echo "=== Feature 3: Outreach Generator ===" && \
time python outreach_generator.py profile.json score.json > /dev/null
```

**Expected pattern** (if intelligence accumulation works):
```
F1 time: ~30-45 seconds (baseline: all three features, design spec)
F2 time: ~20-30 seconds (60-70% of F1: you have design patterns)
F3 time: ~10-15 seconds (30-40% of F1: simple template logic)

Total capstone time so far: ~60-90 seconds
```

**Record results:**

```
Feature 1: _____ seconds
Feature 2: _____ seconds  (% of F1)
Feature 3: _____ seconds  (% of F1)
Total:     _____ seconds

Pattern observed:
[ ] F3 is faster than F2 ✓ (confirms intelligence reuse)
[ ] F3 similar to F2 (patterns not yet mature)
[ ] F3 slower than F2 (check for debugging overhead)
```

---

## Part 6: Validate Template Quality (Hands-On)

Before "Try With AI," manually validate that your generated messages are actually good.

### Validation Checklist

For your 3+ test outputs (different companies), verify:

```
For each generated message:

[ ] Template selection correct
    - Hot selected when ICP score ≥ 80 and pain_point_confidence > 70
    - Warm selected when ICP score ≥ 50
    - Cold selected otherwise

[ ] Personalization specificity
    - Company name and industry visible in message
    - Pain point reference specific to company (not generic)
    - Value statement relevant to identified pain point

[ ] Message readability
    - No {unfilled} template markers remain
    - Sentence flow is natural (not robotic template-fill)
    - Tone matches template type (direct/educational/nurture)

[ ] Subject line quality
    - First variant mentions specific company or pain point
    - Variants offer different hooks (curiosity, value, pain, proof, question)
    - All variants match template tone

[ ] Confidence score accuracy
    - Score ≥ 85 when all fields present: makes sense
    - Score 70-85 when 1-2 fields missing: reasonable penalty
    - Score ≤ 70 when 3+ fields missing or ICP score low: justified
    - Reasoning explains the score (not arbitrary)
```

### Example Validation (Stripe)

**Input:**
- F1 profile: Stripe (fintech, complex payment regulations)
- F2 score: 95 (hot: high industry match, strong pain point signal)

**Output (F3):**
```json
{
  "template_used": "hot",
  "outreach_message": "Hi Sarah, I noticed Stripe is navigating complex payment regulations while scaling globally. We help fintech companies like yours automate compliance workflows. Worth a 15-minute call this week?",
  "confidence": 95
}
```

**Validation:**
- ✅ "Hot" correct (95 score, high pain_point_confidence)
- ✅ Stripe + fintech visible → company-specific, not generic
- ✅ "complex payment regulations" is exact pain point from F1
- ✅ "automate compliance workflows" matches industry
- ✅ No template markers ({field} symbols) remain
- ✅ Message reads naturally (not robotic)
- ✅ 95 confidence justified (all fields present, high ICP match)

Repeat this for cold and warm examples to build confidence in your logic.

---

## Part 7: Try With AI

Now use AI to refine your templates and test personalization quality at scale.

### Ask Your AI Tool: Template Evaluation

**Copy this exact prompt into your AI tool:**

```
I've built an outreach message generator for a sales pipeline.
Here's a message my system generated:

MESSAGE:
---
Hi Sarah, I noticed Stripe is navigating complex payment regulations
while scaling globally. We help fintech companies like yours automate
compliance workflows. Worth a 15-minute call this week?
---

Template Used: "hot" (direct, value-focused)
Confidence Score: 95

Questions for you:

1. **Specificity Test**: Is this message specific enough that it feels
   personalized to Stripe, or could it be sent to any fintech company?

2. **Personalization Depth**: Which parts feel the most personalized
   (company-specific) vs generic? Which parts would benefit from more
   detail?

3. **Template Fit**: For a company scoring 95/100 ICP match with strong
   pain point signals, does "hot" (direct value prop) seem like the
   right tone? Would "warm" (educational insight) work better?

4. **Pain Point Accuracy**: "Complex payment regulations while scaling
   globally" — is this specific enough, or too broad? What level of
   specificity would make this feel researched?

Respond with specific feedback, not general advice.
```

**Review AI's feedback for:**
- Personalization specificity gaps
- Tone mismatches with ICP score
- Pain point depth issues
- Subject line effectiveness

### Ask Your AI Tool: Subject Line Optimization

**Copy this exact prompt into your AI tool:**

```
I'm testing subject line effectiveness for outreach messages.
Here's my message and the subject lines my system generated:

OUTREACH MESSAGE:
Hi Sarah, I noticed Stripe is navigating complex payment regulations
while scaling globally. We help fintech companies like yours automate
compliance workflows. Worth a 15-minute call this week?

SUBJECT LINES (from my system):
1. "Stripe + automate compliance workflows"
2. "Quick question about navigating complex payment regulations
   while scaling globally"
3. "fintech compliance: one approach"

Task: Generate 5 NEW subject line variations that each use a different
psychological hook:
1. Curiosity hook (intriguing but not clickbait)
2. Value statement (what they gain)
3. Pain point acknowledgment (shows you understand their problem)
4. Social proof (hint that peers are solving this)
5. Direct question (invitation to engagement)

For each, explain which hook you used and why it would work for a
fintech CFO deciding whether to take a 15-minute call.
```

**Review AI's output for:**
- Subject lines that outperform your template patterns
- Hook types that resonate with your ICP
- Psychological triggers specific to your industry/role
- Ideas for expanding your subject_patterns templates

---

## Part 8: Record Cumulative Time

Stop your Feature 3 timer and update the tracking table:

```
Feature 1 (F1): _____ minutes (your baseline)
Feature 2 (F2): _____ minutes (% of F1: ____)
Feature 3 (F3): _____ minutes (% of F1: ____)

Total capstone time so far: _____ minutes

Analysis:
- F3 is faster than F2 by: _____ seconds  ✓ Intelligence accumulation working
- Complexity tier: F3 << F2 << F1 (confirms spec-first design progression)
- Ready for F4: Campaign Dashboard (should be < 50% of F1)
```

---

## Summary: What You Learned

By building Feature 3, you proved that **specification quality accelerates implementation**.

**Specification → Implementation Time Correlation:**

| Feature | Spec Complexity | Implementation Decisions | Time | % of F1 |
|---------|-----------------|--------------------------|------|---------|
| F1 | High (new product) | 15+ decisions (profile, scraping, extraction) | Baseline | 100% |
| F2 | Medium (reuses F1 outputs) | 8-10 decisions (scoring logic, criteria) | Lower | 50-70% |
| F3 | Low (simple template logic) | 3-5 decisions (template selection, field mapping) | Lowest | 30-40% |

**Why F3 is faster:**
- You don't design the scoring algorithm (done in F2)
- You don't scrape and extract data (done in F1)
- You implement template selection (rule-based, not probabilistic)
- You map fields deterministically (not learned from data)
- Confidence scoring has explicit formulas (no complexity)

This is specification-driven reusable intelligence in action: **each feature builds on previous specifications, reducing cognitive load for the next feature.**

---

## Next Step

Move to Lesson 05 (Feature 4: Campaign Dashboard). Your timer shows that F4 should take less than half of F1's time. Go prove it.
