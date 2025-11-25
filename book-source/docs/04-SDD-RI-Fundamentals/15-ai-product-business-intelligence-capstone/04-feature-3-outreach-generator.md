---
sidebar_position: 4
title: "Feature 3: Outreach Generator"
---

# Feature 3: Outreach Generator

By Feature 3, the SDD-RI cycle is muscle memory. You've built Lead Profiler (F1) and ICP Scorer (F2). Now build Outreach Generator: takes Lead Profile + ICP Score, selects template, personalizes, outputs ready-to-send message.

**Start your timer (total capstone so far):**
- Feature 1: _____ minutes
- Feature 2: _____ minutes
- Feature 3 start: _____

## Define Outreach Templates

Create the templates you'll use for different ICP categories:

```bash
touch outreach_templates.json
```

```json
{
  "templates": {
    "hot": {
      "tone": "direct",
      "approach": "value_proposition",
      "template": "Hi {contact_name}, I noticed {company_name} is {pain_point_reference}. We help {industry} companies like yours {value_statement}. Worth a 15-minute call this week?"
    },
    "warm": {
      "tone": "educational",
      "approach": "insight_sharing",
      "template": "Hi {contact_name}, I've been researching {industry} companies and noticed an interesting trend around {pain_point_reference}. I wrote a quick analysis that might be relevant to {company_name}. Would you find that useful?"
    },
    "cold": {
      "tone": "nurture",
      "approach": "relationship_building",
      "template": "Hi {contact_name}, I came across {company_name} while researching {industry} trends. I'd love to understand how you're thinking about {pain_point_reference}. No pitch—just curious about your perspective."
    }
  },
  "personalization_fields": [
    "contact_name",
    "company_name",
    "industry",
    "pain_point_reference",
    "tech_indicator_reference",
    "value_statement"
  ]
}
```

Save this file. Open it in your editor and verify the structure.

## Write the Specification

Run `/sp.specify` and write the feature specification:

**Feature**: Outreach Generator

**Intent**: Given a Lead Profile and ICP Score from upstream features, generate a personalized outreach message by selecting the appropriate template, filling personalization fields, and producing a ready-to-send message.

**Input**:
- `lead_profile`: JSON object (from Feature 1 output)
- `icp_score`: JSON object (from Feature 2 output)

**Output**:
```yaml
message: string (personalized outreach message)
template_used: string (hot/warm/cold)
personalization_applied:
  - field: string (name of field)
    value: string (actual value filled)
suggested_subject_line: string
confidence: number (0-100)
```

**Success Criteria**:
- Correct template selected based on ICP score category
- All personalization fields filled with relevant data from lead profile
- Message reads naturally without obvious template markers
- Subject line tone matches template tone
- Confidence score reflects quality of personalization

**Constraints**:
- Must accept output from Feature 2 (icp_score.json format)
- Must use only the three templates defined above
- Personalization must use data present in lead_profile (no hallucinations)

## Plan and Implement

Run the SDD-RI workflow:

```bash
/sp.plan
/sp.tasks
/sp.implement
```

Your implementation will typically include:

1. **Template Selection**: Logic to pick hot/warm/cold based on ICP score category
2. **Field Extraction**: Pull contact_name, company_name, industry, pain_point_reference from lead_profile
3. **Template Filling**: Replace {field} markers with extracted values
4. **Personalization Check**: Verify fields are filled with actual lead data
5. **Subject Line Generation**: Create subject matching template tone
6. **Confidence Scoring**: Rate personalization quality (0-100)

Keep the code modular so it can chain with F1 and F2 outputs.

## Test Full Pipeline (F1 → F2 → F3)

Run the complete three-feature pipeline to verify composition:

```bash
# Sequential execution
python lead_profiler.py https://stripe.com > profile.json
python icp_scorer.py profile.json > score.json
python outreach_generator.py profile.json score.json
```

Or as a piped chain:

```bash
URL="https://stripe.com"
python lead_profiler.py $URL | python icp_scorer.py | python outreach_generator.py
```

Verify the output JSON includes:
- `message`: Personalized text with filled template
- `template_used`: One of hot/warm/cold
- `personalization_applied`: List of fields and values actually used
- `suggested_subject_line`: Subject matching tone
- `confidence`: Numeric score

Example output:

```json
{
  "message": "Hi there, I noticed Stripe is navigating complex payment regulations while scaling globally. We help fintech companies like yours automate compliance workflows. Worth a 15-minute call this week?",
  "template_used": "hot",
  "personalization_applied": [
    { "field": "contact_name", "value": "Sarah" },
    { "field": "company_name", "value": "Stripe" },
    { "field": "industry", "value": "fintech" },
    { "field": "pain_point_reference", "value": "navigating complex payment regulations while scaling globally" },
    { "field": "value_statement", "value": "automate compliance workflows" }
  ],
  "suggested_subject_line": "Quick question about Stripe's compliance workflow",
  "confidence": 85
}
```

Run this test with at least 3 different company URLs (from F1/F2 tests) to ensure the pipeline holds.

**Stop your timer and record results:**

Feature 3 duration: _____ minutes

Update your cumulative tracking:
- Feature 1: _____ minutes
- Feature 2: _____ minutes
- Feature 3: _____ minutes
- **Total capstone time: _____ minutes**

Note the trend: Is F3 faster than F2 (due to pattern mastery)? By how much?

## Try With AI

**Prompt 1**: "I have this generated outreach message: [paste your message JSON output]. Review the personalization. Is it specific enough to the lead's actual situation, or does it feel generic?"

**Prompt 2**: "Generate 5 subject line variations for this outreach message: [paste message text]. Try different angles: curiosity hook, value statement, pain point acknowledgment, social proof, and question."
