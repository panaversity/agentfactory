# Chapter 15 Redesign Specification

## Intent

Transform Chapter 15 from an impractical "AI Sales Assistant" (requiring leads students don't have) into a **Personal AI Business Intelligence System** that uses real, accessible tools (Gemini App, NotebookLM) with data students actually own (their profiles, target companies).

Students build a 4-feature system to analyze their professional presence, research their target market, generate content strategy, and create an actionable dashboard—all while measuring intelligence accumulation (F4 < 50% of F1 time).

## Problem with Previous Design

1. **No practical data source** - Lead profiling requires leads students don't have
2. **Python code** - Students haven't learned Python yet (that's Part 5, Ch 16+)
3. **Sales-only focus** - Chapter title promises "Business Intelligence" not just sales
4. **No accessible tools** - Unlike Ch14's Gemini.google.com + Playwright approach

## New Design Principles

1. **Real data students own** - Their LinkedIn, GitHub, portfolio, target companies
2. **Free accessible tools** - Gemini App, NotebookLM (Google account required)
3. **No student-written code** - AI generates, students validate and refine
4. **Career value** - Students leave with actual personal brand strategy
5. **Business intelligence breadth** - Market research, competitive analysis, content strategy

## Tools Used

| Tool | Purpose | Access |
|------|---------|--------|
| **Gemini App** (gemini.google.com) | Analysis, generation, synthesis | Free with Google account |
| **NotebookLM** (notebooklm.google.com) | Deep research, source synthesis | Free with Google account |
| **Claude Code / Gemini CLI** | SDD-RI workflow (/sp.specify, etc.) | From Ch14 |

## 4 Features

### Feature 1: Personal Brand Profiler
**Tool**: Gemini App
**Input**: Student's LinkedIn About section, GitHub bio, portfolio description
**Process**: Copy-paste into Gemini with structured prompt
**Output**: Brand analysis JSON
```json
{
  "core_strengths": ["...", "..."],
  "positioning_statement": "...",
  "brand_gaps": ["...", "..."],
  "differentiation_opportunities": ["...", "..."],
  "confidence_score": 85
}
```
**Quality Gate**: Must identify 3+ strengths and 2+ gaps

### Feature 2: Market Intelligence Scanner
**Tool**: NotebookLM
**Input**: 3-5 target company pages (About, Careers), job postings, industry articles
**Process**: Upload sources to NotebookLM, generate research brief
**Output**: Market intelligence brief
```json
{
  "industry_trends": ["...", "..."],
  "skill_demands": ["...", "..."],
  "competitor_landscape": {...},
  "opportunity_areas": ["...", "..."],
  "sources_analyzed": 5
}
```
**Quality Gate**: Must synthesize 3+ sources with citations

### Feature 3: Content Strategy Generator
**Tool**: Gemini App
**Input**: F1 brand analysis + F2 market brief
**Process**: Feed both outputs to Gemini with content strategy prompt
**Output**: Content calendar
```json
{
  "content_pillars": ["...", "...", "..."],
  "weekly_schedule": {...},
  "topic_ideas": [...],
  "format_recommendations": [...],
  "first_week_actions": [...]
}
```
**Quality Gate**: Must reference both F1 strengths and F2 trends

### Feature 4: Action Dashboard
**Tool**: Claude Code (markdown generation)
**Input**: F1 + F2 + F3 outputs
**Process**: Run /sp.implement to aggregate into dashboard
**Output**: Unified markdown dashboard
- Brand summary (from F1)
- Market opportunities (from F2)
- Content calendar (from F3)
- Priority action items (synthesized)
- 30/60/90 day goals

**Quality Gate**: Must aggregate all three features without redundancy

## Lesson Structure

| Lesson | Title | Duration | Focus |
|--------|-------|----------|-------|
| 01 | Project Setup + Constitution | 30 min | Define BI quality standards |
| 02 | Feature 1: Personal Brand Profiler | 60 min | Gemini App analysis (baseline) |
| 03 | Feature 2: Market Intelligence Scanner | 45 min | NotebookLM research |
| 04 | Feature 3: Content Strategy Generator | 40 min | Gemini synthesis |
| 05 | Feature 4: Action Dashboard | 30 min | Aggregation (50% target) |
| 06 | Skill Creation + Polish | 45 min | P+Q+P formalization |
| 07 | Ship + Retrospective | 30 min | Measure acceleration |
| 08 | Quiz | 20 min | Assessment |

## Intelligence Accumulation Measurement

Same framework as before:
- F1 = baseline time
- F4 target = < 50% of F1 time
- Track what patterns transfer (prompting strategies, output validation, quality gates)

## Success Criteria

1. Students complete 4 features using only Gemini App + NotebookLM + Claude Code
2. No Python code written by students
3. All outputs are real (student's actual brand, real target companies)
4. F4 completes in < 50% of F1 time (intelligence accumulation proven)
5. Students leave with actionable personal brand strategy

## Non-Goals

- Production deployment
- Persistent storage beyond files
- Visual UI (markdown dashboards acceptable)
- Real-time data feeds
- Integration with LinkedIn/GitHub APIs

## Constraints

- Tools: Only Gemini App, NotebookLM, Claude Code/Gemini CLI
- Data: Only public information + student's own profiles
- No code: Students don't write Python (that's Part 5)
- Time: Each feature < 90 minutes

## Migration from Old Design

| Old | New |
|-----|-----|
| Lead Profiler (Python) | Personal Brand Profiler (Gemini App) |
| ICP Scorer (Python) | Market Intelligence Scanner (NotebookLM) |
| Outreach Generator (Python) | Content Strategy Generator (Gemini App) |
| Campaign Dashboard (Python) | Action Dashboard (markdown) |

Core learning outcomes remain:
- SDD-RI workflow mastery
- Intelligence accumulation measurement
- P+Q+P skill creation
- Specification-driven development

But now with practical, accessible execution.
