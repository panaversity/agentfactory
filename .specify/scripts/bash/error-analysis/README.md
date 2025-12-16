# Error Analysis Scripts

**Purpose**: Helper scripts that provide quantitative metrics for `/sp.error-analysis` command.

**Philosophy**: Scripts provide **data points** (counts, patterns, flags). AI agent interprets **meaning** (context, severity, recommendations).

---

## Scripts Overview

### 1. `artifact-locator.sh`

**Purpose**: Find all artifacts for a chapter (PHR, spec, plan, tasks, lessons, validation report)

**Usage**:

```bash
./artifact-locator.sh 14
```

**Output** (JSON):

```json
{
  "chapter": 14,
  "artifacts": {
    "phr": "history/phr/2025-01-10-chapter-14.md",
    "spec": "specs/part-4-chapter-14/spec.md",
    "plan": "specs/part-4-chapter-14/plan.md",
    "tasks": "specs/part-4-chapter-14/tasks.md",
    "lessons": [
      "apps/learn-app/docs/04-Part-4/14-data-types/01-intro.md",
      "apps/learn-app/docs/04-Part-4/14-data-types/02-numeric.md"
    ],
    "validation_report": "VALIDATION_REPORT_CHAPTER_14.md"
  },
  "status": "complete"
}
```

---

### 2. `triage-metrics.sh`

**Purpose**: Quick quantitative assessment for 5-minute triage

**Usage**:

```bash
./triage-metrics.sh 14
```

**Output** (JSON):

```json
{
  "chapter": 14,
  "metrics": {
    "colearning_elements": {
      "💬_prompts": 8,
      "🎓_commentaries": 5,
      "🚀_challenges": 3,
      "✨_tips": 2
    },
    "lesson_closure_violations": [
      "lesson-3.md has ## Summary after Try With AI (line 456)"
    ],
    "pedagogical_ordering_flags": [
      "lesson-1.md:234 - String methods (may be taught later)"
    ],
    "code_block_count": 45,
    "lesson_count": 5,
    "avg_concepts_per_lesson": 6.2,
    "forward_references_found": 2
  },
  "suggested_analysis_depth": "full"
}
```

**What it detects**:

- CoLearning element counts (💬🎓🚀✨)
- Lesson closure violations (sections after "Try With AI")
- Potential forward references (common patterns)
- Code block count
- Estimated concepts per lesson

**Suggested depth logic**:

- `"full"` if: Forward refs > 2 OR closure violations > 0 OR colearning < 60% expected
- `"quick"` otherwise

---

### 3. `colearning-metrics.sh`

**Purpose**: Detailed colearning element breakdown per lesson

**Usage**:

```bash
./colearning-metrics.sh 14
```

**Output** (JSON):

```json
{
  "chapter": 14,
  "lessons": {
    "01-intro": {
      "💬_prompts": 2,
      "🎓_commentaries": 1,
      "🚀_challenges": 1,
      "✨_tips": 0,
      "prompt_locations": [234, 456],
      "prompt_snippets": [
        "Ask your AI to explain data types",
        "Tell your AI to create variables"
      ]
    },
    "02-numeric": { ... }
  },
  "totals": {
    "💬_prompts": 8,
    "🎓_commentaries": 5,
    "🚀_challenges": 3,
    "✨_tips": 2
  },
  "expected_ranges": {
    "💬_prompts": "5-20 (1-4 per lesson × 5 lessons)",
    "🎓_commentaries": "10-20 (2-4 per lesson)",
    "🚀_challenges": "5-20 (1-4 per lesson)",
    "✨_tips": "5-15 (1-3 per lesson)"
  }
}
```

**What it provides**:

- Per-lesson element counts
- Prompt locations (line numbers)
- First 3 prompt snippets (for AI to read and judge quality)
- Expected ranges based on lesson count

---

### 4. `detect-forward-references.sh`

**Purpose**: Flag potential pedagogical ordering violations

**Usage**:

```bash
./detect-forward-references.sh 14
```

**Output** (JSON):

```json
{
  "chapter": 14,
  "violations_detected": [
    {
      "lesson": "01-intro.md",
      "line": 234,
      "code_snippet": "name.upper()",
      "issue": "String method not introduced",
      "severity_flag": "CRITICAL",
      "context": "name = 'alice' print(name.upper()) # Prints: ALICE"
    }
  ],
  "clean_lessons": ["03-strings.md", "04-booleans.md"],
  "violation_count": 1
}
```

**What it detects** (heuristic patterns):

- String methods: `.upper()`, `.lower()`, `.strip()`, `.split()`, etc.
- Built-in functions: `isinstance()`, `len()`, `type()`, `range()`
- Advanced syntax: `def`, `class`, `lambda`, `import`
- Type constructors: `list()`, `dict()`, `set()`, `tuple()`

**Severity flags** (heuristic):

- `CRITICAL`: `def`, `class`, `isinstance()` (completely blocks beginner)
- `MAJOR`: `len()`, `type()` (confusing but might infer from context)

**Important**: These are **FLAGS**, not confirmed violations. AI agent reads context to judge actual severity.

---

## Usage Pattern

**By AI Agent** (in `/sp.error-analysis` command):

1. **Phase 0**: Run `artifact-locator.sh` to find all files
2. **Phase 1**: Run `triage-metrics.sh` for quick assessment
3. **Phase 2** (if needed):
   - Run `colearning-metrics.sh` for detailed element analysis
   - Run `detect-forward-references.sh` for pedagogical ordering
4. **AI Intelligence**: Read flagged content, interpret context, judge severity, recommend fixes

**NOT by AI Agent**:

- Direct violation scoring (scripts flag, AI judges)
- Automated pass/fail (AI reads and decides)
- Template fill-in (AI writes narrative analysis)

---

## Design Philosophy

### Scripts Provide:

✅ **Quantitative data** (counts, line numbers, patterns)
✅ **Heuristic flags** (potential issues for AI to investigate)
✅ **Structured JSON** (easy for AI to parse)

### AI Provides:

✅ **Qualitative judgment** (is this actually a problem?)
✅ **Context interpretation** (read surrounding text, understand intent)
✅ **Severity assessment** (critical vs. minor based on pedagogy)
✅ **Strategic recommendations** (why it matters, how to fix)

### Example: Forward Reference Detection

**Script says**: "Found `.upper()` at line 234"
**AI reads context**:

```python
# Line 230-240
name = "alice"
print(name.upper())  # No introduction of methods
```

**AI judges**: "CRITICAL - beginner doesn't know what methods are"

vs.

**Script says**: "Found `.upper()` at line 234"
**AI reads context**:

````markdown
### String Methods (New Concept)

Python strings have built-in methods (actions they can do).
One useful method is `.upper()` which converts to uppercase:

```python
name = "alice"
print(name.upper())  # Prints: ALICE
```
````

````
**AI judges**: "ACCEPTABLE - concept introduced inline before use"

---

## JSON Schema

All scripts output valid JSON with this general structure:

```typescript
interface ScriptOutput {
  chapter: number;
  [key: string]: any;  // Script-specific fields
}
````

### Common Fields:

- `chapter`: Chapter number (integer)
- `error`: Error message if script fails (string, optional)

### Script-Specific Fields:

See individual script documentation above.

---

## Maintenance

### Adding New Detection Patterns

Edit `detect-forward-references.sh`:

```bash
declare -A PATTERNS=(
    # Existing patterns...
    ["your_new_pattern"]="Description of issue"
)
```

### Adjusting Severity Heuristics

Edit `detect-forward-references.sh` severity logic:

```bash
SEVERITY="MAJOR"
if [[ "$PATTERN" =~ "your_critical_pattern" ]]; then
    SEVERITY="CRITICAL"
fi
```

### Modifying Expected Ranges

Edit `colearning-metrics.sh` expected ranges:

```bash
"💬_prompts": "$((LESSON_COUNT * 1))-$((LESSON_COUNT * 4))"
```

---

## Testing

```bash
# Test all scripts on an existing chapter
./artifact-locator.sh 14
./triage-metrics.sh 14
./colearning-metrics.sh 14
./detect-forward-references.sh 14

# Verify JSON output is valid
./triage-metrics.sh 14 | jq .
```

---

## Integration with `/sp.error-analysis`

These scripts are **helpers**, not standalone analysis tools. They:

1. Provide quantitative metrics quickly
2. Flag potential issues for AI investigation
3. Structure data in parseable format

**The AI agent** (via `/sp.error-analysis` command):

1. Runs scripts to gather data
2. Reads flagged content in context
3. Applies constitutional principles
4. Judges severity based on pedagogy
5. Recommends strategic fixes
6. Partners with user on decisions

**Result**: AI-centric analysis with quantitative support, not mechanical scoring.
