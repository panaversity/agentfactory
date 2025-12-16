# Constitution-Sync Scripts

**Purpose**: Helper scripts that provide quantitative metrics for `/sp.constitution-sync` command.

**Philosophy**: Scripts provide **data points** (counts, patterns, flags). AI agent interprets **meaning** (context, severity, recommendations).

---

## Scripts Overview

### 1. `artifact-locator.sh`

**Purpose**: Find all artifacts for a chapter (spec, plan, tasks, lessons, validation report)

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

### 2. `compliance-metrics.sh`

**Purpose**: Quantitative compliance metrics for a single lesson

**Usage**:

```bash
./compliance-metrics.sh 14 apps/learn-app/docs/04-Part-4/14-data-types/01-intro.md
```

**Output** (JSON):

```json
{
  "chapter": 14,
  "lesson": "01-intro.md",
  "colearning_elements": {
    "💬_prompts": 0,
    "🎓_commentaries": 0,
    "🚀_challenges": 0,
    "✨_tips": 0
  },
  "lesson_closure": {
    "try_with_ai_exists": true,
    "post_sections": []
  },
  "pedagogical_ordering": {
    "forward_references_found": 0,
    "flagged_lines": []
  },
  "tone_analysis": {
    "conversational_score": 8,
    "exploration_language": "present",
    "ai_partnership": "embedded"
  },
  "code_quality": {
    "python_version": "3.10+",
    "type_hints": "present",
    "security": "no_issues"
  }
}
```

**What it detects**:

- CoLearning element counts (💬🎓🚀✨)
- Lesson closure violations (sections after "Try With AI")
- Forward references (heuristic patterns)
- Tone analysis (conversational vs documentation style)
- Code quality (Python version, type hints, security issues)

---

### 3. `detect-forward-references.sh`

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

- String methods: `.upper()`, `.lower()`, `.strip()`, `.split()`, `.replace()`, `.startswith()`, `.endswith()`
- Built-in functions: `isinstance()`, `len()`, `type()`, `range()`
- Advanced syntax: `def`, `class`, `lambda`, `import`
- Type constructors: `list()`, `dict()`, `set()`, `tuple()`

**Severity flags** (heuristic):

- `CRITICAL`: `def`, `class`, `isinstance()` (completely blocks beginner)
- `MAJOR`: `len()`, `type()` (confusing but might infer from context)

**Important**: These are **FLAGS**, not confirmed violations. AI agent reads context to judge actual severity.

---

## Usage Pattern

**By AI Agent** (in `/sp.constitution-sync` command):

1. **Phase 0**: Run `artifact-locator.sh` to find all files
2. **Phase 1**: Check spec/plan compliance
3. **Phase 2** (per-lesson):
   - Run `compliance-metrics.sh` for quantitative assessment
   - Run `detect-forward-references.sh` for pedagogical ordering
4. **AI Intelligence**: Read flagged content, interpret context, judge severity, decide intervention

**NOT by AI Agent**:

- Direct violation scoring (scripts flag, AI judges)
- Automated pass/fail (AI reads and decides)
- Automatic fixes (AI decides intervention strategy first)

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
✅ **Intervention decision** (surgical edit vs. enhanced regen vs. full regen)

### Example: Forward Reference Detection

**Script says**: "Found `.upper()` at line 234"
**AI reads context**:

```python
# Line 230-240
name = "alice"
print(name.upper())  # No introduction of methods
```

**AI judges**: "CRITICAL - beginner doesn't know what methods are"
**AI decides**: "Full regeneration needed (fundamental pedagogical violation)"

vs.

**Script says**: "Found `.upper()` at line 234"
**AI reads context**:

```markdown
### String Methods (New Concept)

Python strings have built-in methods (actions they can do).
One useful method is `.upper()` which converts to uppercase:

\`\`\`python
name = "alice"
print(name.upper()) # Prints: ALICE
\`\`\`
```

**AI judges**: "ACCEPTABLE - concept introduced inline before use"
**AI decides**: "No change needed"

---

## JSON Schema

All scripts output valid JSON with this general structure:

```typescript
interface ScriptOutput {
  chapter: number;
  [key: string]: any; // Script-specific fields
}
```

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

### Adding New Metrics

Edit `compliance-metrics.sh` to add new quantitative checks:

```bash
# New metric example
NEW_METRIC_COUNT=$(grep -c "pattern" "$LESSON_FILE" 2>/dev/null || echo "0")

# Add to JSON output
"new_metric": {
  "count": $NEW_METRIC_COUNT
}
```

---

## Testing

```bash
# Test all scripts on an existing chapter
./artifact-locator.sh 14
./compliance-metrics.sh 14 apps/learn-app/docs/04-Part-4/14-data-types/01-intro.md
./detect-forward-references.sh 14

# Verify JSON output is valid
./compliance-metrics.sh 14 apps/learn-app/docs/04-Part-4/14-data-types/01-intro.md | jq .
```

---

## Integration with `/sp.constitution-sync`

These scripts are **helpers**, not standalone analysis tools. They:

1. Provide quantitative metrics quickly
2. Flag potential issues for AI investigation
3. Structure data in parseable format

**The AI agent** (via `/sp.constitution-sync` command):

1. Runs scripts to gather data
2. Reads flagged content in context
3. Applies constitutional principles
4. Judges severity based on pedagogy
5. Decides intervention strategy (surgical/enhanced/full/skip)
6. Executes fixes
7. Partners with user on decisions

**Result**: AI-centric analysis with quantitative support, not mechanical scoring.
