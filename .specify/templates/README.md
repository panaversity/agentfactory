# SpecKit Templates

**Version**: 2.0.0
**Last Updated**: 2025-11-10

This directory contains standardized templates for all SpecKit+ workflow artifacts.

---

## Template Inventory

### Core SDD Workflow Templates

1. **`spec-template.md`**
   - Feature specification template
   - Used by: `/sp.specify`
   - Output: `specs/<feature>/spec.md`

2. **`plan-template.md`**
   - Implementation planning template
   - Used by: `/sp.plan`
   - Output: `specs/<feature>/plan.md`

3. **`tasks-template.md`**
   - Task breakdown template
   - Used by: `/sp.tasks`
   - Output: `specs/<feature>/tasks.md`

---

### Historical Artifact Templates

4. **`phr-template.prompt.md`**
   - Prompt History Record template
   - Used by: `/sp.phr`
   - Output: `history/prompts/<feature>/YYYY-MM-DD-<title>.md`

5. **`adr-template.md`**
   - Architectural Decision Record template
   - Used by: `/sp.adr`
   - Output: `history/adrs/<number>-<title>.md`

6. **`error-analysis-report-template.md`** ✨ NEW
   - Error analysis report template
   - Used by: `/sp.error-analysis`
   - Output: `history/error-analysis/YYYY-MM-DD-chapter-N-[type].md`

7. **`constitution-sync-report-template.md`** ✨ NEW
   - Constitution sync report template
   - Used by: `/sp.constitution-sync`
   - Output: `history/constitution-sync/YYYY-MM-DD-chapter-N-sync-vX.X.X.md`

---

### Utility Templates

8. **`checklist-template.md`**
   - Custom checklist template
   - Used by: `/sp.checklist`
   - Output: `specs/<feature>/checklist.md`

9. **`agent-file-template.md`**
   - Agent prompt template
   - Used for: Creating new agents in `.claude/agents/`

---

## Template Structure Standards

All templates follow these conventions:

### 1. Frontmatter (YAML/Markdown)
```yaml
---
id: {{ID}}
title: {{TITLE}}
date: {{DATE_ISO}}
# ... other metadata
---
```

### 2. Placeholder Syntax
- `{{PLACEHOLDER}}` - Required replacement
- `{{OPTIONAL_PLACEHOLDER}}` - Optional field
- HTML comments `<!-- instruction -->` - Guidance, not output

### 3. Section Structure
- Clear headings (`##`, `###`)
- Consistent formatting
- Checkboxes for actionable items
- Tables for structured data

### 4. Human + AI Readable
- Descriptive section titles
- Inline guidance comments
- Examples where helpful
- Clear acceptance criteria

---

## Report Templates Deep Dive

### Error Analysis Report Template

**Purpose**: Standardize error analysis outputs for workflow quality assessment

**Key Sections**:
- **Phase 1**: Quick triage (quantitative metrics + AI judgment)
- **Phase 2**: Deep analysis (4 checks: Co-Learning, Pedagogical, Constitutional, Learner Value)
- **Phase 3**: Root cause analysis & pattern detection
- **Phase 4**: Actionable recommendations (critical/major/systemic)

**Philosophy**: AI intelligence at core (scripts provide data, AI provides judgment)

**Used by**: `/sp.error-analysis` command

**Output naming**:
- Triage: `history/error-analysis/YYYY-MM-DD-chapter-N-triage.md`
- Full: `history/error-analysis/YYYY-MM-DD-chapter-N-full-analysis.md`

---

### Constitution Sync Report Template

**Purpose**: Standardize constitution synchronization reports for chapter maintenance

**Key Sections**:
- **Phase 0**: Constitution delta analysis
- **Phase 1**: Spec/plan compliance check
- **Phase 2**: Per-lesson intelligence (surgical/enhanced/full regen/no change)
- **Phase 3**: Chapter-level validation
- **Phase 4**: Summary & recommendations

**Philosophy**: Intelligent hybrid approach (right intervention per lesson, not per chapter)

**Used by**: `/sp.constitution-sync` command

**Output naming**: `history/constitution-sync/YYYY-MM-DD-chapter-N-sync-vX.X.X.md`

---

## Helper Scripts

Both report templates are supported by helper scripts:

### Error Analysis Scripts
Location: `.specify/scripts/bash/error-analysis/`

- `artifact-locator.sh` - Find all chapter artifacts
- `triage-metrics.sh` - Quick quantitative assessment
- `colearning-metrics.sh` - Detailed element breakdown
- `detect-forward-references.sh` - Flag pedagogical violations

### Constitution Sync Scripts
Location: `.specify/scripts/bash/constitution-sync/`

- `artifact-locator.sh` - Find all chapter artifacts
- `compliance-metrics.sh` - Quantitative compliance per lesson
- `detect-forward-references.sh` - Flag ordering violations

**Philosophy**: Scripts provide quantitative data (counts, patterns, flags). AI provides qualitative intelligence (context, severity, recommendations).

---

## Template Usage Guidelines

### When Creating New Templates

1. **Follow existing patterns** - Look at similar templates
2. **Use placeholders** - `{{UPPERCASE_SNAKE_CASE}}`
3. **Include guidance** - HTML comments with instructions
4. **Provide examples** - Show what good output looks like
5. **Test with AI** - Verify AI can fill all placeholders correctly

### When Using Templates

1. **Read entire template first** - Understand structure
2. **Fill all required placeholders** - No `{{PLACEHOLDERS}}` in output
3. **Remove guidance comments** - Keep output clean
4. **Preserve structure** - Maintain headings and sections
5. **Validate completeness** - Check no missing sections

### When Updating Templates

1. **Document changes** - Update version and changelog
2. **Update dependent commands** - Sync command files
3. **Test backwards compatibility** - Existing reports still valid?
4. **Announce changes** - Update team/documentation

---

## History Directory Structure

Templates output to organized history directories:

```
history/
├── adrs/                    # Architectural decisions
├── prompts/                 # Prompt history records
│   ├── constitution/
│   ├── general/
│   └── <feature-name>/
├── error-analysis/          # Error analysis reports (NEW)
└── constitution-sync/       # Constitution sync reports (NEW)
```

---

## Template Versioning

**Version Scheme**: `MAJOR.MINOR.PATCH`

- **MAJOR**: Breaking changes (structure incompatible)
- **MINOR**: New sections/fields (backwards compatible)
- **PATCH**: Clarifications, typo fixes

**Current Versions**:
- Core SDD templates: `1.x`
- Report templates: `1.0` (newly created)

---

## Contributing New Templates

1. Create template in `.specify/templates/`
2. Add to this README
3. Create associated command in `.claude/commands/`
4. Add helper scripts if needed (in `.specify/scripts/bash/`)
5. Test with real workflow
6. Document in command file

---

## Philosophy: Templates as Contracts

Templates are **executable specifications** for AI agents:

- **Standardization**: Consistent output format
- **Completeness**: No missing sections
- **Quality**: Built-in guidance and examples
- **Traceability**: Structured data for future analysis
- **Co-Learning**: Templates embody our pedagogical values

**Templates ensure AI agents produce publication-ready artifacts, not rough drafts.**

---

**Questions?** See `.specify/templates/book/` for additional book-specific templates.
