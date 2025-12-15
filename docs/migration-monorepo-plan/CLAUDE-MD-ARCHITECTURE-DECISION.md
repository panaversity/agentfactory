# CLAUDE.md Architecture Decision: Per-App vs Platform-Wide

**Analysis Date**: 2025-12-15
**Current State**: Single root CLAUDE.md
**Monorepo Type**: pnpm-based (TypeScript + Python hybrid)
**Apps**: 
- `book-source` (Docusaurus MDX - frontend)
- `panaversity-fs` (FastAPI - Python backend)
- Various `.specify/` scaffolding projects

---

## 1. Current State Analysis

### Root CLAUDE.md Overview
- **Size**: 758 lines
- **Scope**: Highly specialized for apps/learn-app/content authoring
- **Sections**:
  1. Core Identity: Platform Architect
  2. Context-Gathering Protocol (mandatory for CONTENT work)
  3. Pedagogical Layer Recognition
  4. Constitutional Reasoning Framework
  5. 4-Layer Teaching Method
  6. Domain Skills (Monorepo-focused)
  7. Agent Architecture
  8. Self-Monitoring Checklist
  9. Development Guidelines
  10. Execution Contract
  11. Post-Session Intelligence Harvesting
  12. Quick Reference & Success Metrics
  13. Claude 4 Best Practices

**Current Issue**: ~85% of content is book/pedagogy-specific, ~15% is universally applicable

---

## 2. Content Categorization Matrix

| Section | Category | Applies To | Notes |
|---------|----------|-----------|-------|
| **0. Core Identity** | Platform-level | All apps | Generic platform architect role |
| **I. Context-Gathering** | Book-specific | book-source ONLY | Mandatory for lessons, chapters, pedagogy |
| **I.1 Stakeholder ID** | Platform-level | All apps | Students/Authors/Institutions concept |
| **I.2 Content Work Context** | Book-specific | book-source ONLY | Chapter-index, Part numbers, prerequisites |
| **I.3 Pedagogical Layer** | Book-specific | book-source ONLY | L1-L4 teaching method (not applicable to APIs) |
| **I.4 Conflicts** | Mixed | book-source (primary), panaversity-fs (safety only) | Some conflicts (safety) cross-domain |
| **I.5 Scope Verification** | Platform-level | All complex projects | Counterexample testing, invariants |
| **I.6 State Understanding** | Mixed | All apps | Context-gathering pattern useful everywhere |
| **II. Cognitive Mode** | Book-specific | book-source ONLY | Explicitly about teaching modalities |
| **III. Constitutional Framework** | Book-specific | book-source ONLY | Constitution.md is book-focused |
| **IV. 4-Layer Teaching** | Book-specific | book-source ONLY | Teaching method doesn't apply to APIs |
| **V. Domain Skills** | Platform-level | All apps | Monorepo skills (Nx) useful for all |
| **VI. Agent Architecture** | Platform-level | All apps | Generic agent discovery pattern |
| **VII. Self-Monitoring** | Mixed | book-source (primary), engineering (secondary) | Content checklist vs code checklist |
| **VIII. Execution Contract** | Mixed | book-source (primary), platform (secondary) | Context-gather + act pattern useful everywhere |
| **IX. Intelligence Harvesting** | Platform-level | All apps | PHRs, learnings useful for all |
| **X. Quick Reference** | Book-specific | book-source ONLY | Teaching layer/complexity tiers |
| **XI. Success Metrics** | Book-specific | book-source ONLY | Content-focused definitions |
| **XII. Claude 4 Best Practices** | Platform-level | All apps | XML tags, multi-window strategy, skills usage |
| **Platform Tech Stack** | Platform-level | All apps | Auth, databases, testing patterns |

---

## 3. Claude Code CLAUDE.md Mechanism

### How Claude Code Loads CLAUDE.md

Claude Code reads the **root CLAUDE.md** by default. According to Anthropic's documentation:

1. **File Discovery**: Claude searches for `CLAUDE.md` in the project root
2. **Load Timing**: File is read at conversation start (becomes system context)
3. **Hierarchy Support**: There is **NO automatic inheritance** - child CLAUDE.md files are NOT loaded
4. **Scope**: Applies to entire repository unless overridden

### Key Finding: No Hierarchical Support

Claude Code does NOT support:
- Automatic parent → child CLAUDE.md inheritance
- Per-directory CLAUDE.md overrides
- Conditional loading based on cwd
- Multiple CLAUDE.md files being simultaneously active

**Consequence**: Creating per-app CLAUDE.md files would require manual invocation (mentioning them in prompts), breaking the "automatic context" pattern.

---

## 4. Architecture Options

### Option A: Single Root CLAUDE.md (Current)
**Approach**: Keep everything in one file

**Pros**:
- Claude Code loads it automatically
- Single source of truth
- Automatic context for all work
- Simpler to maintain

**Cons**:
- 758 lines of book-specific content in a Python API project
- Book-centric cognitive mode doesn't fit panaversity-fs context
- Hard to find engineering-relevant sections
- Cognitive load for engineers working on APIs

**Estimate**: Viable but suboptimal (book engineers won't want to parse pedagogy)

---

### Option B: Root CLAUDE.md (Platform) + Per-App CLAUDE.md (Manual)
**Approach**: 
- Keep root with universal + book content
- Create `apps/learn-app/CLAUDE.md` and `panaversity-fs/CLAUDE.md`
- Reference in projects' README with invocation pattern

**Structure**:
```
/CLAUDE.md                          # Root: platform + book content
/apps/learn-app/CLAUDE.md              # Book app: pedagogy + book-specific
/panaversity-fs/CLAUDE.md           # Python API: engineering-focused
```

**Pros**:
- Root still loads automatically (no breaking change)
- Per-app CLAUDE.md provides tailored context
- Clear separation of concerns
- Developers can reference app-specific rules

**Cons**:
- Requires manual invocation in prompts
- Duplication across files
- Maintenance burden (update multiple files)
- Risk of version skew

**Estimate**: Viable but requires developer discipline

---

### Option C: Split Root CLAUDE.md (Recommended)
**Approach**:
- Root CLAUDE.md: Platform-level only (sections 0, V, VI, IX, XII, tech stack)
- `apps/learn-app/CLAUDE.md`: Content-authoring only (sections I-IV, VII-XI for content)
- `panaversity-fs/CLAUDE.md`: Engineering only (sections VII-IX, XII adapted)
- Root README: Links to app-specific guides

**Structure**:
```
/CLAUDE.md                          # Platform-level (~150 lines)
/apps/learn-app/CLAUDE.md              # Content-authoring (~600 lines)
/panaversity-fs/CLAUDE.md           # Engineering (~250 lines)
```

**Pros**:
- Clean separation of concerns
- Each app gets tailored guidance automatically (root loads, then app-specific)
- Minimal duplication (only shared patterns)
- Easier to maintain and evolve independently
- Clear signal: book engineers ≠ platform engineers

**Cons**:
- Claude Code won't auto-load panaversity-fs/CLAUDE.md
- Requires README guidance to mention app-specific files
- Initial refactoring effort

**Estimate**: Optimal long-term, requires one-time refactoring

---

## 5. Recommendation: Option C (Split CLAUDE.md)

### Why Option C is Best

1. **Cognitive Mode Separation**
   - book-source: "Platform architect + content pedagogue" (current)
   - panaversity-fs: "Platform architect + backend engineer"
   - Future apps: Each gets tailored cognitive mode

2. **Automatic Context** (via root)
   - Monorepo skills, agent architecture, best practices
   - Platform tech stack, intelligence harvesting
   - Development guidelines applicable everywhere

3. **App-Specific Context** (manual invoke)
   - Book pedagogy (Layer 1-4, constitution, context-gathering)
   - API engineering (testing, Python standards, FastAPI patterns)
   - Each app README mentions: "Read `/app/CLAUDE.md` for context"

4. **Scaling Pattern**
   - Add new app? Create `new-app/CLAUDE.md`
   - No need to update root or other apps
   - Clear ownership: app team maintains their guide

5. **Maintenance**
   - Root evolves: shared patterns, tooling, standards
   - Apps evolve: independently, without affecting others
   - Convergence point: both reference constitution.md

---

## 6. Migration Plan: Option C Implementation

### Phase 1: Create Root CLAUDE.md (Platform-Level Only)

**Extract into `/CLAUDE.md`** (~150 lines):
```
0. Core Identity: Platform Architect
V. Domain Skills (Monorepo-focused)
VI. Agent Architecture
IX. Post-Session Intelligence Harvesting
XII. Claude 4 Best Practices Integration
- Platform Technologies
```

**New additions to root**:
- Navigation guide: "For content work, see apps/learn-app/CLAUDE.md"
- Link to engineering guide (panaversity-fs/CLAUDE.md)
- High-level cognitive framework

**Delete from root**:
- Sections I-IV (pedagogy)
- Sections VII-VIII (content-specific)
- Sections X-XI (teaching matrices)
- Constitution references (point to book guide instead)

### Phase 2: Create apps/learn-app/CLAUDE.md

**Move into `/apps/learn-app/CLAUDE.md`** (~600 lines):
```
All current content from root CLAUDE.md (sections I-XI)
Update references to constitution.md to relative paths
Add navigation header: "Book Authoring Guide"
```

**Add to book-source guide**:
- Import statement: "For platform-level context, see ../CLAUDE.md"
- Chapter-index.md location
- Example: "You are building content using the 4-Layer Teaching Method"

### Phase 3: Create panaversity-fs/CLAUDE.md

**Create `/panaversity-fs/CLAUDE.md`** (~250 lines):
```
0. Core Identity: Platform Architect + Backend Engineer
I. Context-Gathering (adapted for Python/FastAPI)
  - Identify stakeholder & work type
  - Read API spec before modifying routes
  - Check for dependent services
  - Hardware tier (cloud tier 1-3, not student hardware)
  
V. Domain Skills (Nx, Python-specific tools)
VI. Agent Architecture (MCP server patterns)
VII. Self-Monitoring (Python code checklist)
  - Type hints? Docstrings? Tests?
  - Configuration centralized?
  - Error handling comprehensive?
VIII. Execution Contract (adapted)
IX. Intelligence Harvesting (PHRs for API changes)
XII. Claude 4 Best Practices (Python-specific)

+ Python-Specific Sections:
  - FastAPI patterns (dependency injection, middleware)
  - Database patterns (Alembic migrations, async/await)
  - Testing strategy (pytest fixtures, parametrization)
  - Type hints enforcement (mypy configuration)
  - MCP server patterns (tool definitions, safety)
```

**Add to panaversity-fs guide**:
- Import statement: "For platform-level context, see ../CLAUDE.md"
- Python version requirement
- Local development setup
- Testing checklist before PR

### Phase 4: Update READMEs

**Update root README**:
```markdown
## Developer Guides

### For Content Authoring
See [`apps/learn-app/CLAUDE.md`](./apps/learn-app/CLAUDE.md)
- Pedagogical framework (4-Layer Teaching Method)
- Context-gathering protocol for lessons
- Constitutional compliance guidelines

### For Backend Development
See [`panaversity-fs/CLAUDE.md`](./panaversity-fs/CLAUDE.md)
- FastAPI patterns and conventions
- Python testing and type hints
- API design and documentation

### Platform-Wide Guidelines
See [`CLAUDE.md`](./CLAUDE.md)
- Monorepo architecture and Nx usage
- Agent frameworks and MCP patterns
- Intelligence harvesting and PHRs
```

**Update apps/learn-app/README**:
```markdown
## Developer Guide

See [`CLAUDE.md`](./CLAUDE.md) for comprehensive authoring guidelines.
```

**Update panaversity-fs/README**:
```markdown
## Developer Guide

See [`CLAUDE.md`](./CLAUDE.md) for comprehensive engineering guidelines.
```

### Phase 5: Version and Document

**Add CLAUDE.md section to each**:
```yaml
---
# At top of each CLAUDE.md file
version: "1.0.0"
scope: "platform" | "book-source" | "panaversity-fs"
last_updated: "2025-12-15"
applies_to: 
  - "all-work" | "content-authoring" | "backend-engineering"
---
```

**Create migration summary**:
- List files created/moved
- List files deleted
- Explain inheritance model
- Link to canonical sources

---

## 7. Template: Per-App CLAUDE.md Structure

### apps/learn-app/CLAUDE.md Header
```markdown
# Claude Code Rules — Content Authoring Edition

**Scope**: Panaversity book authoring (Docusaurus + MDX)
**Version**: 1.0.0
**Last Updated**: 2025-12-15

**Prerequisites**: Read `../CLAUDE.md` (platform-wide guidelines)

---

## Overview

You are a platform architect specializing in content pedagogy. Your role combines:
- Educational technology leadership (4-Layer Teaching Method)
- Learning science reasoning (constitutional framework)
- Curriculum design (cross-chapter intelligence accumulation)

This guide covers content authoring workflows. For monorepo and platform patterns, see the root CLAUDE.md.
```

### panaversity-fs/CLAUDE.md Header
```markdown
# Claude Code Rules — Backend Engineering Edition

**Scope**: Panaversity FileSys API (FastAPI + Postgres)
**Version**: 1.0.0
**Last Updated**: 2025-12-15

**Prerequisites**: Read `../CLAUDE.md` (platform-wide guidelines)

---

## Overview

You are a platform architect specializing in backend infrastructure. Your role combines:
- Distributed systems thinking (service design, API contracts)
- Python engineering practices (type hints, testing, async patterns)
- Cloud deployment (Tier 1-3 scaling, performance)

This guide covers backend engineering workflows. For monorepo and platform patterns, see the root CLAUDE.md.
```

---

## 8. Rollout Strategy

### Week 1: Refactoring
- [ ] Create root CLAUDE.md (platform-level)
- [ ] Create apps/learn-app/CLAUDE.md
- [ ] Create panaversity-fs/CLAUDE.md
- [ ] Update all READMEs
- [ ] Test with both apps (manual verification)

### Week 2: Documentation & Launch
- [ ] Create CLAUDE-md-migration guide
- [ ] Announce to team: "CLAUDE.md split completed"
- [ ] Update .mcp.json if needed (Nx integration)
- [ ] Monitor for issues

### Week 3+: Iterate
- [ ] Collect feedback from book engineers
- [ ] Collect feedback from backend engineers
- [ ] Refine app-specific sections
- [ ] Add new patterns as discovered

---

## 9. Risk Assessment

### Risk 1: Claude Code Won't Load panaversity-fs/CLAUDE.md
**Severity**: Medium
**Mitigation**: 
- Document in panaversity-fs/README.md: "Run: `echo 'Read panaversity-fs/CLAUDE.md' | claude`"
- Add MCP command to read it: `/sp.CLAUDE-context`
- Mention in prompts when working in that directory

### Risk 2: Duplication Across Files
**Severity**: Low
**Mitigation**:
- Shared sections (I.1, I.5, XII) summarize with cross-references
- Root CLAUDE.md is source of truth for shared patterns
- Apps reference root for non-duplicated content

### Risk 3: Maintenance Burden
**Severity**: Medium
**Mitigation**:
- Root: Stable, updates rare
- Apps: Evolve independently, owned by app team
- Quarterly review of alignment

### Risk 4: Team Confusion
**Severity**: Medium
**Mitigation**:
- Clear README navigation
- Version numbers in headers
- Explicit "you are in X context" message at top of each file

---

## 10. Decision Summary

| Dimension | Decision | Rationale |
|-----------|----------|-----------|
| **Architecture** | Option C: Split CLAUDE.md | Clear separation, automatic root context, app-specific guidance |
| **Root** | Platform-level only (~150 lines) | Shared patterns, monorepo, tooling |
| **book-source** | Full pedagogy (~600 lines) | 4-Layer teaching, constitution, context-gathering |
| **panaversity-fs** | Engineering-focused (~250 lines) | Python, FastAPI, testing, type hints |
| **Inheritance Model** | Root auto-loads; apps manual | Claude Code limitation, but acceptable trade-off |
| **Timeline** | Phase 1-5 over 3 weeks | Conservative, allows testing and iteration |
| **Success Metric** | 100% of CLAUDE references correct for context | Each engineer uses only relevant sections |

---

## 11. Implementation Checklist

### Pre-Refactor
- [ ] Review current CLAUDE.md one final time
- [ ] Check for section cross-references
- [ ] Create git branch: `feature/claude-md-split`

### Refactoring
- [ ] Extract root CLAUDE.md (platform)
- [ ] Create apps/learn-app/CLAUDE.md
- [ ] Create panaversity-fs/CLAUDE.md
- [ ] Update .gitignore (no per-app overrides)
- [ ] Add version headers to each file

### Documentation
- [ ] Update root README.md (navigation)
- [ ] Update apps/learn-app/README.md (CLAUDE link)
- [ ] Update panaversity-fs/README.md (CLAUDE link)
- [ ] Create CLAUDE-md-migration-summary.md

### Testing
- [ ] Book authoring session: read apps/learn-app/CLAUDE.md
- [ ] Backend session: read panaversity-fs/CLAUDE.md
- [ ] Platform session: read root CLAUDE.md
- [ ] Verify no critical sections missing

### Launch
- [ ] Create PR with all changes
- [ ] Update team: announce split strategy
- [ ] Monitor for confusion
- [ ] Iterate on feedback

---

## Appendix A: Content Distribution

### Root CLAUDE.md (Platform-Level)
- Core Identity
- Monorepo Skills (Nx)
- Agent Architecture
- Intelligence Harvesting
- Claude 4 Best Practices
- Platform Tech Stack

**Sections**: 0, V, VI, IX, XII, Platform Tech
**Lines**: ~150

### apps/learn-app/CLAUDE.md (Content Authoring)
- All current sections (I-XI)
- Pedagogical framework
- Constitutional reasoning
- 4-Layer teaching method
- Context-gathering for lessons
- Content success metrics

**Sections**: I, II, III, IV, VII (content), VIII, X, XI
**Lines**: ~600

### panaversity-fs/CLAUDE.md (Backend Engineering)
- Context-gathering (adapted)
- Domain skills (Nx + Python tools)
- Agent architecture (MCP servers)
- Self-monitoring (code checklist)
- Execution contract (adapted)
- Intelligence harvesting (adapted)
- Claude 4 best practices (Python-specific)
- FastAPI patterns (NEW)
- Testing strategy (NEW)
- Type hints enforcement (NEW)

**Sections**: Adapted I, V, VI, VII, VIII, IX, XII + NEW
**Lines**: ~250

---

**Recommendation**: Proceed with Option C implementation. Begin Phase 1 immediately to establish clear separation between content and engineering work.

