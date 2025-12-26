# Part 5 Todo Running Example Integration

**Project**: Issue #407 - Todo Console App incremental build across Python Fundamentals
**Branch**: `046-part5-todo-integration`
**Status**: Planning Phase Complete
**Date**: 2025-12-26

---

## Deliverables

This specification folder contains the complete planning documents for integrating a running Todo example across Part 5 (Python Fundamentals, Chapters 15-32).

### 1. **spec.md** (13 KB)
- Feature specification document
- User scenarios and acceptance criteria
- Success evaluation metrics
- Related issues and audit references

### 2. **plan.md** (34 KB, 866 lines)
- **The primary planning document** — comprehensive implementation roadmap
- 6 distinct implementation phases with clear dependencies
- Per-phase objectives, effort estimates, and success criteria
- Lesson-by-lesson reference tables
- Risk mitigation strategies
- Gold standard references and domain mappings
- Detailed parallelization strategy

**Key Sections**:
- Part 1: Technical Context
- Part 2: Phase Breakdown (6 phases, 74 lessons, 96 hours)
- Part 3: Implementation Workflow and Critical Path
- Part 4: Lesson-by-Lesson Reference
- Part 5: Risk Mitigation and Validation
- Part 6: Effort Estimation
- Part 7: Success Criteria
- Appendix: Reference Materials

### 3. **IMPLEMENTATION_SUMMARY.md** (10 KB)
- Executive summary for project leads and team members
- Quick-start guide
- Phase timeline and parallelization strategy
- Success criteria checklist
- Risk and mitigation table
- Effort breakdown by phase
- How to use the plan (for PM, developers, reviewers)

---

## Quick Reference

### The 6 Phases

| Phase | Name | Lessons | Hours | Duration | Dependencies |
|-------|------|---------|-------|----------|---|
| 1 | Critical Capstones | 4 | 8 | 1-2 days | None |
| 2 | Core OOP Reframes | 9 | 20 | 3-4 days | Needs Phase 1 |
| 3 | Quick Wins (Renaming) | 32 | 16 | 2-3 days | Can parallel |
| 4 | Auxiliary Domain Examples | 29 | 36 | 5-7 days | Can parallel |
| 5 | Category D Restructuring | 13 | 12 | 2-3 days | Can parallel |
| 6 | Connections + READMEs | 23 | 4 | 1 day | Must be last |
| **TOTAL** | **All Phases** | **74** | **96** | **2-3 weeks** | **1-2 person team** |

### Critical Path
```
Phase 1 (Capstones)
  ↓ [1-2 days]
Phase 2 (OOP Reframes)
  ↓ [3-4 days]
Phase 6 (Connections)
```

### Parallelization Opportunity
While Phase 2 runs, execute in parallel:
- Phase 3 (Variable renaming)
- Phase 4 (Auxiliary examples) ← Longest
- Phase 5 (Restructuring)

**Result**: 2-person team can complete in ~10 days vs. 18 days for 1 person.

---

## Phase Details

### Phase 1: Critical Capstones (4 lessons)
Rewrite 4 capstone lessons to focus on building the Todo app:
- **Ch23-05**: Functions → TaskManager function interface
- **Ch25-05**: File I/O → Task JSON persistence
- **Ch27-05**: OOP → Task and TaskManager classes
- **Ch32-06**: CPython/GIL → Batch task processing

**Prerequisite**: None (start immediately)
**Deliverable**: 4 working capstones that sequence together

### Phase 2: Core OOP Reframes (9 lessons)
Replace generic examples (Person, Animal, BankAccount) with Task/TaskManager:
- **Ch27-01 to 27-04**: Classes, attributes, constructors, inheritance
- **Ch28-01 to 28-04**: Encapsulation, class methods, polymorphism, abstract patterns

Each lesson also gets 3+ auxiliary domain examples (Legal, Finance, Healthcare).

**Prerequisite**: Phase 1 (establishes Todo foundation)
**Deliverable**: Ch27-28 fully Todo-focused with consistent Task structure

### Phase 3: Variable Renaming (32 lessons)
Minimal changes to 32 lessons across 7 chapters:
- Rename variables to task-aligned names: `items` → `tasks`
- Add 1-2 sentence connections to Todo context
- No pedagogical changes

**Chapters**: Ch17, 18, 19, 20, 21, 22, 24
**Can Run in Parallel**: Yes (while Phase 2 executes)
**Deliverable**: Consistent variable naming across chapters

### Phase 4: Auxiliary Domain Examples (29 lessons)
Add 2-3 domain examples to lessons with Todo but lacking auxiliaries:
- Legal domain (Case/Contract)
- Finance domain (Invoice/Transaction)
- Healthcare domain (Appointment/Patient)
- Marketing domain (Campaign/Lead)

**Chapters**: Ch17, 18, 19, 20, 21, 23, 25, 26, 29, 30, 31
**Can Run in Parallel**: Yes (independent work)
**Duration**: 5-7 days (longest phase)
**Deliverable**: Domain examples demonstrating pattern universality

### Phase 5: Category D Restructuring (13 lessons)
Add running examples to lessons currently lacking Todo framing:
- Ch16: Introduction → Hello Todo
- Ch19: Type casting → Task data types
- Ch20: Break/Continue → Task filtering
- Ch22: Frozenset → Immutable task categories
- Ch24: Custom exceptions → Task error handling
- Ch26: DateTime → Task scheduling
- Ch32: Batch processing → Task batching

**Can Run in Parallel**: Yes (independent work)
**Deliverable**: All lessons incorporate Todo context naturally

### Phase 6: Connections and READMEs (23 items)
Add narrative throughline and update chapter-level documentation:
- **5 inter-chapter connections**: Explicit transitions between chapters
- **18 README updates**: Each chapter explicitly mentions its Todo contribution

**Must be Last**: Yes (after all content changes)
**Deliverable**: Students can trace Todo evolution from Ch16 through Ch32

---

## Gold Standard Reference

**Location**: Chapter 29, Lesson 04 - Advanced Dataclass Features
`apps/learn-app/docs/05-Python-Fundamentals/29-metaclasses-dataclasses/04-advanced-dataclass-features.md`

**Use this TodoList dataclass as reference for all lessons**:
```python
from dataclasses import dataclass, field

@dataclass
class TodoList:
    name: str
    items: list[str] = field(default_factory=list)
    tags: dict[str, str] = field(default_factory=dict)
    priority: int = 5
```

---

## Domain Entity Mappings (for Phases 2 & 4)

### Legal Domain
- **Entity**: Case (or LegalCase)
- **Attributes**: case_id, status, filed_date, parties
- **Operations**: open_case(), close_case(), get_case_age()

### Finance Domain
- **Entity**: Invoice (or Transaction)
- **Attributes**: invoice_no, amount, paid, due_date, vendor
- **Operations**: mark_paid(), send_reminder(), calculate_overdue()

### Healthcare Domain
- **Entity**: Appointment (or PatientRecord)
- **Attributes**: appointment_id, patient_name, date_time, doctor, status
- **Operations**: schedule(), cancel(), get_reminders()

### Marketing Domain
- **Entity**: Campaign (or Lead)
- **Attributes**: campaign_id, name, status, budget
- **Operations**: launch(), pause(), calculate_roi()

---

## Success Criteria

### Phase 1 ✓
- All 4 capstones have Todo framing
- Code examples are syntactically correct
- Clear transition statements between capstones

### Phase 2 ✓
- Task/TaskManager are primary examples
- Generic examples in secondary (auxiliary) position
- 3+ domain examples per lesson
- No contradictions in Task attributes across Ch27-28

### Phase 3 ✓
- Variable names consistent within chapters
- 1-2 line connecting sentences
- No pedagogical changes

### Phase 4 ✓
- Domain examples follow Todo pattern
- Auxiliary examples don't distract
- All code is runnable

### Phase 5 ✓
- Running examples feel natural (not forced)
- No existing content removed
- Python concepts remain primary focus

### Phase 6 ✓
- Every README mentions running example
- 5 connection points between chapters
- Students can trace Todo evolution

### Final ✓
- Book builds without errors: `pnpm nx build learn-app`
- All Python examples syntactically correct
- Students can follow Todo progression Ch16→Ch32
- Part 6 integration points clear

---

## How to Use This Specification

### For Project Managers
1. Read **IMPLEMENTATION_SUMMARY.md** for executive overview
2. Use Phase timeline to plan team assignments
3. Monitor critical path: Phase 1 → Phase 2 → Phase 6
4. Track effort and parallelization opportunities

### For Developers
1. Read **plan.md** section "Part 2: Phase Breakdown" for your assigned phase
2. Use "Part 4: Lesson-by-Lesson Reference" for specific lessons
3. Refer to "Gold Standard Reference" constantly
4. Check "Per-Phase Quality Checklist" before submitting
5. Test all Python examples locally

### For Code Reviewers
1. Use Success Criteria checklist in IMPLEMENTATION_SUMMARY.md
2. Reference gold standard examples for consistency
3. Verify narrative arc (especially connections in Phase 6)
4. Check that Python pedagogy is unchanged
5. Validate book builds: `pnpm nx build learn-app`

---

## Key Metrics

- **Total Lessons**: 74 (across 18 chapters)
- **Total Effort**: 96 hours
- **Recommended Team**: 1-2 people
- **Timeline**:
  - 1 person: 18 days (3.6 weeks) sequential
  - 2 people: 10 days (2 weeks) with parallelization
  - 3+ people: 8-10 days (subject to coordination overhead)
- **Critical Path**: 8 days minimum (Phase 1 + 2 + 6)

---

## Documentation Structure

```
specs/046-part5-todo-integration/
├── README.md (this file)
├── spec.md (feature specification)
├── plan.md (complete implementation plan — 866 lines)
├── IMPLEMENTATION_SUMMARY.md (executive summary — 317 lines)
└── checklists/ (validation templates)
```

---

## Related Resources

- **Parent Issue**: #406 (Book Framing: One Running Example + Auxiliary Examples)
- **Part 5 Spec Issue**: #407 (Todo Console App incremental build)
- **Audit Guidelines**: #411 (Per-chapter audit template)
- **Audit Results**: `specs/part-5-audit/instructions.md`

---

## Version History

- **2025-12-26**: Planning phase complete
  - plan.md: Full 6-phase implementation roadmap (866 lines)
  - IMPLEMENTATION_SUMMARY.md: Executive summary (317 lines)
  - spec.md: Feature specification (already present)
  - README.md: This guide

---

## Next Steps

1. **Assign Phase 1 developer** (immediate)
2. **Prepare parallel team** for Phases 3-5 (while Phase 2 runs)
3. **Kick off Phase 1**: Capstone rewrites (4 lessons)
4. **Monitor critical path**: Phase 1 → Phase 2 (8 days minimum)
5. **Begin Phase 6** after all content phases complete (final day)
6. **Validate**: Run success criteria checklist and book build

---

**Generated by**: chapter-planner v2.0.0
**Architecture**: Layer 4 Orchestration Specialist (Pedagogical Architect)
**Status**: Ready for Implementation
**Contact**: See plan.md for detailed technical questions
