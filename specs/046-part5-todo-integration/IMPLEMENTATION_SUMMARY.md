# Part 5 Todo Integration — Implementation Summary

**Status**: Planning Complete - Ready for Phase 1 Execution
**Date**: 2025-12-26
**Plan Document**: `plan.md` (866 lines)

---

## Quick Start for Team Leads

### What We're Building
Integrate a **running example** (Todo Console App) across Part 5 (Python Fundamentals, Chapters 15-32). Students will build the app incrementally, with each Python concept contributing to a cohesive project. Auxiliary domain examples (Legal, Finance, Healthcare, Marketing) show pattern universality.

### Why This Matters
Part 6 introduces the TaskManager Agent, which depends on Part 5 establishing the Todo domain model. Without consistent Todo framing in Part 5, Part 6 loses its foundation.

### The Numbers
- **Total Effort**: 96 hours
- **Team Size**: 1-2 people recommended
- **Duration**: 10 days (2 people) or 18 days (1 person)
- **Chapters Affected**: All 18 (Ch15-32)
- **Lessons Affected**: 74 total

---

## Implementation Phases (In Order)

### Phase 1: Critical Capstones (Days 1-2)
**4 Lessons | 8 hours | Prerequisite for Phase 2**

Build the foundation. Rewrite 4 capstone lessons to frame the complete Todo app:
- Ch23-05 (Functions): TaskManager function interface
- Ch25-05 (File I/O): Task save/load JSON
- Ch27-05 (OOP): Task and TaskManager classes
- Ch32-06 (CPython/GIL): Batch task processing

**Deliverable**: All 4 capstones work together sequentially.

**Validation**: Code runs, transitions between lessons clear, no broken examples.

---

### Phase 2: Core OOP Reframes (Days 3-6)
**9 Lessons | 20 hours | Depends on Phase 1**

Reframe Ch27-28 (OOP Part 1 & 2) with Task/TaskManager as primary example.

**Changes**:
- Move generic Person/Animal examples to auxiliary (secondary)
- Add 3+ domain examples per lesson (Legal, Finance, Healthcare minimum)
- Ensure Task/TaskManager attributes consistent across all lessons

**Deliverable**: Ch27-28 fully reframed with Todo focus and auxiliary domain examples.

**Validation**: No contradictions in Task structure, OOP principles still clear.

---

### Phase 3: Variable Renaming (Days 3-5, parallel to Phase 2)
**32 Lessons | 16 hours | Can run in parallel**

Quick wins: Rename variables to Todo-aligned names and add 1-2 connecting sentences.

**Changes Per Lesson**:
- Rename: `items` → `tasks`, `data` → `todo_data`, etc.
- Add: 1-2 lines connecting to Todo context
- No pedagogical changes

**Chapters**: Ch17, 18, 19, 20, 21, 22, 24 (6 chapters, 4-6 lessons each)

**Validation**: Variable names consistent within each chapter, no broken code.

---

### Phase 4: Auxiliary Domain Examples (Days 3-7, parallel to Phase 2)
**29 Lessons | 36 hours | Longest phase | Can run in parallel**

Add domain examples showing pattern universality. **For each lesson**:
- Todo: Task list/dict operations (primary)
- Legal: Case/Contract operations
- Finance: Invoice/Transaction operations
- Healthcare: Appointment/Patient operations

**Chapters**: Ch17, 18, 19, 20, 21, 23, 25, 26, 29, 30, 31

**Validation**: Each domain example correct and runnable, doesn't distract from Python concept.

---

### Phase 5: Category D Restructuring (Days 5-7, parallel to Phase 4)
**13 Lessons | 12 hours | Can run in parallel**

Add running examples where lessons currently lack Todo framing.

**Lessons**:
- Ch16 (Intro): Hello World → "Hello, Todo!"
- Ch19 (Strings): Type casting → task data
- Ch20 (Control Flow): Break/continue → task filtering
- Ch22 (Sets): Frozenset → task categories
- Ch24 (Exceptions): Custom errors → task error handling
- Ch26 (DateTime): Calendar → task scheduling
- Ch32 (CPython/GIL): Batch processing (non-capstone lessons)

**Validation**: Examples feel natural (not forced), pedagogy unchanged.

---

### Phase 6: Connections and READMEs (Day 8, after all others)
**23 Items | 4 hours | Must be last**

Final connections. Add narrative throughline and update chapter READMEs.

**5 Connection Points**:
- Ch16 → Ch17: "Now structure tasks as data"
- Ch21 → Ch23: "Wrap operations in functions"
- Ch23 → Ch25: "Persist to disk"
- Ch25 → Ch27: "Organize with classes"
- Ch27 → Ch31: "Make async for Part 6"

**18 Chapter README Updates**:
- Add 1-2 sentences: What Todo component does this chapter build?
- Optional: Which domains demonstrated?
- Students can trace Todo evolution

**Validation**: Every README mentions running example, narrative arc clear.

---

## Critical Path and Parallelization

### Critical Path (Must be sequential)
```
Phase 1 (2 days)
    ↓
Phase 2 (4 days)
    ↓
Phase 6 (1 day)
```

### Parallel Tracks (While Phase 2 runs)
```
Phase 3 (Renaming): 2-3 days
Phase 4 (Domains): 5-7 days ← Longest
Phase 5 (Restructuring): 2-3 days
```

### Timeline with 2 People

**Week 1**:
- Days 1-2: Person A does Phase 1 (4 capstones)
- Days 3-6: Person A does Phase 2 (9 OOP lessons)
- Days 3-7: Person B does Phase 3, 4, 5 in parallel

**Week 2**:
- Day 1: Phase 6 (READMEs + connections)

**Total: 10 days (2 weeks)**

### Timeline with 1 Person
Sequential: ~18 days (Phase 1 + 2 + 3 + 4 + 5 + 6)

---

## Success Criteria Checklist

### Phase 1 Complete ✓
- [ ] Ch23-05 focuses on TaskManager functions
- [ ] Ch25-05 saves/loads tasks to JSON
- [ ] Ch27-05 has Task and TaskManager classes
- [ ] Ch32-06 demonstrates batch processing
- [ ] All code examples run without error

### Phase 2 Complete ✓
- [ ] Task/TaskManager are primary examples in Ch27-28
- [ ] Generic examples moved to auxiliary (secondary position)
- [ ] 3+ domain examples per lesson (Legal, Finance, Healthcare)
- [ ] Task attributes consistent across Ch27-28
- [ ] OOP concepts still clear (not obscured by Todo)

### Phase 3 Complete ✓
- [ ] Variable names consistent within chapters (e.g., `tasks` everywhere in Ch20)
- [ ] 1-2 connecting sentences per lesson (not disruptive)
- [ ] No broken code examples

### Phase 4 Complete ✓
- [ ] Each domain example follows Todo pattern
- [ ] Auxiliary examples don't distract
- [ ] All code examples are runnable
- [ ] Pattern consistency across domains

### Phase 5 Complete ✓
- [ ] Running examples feel natural
- [ ] No existing content removed
- [ ] Python concept explanation unchanged

### Phase 6 Complete ✓
- [ ] Every README mentions running example
- [ ] 5 connection points added between chapters
- [ ] Students can trace Todo: Ch16 → Ch17 → ... → Ch32

### Final Validation ✓
- [ ] Book builds: `pnpm nx build learn-app`
- [ ] All Python examples syntactically correct
- [ ] Students can follow Todo evolution
- [ ] Part 6 integration points clear

---

## Reference Materials

### Gold Standard
**Location**: Ch29, Lesson 04 - TodoList Dataclass
`/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/05-Python-Fundamentals/29-metaclasses-dataclasses/04-advanced-dataclass-features.md`

Use this structure as reference for all Todo examples:
```python
@dataclass
class TodoList:
    name: str
    items: list[str] = field(default_factory=list)
    tags: dict[str, str] = field(default_factory=dict)
    priority: int = 5
```

### Quality Reference Lesson
**Location**: Part 1, Ch1, Lesson 01
`apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`

Use for YAML frontmatter, narrative opening, code depth, auxiliary examples.

### Domain Entity Mappings

| Domain | Entity | Key Attributes | Operations |
|--------|--------|---|---|
| **Legal** | Case | case_id, status, filed_date, parties | open_case(), close_case() |
| **Finance** | Invoice | invoice_no, amount, paid, due_date | mark_paid(), calculate_overdue() |
| **Healthcare** | Appointment | appt_id, patient, date_time, doctor | schedule(), cancel() |
| **Marketing** | Campaign | campaign_id, name, status, budget | launch(), pause() |

---

## Risks and Mitigations

| Risk | Mitigation |
|------|-----------|
| Inconsistent Task naming | Establish naming in Phase 1; validate in Phase 2 |
| Auxiliary examples distract | Keep brief; place after primary example |
| Breaking existing content | Read full chapter before editing; test examples |
| Phase 4 overruns | Start Phase 4 early; consider 2-person team |
| READMEs feel disconnected | Use consistent framing: "...contributes to Todo app" |

---

## Effort Breakdown

### By Phase
| Phase | Lessons | Hours | Days | Team |
|-------|---------|-------|------|------|
| 1 | 4 | 8 | 1 | 1 |
| 2 | 9 | 20 | 2.5 | 1 |
| 3 | 32 | 16 | 2 | 1-2 (parallel) |
| 4 | 29 | 36 | 4.5 | 1-2 (parallel) |
| 5 | 13 | 12 | 1.5 | 1-2 (parallel) |
| 6 | 23 | 4 | 0.5 | 1 |
| **TOTAL** | **74** | **96** | **12** | **1-2** |

---

## How to Use This Plan

### For Project Manager
1. Review "Implementation Phases" section above
2. Assign Phase 1 to one developer immediately
3. While Phase 1 runs, prepare Phases 3-5 parallel team
4. Monitor critical path: Phase 1 → Phase 2 (4 days minimum)
5. Target completion: 10-18 days depending on team size

### For Developers
1. Read the full `plan.md` for your assigned phase(s)
2. Use the "Lesson-by-Lesson Reference" section (Part 4 of plan.md)
3. Check "Phase X Quality Checklist" before submitting
4. Reference gold standard examples constantly
5. Test all Python code locally

### For Code Reviewers
1. Use "Success Criteria Checklist" above to validate
2. Verify narrative arc (especially Phase 6 connections)
3. Check domain examples match patterns
4. Ensure no pedagogical changes to Python concepts
5. Validate book builds without errors

---

## Next Steps

1. **Assign team**: Recommend 2 people for 10-day timeline
2. **Start Phase 1**: Immediate (critical path)
3. **Prepare Phases 3-5**: Have developers ready for parallelization
4. **Monitor**: Daily standup on critical path (Phase 1 → Phase 2)
5. **Validate**: Run final checklist and book build before merge

---

## Questions?

Refer to the full `plan.md` document (866 lines) for:
- Detailed per-lesson changes
- Specific code examples needed
- Risk mitigation strategies
- Regression testing procedures
- Cross-chapter dependency analysis

---

**Generated by**: chapter-planner v2.0.0
**Date**: 2025-12-26
**Status**: Ready for Implementation Phase 1
