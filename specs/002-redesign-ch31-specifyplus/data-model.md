# Data Model – Chapter 31 Redesign

## Calculator (example)
- Operation: enum { add, subtract, multiply, divide [NEEDS CLARIFICATION: unary?] }
- Operands: number[] (length 2 for binary ops)
- Result: number
- Error: { code, message }

## Grading System (example)
- Student: { id, name }
- Assignment: { id, title, dueDate }
- Submission: { id, studentId, assignmentId, contentRef }
- Rubric: { id, name, criteria: Criterion[] }
- Criterion: { id, name, weight, description }
- Grade: { id, submissionId, rubricId, totalScore (0-100), perCriterion: { criterionId, score }[] }
- Feedback: { id, submissionId, text }
- GradeHistory: { id, gradeId, changedAt, previousScore, newScore, reason }

Validation rules
- Rubric must have 3–5 criteria.
- Score must be 0–100.
- Files within stated size limits (per spec).

