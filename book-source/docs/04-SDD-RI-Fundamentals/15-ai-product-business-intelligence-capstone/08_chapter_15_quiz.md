---
sidebar_position: 8
title: "Chapter 15 Quiz"
---

# Chapter 15 Quiz

Test your understanding of SDD-RI workflow, intelligence accumulation, and skill creation through Spec-Driven Development with Reusable Intelligence.

---

## Questions

### Question 1

What is the correct sequence in the SDD-RI workflow for developing a feature?

A) Implement → Specify → Plan → Tasks
B) Specify → Plan → Tasks → Implement
C) Plan → Specify → Tasks → Implement
D) Tasks → Specify → Plan → Implement

---

### Question 2

What does the "50% target" in the AI Sales Assistant capstone represent?

A) Half of all features should use AI assistance
B) Feature 4 should take approximately 50% of the time required for Feature 1
C) The acceleration should reduce bugs by half
D) At least 50% of code should be reusable

---

### Question 3

True or False: A constitution defines project-wide quality standards and decision constraints that apply consistently across all features.

---

### Question 4

Why does the ICP Scorer feature receive structured input from the Lead Profiler rather than directly fetching data from a URL?

A) It reduces API calls to external services
B) It demonstrates the pipeline architecture principle—each feature's output feeds the next as input
C) It makes the ICP Scorer feature execute faster
D) It simplifies the database schema

---

### Question 5

In the P+Q+P framework for creating reusable skills, what do these three components represent?

A) Process, Questions, Procedures
B) Plan, Queries, Practices
C) Persona, Questions, Principles
D) Purpose, Quality, Patterns

---

### Question 6

Which of the following is a measurable, testable Success Criterion for a feature specification?

A) "The feature should work well and be user-friendly"
B) "Users will be satisfied with the output quality"
C) "Profile extraction identifies company name and extracts at least 2 technology indicators with 95%+ accuracy"
D) "Implementation will take less than a week"

---

### Question 7

Why is Feature 4 able to develop faster than Feature 1, even though both implement new functionality?

A) Feature 4 requires simpler code architecture
B) The team has more experience with the technology stack by Feature 4
C) Reusable patterns, specifications, and skills created in Features 1-3 directly apply to Feature 4, reducing rework and cognitive load
D) Feature 4 uses simpler business logic and fewer edge cases

---

### Question 8

True or False: Skills created using P+Q+P should be specific to a single project and should not be reused in other projects.

---

### Question 9

In the AI Sales Assistant project, what is the primary purpose of the Campaign Dashboard?

A) To execute outreach campaigns automatically
B) To aggregate and display outputs from all upstream features (Lead Profiler, ICP Scorer, Outreach Generator) in a unified interface
C) To score leads based on ICP criteria
D) To identify and generate new leads automatically

---

### Question 10

A pattern becomes worth formalizing into a reusable skill when it:

A) Uses advanced AI techniques or sophisticated algorithms
B) Appears in 2+ features AND involves multiple decision points or variations in application
C) Was difficult to implement initially in any single feature
D) Is suggested by an AI assistant to generalize

---

### Question 11

Which of the following represents a constitution principle being applied effectively across the AI Sales Assistant features?

A) Each feature interprets error handling differently based on its unique context
B) Feature 1 outputs plain text, Feature 2 outputs JSON, Feature 3 outputs CSV—maximum flexibility
C) All features output structured JSON, ensuring that upstream outputs feed downstream inputs consistently
D) Each feature includes its own documentation format to suit its specific purpose

---

### Question 12

What should an honest retrospective document capture about feature development?

A) Only successful implementations and positive team performance metrics
B) Which team members performed well and which need improvement
C) Specific skills that accelerated development, friction points encountered, and learnings transferable to future features
D) Timelines and resource plans for the next project

---

### Question 13 (Short Answer)

Explain in 2-3 sentences why the SDD-RI workflow (Specification → Plan → Tasks → Implement) prioritizes writing specifications before implementation, and how this differs from a code-first approach.

---

### Question 14 (Short Answer)

You've built Features 1-3 and identified a recurring pattern: "structured input validation for JSON data transformation." This pattern appears with 7+ decision points (input schema, output schema, error cases, field mapping, type coercion, null handling, custom transformations). Should you formalize this as a reusable skill? Explain your reasoning using the 2+ features + 5+ decision points criteria.

---

## Answers

<details>
<summary>Click to reveal answers</summary>

**1. B** - Specify → Plan → Tasks → Implement. The SDD-RI workflow prioritizes intent (specification) before implementation, ensuring clarity of goals, constraints, and success criteria before any code is written.

**2. B** - Feature 4 should take approximately 50% of the time required for Feature 1. This 50% target demonstrates that accumulated skills, reusable patterns, and intelligence compound—each subsequent feature development accelerates due to prior work.

**3. True** - A constitution establishes shared standards (output formats, quality constraints, decision principles, non-goals) that guide consistent implementation across all features in a project. It prevents quality drift and ensures coherence.

**4. B** - It demonstrates the pipeline architecture principle—each feature's output feeds the next as input. This shows data flow and composition between features, not just isolated implementations. It also enables testing each feature independently while maintaining integration contracts.

**5. C** - Persona, Questions, Principles.
- **Persona** defines who will use the skill and what values they bring
- **Questions** guide critical decision-making when applying the skill
- **Principles** constrain how the skill should be applied and why those constraints matter

**6. C** - "Profile extraction identifies company name and extracts at least 2 technology indicators with 95%+ accuracy." Success criteria must be measurable, testable, and observable—not subjective statements like "work well" or "be user-friendly."

**7. C** - Reusable patterns, specifications, and skills created in Features 1-3 directly apply to Feature 4, reducing rework and cognitive load. Intelligence accumulation means you're not re-solving similar problems from scratch; instead, you inherit proven approaches and can focus on unique aspects of Feature 4.

**8. False** - Skills should be designed for reuse across multiple projects. The entire value of formalizing a pattern using P+Q+P is to make it applicable beyond the immediate context and transferable to future projects. This is why the pattern must recur in 2+ contexts and have 5+ decision points—criteria that ensure general applicability.

**9. B** - To aggregate and display outputs from all upstream features (Lead Profiler, ICP Scorer, Outreach Generator) in a unified interface. The dashboard serves as the final integration point that makes the entire sales intelligence pipeline visible to end users.

**10. B** - It appears in 2+ features AND involves multiple decision points or variations in application. Patterns that recur and require judgment calls are worth capturing in a reusable skill; isolated, one-time patterns are not. The 5+ decision points criterion ensures the skill captures meaningful complexity worth reusing.

**11. C** - All features output structured JSON, ensuring that upstream outputs feed downstream inputs consistently. This demonstrates the constitution principle of "Structured over Freeform" being applied uniformly. It enables composition: Lead Profiler output becomes ICP Scorer input, which becomes Outreach Generator input, which becomes Dashboard input.

**12. C** - Specific skills that accelerated development, friction points encountered, and learnings transferable to future features. Honest retrospectives balance celebrating successes with capturing honest feedback about what worked, what didn't, and what you learned. This honesty is how intelligence accumulates—you capture not just wins but friction points that future projects should anticipate.

**13. Sample Answer:** Specifications define intent and success criteria before implementation begins, ensuring everyone understands the "what" and "why" before deciding "how." This differs from code-first approaches, which often jump to implementation without clarity, leading to rework when requirements or constraints change. SDD-RI treats specifications as the primary design artifact—the executable contract—not an afterthought written after code is done.

**14. Sample Answer:** Yes, formalize this as a reusable skill. The pattern appears in Features 1-3 (3 features is >2+), and the 7+ decision points exceed the 5+ decision points threshold. With 7 decision points (schema, output, errors, mapping, coercion, nulls, transformations), this pattern has sufficient complexity and variations to justify encapsulation. Future projects will encounter JSON transformation challenges; this skill will save time and ensure consistent handling of tricky edge cases like type coercion and null handling.

</details>

---

## Scoring Guide

**Total Points:** 14 (12 MC @ 1 point each, 2 SA @ 1 point each)

**Passing Score:** 10/14 (71%)

### Grade Breakdown

- **13-14 points (93-100%):** Mastery — Deep understanding of SDD-RI, intelligence accumulation, and skill formalization
- **11-12 points (79-86%):** Proficient — Solid understanding of core concepts with minor gaps
- **10-11 points (71-79%):** Developing — Meets baseline, some concepts need reinforcement
- **Below 10 (Below 71%):** Needs Support — Review Chapter 13-14 before proceeding

### Key Concept Mapping

| Concept | Questions | Weight |
|---------|-----------|--------|
| SDD-RI Workflow | Q1, Q13 | 14% |
| Intelligence Accumulation | Q2, Q7, Q14 | 21% |
| Constitution Application | Q3, Q11 | 14% |
| Pipeline Architecture | Q4, Q9 | 14% |
| P+Q+P Framework | Q5, Q10 | 14% |
| Specification Quality | Q6, Q13 | 14% |
| Retrospective Honesty | Q12 | 7% |

### Individual Question Difficulty

- **Easy (80%+ expected pass):** Q1, Q2, Q3, Q5, Q9
- **Medium (60-80%):** Q4, Q6, Q7, Q8, Q10, Q11, Q12
- **Hard (Below 60%):** Q13, Q14 (short answers require synthesis)
