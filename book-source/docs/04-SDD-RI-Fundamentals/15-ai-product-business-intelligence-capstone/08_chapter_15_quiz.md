---
sidebar_position: 8
title: "Chapter 15 Quiz"
---

# Chapter 15 Quiz

Test your understanding of SDD-RI workflow, intelligence accumulation, and skill creation through Spec-Driven Development with Reusable Intelligence.

---

## Questions

### Question 1

In the SDD-RI workflow, what is the correct sequence for developing a feature?

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

Why does intelligence accumulation cause Feature 4 to develop faster than Feature 1, even though both implement new functionality?

A) Feature 4 requires less complex code
B) The team has more experience with the technology stack by Feature 4
C) Reusable patterns, specifications, and skills created in Features 1-3 directly apply to Feature 4, reducing rework and cognitive load
D) Feature 4 uses simpler business logic

---

### Question 8

True or False: Skills created using P+Q+P should be specific to a single project and should not be reused in other projects.

---

### Question 9

In the AI Sales Assistant project, what is the primary purpose of the Campaign Dashboard?

A) To execute outreach campaigns automatically
B) To aggregate and display outputs from all upstream features (Lead Profiler, ICP Scorer, Email Composer) in a unified interface
C) To score leads based on ICP criteria
D) To identify and generate new leads automatically

---

### Question 10

What makes a pattern worth formalizing into a reusable skill?

A) It was difficult to implement initially
B) It appears in two or more features and involves multiple decision points or variations
C) It uses advanced AI techniques
D) It was suggested by an AI assistant

---

### Question 11

What should an honest retrospective document capture about a feature development workflow?

A) Only positive outcomes and successful implementations
B) Which team members performed well or poorly
C) Specific skills that accelerated development, friction points encountered, and learnings transferable to future features
D) Plans and timelines for the next project only

---

### Question 12 (Short Answer)

Explain in 2-3 sentences why the SDD-RI workflow (Specification → Plan → Tasks → Implement) prioritizes writing specifications before implementation, and how this differs from a code-first approach.

---

## Answers

<details>
<summary>Click to reveal answers</summary>

1. **B** - Specify → Plan → Tasks → Implement. The SDD-RI workflow prioritizes intent (specification) before implementation, ensuring clarity of goals, constraints, and success criteria before any code is written.

2. **B** - Feature 4 should take approximately 50% of the time required for Feature 1. This 50% target demonstrates that accumulated skills, reusable patterns, and intelligence compound—each subsequent feature development accelerates due to prior work.

3. **True** - A constitution establishes shared standards (output formats, quality constraints, decision principles, non-goals) that guide consistent implementation across all features in a project.

4. **B** - It demonstrates the pipeline architecture principle—each feature's output feeds the next as input. This shows data flow and composition between features, not just isolated implementations.

5. **C** - Persona, Questions, Principles. Persona defines who will use the skill, Questions guide decision-making when applying the skill, and Principles constrain how the skill should be applied.

6. **C** - "Profile extraction identifies company name and extracts at least 2 technology indicators with 95%+ accuracy." Success criteria must be measurable, testable, and observable—not subjective statements like "work well" or "be user-friendly."

7. **C** - Reusable patterns, specifications, and skills created in Features 1-3 directly apply to Feature 4, reducing rework and cognitive load. Instead of solving similar problems repeatedly from scratch, developers reuse proven approaches and specifications.

8. **False** - Skills should be designed for reuse across projects. The value of formalizing a pattern using P+Q+P is precisely to make it applicable beyond the immediate context and transferable to future projects.

9. **B** - To aggregate and display outputs from all upstream features (Lead Profiler, ICP Scorer, Email Composer) in a unified interface. The dashboard serves as the final integration point showing the complete workflow output.

10. **B** - It appears in two or more features and involves multiple decision points or variations. Patterns that recur and require judgment calls are worth capturing in a reusable skill; isolated, one-time patterns are not.

11. **C** - Specific skills that accelerated development, friction points encountered, and learnings transferable to future features. Honest retrospectives balance celebrating successes with capturing honest feedback about what worked, what didn't, and what you learned.

12. **Sample answer**: Specifications define intent and success criteria before implementation begins, ensuring everyone understands the "what" and "why" before deciding "how." This differs from code-first approaches, which often jump to implementation without clarity, leading to rework when requirements or constraints change. SDD-RI treats specifications as the primary design artifact, not an afterthought.

</details>
