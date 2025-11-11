---
sidebar_position: 2
title: "Chapter 31: Spec-Kit Plus Hands-On"
---

# Chapter 31: Spec-Kit Plus Hands-On

Welcome to hands-on specification-driven development. This chapter transforms understanding (from Chapter 30) into practice. You won't read about specifications-you'll write them. You won't learn about AI collaboration - you'll build project with your AI companion from specifications to implementation.

---

## From Chapter 30 to Chapter 31

**Chapter 30** taught you the foundations:
- Why specification-driven development matters (the vague code problem)
- What SDD is (spec-first, spec-anchored, spec-as-source levels)
- How to write your first spec (calculator exercise)
- Why teams need shared rules (Constitutions, ADRs, PHRs)
- When SDD became essential (AI convergence moment)
- Which tools exist (Kiro, Spec-Kit, **Spec-Kit Plus**, Tessel)

**Chapter 31** teaches you the workflow:
- How to install and configure Spec-Kit Plus
- How to create project Constitutions
- How to collaborate with AI on specifications
- How to refine specs iteratively
- How to generate plans and document decisions (ADRs)
- How to decompose work with checkpoint-driven execution
- How to implement with validation protocols
- How to complete end-to-end workflows independently

**The Bridge**: Chapter 30 gave you the map. Chapter 31 teaches you to drive.

---

## What You'll Learn

By completing this chapter, you'll master:

1. **Spec-Kit Plus Architecture** - Understand Horizontal Intelligence (ADRs + PHRs) and Vertical Intelligence (orchestrator + subagents)
2. **Evals-First Collaboration** - How to explore problem space with AI BEFORE writing formal specifications
3. **Constitution Creation** - Setting project-wide quality standards that cascade to all features
4. **Specification Writing** - Creating clear, testable requirements using SMART criteria
5. **Iterative Refinement** - Using `/sp.clarify` to identify and resolve specification gaps
6. **Architecture Planning** - Generating implementation plans and documenting decisions with ADRs
7. **Task Decomposition** - Breaking plans into atomic work units with checkpoint-driven execution
8. **AI-Driven Implementation** - Orchestrating code generation with systematic validation protocols
9. **Independent Workflow** - Applying all skills end-to-end without guidance

Most importantly, you'll build the muscle memory for specification-first thinking that makes you dramatically faster in Chapters 32 and beyond. Let's build something real.

<Quiz title="Chapter 2 Quiz" questions={[{"question":"What did Sarah Chen, a solo founder, build in forty-eight hours with the help of Claude Code?","options":{"a":"A simple personal blog.","b":"A complete customer analytics dashboard for 1,200 customers.","c":"A mobile game.","d":"A tutorial on PHP."},"correct_answer":"b","explanation":"The text explicitly states, \u0027In forty-eight hours, she\u0027d built a complete customer analytics dashboard that two months ago would have required a team of five developers and three weeks of work... Her dashboard processed real-time data for 1,200 customers.\u0027"},{"question":"How does the speed of the current AI coding revolution compare to previous transitions in software development?","options":{"a":"It is happening over 10-15 years, similar to past transitions.","b":"It is happening much slower than previous transitions.","c":"It is happening in months, not years.","d":"The speed is exactly the same."},"correct_answer":"c","explanation":"The chapter notes, \u0027Previous transitions... took 10-15 years... The AI coding revolution is happening in months, not years.\u0027"},{"question":"According to the Stack Overflow 2024 Developer Survey cited in the chapter, what percentage of professional developers are already using AI coding tools?","options":{"a":"10%","b":"44%","c":"62%","d":"76%"},"correct_answer":"c","explanation":"The text provides the statistic: \u002776% of professional developers are using or plan to use AI coding tools, with 62% already using them—up from 44% last year (Stack Overflow 2024 Developer Survey).\u0027"},{"question":"What is the chapter\u0027s main answer to questions like \u0027Am I too late?\u0027 or \u0027Will this replace me?\u0027 for developers?","options":{"a":"It is too late for new developers to enter the field.","b":"AI will replace most developers within the next year.","c":"This is the best time in decades to be learning software development because of AI.","d":"Developers should switch to a different career."},"correct_answer":"c","explanation":"The text directly answers, \u0027The answer to all four is the same, and it might surprise you: This is the best time in decades to be learning software development. Not despite AI. Because of it.\u0027"},{"question":"How is the role of a developer evolving in the AI era, according to the chapter?","options":{"a":"From a high-level architect to a low-level coder.","b":"It is not evolving; the tasks remain the same.","c":"From a typist writing code line by line to an orchestrator managing AI agents.","d":"Developers are now primarily focused on hardware and infrastructure."},"correct_answer":"c","explanation":"The chapter states, \u0027Your role as a developer is evolving from typist (writing code line by line) to orchestrator (managing AI agents, making architectural decisions, exercising judgment).\u0027"}]} />

