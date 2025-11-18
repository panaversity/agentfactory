# Chapter 10: Prompt Engineering for AI-Driven Development

Clear prompts equal working code on the first try. Vague prompts equal hours spent debugging AI-generated mistakes. Studies show developers using AI agents are 55% more productive, but only when they master prompting techniques.

This chapter teaches **prompt engineering** as a **specification skill** for AI-native development—the ability to communicate intent, constraints, and success criteria to AI systems with the same precision product managers use to specify requirements. You'll learn the frameworks that Jake Heller (Casetext founder, $650M exit to Thomson Reuters) used to refine AI prompts from 60% accuracy to 97% accuracy through iterative specification writing.

By the end of this chapter, you'll write prompts that consistently produce high-quality AI outputs and create reusable prompt templates for recurring development tasks. This chapter focuses exclusively on **what you SAY** to your AI agent. Chapter 11 will teach context engineering—**what your AI agent KNOWS** when you say it.

## What You'll Learn

By the end of this chapter, you will be able to:

- **Explain prompt engineering as specification skill** — Distinguish between specification prompts (WHAT) and vague requests, analyze prompts for completeness using Intent-Constraints-Success framework
- **Write structured prompts with clear intent** — Construct prompts using Intent → Constraints → Success Criteria structure, apply 8 technical action verbs (Create, Debug, Refactor, Analyze, Optimize, Generate, Explain, Validate)
- **Apply iterative refinement to improve prompt quality** — Identify specification gaps in AI outputs, refine prompts through iteration to achieve 90%+ quality (Jake Heller's 60% → 97% framework)
- **Write specification documents before prompting** — Define measurable success criteria, specify constraints and non-goals explicitly, create validation tests for acceptance criteria
- **Use Question-Driven Development to elicit requirements** — Prompt AI to ask clarifying questions, transform AI questions and answers into structured specifications
- **Create reusable prompt templates** — Recognize recurring prompt patterns (2+ occurrences), design templates with placeholders and usage guidance, evolve templates based on usage patterns
- **Apply template selection decision frameworks** — Classify tasks by category (Diagnostic, Transformative, Generative, Explanatory, Evaluative), evaluate template applicability in &lt;30 seconds, decide between template, custom prompt, or hybrid approach
- **Write peer-reviewable specifications** — Produce specification documents implementable without clarification, define measurable success criteria and validation tests, identify and document open questions
