---
sidebar_position: 33
title: "Chapter 33: Tessl and SpecifyPlus - The Ultimate Workflow"
---

# Chapter 33: Tessl and SpecifyPlus - The Ultimate Workflow

You've mastered SpecifyPlus's opinionated workflow: Constitution → Specify → Plan → Tasks → Implement. The PHRs and ADRs are there. You've learned how specifications serve as the coordination layer for AI agents. Now you're ready for the next evolution: **integrating Tessl with SpecifyPlus** to create the most powerful specification-driven development workflow available in 2025.

This chapter shows you how to marry two complementary approaches:

**SpecifyPlus** provides the workflow structure—the human-gated, multi-phase process that ensures you build the right thing with clear organizational guardrails.

**Tessl** provides the infrastructure—versioned usage specs that prevent API hallucinations, framework-level enforcement of spec-first patterns, and long-term memory through specifications stored in your codebase.

Together, they create something neither achieves alone: a complete system for reliable, professional AI-assisted development that scales from solo projects to enterprise teams.

---

## The Marriage: How SpecifyPlus and Tessl Complement Each Other

Think of this integration like a professional kitchen:

**SpecifyPlus is your recipe and workflow**—it tells you _what_ to cook, _when_ to add each ingredient, and _how_ to verify the dish is correct. It provides structure: constitution sets the standards, specification defines the goals, plan maps the implementation, tasks break it into atomic steps.

**Tessl is your ingredient quality system**—it ensures every ingredient (library, API, framework) is exactly what you think it is. No outdated packages, no hallucinated APIs, no mixed versions. It also provides persistent memory so future chefs (or your future self) remember why certain ingredients were chosen.

## The Big Picture

Tessl represents the evolution of everything you've learned so far:

**Spec-Kit Plus** teaches you to write specifications first and gives you a structured workflow (Constitution → Specify → Plan → Tasks). It's a methodology you adopt with discipline.

**Tessl** enforces that methodology at the framework level. It won't let agents rush to code. It provides versioned library documentation so agents can't hallucinate APIs. It stores specifications in your codebase as long-term memory so agents remember decisions across sessions.

Together, they're more powerful than either alone:

- Use **SpecifyPlus** for your high-level workflow and project structure
- Use **Tessl Registry** to prevent API hallucinations for every library
- Explore **Tessl Framework** to enforce spec-first patterns and regenerate code from specs

This chapter shows you how to combine these tools to build software at a level of reliability and scale that was previously impossible for solo developers or small teams.

---

## Chapter Structure

The lessons build progressively:

**Lessons 1-3** establish the problem and Tessl's solution at a conceptual level. You'll understand _why_ Tessl exists and what makes it different from everything else.

**Lessons 4-6** are hands-on: installing Tessl, creating your first spec, and integrating it with your existing SpecifyPlus workflows.

**Lessons 7-9** go deeper: understanding the three-stage evolution of SDD, publishing custom specs for your organization, and studying real-world production systems.

**Lesson 10** concludes with the future vision: a world where you work primarily at the specification level, with code as a generated artifact you rarely need to see.

---

## Why SpecifyPlus Alone Isn't Enough

You've experienced SpecifyPlus's power in Chapters 30-32. The opinionated workflow works. But you've probably also noticed friction points:

**Scenario**: Your plan says "Use FastAPI 0.104.1 for the REST API."

**What Happens**: The agent suggests code using FastAPI 0.95.x syntax mixed with 0.104.x features because its training data contains both versions.

**Result**: 30 minutes debugging import errors and deprecated methods.

**Tessl Solution**: The Registry provides `tessl/pypi-fastapi@0.104.1` usage spec with exact, version-specific API documentation.

## The Future is Spec-Centric

By the end of 2027, Guy Podjarny predicts developers won't look at code most of the time. They'll work at the specification level, with AI agents handling implementation details.

This chapter prepares you for that future by teaching you to marry SpecifyPlus's opinionated workflow with Tessl's infrastructure. Together, they create the most powerful specification-driven development system available today.

This is how you build production-grade software in the AI era.

This is how specifications become source code.

Let's begin.

<Quiz title="Chapter 33 Quiz" questions={[{"question":"What is the primary purpose of the Model Context Protocol (MCP)?","options":{"a":"To allow Claude Code to work offline without an internet connection.","b":"To act as a bridge that lets Claude Code safely use external tools and data sources like websites and documentation.","c":"To enforce a specific coding style on the user.","d":"To manage user authentication for the Claude.ai website."},"correct_answer":"b","explanation":"The chapter defines MCP as \u0027the bridge that lets Claude Code safely use external tools and data—so it can truly collaborate with you on real tasks.\u0027"},{"question":"What is the key distinction between an MCP Server and an Agent Skill?","options":{"a":"Skills work with external systems, while MCP servers only work with local files.","b":"MCP servers provide access to external data and tools, while skills provide expertise based on local file context.","c":"There is no difference; they are the same.","d":"MCP servers are less secure than skills."},"correct_answer":"b","explanation":"The comparison table clearly shows that MCP Servers are for \u0027External data access\u0027 from \u0027External systems,\u0027 while Skills and Subagents work with \u0027Local files only.\u0027"},{"question":"In the hands-on section, what two MCP servers are added to Claude Code?","options":{"a":"A server for playing music and a server for ordering food.","b":"A server for accessing GitHub and a server for accessing a database.","c":"The Playwright MCP for browsing the web and the Context7 MCP for fetching documentation.","d":"The lesson does not involve adding any MCP servers."},"correct_answer":"c","explanation":"The \u0027Hands-On\u0027 section provides the exact commands to add the Playwright and Context7 MCP servers."},{"question":"In the \u0027Shop Together\u0027 workflow example, what does the Playwright MCP enable Claude Code to do?","options":{"a":"Purchase items directly from Amazon.","b":"Browse the Amazon website, extract details about products, and return a summary.","c":"Access your Amazon account history.","d":"Write a review for a product on Amazon."},"correct_answer":"b","explanation":"The example prompt asks Claude to \u0027browse Amazon. Find 3 men\u0027s casual shirts... Share links, prices, main features...\u0027 which is a web browsing and data extraction task."},{"question":"According to the \u0027Strategic Decision Framework,\u0027 when should you use an MCP Server?","options":{"a":"When you have a repetitive task with clear rules.","b":"When you need to access external tools, data, or websites.","c":"For any exploratory or one-off task.","d":"When you want expertise to be applied automatically."},"correct_answer":"b","explanation":"The framework starts with the question, \u0027Need external tools/data (web, docs, database)?\u0027 and directs the user to \u0027Use MCP Server\u0027 if the answer is yes."}]} />

