---
sidebar_position: 14
title: "Chapter 14: Spec-Kit Plus Hands-On"
slides:
  source: "slides/chapter-14-slides.pdf"
  title: "Chapter 14: Spec-Kit Plus Hands-On"
  height: 700
---

# Chapter 14: Spec-Kit Plus Hands-On — Building AI-Generated Videos

Welcome to hands-on specification-driven development. This chapter transforms understanding (from Chapter 13) into practice. You won't read about specifications—you'll write them. You won't learn about AI collaboration—you'll build a **real product**: AI-generated product demo videos uploaded to YouTube.

**Your Project**: Create product demo videos using Gemini.google.com + Playwright MCP, then upload them to YouTube—all orchestrated through Spec-Kit Plus workflow.

**Why Product Demo Videos?** SaaS companies spend thousands on product demo videos for marketing. You'll build a system that generates them with AI—a valuable, portfolio-worthy skill that demonstrates SDD-RI mastery.

## Prerequisites

Before starting this chapter, ensure you have:

- **Chapter 13 Complete**: You understand SDD-RI theory (specification primacy, intelligence accumulation)
- **Python 3.12+ Installed**: Spec-Kit Plus requires Python. If not installed, complete [Installing Python](https://ai-native.panaversity.org/docs/Python-Fundamentals/introduction-to-python/installing-python) first
- **AI Companion Ready**: Claude Code or Gemini CLI installed and configured
- **Google Account**: For Gemini.google.com and YouTube access (free tier sufficient)
- **Node.js Installed**: Required for Playwright MCP browser automation

## What You'll Learn

1. **Spec-Kit Plus Foundation** — Understand WHAT Spec-Kit Plus is, WHY this book uses it, and HOW it implements SDD-RI concepts (Horizontal + Vertical Intelligence)
2. **Installation & Configuration** — Install Spec-Kit Plus + Playwright MCP, configure your AI tool, set up browser automation with session persistence
3. **Constitution Creation** — Setting project-wide quality standards for video generation (Gemini constraints, output quality, testing requirements)
4. **Specification Writing** — Creating clear, testable requirements for product demo videos using SMART criteria
5. **Iterative Refinement** — Using `/sp.clarify` to identify video-specific edge cases (Gemini availability, session timeout, quality validation)
6. **Architecture Planning** — Generating implementation plans using Playwright MCP + Gemini.google.com approach
7. **Task Decomposition** — Breaking video generation into atomic work units with checkpoint-driven execution
8. **AI-Driven Implementation** — Executing `/sp.implement` to generate and download your video
9. **Reusable Intelligence Design** — Creating `generate-video` and `upload-youtube` skills using P+Q+P framework
10. **Brownfield Adoption** — Adding Spec-Kit Plus to existing projects safely
11. **Capstone Execution** — Applying skills to YouTube upload, demonstrating intelligence acceleration

Let's build something real—and create intelligence that compounds.