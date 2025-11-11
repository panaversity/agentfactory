---
sidebar_position: 6
title: "Understanding Dependencies and Packages"
chapter: 7
lesson: 6
duration_minutes: 40

skills:
  - name: "Package Manager Conceptual Understanding"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information"
    measurable_at_this_level: "Learner understands why package managers exist; recognizes different managers for different languages"

  - name: "Package Installation Dialogue"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information"
    measurable_at_this_level: "Learner can request package installation; understand what gets installed; verify success"

  - name: "Dependency Understanding"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information"
    measurable_at_this_level: "Learner understands that packages have dependencies (other packages they need); recognizes transitive dependencies"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (package managers, installation, dependencies, verification) within A2 limit ✓"
---

# Understanding Dependencies and Packages

## Package Managers Are Like Supplier Networks

Imagine building a bookshelf. You could:
- **Make every piece of wood yourself** (inefficient)
- **Order shelves from a supplier who sources from manufacturers** (efficient)

Package managers work the same way. They're suppliers of software tools. Instead of building everything from scratch, you request what you need, and the package manager finds it, along with all its dependencies (the other things it needs to work).

By the end of this lesson, you'll:
1. **Understand what package managers do** (pip for Python, npm for Node.js, brew for macOS, apt for Linux)
2. **Request package installation** in plain language and understand what happens
3. **Verify packages installed correctly** and recognize dependencies

---

## Use `pip` to Install Python Packages

Here's a conversation about installing a Python package.

**You**: "I want to use the `requests` library to make HTTP calls. How do I get it?"

**Agent**: "I'll install the `requests` package using pip (Python's package manager). When you install a package, pip automatically finds and installs any other packages it needs (its dependencies)."

```
$ pip install requests
Collecting requests
  Downloading requests-2.31.0-py3-none-any.whl (62 kB)
Collecting charset-normalizer<4,>=2 (from requests)
Collecting idna<4,>=2.5 (from requests)
Collecting urllib3<3,>=1.21.1 (from requests)
Collecting certifi>=2017.4.17 (from requests)
  ...
Installing collected packages: certifi, charset-normalizer, idna, urllib3, requests
Successfully installed certifi-2023.7.22 charset-normalizer-2.1.0 idna-3.4 urllib3-1.26.12 requests-2.31.0
```

**Agent**: "Notice what happened:
1. You requested `requests`
2. pip found it and checked its dependencies
3. pip automatically installed 4 other packages that `requests` needs: certifi, charset-normalizer, idna, urllib3
4. All packages are now ready to use in your code"

**You**: "So I asked for one package, but five got installed?"

**Agent**: "Exactly. The `requests` library depends on those other four packages. pip installed them automatically. This is called dependency resolution. It saves you from manually tracking down all the pieces."

---

## Understanding Package Installation

That dialogue shows:

**What package managers do**:
1. Find the package you requested
2. Check what other packages it needs (dependencies)
3. Install the main package and all dependencies
4. Make everything available to your code

**Why this matters**:
- You don't have to manually find and install 5 packages; you ask for 1
- Package managers handle version compatibility
- Packages always get their required dependencies

---

## Use `npm` to Install Node.js Packages

Let's see how it works for JavaScript/Node.js projects.

**You**: "I'm building a web app with Node.js. I want to use the Express framework. How do I add it?"

**Agent**: "I'll install Express using npm (Node's package manager). It works similarly to pip."

```
$ npm install express
npm notice created a lockfile as package-lock.json
npm WARN deprecated body-parser@1.20.0 some new releases are available
added 50 packages, and audited 51 packages in 2s
```

**Agent**: "npm installed Express and 49 other packages that Express depends on. It also created a `package-lock.json` file that ensures the exact same versions install on other machines."

**You**: "Why 50 packages for one framework?"

**Agent**: "Express needs other packages for routing, middleware, parsing requests, etc. Each of those might need other packages. npm handles all that complexity. You just ask for Express."

---

## Understanding Package Managers for Different Languages

Different programming languages have different package managers.

**You**: "What package managers exist? When do I use each?"

**Agent**: "Here's the main ones:

**Python (pip)**:
```
$ pip install numpy pandas requests
```
- Downloads packages from PyPI (Python Package Index)
- Standard for Python projects
- Use when: Working with Python code

**Node.js (npm)**:
```
$ npm install express react lodash
```
- Downloads from npm registry
- Standard for JavaScript/Node projects
- Use when: Working with JavaScript/Node code

**macOS (brew)**:
```
$ brew install git python node
```
- Downloads from Homebrew
- Installs command-line tools and applications
- Use when: Installing system tools on macOS

**Linux (apt)**:
```
$ apt-get install git python3 nodejs
```
- Downloads from Ubuntu/Debian repositories
- Installs system packages
- Use when: Installing system tools on Linux

You don't choose—each language has its standard manager. Python projects use pip. JavaScript uses npm. Your AI will use the right one automatically."

**You**: "So I never memorize which to use?"

**Agent**: "Correct. When you work on a Python project, you use pip. JavaScript project, use npm. Your AI knows the context and chooses the right manager. Your job is understanding that packages and their dependencies are being installed."

---

## Verify Packages Are Installed Correctly

After installing a package, you should verify it works.

**You**: "I installed requests. How do I verify it actually worked?"

**Agent**: "Let me show you verification approaches:

**Method 1: Check with pip**:
```
$ pip show requests
Name: requests
Version: 2.31.0
Summary: Python HTTP for Humans.
Location: /usr/local/lib/python3.11/site-packages
Requires: charset-normalizer, idna, urllib3, certifi
```

This shows requests is installed, its version, location, and dependencies.

**Method 2: Try importing it in Python**:
```
$ python3
>>> import requests
>>> print(requests.__version__)
2.31.0
```

If it imports without error, it's installed.

**Method 3: List all installed packages**:
```
$ pip list | grep requests
requests    2.31.0
```

All three confirm the package is available."

---

## Understanding Dependencies More Deeply

When you install a package, you're creating a dependency chain. Let's trace it:

**You ask for**: `requests`
**requests needs**: `urllib3`, `certifi`, `charset-normalizer`, `idna`
**urllib3 needs**: (its own dependencies)
**And so on...**

The package manager traces this entire tree and installs everything needed. This is called **transitive dependencies** (dependencies of dependencies).

**Why this matters**:
- If a package is missing, nothing works
- If versions conflict, installation can fail
- Your AI understands these chains and resolves conflicts

---

## Try With AI: Side-by-Side Package Installation

Now that you understand package managers, compare what happens when your AI installs a package.

### Comparison Prompt

Open your AI tool and ask:

**Prompt:**
```
Install a package for me (choose: requests, numpy, express, or lodash).
Show me:
1. The installation command
2. The installation output (what gets downloaded and installed)
3. What dependencies were installed and why
4. How to verify it worked in my code
```

**What to Compare**:

| Package Manager | You Would Do | Your AI Does This |
|---|---|---|
| Command | `pip install requests` | (AI's install command) |
| See output | (Installation progress) | (Installation progress) |
| Verify | `pip show requests` | (AI verifies it) |

**Observation**:
- Are the commands the same pattern? (Yes—`pip install`, `npm install`, etc.)
- Does your AI explain what's being installed? (Yes—shows dependencies)
- Can you verify it worked? (Yes—with `pip show`, `npm list`, or import test)

**Key Insight**: Package managers automate dependency resolution. You don't have to find 5+ packages manually—you ask for one, and the manager finds the rest.

---

## Try With AI: Understanding Dependencies

Ask your AI:

**Prompt:**
```
I just installed [package-name] and 50+ other packages appeared.
Explain:
1. Why so many packages when I asked for just one?
2. What are "transitive dependencies"?
3. What happens if one of these dependencies is missing?
4. How would I verify all these dependencies are correctly installed?
```

**Expected Response**:
Your AI will explain that packages depend on other packages, which depend on others. The package manager traces this entire dependency tree and installs everything needed. This is called transitive dependency resolution.

**Key Principle**: Package managers handle complexity you'd never want to manage manually.

---


<Quiz title="Chapter 6 Quiz" questions={[{"question":"The chapter describes the developer\u0027s role as shifting from a \u0027typist\u0027 to what?","options":{"a":"A manager","b":"A data scientist","c":"An orchestrator","d":"A marketer"},"correct_answer":"c","explanation":"The chapter\u0027s main theme is this transition: \u0027Your role shifted from typing code to directing an AI agent. You\u0027re orchestrating, not transcribing.\u0027"},{"question":"Which of the following is NOT one of the four core dimensions of the modern \u0027developer-as-orchestrator\u0027 role?","options":{"a":"Specification \u0026 Intent Communication","b":"Memorizing complex syntax and APIs","c":"Architecture \u0026 System Design","d":"Code Review \u0026 Quality Judgment"},"correct_answer":"b","explanation":"The chapter lists four dimensions, and explicitly contrasts the new skills with the old skill of memorizing syntax. The four dimensions are: Specification \u0026 Intent Communication, Architecture \u0026 System Design, Code Review \u0026 Quality Judgment, and Agent Supervision \u0026 Iteration."},{"question":"According to the chapter, why do experienced developers often see greater productivity gains with AI tools than beginners?","options":{"a":"Because they type faster.","b":"Because their experience allows them to provide more precise specifications and better evaluate the AI\u0027s output.","c":"Because AI tools are only designed for senior developers.","d":"Because they ignore the AI\u0027s suggestions and write the code themselves."},"correct_answer":"b","explanation":"The text explains that experienced developers benefit more because \u0027You know: What to ask for... What to look for... What matters...\u0027"},{"question":"What is the \u0027paradox\u0027 mentioned in the chapter regarding the value of developers?","options":{"a":"As AI tools get better, developers become less valuable.","b":"As AI tools become more powerful, skilled developers become MORE valuable, not less.","c":"AI tools only increase the value of junior developers.","d":"The value of a developer is now measured by how many AI tools they use."},"correct_answer":"b","explanation":"The chapter states, \u0027Here\u0027s what surprises people: As AI tools become more powerful, skilled developers become MORE valuable, not less.\u0027"},{"question":"What analogy does the chapter use to describe the impact of AI coding tools on developers?","options":{"a":"A calculator for a mathematician.","b":"A word processor for a writer.","c":"A GPS for a taxi driver.","d":"An assembly line for a factory worker."},"correct_answer":"b","explanation":"The text makes a direct comparison: \u0027Word processors didn\u0027t make writers obsolete—they freed writers to focus on storytelling... AI coding tools are doing the same for developers.\u0027"}]} />

