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

## Example 1: Installing a Python Package (pip)

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

## Example 2: Installing a Node.js Package (npm)

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

## Example 3: Package Managers for Different Languages

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

## Example 4: Verification—Confirming Installation

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

## Exercise 1: Package Manager Selection

For each scenario, which package manager should be used?

**Scenario 1**: "I'm writing a Python script that needs to read Excel files"
- A) pip
- B) npm
- C) brew

*Answer: A (pip for Python)*

**Scenario 2**: "I'm building a web application with Node.js and need a web framework"
- A) pip
- B) npm
- C) apt

*Answer: B (npm for Node.js)*

**Scenario 3**: "I need to install Git on my macOS computer"
- A) pip
- B) npm
- C) brew

*Answer: C (brew for macOS system tools)*

---

## Exercise 2: Understanding Dependencies

Read this installation output and answer questions:

```
$ npm install react
npm notice
added 200 packages, and audited 201 packages in 5s

packages audited, 201 vulnerabilities found
```

**Questions**:
1. How many packages were installed?
2. You only asked for `react`. Why were 200 other packages installed?
3. What do the "vulnerabilities found" mean?

**Answers**:
1. 201 packages (react + 200 dependencies)
2. react depends on other packages; those depend on others. npm installed the entire dependency tree.
3. Vulnerabilities are security issues in some packages. Your AI should check for these and warn you.

---

## Exercise 3: Verification Practice

Write what command/dialogue you'd use to verify each installation:

1. **Verify Python's `numpy` package installed**:
   - Command: `pip show numpy` or `python3 -c "import numpy; print(numpy.__version__)"`

2. **Verify Node's `express` installed**:
   - Your answer:

3. **Verify multiple packages together**:
   - Your answer:

---

**Model answers**:
2. `npm list express` or `npm ls | grep express`
3. `pip list` (shows all Python packages) or `npm list` (shows all Node packages)

---

## Formative Assessment: Packages and Dependencies

**Question 1**: Why do package managers automatically install dependencies?
- A) To waste disk space
- B) Because the package you want needs them to function
- C) No real reason

*Correct: B. Packages depend on other packages to work.*

**Question 2**: When you install a package with pip, which happens first?
- A) The package is installed immediately
- B) pip checks dependencies, then installs everything together
- C) You have to manually install dependencies

*Correct: B. Package managers resolve dependencies before installing.*

**Question 3**: How many package managers do you need to memorize?
- A) All of them (pip, npm, brew, apt, etc.)
- B) Just the one for your current project (Python=pip, JavaScript=npm)
- C) None—your AI chooses the right one

*Correct: B. Your AI handles the choice; you focus on the package you need.*

---

## Summative Assessment: Install and Verify a Package

Have a real conversation with your AI where you:

1. **Ask your AI to install a package** (for Python: `requests`, `numpy`; for JavaScript: `express`, `lodash`; etc.)
2. **Have your AI explain what dependencies are being installed**
3. **Observe the installation** and ask what's happening
4. **Verify the installation** using one of the methods (pip show, npm list, or try importing)
5. **Understand the dependency chain** by asking "What does this package depend on?"

**Success criteria**:
- You understand why multiple packages were installed (dependencies)
- You can verify the package is available
- You recognize that package managers automate dependency resolution
- You could request a different package with confidence

---

## Try With AI

**Tool**: Claude Code, ChatGPT Code Interpreter, Gemini CLI, or your preferred AI companion

**Setup**: You're going to install a real package and understand the dependency chain.

### Prompt 1: Install a Package and Explain

Copy and paste this prompt:

```
I want to install a useful Python/Node package for my work.
Install [choose one: requests, numpy, express, or lodash].
After installing, explain:
1. What the main package does
2. What dependencies were installed and why
3. How to verify it's available in my code
4. How to use a specific version if I needed to

Show each step clearly.
```

**Expected Outcome**:
- You see the actual installation output
- Your AI explains what each part means
- You understand the dependency chain
- You know how to verify the package works

### Prompt 2: Dependency Deep Dive

```
The package I just installed has lots of dependencies.
Show me:
1. All the dependencies it directly needs
2. Why it needs them (what functionality)
3. What happens if one dependency is missing

Make it concrete, not theoretical.
```

**Expected Outcome**: You understand that packages form a web of dependencies, and package managers navigate that web automatically

### Prompt 3: Verification Methods

```
I want to verify [package-name] is installed and usable in my code.
Show me 3 different ways to verify it:
1. Using the package manager directly
2. Trying to use it in code
3. Checking version information

Show me the exact commands/code for each.
```

**Expected Outcome**: You have multiple approaches to verify installations
