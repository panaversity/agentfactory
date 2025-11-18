# Project Context for Codebase Analysis

**Analysis Date**: [YYYY-MM-DD]
**Analyzed By**: [Your Name/Role]
**Analysis Purpose**: [Vendor Evaluation / Competitive Analysis / Feasibility Assessment / Onboarding]

---

## Project Overview (Layer 1: Foundation Context)

### Business Context
**What problem does this codebase solve?**
[Describe the business problem or domain this software addresses. Example: "E-commerce payment processing API" or "Real-time notification system for mobile apps"]

**Target Users/Customers**:
[Who uses this software? Example: "Internal developers" or "External third-party integrators" or "End consumers via mobile app"]

**Scale/Maturity**:
- Lines of Code: [Approximate count]
- Active Development: [Active / Maintained / Legacy]
- Production Status: [Production / Beta / Prototype]
- Team Size: [Number of developers]

### Technology Stack
**Primary Language(s)**: [Python, JavaScript, Go, etc.]

**Framework(s)**: [Django, React, FastAPI, Express, etc.]

**Database(s)**: [PostgreSQL, MongoDB, Redis, etc.]

**Infrastructure**: [AWS, GCP, Azure, self-hosted]

**Key Dependencies**: [List 3-5 critical external libraries/services]

---

## Architecture Context (Layer 2: Code Structure)

### Entry Points
**How does code execution start?**
[Example: "main.py launches FastAPI app" or "index.js bootstraps React app"]

**Key Entry Files**:
- [File path 1]: [Purpose]
- [File path 2]: [Purpose]
- [File path 3]: [Purpose]

### Module Organization
**How is code organized?**
```
/
├── [module-1]/     # [Purpose/responsibility]
├── [module-2]/     # [Purpose/responsibility]
├── [module-3]/     # [Purpose/responsibility]
└── [config/tests/docs]/
```

**Critical Modules** (focus analysis here):
1. **[Module Name]**: [Responsibility, why it's critical]
2. **[Module Name]**: [Responsibility, why it's critical]
3. **[Module Name]**: [Responsibility, why it's critical]

### Data Flow
**How does data move through the system?**
[Example: "User request → API Gateway → Auth middleware → Business logic → Database → Response"]

**External Integrations**:
- [Service 1]: [Purpose, data exchanged]
- [Service 2]: [Purpose, data exchanged]

---

## Analysis Constraints (Layer 3: Goals & Requirements)

### Analysis Goals
**What specific questions must this analysis answer?**
1. [Example: "Is the architecture scalable to 10x current load?"]
2. [Example: "What are the top 3 security risks?"]
3. [Example: "How hard would it be to integrate with our existing auth system?"]

### Success Criteria
**What makes this analysis successful?**
- [ ] [Example: "Identify architectural style (monolith, microservices, etc.)"]
- [ ] [Example: "Document all external API dependencies"]
- [ ] [Example: "Flag any hardcoded credentials or security anti-patterns"]
- [ ] [Example: "Estimate technical debt level (Low/Medium/High)"]

### Time Constraints
**Analysis Deadline**: [Date/time]
**Time Budget**: [Hours allocated for analysis]

### Focus Areas (Prioritized)
**Must analyze**:
1. [Example: "Authentication and authorization logic"]
2. [Example: "Database schema and migrations"]

**Should analyze if time permits**:
1. [Example: "Test coverage and quality"]
2. [Example: "Deployment configuration"]

**Can skip**:
- [Example: "Frontend UI components (backend-focused analysis)"]
- [Example: "Legacy migration scripts (deprecated)"]

---

## Analyst Context (Layer 4: Your Background)

### Your Role
**Position**: [Product Manager / Technical Lead / Founder / Engineer]

**Technical Background**:
[Example: "Strong Python experience, familiar with REST APIs, limited DevOps knowledge"]

### Analysis Preferences
**Communication Style**:
[Example: "Explain in business terms, not deep technical jargon" OR "Use technical precision, I'm an engineer"]

**Output Format Preferences**:
- Architecture diagrams: [Yes/No]
- Code snippets: [Yes, with explanations / Minimal / No]
- Risk severity scoring: [High/Medium/Low OR 1-10 scale]

**Assumptions**:
[Example: "Assume I understand HTTP/REST but not this specific framework" OR "Assume I need every acronym explained"]

---

## Analysis Workflow Guidance

### Step 1: High-Level Architecture Mapping
**Objective**: Understand system structure before diving into code details.

**Suggested prompts for Claude Code**:
```
"Using Glob tool, show me the directory structure. Then using Read tool, examine
[entry-point-file] to understand how the application bootstraps. Create a high-level
architecture diagram showing: main components, their responsibilities, and data flow."
```

### Step 2: Technology Stack Verification
**Objective**: Validate and document all technology choices.

**Suggested prompts**:
```
"Using Read tool on package.json (or requirements.txt/go.mod), list all dependencies
with versions. Flag any dependencies that are: (a) deprecated, (b) have known security
vulnerabilities, (c) are unmaintained."
```

### Step 3: Security & Risk Assessment
**Objective**: Identify security anti-patterns and technical debt.

**Suggested prompts**:
```
"Using Grep tool, search for common security issues:
1. Hardcoded credentials (grep for 'password', 'api_key', 'secret')
2. SQL injection risks (grep for string concatenation in SQL)
3. Missing authentication checks (grep for public endpoints)
Provide findings with severity (Critical/High/Medium/Low)."
```

### Step 4: Code Quality Evaluation
**Objective**: Assess maintainability and technical debt.

**Suggested prompts**:
```
"Using Read tool, examine [critical-module]. Assess:
1. Code complexity (are functions >50 lines? deeply nested?)
2. Documentation (docstrings, comments)
3. Error handling (try/catch blocks, validation)
4. Test coverage (do tests exist for critical paths?)
Rate technical debt as Low/Medium/High with rationale."
```

### Step 5: Integration Feasibility
**Objective**: Understand effort to integrate with existing systems.

**Suggested prompts**:
```
"Based on the authentication system in [auth-module] and our requirement to integrate
with [our-system], estimate integration complexity:
1. What needs to change in their codebase?
2. What needs to change in our codebase?
3. Are there blockers (incompatible auth schemes, data formats)?
Provide effort estimate: Small (days), Medium (weeks), Large (months)."
```

---

## Validation Checklist

Before finalizing analysis, verify:

- [ ] **Architecture diagram matches actual code structure** (cross-reference with Read tool)
- [ ] **All technical claims can be traced to specific files/lines** (provide evidence)
- [ ] **Security findings are confirmed** (not hallucinated patterns)
- [ ] **Technology stack list is complete** (checked package manifests)
- [ ] **Recommendations are actionable** (specific next steps, not vague advice)

---

## Output Format: Technical Assessment Report

**Target**: 2-page executive summary (see capstone-spec-template.md for full format)

**Page 1: Architecture Overview**
- System diagram
- Technology stack inventory
- Key modules and responsibilities

**Page 2: Strategic Assessment**
- Security findings (3-5 risks with severity)
- Technical debt score (Low/Medium/High with evidence)
- Recommendation (Acquire / Partner / Build Alternative / Pass)

---

## Best Practices for Claude Code Analysis

### Use Glob Before Read
**Why**: Get directory structure overview before reading individual files.
**Example**: `"Use Glob to show all .py files, then Read the main entry point"`

### Use Grep for Pattern Detection
**Why**: Faster than reading every file when looking for specific patterns.
**Example**: `"Use Grep to find all database queries, then analyze for SQL injection risks"`

### Validate AI Claims
**Why**: AI can hallucinate security issues or architectural patterns.
**How**: Cross-reference AI's findings with actual code using Read tool.

### Iterate Context Provision
**Why**: Adding all context upfront can overwhelm. Build progressively.
**Pattern**:
1. Start with Layer 1 (project overview)
2. AI asks clarifying questions → Add Layer 2 (code structure)
3. Refine based on AI's analysis → Add Layer 3 (constraints)

### Document Sources
**Why**: Stakeholders need to verify claims.
**Pattern**: "Finding: Hardcoded API key. Source: src/config.py line 42"

---

## Sample Analysis Session

**Scenario**: Evaluating a FastAPI codebase for acquisition.

**Initial Prompt** (with all 4 layers):
```
I'm evaluating a FastAPI-based REST API for potential acquisition.

LAYER 1 (Project Context):
- Business: E-commerce order management API
- Scale: 50K LOC, production, 5-person team
- Stack: Python 3.11, FastAPI, PostgreSQL, deployed on AWS

LAYER 2 (Code Structure):
- Entry: main.py launches FastAPI app
- Key modules: /auth (authentication), /orders (business logic), /payments (Stripe integration)
- External integrations: Stripe, SendGrid, Redis cache

LAYER 3 (Constraints):
- Must answer: Can we integrate with our existing OAuth2 system?
- Must identify: Top 3 security risks
- Success: Produce 2-page technical assessment in 2 hours

LAYER 4 (My Context):
- Role: Product Manager
- Background: Understand REST APIs, limited Python experience
- Preference: Explain security findings in business impact terms

Using Claude Code tools (Glob, Read, Grep), help me analyze this codebase. Start with
high-level architecture mapping, then security assessment.
```

**Expected AI Response**:
AI uses Glob → Read → Grep tools to produce architecture diagram, security findings, and integration assessment. You validate findings by cross-referencing with actual code.

---

## Anti-Patterns to Avoid

❌ **Vague analysis goals**: "Analyze this codebase"
✅ **Specific questions**: "Identify security risks in authentication module"

❌ **No context layers**: "Here's a repo link, analyze it"
✅ **4-layer context**: Project + Code + Constraints + Analyst background

❌ **Trusting AI blindly**: Accept all findings without verification
✅ **Validate claims**: Cross-reference AI output with actual code

❌ **Analysis paralysis**: Try to understand every file
✅ **Prioritized focus**: Analyze critical paths first (auth, data access, external APIs)

❌ **Generic outputs**: "Code quality is good"
✅ **Evidence-based**: "Technical debt: Medium. Evidence: 12 functions >100 lines in orders.py, missing error handling in payment.py line 67"

---

**Template Version**: 1.0.0
**Last Updated**: 2025-01-18
**Source**: Chapter 10, Lesson 6 (Project Memory Files)
**Constitution**: v6.0.0 Compliance

**Usage**: Copy this template to your analysis project root as `CLAUDE.md`. Fill in all bracketed placeholders. Use as context when prompting Claude Code for codebase analysis.
