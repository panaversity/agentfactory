# Capstone Project: Technical Assessment Report Template

**Lesson 8: Mastery Integration**
**Stage 4: Spec-Driven Integration**

---

## Project Overview

**Scenario**: You are a [Product Manager / Technical Lead / Founder] evaluating a codebase for [acquisition / partnership / competitive analysis / integration feasibility].

**Codebase Analyzed**: [Repository name/URL]

**Analysis Date**: [YYYY-MM-DD]

**Time Budget**: 2-3 hours (1 hour analysis, 1-2 hours report writing)

---

## Specification: What This Assessment Must Deliver

### Intent
**Primary Question**: [Example: "Should we acquire this company?" OR "Can we integrate this authentication system?" OR "What would it take to build a competing feature?"]

### Success Criteria
By the end of this assessment, stakeholders should be able to:

- [ ] **Understand the system architecture** (components, data flow, technology choices)
- [ ] **Identify critical security risks** (severity-rated, actionable findings)
- [ ] **Estimate technical debt** (qualitative assessment with evidence)
- [ ] **Make informed strategic decision** (Acquire / Partner / Build / Pass)

### Constraints
**Must complete in**: [2-3 hours total]

**Tools available**: [Claude Code OR Gemini CLI]

**Codebase size**: [Small: 5-15 files / Medium: 20-40 files / Large: 50+ files]

**Analysis focus**: [Full-stack / Backend-only / Security-focused / Integration-focused]

### Non-Goals
What this assessment will NOT cover:
- [ ] Exhaustive code review of every file
- [ ] Professional penetration testing
- [ ] Refactoring recommendations (unless critical to decision)
- [ ] UI/UX evaluation (unless specifically scoped)

---

## Technical Assessment Report Format (2 Pages)

### Page 1: Architecture Overview

#### System Architecture Diagram
**Purpose**: Visual representation of system components and data flow.

**Required Elements**:
- Main components (boxes with labels)
- Data flow (arrows showing request/response paths)
- External integrations (third-party services, APIs, databases)
- Entry points (how requests enter the system)

**Example Format**:
```
[User Request] → [API Gateway] → [Auth Middleware] → [Business Logic]
                                                            ↓
                                                      [Database]
                                                            ↓
                                                    [External APIs: Stripe, SendGrid]
```

**Deliverable**: ASCII diagram, Mermaid diagram, or hand-drawn scan (whichever is clearest).

---

#### Technology Stack Inventory
**Purpose**: Document all technology choices.

**Format**:
| Category | Technology | Version | Notes |
|----------|-----------|---------|-------|
| **Language** | [Python] | [3.11] | [Modern version, well-supported] |
| **Framework** | [FastAPI] | [0.104] | [Latest stable] |
| **Database** | [PostgreSQL] | [15.2] | [Production-grade] |
| **Caching** | [Redis] | [7.0] | [For session management] |
| **External APIs** | [Stripe, SendGrid] | [Latest] | [Payment, email] |
| **Infrastructure** | [AWS ECS] | [Fargate] | [Containerized deployment] |

**Key Observations**:
- ✅ **Strengths**: [Example: "Modern Python version, well-maintained dependencies"]
- ⚠️ **Concerns**: [Example: "One unmaintained library (package-xyz v1.2) with known CVE"]
- ❌ **Red Flags**: [Example: "Using deprecated Django 2.x with no upgrade path"]

---

#### Entry Points & Key Modules
**Purpose**: Map how code is organized and where execution starts.

**Entry Points**:
1. **[File path]**: [Purpose, e.g., "main.py — FastAPI application bootstrap"]
2. **[File path]**: [Purpose, e.g., "worker.py — Background task processor"]

**Key Modules** (critical paths to understand):
1. **[Module name]**: [Responsibility, why critical]
   - **Files**: [List 2-3 key files]
   - **Purpose**: [What this module does]
   - **Dependencies**: [What it depends on]

2. **[Module name]**: [Responsibility]
   - **Files**: [Key files]
   - **Purpose**: [Functionality]

3. **[Module name]**: [Responsibility]
   - **Files**: [Key files]
   - **Purpose**: [Functionality]

---

### Page 2: Strategic Assessment

#### Security Findings
**Purpose**: Identify risks that could impact acquisition decision or require remediation.

**Format**: Risk table with severity ratings.

| # | Finding | Severity | Location | Impact | Remediation |
|---|---------|----------|----------|--------|-------------|
| 1 | [Hardcoded API key in config] | **Critical** | config.py:42 | [Credential exposure if repo leaked] | [Move to environment variables] |
| 2 | [SQL injection risk in search] | **High** | search.py:78 | [Database compromise] | [Use parameterized queries] |
| 3 | [Missing rate limiting on auth] | **Medium** | auth.py:112 | [Brute force attacks possible] | [Add rate limiter middleware] |
| 4 | [Weak password requirements] | **Medium** | validators.py:23 | [Account takeovers] | [Enforce 12+ chars, complexity] |
| 5 | [Outdated dependency with CVE] | **Low** | requirements.txt | [Potential future exploit] | [Update library to latest] |

**Severity Definitions**:
- **Critical**: Immediate exploitation possible, severe business impact
- **High**: Exploitable with moderate effort, significant impact
- **Medium**: Requires specific conditions, moderate impact
- **Low**: Theoretical risk or minor impact

**Overall Security Posture**: [Low Risk / Medium Risk / High Risk]

**Rationale**: [Example: "3 high-severity findings in authentication layer suggest Medium-High risk. Requires security audit before acquisition."]

---

#### Technical Debt Score
**Purpose**: Assess code maintainability and long-term costs.

**Score**: [Low / Medium / High]

**Evidence**:

**Code Quality Indicators**:
- **Function Complexity**: [Example: "15 functions >100 lines in core modules"]
- **Documentation**: [Example: "60% of functions lack docstrings"]
- **Error Handling**: [Example: "Inconsistent error handling; 40% of API endpoints don't catch exceptions"]
- **Test Coverage**: [Example: "Est. 30% coverage based on test file count vs. source files"]
- **Dependencies**: [Example: "3 unmaintained libraries, 5 packages 2+ years outdated"]

**Architecture Debt**:
- [Example: "Monolith with 50K LOC — no module boundaries, tightly coupled"]
- [Example: "Direct database access from API layer — violates separation of concerns"]

**Deployment Debt**:
- [Example: "Manual deployment scripts — no CI/CD automation"]
- [Example: "No infrastructure-as-code — AWS resources manually configured"]

**Estimated Refactoring Cost**: [Low: &lt;1 month / Medium: 1-3 months / High: 6+ months]

**Rationale**: [Example: "Medium technical debt. Core logic is functional but poorly structured. Would require 2-3 months of refactoring for production-grade quality."]

---

#### Strategic Recommendation
**Purpose**: Actionable decision for stakeholders.

**Recommendation**: [Acquire / Partner / Build Alternative / Pass]

**Rationale** (2-3 paragraphs):

[Example for "Acquire with Conditions"]:
This codebase demonstrates solid functional implementation with concerning technical debt and security gaps. The core business logic (order management, payment processing) is well-tested and production-proven. However, 3 high-severity security findings require immediate remediation before acquisition closes.

**If we acquire**:
- **Immediate actions** (0-30 days): Fix Critical/High security findings, add rate limiting, migrate secrets to environment variables
- **Short-term** (1-3 months): Improve test coverage to 70%+, document undocumented modules, upgrade dependencies
- **Medium-term** (3-6 months): Refactor monolith into services, implement CI/CD, add infrastructure-as-code

**Acquisition should proceed ONLY IF**: Vendor agrees to fix Critical/High security findings pre-close OR acquisition price reflects 2-3 month remediation cost ($150K-$300K estimated engineering time).

**Alternative considered**: Building competing system from scratch would take 6-9 months. Acquiring and refactoring is faster path to market IF security remediation is acceptable risk.

---

**Decision Matrix**:

| Factor | Score (1-5) | Weight | Weighted |
|--------|-------------|--------|----------|
| **Technical Quality** | 3 | 30% | 0.9 |
| **Security Posture** | 2 | 25% | 0.5 |
| **Architecture Soundness** | 3 | 20% | 0.6 |
| **Integration Ease** | 4 | 15% | 0.6 |
| **Technology Stack Fit** | 4 | 10% | 0.4 |
| **Total** | | | **3.0 / 5.0** |

**Interpretation**: 3.0/5.0 suggests "Proceed with caution — viable if remediation plan is acceptable."

---

## Validation Checklist (Before Submitting Report)

**Architecture Section**:
- [ ] Diagram matches actual code structure (verified with Read/Glob tools)
- [ ] All technology stack items have versions listed
- [ ] Entry points confirmed by reading actual files
- [ ] Key modules accurately represent code organization

**Security Section**:
- [ ] All findings cite specific file paths and line numbers
- [ ] Security risks verified by examining actual code (not hallucinated)
- [ ] Severity ratings justified with business impact explanation
- [ ] Remediation steps are specific and actionable

**Technical Debt Section**:
- [ ] Code quality claims backed by examples (file paths, line counts)
- [ ] Architecture debt observations reflect actual code structure
- [ ] Refactoring cost estimate is realistic (not arbitrary)

**Recommendation Section**:
- [ ] Decision (Acquire/Partner/Build/Pass) clearly stated
- [ ] Rationale ties to findings in Security and Technical Debt sections
- [ ] Conditions for acquisition are specific and measurable
- [ ] Alternative options considered (build vs. buy analysis)

**Stakeholder Readiness**:
- [ ] Report understandable by non-technical stakeholders (no unexplained jargon)
- [ ] Recommendations are actionable (next steps clear)
- [ ] Business impact emphasized over technical minutiae

---

## Sample Assessment (Condensed Example)

**Scenario**: Evaluating FastAPI e-commerce API for acquisition.

**Codebase**: 30 files, 10K LOC, FastAPI + PostgreSQL, 6 months old.

**Time Spent**: 2 hours (1 hour Claude Code analysis, 1 hour report writing).

---

**PAGE 1: ARCHITECTURE OVERVIEW**

**System Diagram**:
```
[Customer API] → [FastAPI Router] → [Auth Middleware] → [Order Service]
                                                              ↓
                                                        [PostgreSQL]
                                                              ↓
                                                  [Stripe API (payments)]
                                                  [SendGrid API (emails)]
```

**Tech Stack**:
- Python 3.11, FastAPI 0.104, PostgreSQL 15, Redis 7
- External: Stripe, SendGrid
- Deploy: AWS ECS Fargate

**Key Observations**: Modern stack, production-ready, one outdated library (pydantic 1.x).

**Key Modules**:
1. **auth/** — JWT authentication (auth.py, validators.py)
2. **orders/** — Business logic (order.py, payment.py)
3. **api/** — FastAPI routes (routes.py)

---

**PAGE 2: STRATEGIC ASSESSMENT**

**Security Findings**:
1. **Hardcoded Stripe key** (Critical, config.py:42) → Move to env vars
2. **SQL injection risk** (High, search.py:78) → Use parameterized queries
3. **No rate limiting** (Medium, auth.py) → Add rate limiter

**Security Posture**: Medium-High risk (3 findings requiring immediate fix).

**Technical Debt**: Medium
- **Evidence**: 60% functions lack docstrings, 40% lack error handling, estimated 35% test coverage
- **Refactoring Cost**: 2-3 months for production-grade quality

**Recommendation**: **Acquire with Conditions**
- Vendor MUST fix Critical/High security findings pre-close
- OR reduce acquisition price by $200K (estimated 3-month remediation)
- Core business logic is sound; security gaps are remediable

**Decision**: 3.2/5.0 — Proceed if security remediation is acceptable.

---

## Grading Rubric (Self-Assessment)

**Architecture Section (30 points)**:
- [ ] Diagram present and accurate (10 pts)
- [ ] Tech stack complete with versions (10 pts)
- [ ] Entry points and modules correctly identified (10 pts)

**Security Section (30 points)**:
- [ ] 3+ findings with severity ratings (15 pts)
- [ ] All findings cite file/line evidence (10 pts)
- [ ] Remediation steps specific (5 pts)

**Technical Debt Section (20 points)**:
- [ ] Debt score (Low/Med/High) with evidence (10 pts)
- [ ] Refactoring cost estimate justified (10 pts)

**Recommendation Section (20 points)**:
- [ ] Clear decision stated (5 pts)
- [ ] Rationale ties to findings (10 pts)
- [ ] Alternative options considered (5 pts)

**Total**: 100 points

**Success Threshold**: 70+ points (engineering team rates as "actionable")

---

**Template Version**: 1.0.0
**Last Updated**: 2025-01-18
**Source**: Chapter 10, Lesson 8 (Capstone)
**Constitution**: v6.0.0 Compliance

**Usage**: Copy template. Fill in all bracketed placeholders. Complete analysis within 2-3 hour time budget. Validate against checklist before submitting.
