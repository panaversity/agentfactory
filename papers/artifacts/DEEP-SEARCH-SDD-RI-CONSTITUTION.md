# Constitution for Spec-Driven Development with Reusable Intelligence (SDD-RI)

**Project:** [Your Project Name]  
**Version:** 1.0.0  
**Ratified:** 2025-01-16  
**Last Amended:** 2025-01-16  
**Applicable Scope:** All specifications, agent architectures, and reusable intelligence components

---

## Preamble: The Paradigm Shift

Software development is experiencing a fundamental transformation. **Specifications and reusable intelligence components—not source code—are now the primary artifacts of value.** Code has become a regenerable output, while specifications, agent architectures, subagents, skills, and tools represent the durable strategic assets.

This constitution establishes the architectural and ethical principles governing all development in [Project Name]. These principles are **non-negotiable and actively enforced** through Phase -1 gates during planning and implementation.

**Core Insight:** In the AI-native era, your ability to articulate intent precisely (specification) and design reusable intelligence (agents, skills, tools) determines organizational capability. The teams that master this shift will build systems 10x-99x faster than those clinging to code-centric workflows.

**What Makes This Different:** Traditional software development optimized for human-readable code. SDD-RI optimizes for **machine-executable specifications** and **composable intelligence components**. You are not writing better code—you are architecting better minds and workflows for machines to inhabit.

---

## I. Foundational Articles (Tier 1: Immutable Principles)

These principles are **permanent and non-negotiable**. They establish the ethical, architectural, and quality character of all work. Implementation details may evolve, but these foundational standards remain constant.

### Article I: Specifications Are Primary Artifacts

**Statement:**  
Specifications are THE source code. All value creation begins with clear, precise, testable specifications. Code is a regenerable manifestation, not the canonical representation of system knowledge.

**Rationale:**  
In AI-native development, specification quality directly determines output quality. Poor specs → broken systems. Exceptional specs → production-ready systems. The bottleneck has shifted from "how fast can we type code" to "how clearly can we express intent."

**Enforcement:**  
- **Phase -1 Gate:** No implementation work begins without approved specification
- All specifications MUST include: requirements, acceptance criteria, constraints, non-goals, dependencies
- Specifications MUST be testable with objective success criteria (evals)
- Every code example MUST show the specification that produced it
- Failed outputs trigger specification refinement, never manual patching

**Anti-Patterns (NEVER DO THIS):**
- ❌ Writing code first, specs later ("documentation after the fact")
- ❌ Vague requirements without measurable criteria ("make it fast")
- ❌ Specifications that can't be validated objectively
- ❌ Treating specifications as optional planning documents
- ❌ Manually patching AI-generated code instead of refining specs

**Examples:**

✅ **Compliant Specification:**
```markdown
## Authentication Service Specification

### Requirements
- User registration with email/password
- JWT-based session management
- Password hashing with bcrypt (cost factor 12)
- Rate limiting: 5 login attempts per 15 minutes

### Acceptance Criteria
- 100% test coverage on authentication logic
- P95 latency < 200ms for login requests
- Zero plaintext password storage (verified by security scan)
- Graceful degradation when rate limit exceeded

### Constraints
- MUST use PostgreSQL for user storage (existing infrastructure)
- MUST NOT introduce new authentication libraries (WorkOS already approved)
- Token expiry MUST NOT exceed 1 hour (security requirement)

### Non-Goals
- Social authentication (OAuth deferred to Phase 2)
- Multi-factor authentication (separate project)
```

❌ **Violation:**
```markdown
## Authentication

Build a login system that's secure and fast.
```

### Article II: Reusable Intelligence as Strategic Asset

**Statement:**  
Subagents, skills, and tools are the primary units of reuse and organizational capability. Code reuse is secondary to intelligence reuse.

**Rationale:**  
Traditional software engineering optimized for reusable code (libraries, frameworks). AI-native engineering optimizes for reusable intelligence—structured knowledge and decision-making capabilities that agents can apply consistently across projects. The competitive advantage shifts from "who has the best code" to "who has the best intelligence architecture."

**Enforcement:**  
- All reusable components (subagents, skills, tools) MUST be designed for cross-project portability
- Component specifications MUST define: single responsibility, explicit interfaces, tool permissions, handoff protocols
- Backward compatibility MUST be maintained through semantic versioning
- Breaking changes REQUIRE major version bump + migration guide
- All reusable components REQUIRE 85% test coverage minimum (higher than application code)

**Anti-Patterns (NEVER DO THIS):**
- ❌ Building monolithic multi-purpose agents instead of specialized subagents
- ❌ Hardcoding project-specific paths/assumptions in reusable components
- ❌ Granting universal tool permissions instead of least-privilege access
- ❌ Skipping documentation because "the code is self-explanatory"
- ❌ Breaking changes without version bumps or migration paths

**Examples:**

✅ **Compliant Subagent Specification:**
```markdown
## Security Reviewer Subagent

### Purpose (Single Responsibility)
Analyze code changes for OWASP Top 10 vulnerabilities and produce severity-prioritized findings with fix recommendations.

### Inputs
- Git diff or file paths
- Language/framework context
- Security baseline requirements

### Outputs
- Structured JSON report: [{severity, vulnerability_type, location, description, fix_recommendation}]
- Exit code: 0 (pass), 1 (warnings), 2 (critical)

### Tool Permissions (Least Privilege)
- Read: file system (analysis only)
- Execute: security scanning tools (semgrep, bandit, npm audit)
- NO Write permissions
- NO Network access

### Handoff Protocol
- Triggered: After code generation, before commit
- Returns control to: Implementer agent for remediation
- Escalation: Critical findings block merge until resolved
```

❌ **Violation:**
```markdown
## Code Helper

Helps with implementation tasks and reviews code when needed.
```

### Article III: Validation-First Safety

**Statement:**  
NEVER trust, ALWAYS verify. All AI-generated outputs MUST be read, understood, tested, and validated against specifications before use.

**Rationale:**  
AI agents can hallucinate, misunderstand requirements, or generate insecure code. Blind trust in AI outputs is unprofessional and dangerous. Validation skills are as important as specification skills. This is the critical safety mechanism in AI-native workflows.

**Enforcement:**  
- All generated code MUST be reviewed by humans before execution
- All code MUST pass automated testing (unit, integration, end-to-end)
- All code MUST pass security scanning (no hardcoded secrets, injection vulnerabilities)
- All code MUST align with specification (automated diff validation where possible)
- Failed validation triggers specification refinement → regeneration (NOT manual fixes)

**Anti-Patterns (NEVER DO THIS):**
- ❌ Running AI-generated code without reading it first
- ❌ Accepting code that "seems to work" without tests
- ❌ Skipping security scans for "simple" code
- ❌ Manually patching generated code instead of improving specs
- ❌ Trusting AI explanations without independent verification

**Examples:**

✅ **Compliant Validation Workflow:**
```markdown
## Validation Checklist (All MUST Pass)

### Phase 1: Human Review
- [ ] Code reviewed line-by-line for understanding
- [ ] Logic matches specification requirements
- [ ] No obvious security vulnerabilities
- [ ] No hardcoded secrets or sensitive data
- [ ] Error handling present and appropriate

### Phase 2: Automated Testing
- [ ] All unit tests pass (pytest -v)
- [ ] All integration tests pass
- [ ] Code coverage ≥ 85%
- [ ] Type checking passes (mypy --strict)
- [ ] Linting passes (black, ruff)

### Phase 3: Security Validation
- [ ] Security scan passes (bandit, semgrep)
- [ ] No SQL injection vulnerabilities
- [ ] No XSS vulnerabilities
- [ ] Input validation on all external data
- [ ] Secrets in environment variables, not code

### Phase 4: Specification Alignment
- [ ] All acceptance criteria met
- [ ] All constraints satisfied
- [ ] All non-goals respected
- [ ] Performance benchmarks achieved

Gate Status: ✅ PASS → Proceed | ❌ FAIL → Refine spec and regenerate
```

❌ **Violation:**
```markdown
## Validation

- [x] Ran the code, looks good!
```

### Article IV: Evals-First Development

**Statement:**  
Define success criteria and evaluation methods BEFORE writing specifications or code. Professional AI-native development follows: **Evals → Spec → Implement → Validate**.

**Rationale:**  
Traditional development tested after coding. AI-native development defines "what good looks like" first, then builds toward it. This prevents drift, ensures alignment, and makes validation objective. Evals must connect to business goals, not arbitrary technical metrics.

**Enforcement:**  
- Evals MUST be defined before specifications
- Evals MUST be measurable, objective, and automated where possible
- Evals MUST connect to user/business outcomes
- All implementations MUST be validated against evals
- Failing evals triggers specification refinement, not eval relaxation

**Anti-Patterns (NEVER DO THIS):**
- ❌ Writing specs without defining success criteria first
- ❌ Vague evals ("should be good quality")
- ❌ Technical metrics disconnected from user value
- ❌ Changing evals to match outputs instead of improving outputs
- ❌ Skipping evals for "simple" features

**Examples:**

✅ **Compliant Evals-First Workflow:**
```markdown
## Feature: User Onboarding Flow

### Evals (Define Success FIRST)

**Business Goal:** Increase activation rate from 45% to 70%

**Evaluation Criteria:**
1. **User Success Rate:**
   - Metric: % of users completing onboarding within 5 minutes
   - Target: ≥ 70% (current: 45%)
   - Measurement: Analytics tracking (onboarding_complete event)

2. **Comprehension:**
   - Metric: % of users answering post-onboarding quiz correctly
   - Target: ≥ 80% score on 5-question quiz
   - Measurement: Quiz results stored in database

3. **Error Rate:**
   - Metric: % of sessions with validation errors
   - Target: < 5% (current: 18%)
   - Measurement: Error logging + session tracking

4. **Accessibility:**
   - Metric: Lighthouse accessibility score
   - Target: ≥ 95
   - Measurement: Automated Lighthouse CI check

### Specification (Write AFTER Evals)
[detailed specification targeting above criteria]

### Implementation (Generate from Spec)
[AI generates code to meet spec + evals]

### Validation (Verify Against Evals)
[automated testing confirms all eval criteria met]
```

❌ **Violation:**
```markdown
## Feature: User Onboarding

Build a nice onboarding flow that helps users get started.

[specification and code written without defining success criteria]
```

### Article V: Bilingual Development (Python + TypeScript)

**Statement:**  
Professional AI-native developers MUST be proficient in both Python (reasoning/backend) and TypeScript (interaction/frontend). Both languages are first-class citizens.

**Rationale:**  
Modern AI systems have two layers: Reasoning Layer (Python for agents, data processing, logic) and Interaction Layer (TypeScript for UIs, real-time systems, voice interfaces). Single-language fluency limits what you can build. Full-stack AI-native development requires bilingual capability.

**Enforcement:**  
- Python MUST use 3.13+ with mandatory type hints (mypy --strict)
- TypeScript MUST use 5.3+ with strict mode and ES2024 target
- All function signatures MUST include type annotations (zero exceptions)
- Specifications MUST address both languages where appropriate
- Projects integrating both languages MUST demonstrate proper boundaries and communication

**Anti-Patterns (NEVER DO THIS):**
- ❌ Python-only solutions when TypeScript interaction layer needed
- ❌ TypeScript-only solutions when Python reasoning layer needed
- ❌ Missing type hints ("I'll add them later")
- ❌ Loose typing modes (TypeScript without strict, Python without mypy)
- ❌ Treating one language as "secondary" or "nice to have"

**Examples:**

✅ **Compliant Type Annotations:**

**Python (3.13+ with strict typing):**
```python
from typing import Protocol
from datetime import datetime

class AuthService(Protocol):
    async def authenticate(
        self, 
        username: str, 
        password: str
    ) -> tuple[str, datetime] | None:
        """Returns (token, expiry) or None if auth fails."""
        ...

async def login_user(
    auth: AuthService,
    username: str,
    password: str
) -> dict[str, str | int]:
    """
    Authenticate user and return session data.
    
    Returns:
        {"token": str, "expires_in": int} on success
        {"error": str, "code": int} on failure
    """
    result = await auth.authenticate(username, password)
    if result is None:
        return {"error": "Invalid credentials", "code": 401}
    
    token, expiry = result
    return {
        "token": token, 
        "expires_in": int((expiry - datetime.now()).total_seconds())
    }
```

**TypeScript (5.3+ with strict mode):**
```typescript
// tsconfig.json: "strict": true, "target": "ES2024"

interface AuthResponse {
  token: string;
  expiresIn: number;
}

interface AuthError {
  error: string;
  code: number;
}

type LoginResult = AuthResponse | AuthError;

async function loginUser(
  username: string,
  password: string
): Promise<LoginResult> {
  const response = await fetch('/api/auth/login', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ username, password })
  });
  
  if (!response.ok) {
    return {
      error: 'Authentication failed',
      code: response.status
    };
  }
  
  return await response.json() as AuthResponse;
}
```

❌ **Violations:**

**Python (missing types):**
```python
async def login_user(auth, username, password):  # ❌ No type hints
    result = await auth.authenticate(username, password)
    if result is None:
        return {"error": "Invalid credentials"}
    return {"token": result[0]}
```

**TypeScript (loose typing):**
```typescript
// ❌ No return type, implicit any
async function loginUser(username, password) {
  const response = await fetch('/api/auth/login', {
    method: 'POST',
    body: JSON.stringify({ username, password })
  });
  return response.json();  // ❌ No type assertion
}
```

### Article VI: Production-Ready Deployment Standards

**Statement:**  
All projects MUST demonstrate production deployment with cloud-native patterns. "Works on my laptop" is not professional software.

**Rationale:**  
AI-native developers must understand containerization, orchestration, state management, and scalability. Deployment is not optional—it's the measure of whether software is actually useful. Local-only development teaches outdated workflows.

**Enforcement:**  
- All projects MUST include Docker containerization
- Multi-service projects MUST include Kubernetes manifests
- All services MUST implement health checks, structured logging, graceful shutdown
- Secrets MUST use environment variables or secret management (never hardcoded)
- All deployments MUST include observability (logs, metrics, traces)
- Production configurations MUST differ from development (no debug mode in prod)

**Anti-Patterns (NEVER DO THIS):**
- ❌ No deployment documentation ("just run python main.py")
- ❌ Development-only configurations without production path
- ❌ Hardcoded secrets in Dockerfiles or manifests
- ❌ Missing health checks (Kubernetes will kill your pods)
- ❌ No structured logging (impossible to debug in production)
- ❌ No graceful shutdown (data loss on deployment)

**Examples:**

✅ **Compliant Production Deployment:**

**Dockerfile (Multi-stage build):**
```dockerfile
# Build stage
FROM python:3.13-slim AS builder
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Production stage
FROM python:3.13-slim
WORKDIR /app
COPY --from=builder /usr/local/lib/python3.13/site-packages /usr/local/lib/python3.13/site-packages
COPY src/ ./src/

# Non-root user for security
RUN useradd -m -u 1000 appuser && chown -R appuser /app
USER appuser

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD python -c "import requests; requests.get('http://localhost:8000/health')"

CMD ["python", "-m", "uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Kubernetes Deployment:**
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: auth-service
spec:
  replicas: 3
  selector:
    matchLabels:
      app: auth-service
  template:
    metadata:
      labels:
        app: auth-service
    spec:
      containers:
      - name: auth-service
        image: myregistry/auth-service:1.0.0
        ports:
        - containerPort: 8000
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: auth-secrets
              key: database-url
        - name: JWT_SECRET
          valueFrom:
            secretKeyRef:
              name: auth-secrets
              key: jwt-secret
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 5
        resources:
          requests:
            memory: "128Mi"
            cpu: "100m"
          limits:
            memory: "256Mi"
            cpu: "500m"
```

❌ **Violations:**
```dockerfile
# ❌ No multi-stage build (large image)
# ❌ Running as root
# ❌ No health check
FROM python:3.13
COPY . /app
WORKDIR /app
RUN pip install -r requirements.txt
CMD python main.py
```

### Article VII: Constitutional AI Principles

**Statement:**  
All AI agents MUST operate within ethical, legal, and safety boundaries. These constraints are non-negotiable and actively enforced.

**Rationale:**  
AI agents with autonomy and tool access can cause harm if unconstrained. Constitutional principles ensure agents behave responsibly, respect human rights, comply with laws, and prioritize safety. This is professional standard, not optional consideration.

**Enforcement:**  
- All agent specifications MUST include constitutional constraints
- Agents MUST refuse illegal or harmful actions even when instructed
- Agents MUST escalate uncertain legal/ethical situations to humans
- Agents MUST explain refusals transparently without preaching
- Agents MUST maintain audit trails for all high-stakes operations

**Core Constitutional Principles:**
1. **Helpful, Honest, Harmless:** Assist users effectively while providing accurate information and avoiding harmful outputs
2. **Human Rights Respect:** Align with UN Declaration of Human Rights principles
3. **Legal Compliance:** Refuse actions that violate constitutional law, criminal law, core tort law, or regulatory requirements (GDPR, HIPAA, etc.)
4. **Transparency:** Acknowledge uncertainty and capability boundaries; escalate novel situations
5. **Privacy Protection:** Minimize data collection, encrypt sensitive data, honor opt-out requests
6. **Fairness:** Avoid discrimination, ensure equitable access, detect and mitigate bias
7. **Accountability:** Maintain audit trails, enable forensic investigation, log decision rationale

**Anti-Patterns (NEVER DO THIS):**
- ❌ Agents that execute all instructions without ethical review
- ❌ Claiming capabilities beyond actual limits
- ❌ Hiding refusals behind vague errors instead of transparent explanations
- ❌ Collecting unnecessary user data without consent
- ❌ Operating without audit logs (impossible to debug or investigate)

**Examples:**

✅ **Compliant Agent Constitutional Constraints:**
```markdown
## Data Processor Agent - Constitutional Constraints

### Mandatory Refusals
Agent MUST refuse to:
- Process personally identifiable information (PII) without explicit user consent
- Execute data deletion without confirmation + authorization check
- Bypass access controls or authentication mechanisms
- Generate code that introduces security vulnerabilities
- Perform actions that violate GDPR, CCPA, or HIPAA requirements

### Escalation Protocol
Agent MUST escalate to human review when:
- Legal uncertainty exists (search authoritative sources first, escalate if still unclear)
- High-stakes operations requested (financial transactions, production deployments, data deletion)
- Novel situation not covered by training or guidelines
- Conflicting requirements create ethical dilemma

### Transparency Requirements
When refusing requests, agent MUST:
- Explain the specific constitutional principle violated
- Suggest compliant alternatives where possible
- Provide reasoning without lengthy ethical lectures
- Document refusal in audit log with justification

### Audit Trail Requirements
Agent MUST log:
- All tool invocations (tool name, parameters, timestamp, result)
- All data access operations (what data, why accessed, by whom)
- All refusals (request, constitutional principle, reasoning)
- All escalations to humans (situation, reason, outcome)
```

❌ **Violation:**
```markdown
## Data Processor Agent

Processes data as instructed. Handles all user requests.

[No constitutional constraints defined]
```

### Article VIII: Progressive Disclosure Architecture

**Statement:**  
Reusable components (especially skills) MUST use tiered information architecture to prevent context window saturation while maintaining universal availability.

**Rationale:**  
Loading all component details upfront exhausts context windows and degrades agent performance. Progressive disclosure loads metadata first (discovery), full instructions when relevant (execution), and resources only when needed (optimization). This architectural pattern is essential for scaling agent systems.

**Enforcement:**  
- Skills MUST provide metadata layer (~100 tokens): title, description, applicability criteria
- Skills MUST provide instruction layer (<5k tokens): detailed procedures, examples, edge cases
- Skills MUST provide resource layer (on-demand): scripts, templates, schemas
- Subagents MUST declare tool permissions explicitly (no implicit universal access)
- Tools MUST implement lazy loading for large data structures

**Anti-Patterns (NEVER DO THIS):**
- ❌ Loading all skill content upfront regardless of relevance
- ❌ No metadata layer (agents can't discover skills efficiently)
- ❌ Monolithic skill files exceeding 10k tokens
- ❌ Implicit universal tool permissions (security risk + context waste)
- ❌ Eager loading of large reference documents

**Examples:**

✅ **Compliant Progressive Disclosure (Skill Structure):**

**Metadata Layer (skill-metadata.json):**
```json
{
  "name": "secure-api-design",
  "version": "2.1.0",
  "description": "Design secure REST APIs with authentication, authorization, rate limiting, and input validation",
  "applicability": [
    "When designing public-facing APIs",
    "When implementing authentication systems",
    "When security requirements include PII protection"
  ],
  "categories": ["security", "api-design", "backend"],
  "prerequisites": ["REST fundamentals", "HTTP protocol"],
  "estimated_tokens": 4200
}
```

**Instruction Layer (SKILL.md) - Loaded when relevant:**
```markdown
# Secure API Design Skill

## Core Principles
[detailed procedures, ~4k tokens]

## Authentication Patterns
[JWT, OAuth 2.0, API keys - examples and tradeoffs]

## Rate Limiting Strategies
[token bucket, leaky bucket, sliding window - with code examples]

## Input Validation
[schema validation, sanitization, type checking]

## Common Vulnerabilities
[OWASP API Top 10 with mitigation strategies]
```

**Resource Layer (resources/) - Loaded on-demand:**
```
resources/
├── openapi-template.yaml      # Load when generating API spec
├── jwt-implementation.py      # Load when implementing JWT auth
├── rate-limiter.py            # Load when adding rate limiting
└── validation-schemas.json    # Load when designing input validation
```

❌ **Violation (Monolithic skill):**
```markdown
# API Design Mega-Skill

[15,000 tokens of content covering everything about APIs]
[No metadata layer - agents can't determine relevance]
[No resource separation - all loaded at once]
[Context window saturated, agent performance degraded]
```

---

## II. Implementation Rules (Tier 2: Context-Dependent)

These rules are **project-specific and evolve** based on technology choices, team standards, and organizational context. They define HOW to implement foundational principles.

### Technology Stack Standards

**Current Approved Stack:**
- **Languages:** Python 3.13+, TypeScript 5.3+
- **Agent Frameworks:** OpenAI Agents SDK, Anthropic Claude SDK
- **MCP Servers:** Standard Model Context Protocol implementations
- **Containerization:** Docker 24+
- **Orchestration:** Kubernetes 1.28+
- **Databases:** PostgreSQL 16+, Redis/Valkey 7+
- **Message Brokers:** Kafka 3.6+ or Dapr 1.12+

**Modification Process:**
- Technology changes require ADR (Architecture Decision Record)
- Breaking changes require impact analysis across all components
- New dependencies require security review and license compliance check

### Coding Standards

**Python:**
- Formatter: `black` (line length 100)
- Linter: `ruff` (all rules enabled except specific waivers documented in pyproject.toml)
- Type checker: `mypy --strict`
- Import sorting: `isort` with black compatibility
- Docstrings: Google style

**TypeScript:**
- Formatter: `prettier`
- Linter: `eslint` with strict config
- Type checker: `tsc --strict`
- Target: ES2024
- Module: ESNext

### File Organization Standards

**SpecKit Plus Structure:**
```
<project-root>/
├── .specify/
│   ├── memory/
│   │   └── constitution.md              # THIS DOCUMENT
│   ├── scripts/
│   ├── templates/
│   └── specs/
│       └── 001-feature-name/
│           ├── spec.md                  # Requirements
│           ├── plan.md                  # Implementation plan
│           ├── tasks.md                 # Task checklist
│           ├── adrs/                    # Architecture decisions
│           └── phrs/                    # Prompt history records
├── src/
│   ├── agents/                          # Subagent implementations
│   ├── skills/                          # Skill packages
│   └── tools/                           # MCP tool servers
├── tests/
├── k8s/                                 # Kubernetes manifests
└── docs/
```

### Testing Standards

**Coverage Requirements:**
- Reusable components (agents, skills, tools): **≥ 85% coverage**
- Application code: **≥ 70% coverage**
- Critical security functions: **100% coverage**

**Test Types (all required):**
- Unit tests (fast, isolated)
- Integration tests (services, databases, APIs)
- End-to-end tests (user workflows)
- Security tests (OWASP, penetration, fuzzing)

### Security Standards

**Secrets Management:**
- Environment variables for development
- Kubernetes Secrets for production
- Never commit secrets to git (pre-commit hooks enforced)

**Authentication:**
- WorkOS for user authentication (approved vendor)
- JWT tokens with 1-hour expiry maximum
- Refresh tokens stored in httpOnly cookies

**Authorization:**
- Role-Based Access Control (RBAC) via Kubernetes
- Principle of least privilege for all service accounts
- Audit logging for all privileged operations

---

## III. Anti-Patterns Catalog (NEVER DO THIS)

This section explicitly identifies failure modes to prevent convergence on mediocre patterns. **If you see these patterns, you are building wrong. Stop and refine.**

### Specification Anti-Patterns

**"Vague Intent"**
```markdown
❌ BAD: Build a user authentication system that's secure and easy to use.

✅ GOOD: Implement JWT-based authentication with:
- Email/password registration (bcrypt cost factor 12)
- Token expiry: 1 hour (configurable via env var)
- Rate limiting: 5 failed attempts per 15 minutes per IP
- Acceptance criteria: 100% test coverage on auth logic, P95 latency < 200ms
```

**"Implementation Disguised as Specification"**
```markdown
❌ BAD: Use FastAPI to create an endpoint that takes username and password,
validates them against the database, and returns a JWT token using the
PyJWT library with HS256 algorithm...

✅ GOOD: Authentication endpoint specification:
- Input: username (string), password (string)
- Output: {token: string, expires_at: ISO8601} | {error: string, code: int}
- Constraints: MUST validate against PostgreSQL user table, MUST use existing
  WorkOS integration (no new auth library)
```

**"Feature List Without Context"**
```markdown
❌ BAD:
- User login
- User registration  
- Password reset
- Profile management

✅ GOOD: User Identity Management System
Business Goal: Enable secure self-service account management to reduce support tickets by 60%
Core Workflows:
1. Registration (new user acquisition)
2. Authentication (secure access)
3. Self-service password recovery (reduce support load)
4. Profile updates (data currency)
Context: Existing system has 18% support tickets related to password resets; cost: $45/ticket
```

### Reusable Component Anti-Patterns

**"God Agent" (Monolithic Multi-Purpose Agent)**
```markdown
❌ BAD: "Developer Assistant Agent"
- Writes code
- Reviews code
- Writes tests
- Writes documentation
- Deploys to production
- Monitors systems
- Fixes bugs
- Answers questions

✅ GOOD: Specialized Subagents
- Code Generator: Takes spec → generates implementation
- Security Reviewer: Analyzes code → produces vulnerability report
- Test Engineer: Takes code → generates comprehensive test suite
- Documentation Writer: Takes code + spec → produces user/API docs
[Each has single responsibility, clear interfaces, explicit tool permissions]
```

**"Skill Blob" (Monolithic Undifferentiated Knowledge)**
```markdown
❌ BAD: "Python Best Practices" skill (18,000 tokens)
- Everything about Python in one giant document
- No progressive disclosure
- Context window saturated
- Agent performance degraded

✅ GOOD: Modular Python Skills
- python-typing: Type hints and mypy patterns (~2k tokens)
- python-async: Asyncio patterns and best practices (~3k tokens)
- python-security: Common vulnerabilities and fixes (~4k tokens)
- python-testing: Pytest patterns and fixtures (~2.5k tokens)
[Each focused, independently loadable, composable]
```

**"Implicit Permissions" (Security Risk)**
```markdown
❌ BAD: Subagent specification with no tool declarations
→ Result: Agent gets ALL tools by default (security violation)

✅ GOOD: Explicit least-privilege permissions
```yaml
tools:
  read:
    - file_system: ["src/**/*.py", "tests/**/*.py"]
    - git: ["log", "diff", "blame"]
  write:
    - NONE  # Read-only agent
  execute:
    - security_scanners: ["bandit", "semgrep"]
```
```

### Validation Anti-Patterns

**"Looks Good To Me" (LGTM without verification)**
```markdown
❌ BAD:
- [x] Generated code
- [x] Ran it, seems to work
- [x] No errors

✅ GOOD: Comprehensive validation checklist
- [x] Read and understood all generated code
- [x] Verified logic matches specification requirements
- [x] All unit tests pass (pytest -v)
- [x] All integration tests pass
- [x] Security scan passes (bandit --severity high)
- [x] Type checking passes (mypy --strict)
- [x] No hardcoded secrets (manual review + automated scan)
- [x] Performance benchmarks met (load testing results attached)
- [x] Acceptance criteria verified (spec.md checklist)
```

**"Manual Patching Loop" (Anti-SDD)**
```markdown
❌ BAD: AI generates code → doesn't work → manually fix → repeat
[This defeats the purpose of SDD; you're back to manual coding]

✅ GOOD: Specification refinement loop
AI generates code → doesn't work → analyze failure →
refine SPECIFICATION → regenerate → validate
[Specification improves over time; knowledge captured; reproducible]
```

### Deployment Anti-Patterns

**"Localhost Hero"**
```markdown
❌ BAD:
README: "Just run python main.py and it works on my machine!"
[No Docker, no environment docs, no deployment path, not reproducible]

✅ GOOD:
- Dockerfile with multi-stage build
- docker-compose.yml for local development
- Kubernetes manifests for production
- Environment variable documentation
- Health checks and observability
- CI/CD pipeline configuration
```

**"Debug Mode in Production"**
```dockerfile
❌ BAD:
ENV FLASK_ENV=development
ENV DEBUG=true
CMD flask run --host=0.0.0.0 --debug

✅ GOOD:
ENV ENVIRONMENT=production
ENV LOG_LEVEL=info
CMD gunicorn --workers 4 --bind 0.0.0.0:8000 main:app
```

---

## IV. Phase -1 Gates (Enforcement Mechanism)

Constitutional principles are enforced through validation checkpoints BEFORE implementation begins. **These are not suggestions—these are mandatory gates.**

### Gate 1: Specification Validation (Before Planning)

**Entry Criteria:**
- Problem statement defined
- Business goals articulated
- Success criteria (evals) specified

**Validation Checklist:**
```markdown
## Specification Quality Gate

### Clarity
- [ ] Requirements are unambiguous and testable
- [ ] Acceptance criteria are objective and measurable
- [ ] Constraints are explicit (no hidden assumptions)
- [ ] Non-goals clearly stated (what we're NOT building)

### Completeness
- [ ] All inputs and outputs specified
- [ ] All dependencies identified
- [ ] All failure modes considered
- [ ] All stakeholders identified

### Testability
- [ ] Success criteria (evals) defined FIRST
- [ ] Evals connect to business goals
- [ ] Automated validation possible
- [ ] Edge cases documented

### Reusability (for reusable components)
- [ ] Single responsibility principle satisfied
- [ ] Clear interface boundaries
- [ ] No project-specific assumptions
- [ ] Version compatibility declared

Gate Status: ✅ PASS → Proceed to Planning | ⚠️ WARNING → Document justification | ❌ FAIL → Refine specification
```

### Gate 2: Architectural Validation (Before Implementation)

**Entry Criteria:**
- Specification approved
- Plan.md created
- Tasks.md created

**Validation Checklist:**
```markdown
## Architecture Quality Gate

### Constitutional Alignment
- [ ] Article I: Specifications primary artifacts (spec exists and approved)
- [ ] Article II: Reusable intelligence designed correctly (if applicable)
- [ ] Article III: Validation plan defined
- [ ] Article IV: Evals defined and connected to business goals
- [ ] Article V: Both Python and TypeScript addressed (if applicable)
- [ ] Article VI: Deployment path documented
- [ ] Article VII: Constitutional AI constraints specified (for agents)
- [ ] Article VIII: Progressive disclosure applied (for skills)

### Implementation Standards
- [ ] Technology stack approved (Tier 2 rules)
- [ ] File organization follows standards
- [ ] Testing strategy defined (unit, integration, e2e)
- [ ] Security considerations documented
- [ ] Observability plan specified (logs, metrics, traces)

### Dependency Analysis
- [ ] All dependencies identified and justified
- [ ] License compatibility verified
- [ ] Security vulnerabilities checked
- [ ] Version pinning specified

Gate Status: ✅ PASS → Proceed to Implementation | ⚠️ WARNING → Document justification | ❌ FAIL → Revise plan
```

### Gate 3: Implementation Validation (Before Merge)

**Entry Criteria:**
- Code generated
- Tests written
- Documentation updated

**Validation Checklist:**
```markdown
## Implementation Quality Gate

### Code Quality
- [ ] Read and understood all generated code
- [ ] Logic matches specification
- [ ] Type hints present on all functions (Python)
- [ ] Strict mode enabled (TypeScript)
- [ ] No obvious security vulnerabilities
- [ ] Error handling appropriate
- [ ] Logging structured and meaningful

### Testing
- [ ] All unit tests pass
- [ ] All integration tests pass
- [ ] Coverage meets requirements (85% for reusable, 70% for app)
- [ ] Edge cases tested
- [ ] Security tests included

### Security
- [ ] No hardcoded secrets (automated scan + manual review)
- [ ] No SQL injection vulnerabilities
- [ ] No XSS vulnerabilities
- [ ] Input validation on all external data
- [ ] Security scan passes (bandit, semgrep, npm audit)

### Specification Alignment
- [ ] All acceptance criteria met
- [ ] All constraints satisfied
- [ ] All evals pass
- [ ] Performance benchmarks achieved

### Documentation
- [ ] README updated (if applicable)
- [ ] API documentation generated (if applicable)
- [ ] ADR created for significant decisions
- [ ] PHR updated (prompt history record)

Gate Status: ✅ PASS → Merge to main | ⚠️ WARNING → Document risks and proceed | ❌ FAIL → Refine spec and regenerate
```

**Critical Notes:**
- ⚠️ WARNING allows proceeding with documented technical debt
- ❌ FAIL blocks merge and requires iteration
- Gates are automated where possible (CI/CD pipelines)
- Human review required for all ❌ FAIL → ✅ PASS overrides

---

## V. Governance and Amendment Process

### Authority and Enforcement

**This Constitution is the supreme governing document for [Project Name].**

- All specifications, agent architectures, and reusable components MUST align with foundational articles (Tier 1)
- Implementation rules (Tier 2) can be amended based on project evolution
- Anti-patterns catalog should grow as new failure modes are discovered
- Phase -1 gates enforce constitutional principles automatically where possible

### Amendment Process

**For Tier 2 Changes** (Implementation rules - technology, standards, processes):
- Create Architecture Decision Record (ADR) documenting rationale
- Update constitution with new standards
- Increment MINOR version (1.0.0 → 1.1.0)
- No impact on existing components (backward compatible)

**For Tier 1 Changes** (Foundational principles):
- **REQUIRES:** Written justification with evidence of problem
- **REQUIRES:** Impact analysis across all existing components
- **REQUIRES:** Review and approval by [governance body - define who]
- **REQUIRES:** Migration guide for affected systems
- Increment MAJOR version (1.0.0 → 2.0.0)
- Update "Last Amended" date
- Document in Amendment History appendix

**Proposal Template:**
```markdown
## Constitutional Amendment Proposal

### Current Problem
[What's broken/missing/inadequate?]

### Proposed Change
[Exact constitution text to add/modify/remove]

### Rationale
[Why is this change necessary? Evidence?]

### Impact Analysis
- Components affected: [list]
- Breaking changes: [yes/no - specify]
- Migration effort: [low/medium/high]
- Timeline: [when to implement]

### Alternatives Considered
[What other options were evaluated?]

### Risks
[What could go wrong with this change?]
```

### Amendment History

**Version 1.0.0 - 2025-01-16**
- Initial constitution ratified
- Established 8 foundational articles (Tier 1)
- Defined implementation rules (Tier 2)
- Created anti-patterns catalog
- Specified Phase -1 gate enforcement

---

## VI. Success Metrics

This constitution is successful when:

**Quality Metrics:**
- [ ] Zero critical Phase -1 gate violations in production
- [ ] 95%+ specifications pass Gate 1 on first attempt (clarity improving)
- [ ] 90%+ implementations pass Gate 3 on first validation (alignment strong)
- [ ] 85%+ test coverage maintained across all reusable components
- [ ] Zero security vulnerabilities in production (automated scanning catches issues)

**Velocity Metrics:**
- [ ] Specification → working implementation cycle: <48 hours for features
- [ ] Specification refinement iterations: <3 per feature (improving clarity)
- [ ] Time to production deployment: <24 hours post-validation
- [ ] Code regeneration from spec updates: <4 hours (demonstrating spec value)

**Reusability Metrics:**
- [ ] Subagents used across 3+ projects without modification
- [ ] Skills reused 10+ times across different contexts
- [ ] Tools integrated with 5+ different agent systems
- [ ] Zero "copy-paste-modify" reuse (proper semantic versioning)

**Business Outcomes:**
- [ ] Development velocity: 10x increase over traditional code-first (measured)
- [ ] Bug density: 50% reduction due to specification quality
- [ ] Security incidents: 90% reduction due to validation gates
- [ ] Developer satisfaction: 80%+ report preference for SDD-RI over traditional

**Knowledge Capture:**
- [ ] Specifications are the go-to documentation (not code comments)
- [ ] New team members productive in <1 week (clear specs + reusable intelligence)
- [ ] Tribal knowledge captured in specifications and components (not in people's heads)
- [ ] Agent performance predictable and consistent (constitutionally governed)

---

## VII. Conclusion: Excellence Through Constraints

**The Forcing Function Mindset:**

This constitution is designed with forcing functions that prevent mediocrity:
- **Specifications MUST be clear** → No vague requirements accepted
- **Validation MUST be rigorous** → No "looks good" shortcuts
- **Components MUST be reusable** → No project-specific coupling
- **Deployment MUST be production-ready** → No "works on my laptop" excuses

These are not bureaucratic obstacles. These are **excellence mechanisms** that prevent convergence on generic, mediocre patterns. They elevate your work from "AI-generated code" to "systematically excellent systems."

**The Paradigm Shift:**

You are not learning to type code faster. You are learning to think in specifications that machines execute flawlessly. You are not building libraries—you are architecting intelligence that compounds over time. You are not "coding"—you are **designing minds and workflows for machines to inhabit.**

**The Competitive Advantage:**

Two organizations with access to identical AI models will achieve vastly different outcomes. The difference is constitutional governance:
- Organization A: Vague specs → mediocre code → manual patches → technical debt
- Organization B: Clear specs → excellent code → systematic refinement → compounding capability

This constitution puts you in Organization B.

**The Call to Action:**

Master specification design. Build reusable intelligence. Validate rigorously. Deploy professionally. Refuse mediocrity.

**This is not optional. This is the standard.**

---

**Document Version:** 1.0.0  
**Last Updated:** 2025-01-16  
**Next Review:** 2025-04-16 (quarterly)  
**Maintained By:** [Project governance team]  
**Questions/Proposals:** [Contact information or process]
