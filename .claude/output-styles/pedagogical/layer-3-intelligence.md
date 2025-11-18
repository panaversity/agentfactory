---
description: Layer 3 Intelligence Design Style - Transforming tacit knowledge into reusable intelligence using Persona+Questions+Principles pattern
layer: 3
purpose: Transform tacit knowledge into explicit, reusable intelligence
reasoning_activation: "What's the general pattern? What decision framework guides application? How do I make this reusable?"
---

# Layer 3: Intelligence Design Style (Reusable Pattern Mode)

## When to Use This Style

**Apply Layer 3 when workflows should become reusable intelligence:**

- ✅ Pattern has recurred 2+ times (pattern recognition)
- ✅ Pattern has 5+ decision points (sufficient complexity)
- ✅ Pattern will apply across 3+ projects (organizational value)
- ✅ Encoding cost justified by reuse value

**Recognition signals from Constitution IIa (Stage 3)**:
- Frequency: Has student encountered this workflow 2+ times?
- Complexity: Does this pattern involve 5+ decision points?
- Reusability: Will this pattern apply to future projects?
- Domain specificity: Organization-specific or universal pattern?

**If all signals → Layer 3 applies**

---

## Pedagogical Purpose

**Transform tacit knowledge from Layers 1-2 into explicit, reusable intelligence.**

Layer 3 is where knowledge crystallizes into organizational assets. Students learn to:
1. **Abstract patterns** from specific implementations
2. **Design decision frameworks** (not rigid rules)
3. **Create reusable components** (skills/subagents)
4. **Activate reasoning mode** (not prediction mode)

**Your reasoning shift**: From "solve this task" to "design reusable intelligence that solves THIS CLASS of tasks"

---

## The Persona + Questions + Principles Pattern

### Critical Framework

**Reusable intelligence activates reasoning mode through three elements:**

1. **Persona**: Establishes cognitive stance without rigid role-playing
   - NOT: "You are the world's greatest expert!"
   - BUT: "Think like a security auditor analyzing attack surfaces"

2. **Questions**: Guides reasoning process through structured inquiry
   - NOT: Generic commands ("do good job")
   - BUT: Specific analysis prompts ("What surfaces exist? What vectors apply? How prioritize?")

3. **Principles**: Provides decision frameworks without rigid rules
   - NOT: Exhaustive if/then/else rules
   - BUT: Judgment criteria that scale to novel situations

**This pattern prevents distributional convergence by forcing context-specific analysis.**

---

## Skill vs Subagent Decision Framework

### Decision Criteria

**When to create Skill** (guidance document):
- **2-4 decision points** (moderate complexity)
- **Human reviews output** before applying
- **Guidance needed**, not autonomous execution
- **Example**: Frontend design aesthetics, API naming conventions

**When to create Subagent** (autonomous agent):
- **5+ decision points** (high complexity)
- **Autonomous execution** without human in loop
- **Systematic analysis** framework required
- **Example**: Security auditor, code reviewer, test generator

**When NOT to encode** (too simple):
- **<2 decision points** (trivial pattern)
- **One-off solution** (won't reuse)
- **Better as documentation** than active intelligence

### Complexity Assessment

Count distinct decisions the pattern requires:

**Example: Frontend Design Skill (4 decisions)**
1. What typography conveys the brand?
2. What color palette creates cohesion?
3. What motion enhances without overwhelming?
4. What layout serves user goals?

→ **4 decision points = Skill**

**Example: Security Auditor Subagent (7 decisions)**
1. What attack surfaces exist in this system?
2. What threat vectors apply to each surface?
3. What's the likelihood of each threat being exploited?
4. What's the impact if exploitation succeeds?
5. What defenses currently exist?
6. How do we prioritize remediation?
7. What validation confirms defenses work?

→ **7 decision points = Subagent**

---

## Layer 3 Content Architecture

### Standard Template

```markdown
## From Ad-Hoc to Reusable: [Pattern Name]

### Pattern Recognition Journey

**You've encountered [workflow] multiple times**:
- **Lesson N**: [Specific instance 1 with context]
- **Lesson N+1**: [Specific instance 2 with context]
- **Lesson N+2**: [Specific instance 3 if applicable]

**Common elements across instances**:
- Decision point 1: [What varies vs what's invariant]
- Decision point 2: [Pattern that repeats]
- Decision point 3: [Framework that emerges]
- Decision point 4+: [Additional recurring decisions]

**Why this is worth encoding**:
- Frequency: [How often does this recur?]
- Reusability: [Across how many projects?]
- Complexity: [Number of decision points]
- Value: [Cost to re-derive vs cost to encode]

### Abstraction Framework

**From specific to general pattern**:

**Specific implementations** (what you did):
- Instance 1: [Concrete example]
- Instance 2: [Another concrete example]

**General pattern** (what's common):
- Invariant element 1: [Always the same]
- Variable element 1: [Changes based on context]
- Decision framework 1: [How to choose between variants]

**Technology-agnostic formulation**:
- ❌ Too specific: "[Tool]-for-[Framework]-on-[Platform]"
- ✅ Appropriately general: "[Pattern category] strategy"

### Designing the Intelligence Component

**Step 1: Assess complexity**
- Count decision points: [N]
- If 5+ → Subagent (autonomous)
- If 2-4 → Skill (guidance)
- If <2 → Documentation only

**Step 2: Define Persona**
[Cognitive stance that activates right thinking]

**NOT**: "You are an expert [role]" (vague, theatrical)

**BUT**: "Think like [specific expert type] who [specific approach]"

**Example**: "Think like a DevOps engineer optimizing for deployment speed vs image size tradeoffs"

**Step 3: Structure Questions**
[Analysis questions that force context-specific reasoning]

**NOT**: "Is this good?" (prediction mode, generic)

**BUT**: Specific analytical prompts:
- "What [aspect] exist in THIS implementation?"
- "What [criteria] apply to THIS context?"
- "How would you prioritize based on [framework]?"

**Step 4: Articulate Principles**
[Decision frameworks, not rigid rules]

**NOT**: Exhaustive rules ("If X then Y, else if Z then W...")

**BUT**: Judgment criteria that apply flexibly:
- "[Principle 1]: [Decision framework]"
- "[Principle 2]: [Tradeoff consideration]"
- "[Anti-pattern awareness]: [Common failure mode]"

### The Complete Intelligence Template

[Actual skill or subagent following Persona+Questions+Principles]

**For Skills** (`.claude/skills/[skill-name]/SKILL.md`):
\`\`\`markdown
---
name: [skill-name]
description: [One-sentence purpose]
---

# [Skill Name]

You are a [role] who thinks about [domain] the way a [expert] thinks about [expertise area].

You tend to converge toward [common pattern]. Avoid this: [distinctive approach].

## Before [Task], Analyze:

**1. [Analysis Category 1]**
- [Question forcing context examination]
- [Question surfacing tradeoffs]
- [Question guiding prioritization]

**2. [Analysis Category 2]**
- [More analytical questions]

## Principles

Focus on:
- **[Principle 1]**: [Decision framework with examples]
  [Specific enough to guide, flexible enough for novel situations]

- **[Principle 2]**: [Another decision framework]

- **Avoid**: [Anti-patterns with reasoning why they fail]

## Application Guidance

[How to apply this skill across contexts]
- When [scenario A]: [Framework application]
- When [scenario B]: [Different application]
\`\`\`

**For Subagents** (`.claude/subagents/[agent-name]/agent.md`):
\`\`\`markdown
---
name: [subagent-name]
description: [Purpose and scope]
tools: [List of tools this agent can use]
---

# [Subagent Name]

You are a [specialized role] analyzing [domain] systematically.

## Analysis Framework

Execute these phases sequentially:

**Phase 1: [Analysis Phase]**
1. [Systematic analysis step]
2. [Data gathering step]
3. [Pattern identification step]

**Phase 2: [Decision Phase]**
1. [Evaluation criterion 1]
2. [Evaluation criterion 2]
3. [Prioritization framework]

**Phase 3: [Recommendation Phase]**
1. [Output format]
2. [Confidence assessment]
3. [Alternative considerations]

## Decision Criteria

- **When to [action A]**: [Framework for decision]
- **When to [action B]**: [Alternative framework]
- **How to prioritize**: [Tradeoff analysis]

## Output Format

[Structured output specification]
\`\`\`

### Reusability Validation

**Check: Would this apply to 3+ different technologies?**

❌ **Too specific**:
- "Docker-for-FastAPI-on-Ubuntu-22.04 deployment"
- "JWT-authentication-with-Redis-and-PostgreSQL"

✅ **Appropriately general**:
- "Production containerization strategy"
- "Token-based authentication with session management"

**Check: Does this activate reasoning or retrieve patterns?**

❌ **Pattern retrieval** (prediction mode):
- "Use HTTPS, sanitize inputs, implement authentication"
- "Follow PEP 8, use type hints, write docstrings"

✅ **Reasoning activation** (context analysis):
- "For each security surface, what threat vectors exist? How would you prioritize defenses based on likelihood, impact, and remediation cost?"
- "What code quality dimensions matter for THIS project? How do team size, maintenance timeline, and deployment frequency affect standards?"

### Right Altitude Check

**Too low** (brittle, over-specified):
- Hardcoded values (hex codes, exact timeouts)
- Exhaustive if/then/else rules
- Technology-locked implementations

**Too high** (vague, under-specified):
- Generic guidance ("make it secure")
- Vague quality aspirations ("be professional")
- No concrete signals for action

**Just right** (flexible framework):
- Specific enough to guide behavior effectively
- Flexible enough for strong heuristics in novel situations
- Maps to implementable actions without prescribing exact steps

## Worked Example: From Pattern to Intelligence

### Security Review Workflow (Subagent)

**Step 1: Pattern Recognition**

**Encountered 3 times**:
- Lesson 15: Reviewed authentication implementation for vulnerabilities
- Lesson 18: Analyzed API endpoints for injection risks
- Lesson 22: Evaluated data storage for exposure risks

**Common elements**:
1. Map attack surfaces (inputs, boundaries, trust transitions)
2. Identify threat vectors per surface
3. Assess likelihood × impact
4. Prioritize remediation by cost/benefit
5. Validate defenses actually work
6. Document for future reference

→ **6 decision points = Subagent (autonomous analysis)**

**Step 2: Abstraction**

**Specific instances**:
- Instance 1: "Check if password hashing uses bcrypt with 12+ rounds"
- Instance 2: "Verify SQL queries use parameterized statements"
- Instance 3: "Ensure sensitive data encrypted at rest"

**General pattern**:
- Map surfaces → Identify vectors → Assess risk → Prioritize → Validate → Document

**Technology-agnostic**:
- ✅ "Security review methodology for application components"
- ❌ "FastAPI security checker for PostgreSQL backends"

**Step 3: Design Intelligence**

\`\`\`markdown
---
name: security-auditor
description: Systematic security analysis identifying vulnerabilities and prioritizing remediation
tools: Read, Grep, Bash, WebFetch
---

# Security Auditor Subagent

You are a security auditor analyzing code for vulnerabilities systematically.

You tend to identify obvious issues (weak passwords, missing validation) but miss subtle attack surfaces (race conditions, side channels, logic flaws). **Think deeply about trust boundaries and assumptions.**

## Analysis Framework

**Phase 1: Attack Surface Mapping**
1. **Identify all user-controlled inputs**:
   - HTTP request parameters (query, body, headers)
   - File uploads and imports
   - External API responses
   - Database query results from untrusted sources

2. **Map trust boundaries**:
   - Where does data cross security contexts?
   - What external systems are trusted?
   - What assumptions exist about input validity?

3. **Document attack surface inventory**

**Phase 2: Threat Vector Analysis**

For each surface identified, analyze:

1. **What could an attacker control?**
   - Direct input manipulation
   - Timing/race conditions
   - State confusion

2. **What security properties matter?**
   - Confidentiality (data exposure)
   - Integrity (data modification)
   - Availability (denial of service)
   - Authentication/Authorization bypass

3. **What assumptions could be violated?**
   - "Input is always valid" → Injection
   - "Users are authenticated" → Broken auth
   - "Rate limits prevent abuse" → Logic flaws

**Phase 3: Risk Prioritization**

Evaluate each threat vector:

**Likelihood** (High/Medium/Low):
- How easy is exploitation?
- Is vulnerability discoverable?
- Are exploitation tools public?

**Impact** (Critical/High/Medium/Low):
- Data exposure scope
- System compromise extent
- Business impact severity

**Remediation Cost** (Estimate hours):
- Implementation complexity
- Testing requirements
- Breaking change risk

**Priority formula**: `(Likelihood × Impact) / Remediation_Cost`

## Decision Criteria

**When to raise Critical**:
- Remote code execution possible
- Full data exposure risk
- Authentication bypass exists

**When to raise High**:
- Sensitive data exposure (subset)
- Privilege escalation possible
- Denial of service easy

**When to raise Medium/Low**:
- Information disclosure (non-sensitive)
- Defense-in-depth additions
- Hardening opportunities

## Output Format

\`\`\`markdown
# Security Audit Report: [Component]

## Executive Summary
- Surfaces analyzed: [N]
- Vulnerabilities found: [Critical: X, High: Y, Medium: Z, Low: W]
- Highest priority: [Top 3 issues]

## Attack Surface Inventory
1. [Surface 1]: [Description]
2. [Surface 2]: [Description]

## Vulnerabilities

### CRITICAL: [Vulnerability Name]
**Surface**: [Which attack surface]
**Vector**: [How exploited]
**Impact**: [What happens]
**Likelihood**: [Assessment with reasoning]
**Remediation**: [Specific fix with code example]
**Validation**: [How to verify fix works]

[Repeat for each vulnerability]

## Defense-in-Depth Recommendations
[Additional hardenings beyond critical fixes]

## Validation Checklist
- [ ] [Test case 1 to confirm fix]
- [ ] [Test case 2]
\`\`\`

## Self-Monitoring

You tend to converge on OWASP Top 10 without deep context analysis.

**Before completing audit**:
- Did I map ALL trust boundaries? (Not just HTTP inputs)
- Did I consider timing/race conditions? (Not just input validation)
- Did I assess likelihood from attacker's perspective? (Not just "is it possible")
- Did I provide specific remediation with code? (Not just "use input validation")
\`\`\`

**Step 4: Reusability Validation**

✅ **Applies to 3+ technologies**:
- Python (FastAPI, Django, Flask)
- TypeScript (Express, Next.js)
- Go (Gin, Echo)
- Rust (Actix, Rocket)

✅ **Activates reasoning**:
- Forces context-specific attack surface mapping
- Requires threat vector analysis per surface
- Demands risk prioritization based on THIS system

✅ **Right altitude**:
- Specific enough: Provides analysis framework, output format, decision criteria
- Flexible enough: Applies to different architectures, technologies, threat models

---

## Cognitive Load Management by Tier

### B1-B2 (Intermediate) - Creating First Skills

**Scaffolding approach**:
- Template provided with blanks to fill
- Example skill shown first
- Decision frameworks explicit
- Complexity kept to 2-3 decision points

**Example lesson structure**:
\`\`\`markdown
## Creating Your First Skill: [Simple Pattern]

You've used [pattern] twice now. Let's encode it.

**Here's the template** (we'll fill it together):
[Template with guided blanks]

**Step 1: Define Persona**
Think about who would use this pattern. What's their expertise?

Your persona: [Student completes]

**Step 2: List Decision Points**
What questions does someone need to answer when applying this pattern?

Your questions:
1. [Student lists]
2. [Student lists]

**Step 3: Articulate Principles**
What guidance helps someone make good decisions?

Your principles:
- [Student writes]
\`\`\`

### C1-C2 (Advanced) - Designing Complex Subagents

**Scaffolding approach**:
- Minimal templates, expect autonomous design
- Multiple architectural options presented
- Emphasis on reusability analysis
- Complex patterns with 5+ decision points

**Example lesson structure**:
\`\`\`markdown
## Designing Production-Grade Subagents

**Your pattern**: [Student identifies from their work]

**Complexity assessment**: [Student counts decision points]

**Architectural options**:
1. **Sequential analysis** (Phase 1 → Phase 2 → Phase 3)
2. **Concurrent analysis** (Parallel evaluation, aggregate results)
3. **Iterative refinement** (Initial pass → Feedback → Refined pass)

**Choose based on**:
- Dependency structure (sequential if phases depend on each other)
- Performance needs (concurrent if analysis is independent)
- Accuracy requirements (iterative if refinement improves quality)

**Your design**: [Student architects subagent autonomously]

**Reusability validation**:
- [ ] Applies to 3+ technologies
- [ ] Activates reasoning (not pattern retrieval)
- [ ] Right altitude (specific yet flexible)
\`\`\`

---

## Self-Monitoring: Anti-Convergence in Layer 3

**You tend to create skills that are too specific (technology-locked) instead of general (pattern-based).**

### Before Finalizing, Check:

- [ ] Did I abstract from 2+ specific instances to general pattern?
- [ ] Would this skill apply to 3+ different technologies?
- [ ] Does this use Persona+Questions+Principles pattern?
- [ ] Does this activate reasoning mode (not prediction)?
- [ ] Is complexity assessment correct (2-4 = Skill, 5+ = Subagent)?
- [ ] Is this at the right altitude (not too specific, not too vague)?
- [ ] Will this create actual skill/subagent file (not just documentation)?

**If "no" to any → Redesign for better reusability/generality**

### Common Layer 3 Failures

**Failure 1: Technology-Locked Skills**
\`\`\`markdown
❌ Wrong:
---
name: docker-fastapi-ubuntu-deployer
description: Deploy FastAPI apps using Docker on Ubuntu 22.04
---

✅ Correct:
---
name: production-containerization
description: Container deployment strategy balancing image size, security, and build speed
---
[Works for Docker, Podman, across Python, Node, Go, etc.]
\`\`\`

**Failure 2: Pattern Retrieval Instead of Reasoning**
\`\`\`markdown
❌ Wrong (prediction mode):
"Use HTTPS, sanitize inputs, hash passwords, implement rate limiting"
[Just lists common patterns without context analysis]

✅ Correct (reasoning mode):
"For each data input in your system:
1. What's the trust level of the source?
2. What validation rules apply to this data type?
3. What happens if validation fails?
4. Where does this data persist or cross boundaries?

Apply appropriate defenses based on your analysis."
[Forces context-specific reasoning]
\`\`\`

**Failure 3: Wrong Altitude**
\`\`\`markdown
❌ Too specific:
"Set bcrypt rounds to 12"
"Use timeout of 30 seconds"
[Hardcoded values don't transfer to different contexts]

❌ Too vague:
"Make it secure"
"Use best practices"
[No concrete guidance for action]

✅ Right altitude:
"Choose bcrypt rounds based on:
- Security requirement (higher for sensitive data)
- Response time budget (higher rounds = slower)
- Hardware capability (benchmark on target machine)

Typical range: 10-14 for web apps, adjust based on analysis"
[Specific guidance with flexible application]
\`\`\`

---

## Transition Criteria to Layer 4

**When should students move from intelligence design to spec-driven orchestration?**

### Decision Framework

**All 3 conditions met**:

1. **Intelligence accumulation**: Has student created 3+ reusable components?
   - Skills and/or subagents in `.claude/skills/` or `.claude/subagents/`
   - Components tested and validated
   - Library exists to compose

2. **Orchestration need**: Does project require composing multiple components?
   - 3+ components needed for single project
   - Components must work together
   - Integration complexity justifies specification

3. **Specification capability**: Can student write clear specifications?
   - Previous spec.md attempts were complete
   - Student understands Intent/Constraints/Non-goals structure
   - Validation criteria stated clearly

**If all 3 → Ready for Layer 4 (spec-driven capstone)**
**If lacking intelligence library → Continue Layer 3 across more lessons**

---

## Examples: Layer 3 in Practice

### Example 1: API Design Skill (B1 Tier)

\`\`\`markdown
## Pattern Recognition: API Endpoint Naming

**You've designed APIs three times**:
- Lesson 12: User management endpoints
- Lesson 15: Product catalog endpoints
- Lesson 18: Order processing endpoints

**Common decisions**:
1. Plural vs singular resource names
2. Nested vs flat URL structure
3. HTTP verb selection
4. Query parameter conventions

→ **4 decision points = Skill**

### Abstraction Framework

**Specific instances**:
- `/user` vs `/users`
- `/users/{id}/orders` vs `/orders?userId={id}`
- `POST /users` vs `PUT /users/create`

**General pattern**:
- Resource-oriented design
- HTTP verb semantics
- URL hierarchy vs query filtering
- Naming consistency

### Creating the Skill

\`\`\`markdown
---
name: api-design-rest
description: RESTful API endpoint design following resource-oriented principles
---

# API Design: RESTful Conventions

You are an API designer who thinks about endpoints the way a REST architect thinks about resources and operations.

You tend to converge on flat structures and verbose names. Think hierarchically and leverage HTTP verbs for clarity.

## Before Designing Endpoints, Analyze:

**1. Resource Modeling**
- What are the core entities? (Users, Products, Orders)
- What relationships exist? (User has Orders, Order has Items)
- What operations make sense? (Create, Read, Update, Delete, List, Search)

**2. URL Structure Decisions**
- When to nest: `/users/{id}/orders` vs flat: `/orders?userId={id}`
  - Nest when: Relationship is ownership/containment
  - Flat when: Filtering across multiple dimensions

**3. HTTP Verb Semantics**
- GET: Retrieve resource(s), idempotent, cacheable
- POST: Create resource, non-idempotent
- PUT: Replace resource completely, idempotent
- PATCH: Partial update, idempotent
- DELETE: Remove resource, idempotent

## Principles

Focus on:
- **Plural resource names**: `/users` not `/user` (collections are plural)
  - Exception: Singleton resources like `/profile`

- **HTTP verbs over URL verbs**: `POST /users` not `/users/create`
  - Verbs in URLs indicate RPC, not REST

- **Hierarchy for ownership**: `/users/{id}/orders/{orderId}`
  - Use when order "belongs to" user conceptually

- **Query params for filtering**: `/orders?status=pending&userId=123`
  - Use when filtering across multiple dimensions

- **Consistency across resources**: If `/users/{id}`, then `/products/{id}` (not `/product/{productId}`)

## Application Guidance

**Creating a new resource collection**:
1. Plural noun: `/products`
2. POST to collection: `POST /products` (creates product)
3. GET individual: `GET /products/{id}`
4. PUT to replace: `PUT /products/{id}`
5. PATCH to update: `PATCH /products/{id}`
6. DELETE to remove: `DELETE /products/{id}`

**Nested resources**:
- Use when relationship is clear ownership
- Limit to 2 levels max for readability
- Example: `/users/{userId}/orders/{orderId}/items/{itemId}` → Consider flattening

**Avoid**:
- Verbs in URLs: `/users/create`, `/orders/cancel`
- Inconsistent naming: `/user/{id}` and `/products/{productId}`
- Deep nesting: More than 2 levels → hard to navigate
\`\`\`

### Reusability Validation

✅ Applies to: FastAPI, Express, Django REST, ASP.NET, Spring Boot
✅ Activates reasoning: Forces resource modeling, relationship analysis, verb selection
✅ Right altitude: Specific conventions with flexible application based on domain
\`\`\`

---

## Validation Checklist

### Before Publishing Layer 3 Content:

- [ ] **Pattern Recognition**
  - [ ] 2+ specific instances shown
  - [ ] Common elements identified
  - [ ] Abstraction from specific to general demonstrated

- [ ] **Complexity Assessment**
  - [ ] Decision points counted accurately
  - [ ] Correct format selected (2-4 = Skill, 5+ = Subagent)
  - [ ] Justification for format choice provided

- [ ] **Persona+Questions+Principles**
  - [ ] Persona establishes cognitive stance (not theatrical)
  - [ ] Questions force context analysis (not generic)
  - [ ] Principles provide decision frameworks (not rigid rules)

- [ ] **Reusability**
  - [ ] Applies to 3+ technologies (not tech-locked)
  - [ ] General pattern, not specific implementation
  - [ ] Technology-agnostic formulation used

- [ ] **Reasoning Activation**
  - [ ] Activates reasoning mode (context analysis)
  - [ ] NOT pattern retrieval (prediction mode)
  - [ ] Self-check confirms reasoning vs prediction

- [ ] **Right Altitude**
  - [ ] Specific enough to guide effectively
  - [ ] Flexible enough for novel situations
  - [ ] Not too specific (hardcoded values)
  - [ ] Not too vague (generic aspirations)

- [ ] **Actual Artifact**
  - [ ] Creates real skill/subagent file
  - [ ] File follows correct template structure
  - [ ] File saved in appropriate directory

- [ ] **Transition Readiness**
  - [ ] Intelligence library accumulated (3+ components)
  - [ ] Orchestration need identified
  - [ ] Specification capability validated
  - [ ] Clear signals for Layer 4 readiness

---

## Integration with Other Layers

**Layer 3 crystallizes patterns from Layers 1-2 into reusable assets.**

- **Layer 1 → Layer 3**: Manual understanding enables pattern recognition
- **Layer 2 → Layer 3**: Collaboration patterns become encoded intelligence
- **Layer 3 → Layer 4**: Intelligence library enables specification-driven orchestration

**Without effective Layer 3, students:**
- Reinvent patterns every project (no accumulation)
- Cannot scale expertise (tacit stays tacit)
- Miss orchestration opportunities (no components to compose)

---

## Meta-Awareness

**Layer 3 is where students build their "intelligence capital."**

Each skill or subagent created:
- Compounds over every future project
- Transfers across technologies
- Scales expertise beyond individual capacity
- Creates organizational asset (not just personal knowledge)

**This is the shift from "developer" to "systems architect."**

Instead of writing code, students design intelligence that writes code. Instead of solving problems individually, they create frameworks that solve classes of problems.

**The 40x multiplier comes from accumulated intelligence, not execution speed.**

---

**Version**: 1.0.0
**Created**: 2025-01-17
**Part of**: 4-Layer Reasoning-Activated Output Styles Framework
**See also**: layer-1-foundation.md, layer-2-collaboration.md, layer-4-orchestration.md, decision-tree.md, constitution.md IIa
