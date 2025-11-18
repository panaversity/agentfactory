---
description: Layer 4 Orchestration Style - Spec-driven mode for orchestrating accumulated intelligence at scale
layer: 4
purpose: Design systems through specifications that compose accumulated intelligence
reasoning_activation: "What specification is sufficient? How do I compose intelligence? How do I validate spec↔implementation alignment?"
---

# Layer 4: Orchestration Style (Spec-Driven Mode)

## When to Use This Style

**Apply Layer 4 when students ready for specification-driven capstone projects:**

- ✅ Student has accumulated 3+ reusable components (intelligence library exists)
- ✅ Project requires orchestrating multiple components (3+ skills/subagents)
- ✅ Complexity justifies specification-first approach (10+ coordinated operations)
- ✅ Student can write clear specifications (capability validated)

**Recognition signals from Constitution IIa (Stage 4)**:
- Has student accumulated 3+ reusable components?
- Does project require composing multiple components?
- Is project complexity 10+ coordinated operations?
- Can student write clear specifications?

**If all signals → Layer 4 applies**

---

## Pedagogical Purpose

**Design systems through specifications that orchestrate accumulated intelligence at scale.**

Layer 4 is NOT about writing more code—it's about:
1. **Specifying intent** clearly before implementation
2. **Composing intelligence** from accumulated components
3. **Orchestrating execution** through specification-driven workflows
4. **Validating alignment** between spec and implementation

**Your reasoning shift**: From "implement feature" to "validate specification sufficiency for orchestrated execution"

---

## The Specification Quality Framework

### Critical Components

**Every Layer 4 specification MUST include**:

1. **Intent** (WHAT, not HOW)
   - System purpose and goals
   - Success criteria (measurable, falsifiable)
   - Business value or learning outcome

2. **Constraints** (Explicit boundaries)
   - Technical constraints (technology, platform, scale)
   - Architectural constraints (patterns, standards)
   - Resource constraints (time, complexity, scope)

3. **Non-Goals** (Prevent scope creep)
   - What we're NOT building
   - Why those features are excluded
   - Where they might belong later

4. **Acceptance Criteria** (Testable validation)
   - Specific, measurable outcomes
   - Verification methods
   - Definition of "done"

**Without complete specification, implementation will be unpredictable.**

---

## Layer 4 Content Architecture

### Standard Template

```markdown
## Capstone Project: [Project Name]

[CONTEXT: 2-3 paragraphs establishing why this project matters, what skills it synthesizes, how it demonstrates mastery]

This project brings together everything you've learned in Chapters [N-M]:
- Layer 1: [Foundational concepts mastered]
- Layer 2: [Collaboration patterns practiced]
- Layer 3: [Intelligence components created]
- Layer 4: [Now orchestrating at scale]

### Intelligence Inventory

**Accumulated components from previous lessons**:

| Component | Type | Purpose | Decision Points | Created in |
|-----------|------|---------|----------------|------------|
| `[component-1]` | Skill | [Decision framework for domain] | 3 | Lesson N |
| `[component-2]` | Subagent | [Autonomous capability] | 6 | Lesson N+1 |
| `[component-3]` | Skill | [Guidance for pattern] | 4 | Lesson N+2 |

**Ready to compose**: You now orchestrate intelligence, not write from scratch.

**What this means**:
- DON'T re-implement patterns (use components)
- DO compose components through specifications
- DO validate that components integrate correctly

### Specification-First Workflow

**Critical principle**: Write spec.md BEFORE any implementation.

**Why this matters**:
- Specification clarifies intent before execution
- Components can't integrate without clear specification
- Validation requires predefined success criteria
- Iteration is cheaper in specification than code

**The workflow**:
1. Write specification (this section)
2. Identify which components apply (composition map)
3. Review specification quality (validation)
4. Implement using components (not from scratch)
5. Validate against acceptance criteria

### Project Specification

**Create this as `specs/[project-name]/spec.md`**

\`\`\`markdown
# Project: [Name]

## Intent

**Purpose**: [What system should accomplish - 2-3 sentences]

**Goals**:
- [Measurable goal 1]
- [Measurable goal 2]
- [Measurable goal 3]

**Success criteria**:
- [Specific, testable outcome 1]
- [Specific, testable outcome 2]
- [Specific, testable outcome 3]

## Constraints

**Technical**:
- Use [technology/framework] because [rationale]
- Must run on [platform] with [resources]
- Performance target: [specific metric]

**Architectural**:
- Follow [pattern] for [aspect]
- Integrate with [existing component] at [interface]
- Maintain [quality standard]

**Resource**:
- Complete in [timeframe]
- Complexity limit: [scope boundary]
- Dependencies: [what must exist before this]

## Non-Goals

**Explicitly NOT building**:
- [Feature X] — Out of scope, defer to [future version/different project]
- [Capability Y] — Separate concern, belongs in [other system]
- [Integration Z] — Not required for core functionality

**Rationale**: Focus on [core value], avoid [scope creep pattern]

## Acceptance Criteria

How we know this is complete and correct:

- [ ] **[Criterion 1]**: [Specific, measurable, testable]
  - Verification method: [How to validate]
  - Success signal: [What we observe when passing]

- [ ] **[Criterion 2]**: [Another specific criterion]
  - Verification method: [Test approach]
  - Success signal: [Observable outcome]

- [ ] **[Criterion 3]**: [More criteria as needed]

**Definition of Done**: All acceptance criteria pass validation
\`\`\`

### Specification Quality Validation

**Before implementation, check**:

**Intent Clarity** (answers WHAT):
- [ ] Can AI read this and make informed implementation decisions?
- [ ] Are success criteria measurable? (Not vague like "works well")
- [ ] Is purpose clear without implementation details?

**Constraint Definition** (answers BOUNDARIES):
- [ ] What's explicitly EXCLUDED (non-goals)?
- [ ] Are architectural constraints specific?
- [ ] Are resource limits defined?

**Acceptance Criteria** (answers DONE):
- [ ] Are criteria specific and testable?
- [ ] Can we verify each criterion objectively?
- [ ] Do criteria cover all goals?

**If any "no" → Refine specification before implementation**

### Intelligence Composition Map

**Which components handle which requirements?**

\`\`\`markdown
## Component Mapping

[Diagram or table showing requirement → component mapping]

**Example**:

| Requirement | Intelligence Component | Responsibility | Why This Component |
|-------------|----------------------|----------------|-------------------|
| User authentication | `jwt-auth` subagent | Token generation, validation | Autonomous security logic |
| API endpoint design | `api-design-rest` skill | URL structure, verb selection | Consistency decisions |
| Error handling | `error-resilience` skill | Retry, logging, graceful failure | Defense-in-depth guidance |
| Data validation | `input-validation` subagent | Schema enforcement, sanitization | Systematic validation |

**Interfaces between components**:
- `jwt-auth` → `api-design-rest`: Protected endpoints follow REST conventions
- `error-resilience` → all components: Error handling patterns apply universally
- `input-validation` → `jwt-auth`: Validate credentials before authentication attempt

**Gap analysis**:
- [ ] All requirements mapped to components?
- [ ] Any missing capabilities needing new components?
- [ ] Do component interfaces align?
\`\`\`

### Orchestration Patterns

**How components work together**:

**Pattern 1: Sequential Composition**
\`\`\`
Request → input-validation → jwt-auth → api-logic → response
         (component A)      (component B)   (component C)

Output of A feeds into B, output of B feeds into C
\`\`\`

**Pattern 2: Concurrent Analysis**
\`\`\`
Requirement → ┌─ security-auditor (threat analysis)
              ├─ performance-reviewer (latency check)
              ├─ code-quality (style validation)
              └─ integration-tester (e2e verification)
                        ↓
              Aggregate results → Final validation
\`\`\`

**Pattern 3: Iterative Refinement**
\`\`\`
Spec → Component generates → Human validates → Feedback → Component refines → Validate → Done
\`\`\`

**Choose pattern based on**:
- Sequential: When components have data dependencies
- Concurrent: When analyses are independent
- Iterative: When refinement improves quality

### Implementation Guidance

**NOT prescriptive HOW, but orchestration approach**:

**Step 1: Set up composition environment**
- Load all required components
- Verify component versions compatible
- Test component interfaces

**Step 2: Execute sequential compositions**
- Components with dependencies run in order
- Output validation between stages
- Error handling propagates correctly

**Step 3: Run concurrent analyses**
- Independent components execute in parallel
- Results aggregate correctly
- Conflicts surface for resolution

**Step 4: Iterate for refinement**
- Initial pass with components
- Human review for gaps/issues
- Component re-execution with feedback
- Convergence to specification

**Step 5: Validate against acceptance criteria**
- Run all verification methods
- Check all success signals
- Document results

### Validation Protocol

**Post-implementation checks**:

**Specification Alignment**:
- [ ] Does implementation fulfill intent?
- [ ] Do outputs match acceptance criteria?
- [ ] Were constraints respected?
- [ ] Were non-goals avoided?

**Component Utilization**:
- [ ] Were reusable components applied (not reinvented)?
- [ ] Did components integrate correctly?
- [ ] Were interfaces compatible?
- [ ] Did orchestration pattern work as expected?

**Quality Standards**:
- [ ] Does implementation meet quality bar?
- [ ] Are edge cases handled?
- [ ] Is error handling robust?
- [ ] Is documentation complete?

**Reasoning Transparency**:
- [ ] Can student explain design decisions?
- [ ] Are tradeoffs articulated?
- [ ] Is component selection justified?

### Reflection and Meta-Learning

**After project completion**:

**What did this project teach about systems design?**
- [Insight about composition patterns]
- [Learning about component interfaces]
- [Understanding of specification quality impact]

**What would you do differently?**
- [Specification improvements]
- [Component design refinements]
- [Orchestration pattern adjustments]

**What new components emerged as valuable?**
- [Patterns worth encoding for next project]
- [Gaps in intelligence library]
- [Reusability opportunities]

**How did specification-first improve outcomes?**
- [Comparison to ad-hoc implementation]
- [Iteration efficiency gains]
- [Quality improvements from upfront clarity]
```

---

## Cognitive Load Management by Tier

### B2-C1 (Professional) - Production Capstones

**Load characteristics**:
- No artificial concept limits
- Professional-level complexity expected
- Minimal scaffolding, assume autonomy
- Systems thinking required

**Scaffolding approach**:
- Specification templates provided, students customize
- Component composition left to student design
- Validation criteria student-defined
- Professional decision-making expected

**Example structure**:
\`\`\`markdown
## Capstone: Production API Gateway

**Your challenge**: Design specification for API gateway orchestrating:
- Authentication (`jwt-auth` subagent)
- Rate limiting (`rate-limiter` subagent)
- Request validation (`input-validation` subagent)
- Logging (`observability` skill)
- Error handling (`error-resilience` skill)

**Constraints**:
- Must handle 1000 req/sec
- Sub-50ms latency overhead
- Graceful degradation on auth service failure

**Your tasks**:
1. Write complete specification (Intent, Constraints, Non-goals, Acceptance)
2. Map requirements to components
3. Design orchestration pattern
4. Implement using components
5. Validate against your own acceptance criteria

**No step-by-step guidance** — you're architecting this system.
\`\`\`

---

## Self-Monitoring: Anti-Convergence in Layer 4

**You tend to accept vague specifications and "fill in gaps" with assumptions.**

**This is WRONG. Vague specs produce unpredictable implementations.**

### Before Proceeding, Check:

- [ ] Does spec answer: Intent (WHAT)? Constraints (BOUNDARIES)? Non-goals (NOT)? Acceptance (DONE)?
- [ ] Can I execute without making architectural assumptions?
- [ ] Are acceptance tests specific and testable?
- [ ] Are 3+ components identified for composition?
- [ ] Is orchestration pattern selected and justified?
- [ ] Will components integrate without modification?

**If "no" to any → Request specification refinement from student**

### Common Layer 4 Failures

**Failure 1: Vague Specifications**
\`\`\`markdown
❌ Wrong:
"Build authentication using JWT tokens. Make it secure."

- Intent unclear: What kind of auth? For what system?
- No constraints: What security standard? What performance target?
- No non-goals: OAuth? MFA? Password reset?
- No acceptance: How do we know "secure"?

✅ Correct:
**Intent**: Build JWT-based authentication for REST API
**Constraints**:
- bcrypt with 12+ rounds for password hashing
- Access tokens: 15min expiry, Refresh tokens: 7 days
- Rate limit: 5 login attempts per minute per IP
**Non-goals**: OAuth/SSO (defer v2), MFA (separate feature), Password reset (separate)
**Acceptance**:
- [ ] User registration creates account in DB
- [ ] Invalid credentials return 401 with rate limiting
- [ ] Valid credentials return JWT pair (access + refresh)
- [ ] Expired access token rejected, valid refresh issues new access
\`\`\`

**Failure 2: Reinventing Instead of Composing**
\`\`\`markdown
❌ Wrong:
Student writes new authentication logic from scratch despite having `jwt-auth` subagent

✅ Correct:
"Use `jwt-auth` subagent for token generation/validation. Configure with:
- Access token TTL: 900 seconds
- Refresh token TTL: 604800 seconds
- Secret: Load from environment variable"
\`\`\`

**Failure 3: Missing Component Integration**
\`\`\`markdown
❌ Wrong:
Components listed but no integration plan:
- `jwt-auth` subagent
- `rate-limiter` subagent
- `api-design-rest` skill

(How do they work together?)

✅ Correct:
**Sequential composition**:
1. Request arrives → `api-design-rest` validates endpoint structure
2. → `rate-limiter` checks request allowance
3. → If allowed, `jwt-auth` validates token
4. → If valid, proceed to application logic

**Interfaces**:
- `rate-limiter` needs identifier (IP + username)
- `jwt-auth` needs token from Authorization header
- All errors formatted per `api-design-rest` conventions
\`\`\`

---

## Worked Example: E-Commerce Checkout

### Intelligence Inventory

**From previous lessons**:
- Lesson 15: `payment-processing` skill (gateway integration patterns)
- Lesson 18: `inventory-management` subagent (stock validation, reservation)
- Lesson 20: `transaction-coordinator` skill (distributed transaction patterns)
- Lesson 22: `notification-sender` subagent (email/SMS delivery)
- Lesson 24: `error-resilience` skill (retry, circuit breaker, fallback)

### Project Specification

\`\`\`markdown
# Project: E-Commerce Checkout Flow

## Intent

**Purpose**: Implement production-grade checkout that coordinates payment, inventory, and notifications with resilience to partial failures.

**Goals**:
- Process customer payment through payment gateway
- Reserve inventory items (prevent overselling)
- Send order confirmation to customer
- Handle partial failures gracefully (payment succeeds but email fails)

**Success criteria**:
- 99.9% of successful payments result in inventory reservation
- Customers receive confirmation within 5 minutes of payment
- No double-charging even if process retries

## Constraints

**Technical**:
- Payment gateway: Stripe API (PCI compliance requirement)
- Inventory: PostgreSQL with row-level locking
- Notifications: SendGrid for email, Twilio for SMS
- Transaction timeout: 30 seconds total

**Architectural**:
- Use Saga pattern for distributed transaction (not 2PC)
- Idempotency keys for payment gateway (prevent double-charge)
- Circuit breaker for notification service (degrade gracefully)

**Resource**:
- Must complete implementation in 4 hours
- Cannot modify payment gateway API (external service)
- Existing inventory system must remain compatible

## Non-Goals

**NOT building**:
- Multi-currency support (USD only for v1)
- Subscription/recurring payments (one-time only)
- Guest checkout (authenticated users only)
- Cart abandonment recovery (separate feature)

**Rationale**: Focus on reliable core flow, avoid scope creep into edge cases

## Acceptance Criteria

- [ ] **Successful purchase end-to-end**:
  - Verification: Make test purchase with test credit card
  - Success signal: Payment charged, inventory decremented, email received

- [ ] **Idempotency protection**:
  - Verification: Submit same payment request twice rapidly
  - Success signal: Only one charge, inventory decremented once

- [ ] **Inventory reservation accuracy**:
  - Verification: Concurrent purchases of last item in stock
  - Success signal: Only one purchase succeeds, other gets out-of-stock

- [ ] **Graceful notification failure**:
  - Verification: Kill notification service mid-checkout
  - Success signal: Payment and inventory succeed, notification queued for retry

- [ ] **Payment failure rollback**:
  - Verification: Submit purchase with invalid card
  - Success signal: No inventory reserved, customer sees clear error
\`\`\`

### Intelligence Composition Map

| Requirement | Component | Responsibility |
|-------------|-----------|----------------|
| Payment processing | `payment-processing` skill | Stripe integration, idempotency patterns |
| Inventory reservation | `inventory-management` subagent | Stock validation, reservation, rollback |
| Transaction coordination | `transaction-coordinator` skill | Saga pattern, compensation logic |
| Notification | `notification-sender` subagent | Email/SMS delivery with retry |
| Error handling | `error-resilience` skill | Circuit breaker, retry, fallback |

**Orchestration pattern**: Sequential with compensation (Saga)

\`\`\`
1. Validate inventory (inventory-management)
   ↓ Success
2. Charge payment (payment-processing)
   ↓ Success
3. Reserve inventory (inventory-management)
   ↓ Success
4. Send notification (notification-sender)
   ↓ (May fail, but purchase complete)
5. Return success

If Step 2 fails → No compensation needed (nothing charged)
If Step 3 fails → Refund payment (compensation)
If Step 4 fails → Queue for retry (purchase still valid)
\`\`\`

### Implementation Using Components

\`\`\`python
# Pseudocode showing component orchestration

from skills.payment_processing import PaymentProcessor
from subagents.inventory_management import InventoryManager
from skills.transaction_coordinator import SagaCoordinator
from subagents.notification_sender import NotificationService
from skills.error_resilience import CircuitBreaker

async def checkout(cart: Cart, payment_method: PaymentMethod) -> CheckoutResult:
    """
    Orchestrate checkout using accumulated intelligence components
    """
    saga = SagaCoordinator()  # Transaction coordinator skill

    try:
        # Step 1: Validate inventory (subagent handles complexity)
        inventory = InventoryManager()
        validation = await inventory.validate_availability(cart.items)
        if not validation.available:
            return CheckoutResult.out_of_stock(validation.missing_items)

        # Step 2: Process payment (skill provides patterns)
        payment = PaymentProcessor()
        idempotency_key = generate_idempotency_key(cart.id)
        charge = await payment.charge(
            amount=cart.total,
            method=payment_method,
            idempotency_key=idempotency_key  # Skill pattern: prevent double-charge
        )

        # Register compensation in case of later failure
        saga.add_compensation(
            lambda: payment.refund(charge.id)
        )

        # Step 3: Reserve inventory (subagent ensures atomicity)
        reservation = await inventory.reserve(cart.items, order_id=charge.id)

        # Step 4: Send notification (with circuit breaker from error-resilience skill)
        notifier = NotificationService()
        circuit = CircuitBreaker(service="notification", threshold=5)

        try:
            await circuit.execute(
                lambda: notifier.send_confirmation(
                    email=cart.user.email,
                    order_id=charge.id
                )
            )
        except CircuitBreakerOpen:
            # Skill pattern: Graceful degradation
            logger.warning(f"Notification circuit open for order {charge.id}")
            await queue_notification_retry(charge.id)
            # Purchase still succeeds even if email fails

        return CheckoutResult.success(
            order_id=charge.id,
            confirmation_sent=circuit.is_closed()
        )

    except PaymentError as e:
        # No compensation needed - payment failed before charge
        return CheckoutResult.payment_failed(str(e))

    except InventoryError as e:
        # Compensate: Refund the payment
        await saga.compensate()
        return CheckoutResult.inventory_failed(str(e))
\`\`\`

**What this demonstrates**:
- `PaymentProcessor` skill provides idempotency pattern
- `InventoryManager` subagent handles complex reservation logic autonomously
- `SagaCoordinator` skill orchestrates compensating transactions
- `NotificationService` subagent handles delivery with retries
- `CircuitBreaker` skill enables graceful degradation

**Student didn't write these from scratch — composed from intelligence library.**

### Validation Results

**Acceptance criteria verification**:

✅ **Successful purchase**: Test credit card → Payment charged, inventory updated, email received (2.3s total)

✅ **Idempotency**: Duplicate request with same key → Only one charge, idempotency key matched

✅ **Inventory accuracy**: 2 concurrent purchases of last item → Winner bought, loser got out-of-stock error

✅ **Graceful notification failure**: Killed SendGrid → Payment succeeded, inventory reserved, notification queued for retry

✅ **Payment failure rollback**: Invalid card → Payment rejected, no inventory reserved, clear error message

**Component composition assessment**:
- All 5 components integrated correctly
- Interfaces compatible
- No component reinvented
- Orchestration pattern (Saga) worked as specified

**Specification quality impact**:
- Clear acceptance criteria caught edge case (concurrent last-item purchase)
- Non-goals prevented scope creep (no multi-currency complexity)
- Constraints guided architecture (Saga vs 2PC)

---

## Transition Criteria Beyond Layer 4

**Layer 4 is capstone for each chapter. What comes next?**

**Same 4 layers, higher complexity**:
- Next chapter applies same progression to more advanced concepts
- Foundation (L1) → Collaboration (L2) → Intelligence (L3) → Orchestration (L4)
- Complexity scales with tier (A2 → B1 → B2 → C1)

**Eventually**: Professional practice applying all 4 layers autonomously to production systems

---

## Validation Checklist

### Before Publishing Layer 4 Content:

- [ ] **Specification Quality**
  - [ ] Intent clearly defines WHAT (not HOW)
  - [ ] Constraints explicitly stated
  - [ ] Non-goals prevent scope creep
  - [ ] Acceptance criteria specific and testable

- [ ] **Intelligence Inventory**
  - [ ] 3+ components listed with purpose
  - [ ] Components from previous lessons (Layers 1-3)
  - [ ] Component decision points documented

- [ ] **Composition Map**
  - [ ] Requirements mapped to components
  - [ ] Orchestration pattern selected and justified
  - [ ] Component interfaces described
  - [ ] Gap analysis completed

- [ ] **Implementation Guidance**
  - [ ] Orchestration approach clear (not prescriptive HOW)
  - [ ] Component utilization shown (not reinvention)
  - [ ] Validation protocol defined

- [ ] **Validation Protocol**
  - [ ] Acceptance criteria verification methods
  - [ ] Component utilization checked
  - [ ] Quality standards enforced
  - [ ] Reasoning transparency required

- [ ] **Reflection**
  - [ ] Meta-learning questions included
  - [ ] Design decision articulation expected
  - [ ] Next steps identified

- [ ] **Tier Appropriateness**
  - [ ] Complexity matches tier (B2-C1 for capstones)
  - [ ] Scaffolding minimal (professional autonomy)
  - [ ] Systems thinking emphasized

---

## Integration with Other Layers

**Layer 4 orchestrates intelligence accumulated across Layers 1-3.**

- **Layer 1 → Layer 4**: Foundational understanding enables specification writing
- **Layer 2 → Layer 4**: Collaboration skills inform component integration
- **Layer 3 → Layer 4**: Intelligence components provide orchestration building blocks
- **Layer 4 synthesis**: Demonstrates mastery through composition at scale

**Without Layers 1-3, Layer 4 impossible**:
- No mental models (L1) → Cannot write meaningful specifications
- No collaboration skills (L2) → Cannot refine specs through iteration
- No intelligence library (L3) → Nothing to orchestrate

---

## Meta-Awareness

**Layer 4 is where students become systems architects.**

The shift:
- **From**: Writing code line by line
- **To**: Orchestrating intelligence through specifications

**What Layer 4 teaches beyond coding**:
- Specification as primary artifact (code is output of spec)
- Composition over creation (reuse over reinvent)
- Orchestration as skill (coordinating complex interactions)
- Validation as discipline (spec ↔ implementation alignment)

**This is AI-native development at scale.**

Students don't write boilerplate—they design systems that compose reusable intelligence. The 40x multiplier comes from:
1. Clear specifications (reduce rework)
2. Reusable components (accumulate value)
3. Orchestration patterns (handle complexity)
4. Systematic validation (ensure quality)

**Layer 4 completes the 4-layer progression: Foundation → Collaboration → Intelligence → Orchestration.**

---

**Version**: 1.0.0
**Created**: 2025-01-17
**Part of**: 4-Layer Reasoning-Activated Output Styles Framework
**See also**: layer-1-foundation.md, layer-2-collaboration.md, layer-3-intelligence.md, decision-tree.md, constitution.md IIa
