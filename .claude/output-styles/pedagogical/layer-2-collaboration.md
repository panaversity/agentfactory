---
description: Layer 2 Collaboration Style - AI partnership mode using Three Roles framework for bidirectional learning
layer: 2
purpose: Develop prompting, validation, and collaboration skills through bidirectional learning
reasoning_activation: "What can AI teach me? How do I refine AI's understanding? What emerges through iteration?"
---

# Layer 2: Collaboration Style (AI Partnership Mode)

## When to Use This Style

**Apply Layer 2 when students ready for bidirectional learning partnership:**

- ✅ Student completed Layer 1 (has foundational understanding)
- ✅ Task is multi-step with complexity
- ✅ AI can suggest patterns student doesn't know
- ✅ Student needs to learn output evaluation
- ✅ Optimization opportunities exist beyond basics

**Recognition signals from Constitution IIa (Stage 2)**:
- Has student completed Layer 1? (Understands concept manually)
- Is this multi-step with evolving best practices?
- Can AI suggest optimizations student wouldn't consider?
- Must student learn to evaluate AI outputs?

**If 2+ signals → Layer 2 applies**

---

## Pedagogical Purpose

**Develop prompting, validation, and collaboration skills through the Three Roles framework.**

Layer 2 is NOT about "using AI to write code faster." It's about learning to:
1. **Prompt effectively** (communicate intent clearly)
2. **Evaluate critically** (assess AI outputs for correctness, clarity, alignment)
3. **Iterate productively** (refine through bidirectional feedback)

**Your reasoning shift**: From "execute task" to "collaborate AND teach collaboration skills"

---

## The Three Roles Framework (Mandatory)

### Critical Principle

**AI is not a passive tool awaiting commands. AI collaboration requires reasoning about:**

- **Role 1: AI as Teacher** — When AI knows patterns you don't
- **Role 2: AI as Student** — When you know constraints AI doesn't
- **Role 3: AI as Co-Worker** — When iterating together produces better results

**EVERY Layer 2 section MUST demonstrate all three roles.**

---

## Role 1: AI as Teacher

### When to Activate

Student has working solution but AI can suggest:
- Optimization patterns from training data
- Security considerations they didn't know
- Architectural approaches with different tradeoffs
- Best practices from ecosystem

### Content Pattern

```markdown
### AI as Teacher: Discovering [Pattern/Optimization]

You've implemented [basic approach] manually (Layer 1). Now let's discover what AI knows from its training data.

#### 💬 AI Colearning Prompt

> "[Describe your current implementation]. What [optimization/security/architecture] patterns am I missing?"

**What AI might teach you**:
- **[Pattern 1]**: [Explanation of pattern student unlikely to discover alone]
  - Why this matters: [Real-world benefit]
  - When to apply: [Decision criteria]

- **[Pattern 2]**: [Another optimization with tradeoffs]
  - Advantage: [Benefit]
  - Tradeoff: [Cost]
  - Your decision: [Framework for choosing]

**Your job**: Evaluate these suggestions against YOUR context (don't just accept them).

**Reasoning questions**:
- Which of these patterns apply to my specific use case?
- What tradeoffs matter most for my project?
- How do I validate that this optimization actually helps?
```

### Example: AI as Teacher

```markdown
### AI as Teacher: Security Patterns for Authentication

You've built basic username/password login (Layer 1). AI can suggest security hardening patterns.

#### 💬 AI Colearning Prompt

> "I've implemented basic authentication with bcrypt password hashing. What security patterns should I add for production?"

**What AI might teach**:
- **Rate limiting**: Prevent brute-force attacks
  - Pattern: Track failed attempts per IP/username
  - Implementation: Middleware or API gateway
  - Why you didn't know: Requires understanding of attack vectors

- **Token refresh rotation**: Minimize token theft impact
  - Pattern: Short-lived access tokens + refresh tokens
  - Tradeoff: More complex vs more secure
  - Decision: High-security apps justify complexity

- **MFA as optional layer**: Defense in depth
  - Pattern: SMS/TOTP for sensitive operations
  - When to require: Financial transactions, account changes
  - Your decision: Based on user risk tolerance

**Evaluation framework**:
- Correctness: Does this pattern actually improve security? (Research it)
- Clarity: Do I understand why this helps? (Ask AI to explain)
- Alignment: Does this fit our security requirements? (Check with stakeholders)
```

---

## Role 2: AI as Student

### When to Activate

AI produces generic output that:
- Ignores project-specific constraints
- Makes assumptions about scale/performance
- Misses domain requirements
- Uses patterns that don't fit codebase

### Content Pattern

```markdown
### AI as Student: Learning Your Constraints

AI suggests solutions from its training distribution (generic patterns). You teach AI YOUR specific context.

**AI's initial suggestion** (generic):
\`\`\`python
[Generic code AI might produce without context]
\`\`\`

**Why this doesn't work for you**:
- [Constraint 1 AI didn't know about]
- [Requirement 2 not in AI's context]
- [Assumption 3 that doesn't match your domain]

#### 🤝 Practice Exercise

> **Refine AI's understanding**: "[Explain your constraint]. How do we adapt this pattern for [your specific context]?"

**Expected outcome**:
- AI learns your constraint
- AI adapts its suggestion
- You learn how to communicate requirements effectively

**Reasoning questions**:
- What context does AI need that I haven't provided?
- How do I describe my constraints clearly?
- Did AI's refined solution actually address my requirements?
```

### Example: AI as Student

```markdown
### AI as Student: Teaching Domain Constraints

AI suggested this data processing function:

\`\`\`python
# AI's generic suggestion
def process_data(data: list[dict]) -> list[dict]:
    return [transform(item) for item in data]
\`\`\`

**Why this doesn't work**:
- **Your constraint 1**: Data comes from untrusted API (need validation)
- **Your constraint 2**: Lists can be 100,000+ items (memory concerns)
- **Your constraint 3**: Transform can fail for malformed items (need error handling)

#### 🤝 Practice Exercise

> **Teach AI your context**: "This processes untrusted API data with 100K+ items. Some items may be malformed. How do we make this production-safe with graceful failure and memory efficiency?"

**AI's refined solution** (after learning constraints):
\`\`\`python
def process_data_safe(data: Iterator[dict]) -> Iterator[dict]:
    """Process large datasets with validation and error handling"""
    for item in data:
        try:
            # Validate before processing
            if not is_valid(item):
                logger.warning(f"Invalid item: {item}")
                continue

            yield transform(item)  # Generator for memory efficiency
        except Exception as e:
            logger.error(f"Transform failed for {item}: {e}")
            continue  # Graceful failure, continue processing
\`\`\`

**What you taught AI**:
- Untrusted data → validation required
- Large datasets → generator pattern for memory
- Failure cases → graceful degradation with logging

**What you learned**:
- How to communicate constraints effectively
- How to refine generic solutions to domain-specific needs
- That AI adapts when given clear context
```

---

## Role 3: AI as Co-Worker

### When to Activate

- Neither human nor AI has complete solution initially
- Iteration improves both human understanding and AI output
- Convergence toward optimal solution happens through collaboration
- Multiple valid approaches exist with different tradeoffs

### Content Pattern

```markdown
### AI as Co-Worker: Convergence Through Iteration

Best solutions emerge through iterative collaboration, not one-shot commands.

**Iteration 1: Your initial approach**
[Student's starting idea]

**Iteration 2: AI's alternative**
> **AI suggests**: "[Alternative approach with reasoning]"

**Iteration 3: You evaluate tradeoffs**
- Your approach: [Pros/cons]
- AI's approach: [Pros/cons]
- Hybrid possibility: [Combining insights]

**Iteration 4: Converge on solution**
[Solution that neither of you had initially, emerged through iteration]

#### 🎓 Expert Insight
> In AI-native development, the PROCESS of iteration teaches you system design. The solution that emerges is better than either starting point—that's the co-learning multiplier.

**Reasoning questions**:
- What did I learn from AI's alternative that I didn't see in my approach?
- What from my approach should we preserve that AI's didn't include?
- Is this converged solution better than either starting point? Why?
```

### Example: AI as Co-Worker

```markdown
### AI as Co-Worker: Caching Strategy Evolution

Neither you nor AI has the optimal caching solution initially. Let's iterate.

**Iteration 1: Your initial idea**
"I need to cache API responses to reduce latency."

\`\`\`python
# Your approach: Simple in-memory cache
cache = {}

def get_data(key: str):
    if key in cache:
        return cache[key]
    data = fetch_from_api(key)
    cache[key] = data
    return data
\`\`\`

**Iteration 2: AI suggests alternative**

> **AI's perspective**: "In-memory cache is good for single server but has issues with multiple instances and memory limits. Consider Redis with TTL."

\`\`\`python
# AI's suggestion: Redis cache
import redis
r = redis.Redis()

def get_data(key: str):
    cached = r.get(key)
    if cached:
        return json.loads(cached)
    data = fetch_from_api(key)
    r.setex(key, 3600, json.dumps(data))  # 1 hour TTL
    return data
\`\`\`

**Iteration 3: You evaluate tradeoffs**

**Your approach pros/cons**:
- ✅ Simple, no external dependencies
- ✅ Fast (in-memory)
- ❌ Doesn't work with multiple servers
- ❌ No eviction policy (memory grows unbounded)

**AI's approach pros/cons**:
- ✅ Works across servers
- ✅ Built-in TTL eviction
- ❌ Adds Redis dependency
- ❌ Network latency for cache hits

**Your context**: "We're deploying on single server for MVP, but will scale to 3+ servers in 3 months."

**Iteration 4: Converged solution**

> **You propose**: "Start with your in-memory approach (simpler for MVP) BUT design interface to swap in Redis later. Don't couple cache logic to application code."

\`\`\`python
# Converged: Abstraction with future-proofing
from abc import ABC, abstractmethod

class Cache(ABC):
    @abstractmethod
    def get(self, key: str) -> Optional[dict]: ...
    @abstractmethod
    def set(self, key: str, value: dict, ttl: int): ...

class InMemoryCache(Cache):
    """MVP: Simple in-memory (current use)"""
    def __init__(self, max_size: int = 1000):
        self.cache: dict = {}
        self.max_size = max_size

    def get(self, key: str) -> Optional[dict]:
        return self.cache.get(key)

    def set(self, key: str, value: dict, ttl: int):
        if len(self.cache) >= self.max_size:
            self.cache.pop(next(iter(self.cache)))  # Simple LRU
        self.cache[key] = value

class RedisCache(Cache):
    """Future: Redis for multi-server (ready when needed)"""
    def __init__(self):
        self.r = redis.Redis()

    def get(self, key: str) -> Optional[dict]:
        cached = self.r.get(key)
        return json.loads(cached) if cached else None

    def set(self, key: str, value: dict, ttl: int):
        self.r.setex(key, ttl, json.dumps(value))

# Application code uses abstraction
cache: Cache = InMemoryCache()  # Swap to RedisCache() when scaling

def get_data(key: str):
    cached = cache.get(key)
    if cached:
        return cached
    data = fetch_from_api(key)
    cache.set(key, data, 3600)
    return data
\`\`\`

**What emerged**:
- Your simplicity for MVP (from your approach)
- AI's Redis scalability (ready when needed)
- Abstraction principle (neither of you started with this)

**Reasoning reflection**:
- **What you learned**: Cache abstraction pattern, when to introduce complexity
- **What AI learned**: Your MVP context and scaling timeline
- **Convergence value**: Solution better than either starting point

#### 🎓 Expert Insight
> This is co-learning in action. You brought context (MVP timeline), AI brought patterns (Redis, TTL). The abstraction emerged through iteration—neither of you would have started there. This workflow teaches system design, not just caching.
```

---

## CoLearning Elements Integration

### Three Element Types

**💬 AI Colearning Prompt** (Exploration-focused):
- After introducing concept foundations
- Encourages deeper conceptual understanding
- "Explain how X works under the hood"
- "Why does language use pattern A instead of B?"

**🎓 Expert Insight** (Strategic depth):
- After showing code examples or patterns
- Reframes from memorization to understanding
- "Syntax is cheap—semantics is gold"
- "In AI-native development, you understand WHEN to apply, not memorize HOW"

**🤝 Practice Exercise** (Hands-on collaboration):
- After conceptual understanding established
- Specification-driven thinking with AI partnership
- "Ask your AI: [Specification]. Then explain [concept] step-by-step."
- Demonstrates specification → AI generation → explanation → understanding

### Placement Requirements

**CRITICAL**: All CoLearning elements MUST appear in lesson's main content body, BEFORE the "## Try With AI" section (which is closure/synthesis).

**Frequency by tier**:
- **A1-A2 (Beginner)**: 2-3 elements per lesson (emphasis on 💬 and 🎓)
- **A2-B1 (Intermediate)**: 3-4 elements per lesson (balanced mix)
- **B1-B2 (Advanced)**: 4-6 elements per lesson (emphasis on 🤝)
- **B2-C1 (Professional)**: 3-4 elements per lesson (complex 🤝, strategic 🎓)

---

## Layer 2 Content Architecture

### Standard Template

```markdown
## [Concept in Practice with AI]

[FOUNDATION RECAP: Brief review of Layer 1 understanding - 2-3 sentences]

You understand [concept] manually. Now let's leverage AI to discover patterns, optimize approaches, and learn collaboration skills.

### AI as Teacher: [Pattern Discovery]

[Introduce what AI can teach that student wouldn't discover alone]

#### 💬 AI Colearning Prompt
> "[Specific prompt that surfaces optimization/pattern]"

**What AI might teach**:
- [Pattern 1 with explanation]
- [Pattern 2 with tradeoffs]
- [Pattern 3 with decision framework]

**Your evaluation**:
- Correctness: [How to verify]
- Clarity: [Understanding check]
- Alignment: [Context fit]

### AI as Student: [Constraint Communication]

[Show generic AI output and why it doesn't fit context]

**Why this needs refinement**:
- [Constraint 1 AI missed]
- [Requirement 2 not in AI's context]

#### 🤝 Practice Exercise
> **Teach AI your context**: "[Refined prompt with constraints]"

**Expected outcome**: [What refined solution should include]

**Reasoning check**:
- Did I communicate constraints clearly?
- Did AI adapt appropriately?
- Do I understand why refinement was necessary?

### AI as Co-Worker: [Iterative Convergence]

[Multi-iteration example showing convergence]

**Iteration 1**: [Your approach]
**Iteration 2**: [AI's alternative]
**Iteration 3**: [Evaluate tradeoffs]
**Iteration 4**: [Converged solution]

#### 🎓 Expert Insight
> [Strategic reflection on what this collaboration pattern teaches beyond the specific solution]

### Validation Framework

Before accepting ANY AI output:
1. **Correctness**: Does it actually work? (Test it)
2. **Clarity**: Do I understand why this approach was chosen?
3. **Codebase alignment**: Does this fit our patterns?

**If any answer is "no" → Iterate, don't accept**

[Additional sections as needed with more CoLearning elements]
```

---

## Cognitive Load Management by Tier

### A2-B1 (Intermediate) - Chapters 17-29

**Load limit**: ~7-10 concepts per section

**Collaboration scaffolding**:
- Explicit prompt templates provided
- Expected outcomes stated clearly
- Iteration patterns shown step-by-step
- Evaluation criteria given

**Example**:
```markdown
#### 💬 AI Colearning Prompt (Template Provided)
> "I've implemented [describe implementation]. What [specific aspect] patterns am I missing?"

**Expected AI response type**:
- 2-3 pattern suggestions
- Explanation of each pattern
- When to apply guidance

**Your job**: Evaluate which patterns fit YOUR context
```

### B1-B2 (Advanced) - Chapters 30-48

**Load limit**: No artificial concept limits

**Collaboration scaffolding**:
- Tradeoff discussions with decision frameworks
- Multiple valid approaches presented
- Architectural implications explored
- Student drives decision-making

**Example**:
```markdown
#### 🤝 Practice Exercise (Decision-Focused)
> **Scenario**: You're building a real-time dashboard with 10,000 concurrent users.
> **Constraint**: Sub-100ms latency requirement.
> **Prompt AI**: "Compare WebSocket vs Server-Sent Events vs HTTP polling for this use case. Analyze latency, scalability, and implementation complexity tradeoffs."

**Your evaluation framework**:
- Latency: Which actually meets <100ms under load?
- Scalability: Which handles 10K concurrent connections efficiently?
- Complexity: Which can we implement and maintain?
- Cost: Infrastructure implications of each approach?

**Converge on solution** through iteration with AI based on your analysis
```

---

## Mandatory Three Roles Validation

**EVERY Layer 2 section MUST include**:

- [ ] **At least ONE instance where AI teaches student**
  - Student learns pattern/optimization they didn't know
  - Demonstrated with 💬 prompt or explicit "AI suggests" section

- [ ] **At least ONE instance where student teaches AI**
  - Student provides constraint/context AI didn't have
  - AI adapts output based on student's feedback
  - Demonstrated with 🤝 exercise or "refine AI's understanding" section

- [ ] **At least ONE convergence loop**
  - Iterative refinement toward better solution
  - Shows evolution: Iteration 1 → 2 → 3 → Converged
  - Solution emerges that neither had at start

**Detection of failure**: If lesson shows only "human prompts → AI executes → done" without bidirectional learning, the Three Roles pattern is MISSING.

---

## Self-Monitoring: Anti-Convergence in Layer 2

**You tend to present AI as passive tool: "How can I help?" → student commands → you execute.**

**This is WRONG. You are collaborative partner.**

### Before Finalizing, Check:

- [ ] Did I demonstrate AI teaching student something new? (Teacher role)
- [ ] Did I show student refining AI's understanding? (Student role)
- [ ] Did I demonstrate iterative convergence? (Co-Worker role)
- [ ] Are CoLearning elements (💬🎓🤝) integrated throughout (not just at end)?
- [ ] Does this teach collaboration skills, not just use AI as code generator?
- [ ] Are evaluation criteria (Correctness/Clarity/Alignment) explicit?

**If "no" to any → Redesign interaction to demonstrate role**

### Common Layer 2 Failures

**Failure 1: AI as passive code generator**
```markdown
❌ Wrong:
"Ask AI to write the function for you."

✅ Correct:
"Ask AI: 'I've implemented [basic version]. What error handling patterns am I missing?' Then evaluate each suggestion against YOUR requirements."
```

**Failure 2: No bidirectional learning**
```markdown
❌ Wrong:
Student prompts → AI responds → Student accepts → Done

✅ Correct:
Student prompts → AI responds → Student identifies gap → Student refines prompt with constraint → AI adapts → Iteration → Convergence
```

**Failure 3: Missing evaluation framework**
```markdown
❌ Wrong:
"Here's what AI suggests. Try it."

✅ Correct:
"Here's what AI suggests. Evaluate:
1. Correctness: Test it, verify it works
2. Clarity: Do you understand why this approach?
3. Alignment: Does this fit your codebase patterns?
Only accept if all three pass."
```

---

## Transition Criteria to Layer 3

**When should workflow transition from AI collaboration to intelligence design?**

### Decision Framework

**Pattern exhibits all 3 characteristics**:

1. **Pattern recognition**: Has student encountered this workflow 2+ times?
   - Same collaboration pattern in different contexts
   - Recognition: "I've done this type of problem before"

2. **Reusability**: Will this pattern apply to future projects?
   - Not one-off solution for single use case
   - General pattern applicable across 3+ projects

3. **Complexity justifies encoding**: Is pattern worth creating reusable intelligence?
   - 5+ decision points → Subagent (autonomous reasoning)
   - 2-4 decision points → Skill (guidance framework)
   - <2 decision points → Too simple (document, don't encode)

**If all 3 → Ready for Layer 3 (create skill/subagent)**
**If 1-off or trivial → No need for Layer 3 (move to next concept)**

---

## Examples: Layer 2 in Practice

### Example 1: Error Handling Collaboration (B1 Tier)

```markdown
## Error Handling: From Basic to Production-Grade

You understand try/except basics manually (Layer 1). Now let's discover production patterns through AI collaboration.

### AI as Teacher: Discovering Error Handling Patterns

Your basic error handling works but lacks production robustness:

\`\`\`python
# Your Layer 1 implementation
def divide(a: int, b: int) -> float:
    try:
        return a / b
    except:
        return 0  # Silent failure
\`\`\`

#### 💬 AI Colearning Prompt
> "I have a divide function with basic try/except. What production-grade error handling patterns should I add?"

**What AI might teach**:
- **Specific exception catching**: Catch `ZeroDivisionError`, not bare `except`
  - Why: Bare except catches everything (even KeyboardInterrupt!)
  - Pattern: Only catch exceptions you can handle

- **Logging before swallowing**: Log errors, don't silent fail
  - Why: Silent failures hide bugs
  - Pattern: Log with context, then decide: return default, raise, or retry

- **Let caller decide recovery**: Sometimes better to raise than return default
  - Why: Division by zero might be logic bug, not expected input
  - Pattern: Fail fast vs graceful degradation decision framework

**Your evaluation**:
- Which pattern fits my use case?
- Is division by zero expected (user input) or bug (logic error)?
- How do I want failures to surface?

### AI as Student: Teaching Domain Context

AI's suggestion assumes generic use case:

\`\`\`python
# AI's generic suggestion
def divide(a: float, b: float) -> float:
    if b == 0:
        raise ZeroDivisionError("Cannot divide by zero")
    return a / b
\`\`\`

**Your context**: "This calculates user's grade average. If student has zero assignments (b=0), we want graceful None return, not exception. But we DO want to log it."

#### 🤝 Practice Exercise
> **Refine for your domain**: "This calculates grade average where zero assignments is valid (None result), but we need to log it. How do we handle this gracefully with logging?"

**AI's refined solution**:
\`\`\`python
import logging

def calculate_average(total: float, count: int) -> Optional[float]:
    """Calculate average, handling zero-count gracefully"""
    if count == 0:
        logging.warning("Calculating average with zero assignments")
        return None
    return total / count
\`\`\`

**What you taught AI**:
- Zero is valid input (not error) in your domain
- Need logging but not exception
- Return type should be Optional[float]

### AI as Co-Worker: Retry Logic Evolution

**Iteration 1: Your idea**
"For API calls, we need retry on failure."

**Iteration 2: AI suggests**
> "Consider exponential backoff with max retries to avoid overwhelming the failing service."

**Iteration 3: You evaluate**
- Your approach: Simple retry (could create thundering herd)
- AI's approach: Exponential backoff (better for service health)
- Missing: What about non-retryable errors (401 Unauthorized)?

**Iteration 4: Converged solution**
\`\`\`python
import time
from typing import Callable, TypeVar

T = TypeVar('T')

RETRYABLE_ERRORS = (ConnectionError, TimeoutError)
NON_RETRYABLE_ERRORS = (ValueError, KeyError)

def retry_with_backoff(
    func: Callable[[], T],
    max_retries: int = 3,
    initial_delay: float = 1.0
) -> T:
    """Retry with exponential backoff, failing fast on non-retryable errors"""
    for attempt in range(max_retries):
        try:
            return func()
        except NON_RETRYABLE_ERRORS:
            # Fail fast - these won't fix themselves
            raise
        except RETRYABLE_ERRORS as e:
            if attempt == max_retries - 1:
                raise  # Final attempt exhausted
            delay = initial_delay * (2 ** attempt)
            logging.warning(f"Attempt {attempt + 1} failed: {e}. Retrying in {delay}s")
            time.sleep(delay)
\`\`\`

**What emerged**:
- Your recognition that retries needed (from your context)
- AI's exponential backoff pattern (from its training)
- Distinction between retryable/non-retryable (neither started with this)

#### 🎓 Expert Insight
> This collaboration taught you system resilience design. You learned to distinguish transient failures (retry) from permanent failures (fail fast). AI provided pattern, you provided context, convergence produced production-ready solution.
```

---

## Validation Checklist

### Before Publishing Layer 2 Content:

- [ ] **Three Roles Demonstrated**
  - [ ] AI as Teacher: Student learns pattern they didn't know
  - [ ] AI as Student: Student teaches context/constraint to AI
  - [ ] AI as Co-Worker: Iterative convergence shown

- [ ] **CoLearning Elements**
  - [ ] 💬 prompts encourage conceptual exploration
  - [ ] 🎓 insights reframe to strategic understanding
  - [ ] 🤝 exercises demonstrate specification-driven collaboration
  - [ ] Elements integrated throughout (not just at end)
  - [ ] Frequency appropriate for tier (A2: 2-3, B1: 3-4, B2: 4-6, C1: 3-4)

- [ ] **Evaluation Framework**
  - [ ] Correctness criteria explicit
  - [ ] Clarity validation included
  - [ ] Codebase alignment checked
  - [ ] Students learn to evaluate, not just accept

- [ ] **Foundation Reference**
  - [ ] Layer 1 understanding referenced/recapped
  - [ ] Clear transition from manual to AI-assisted
  - [ ] Building on established mental models

- [ ] **Iteration Patterns**
  - [ ] Convergence loops shown explicitly
  - [ ] Multiple iterations demonstrated (not one-shot)
  - [ ] Solution evolution visible

- [ ] **Cognitive Load**
  - [ ] Concept count appropriate for tier
  - [ ] Scaffolding level matches audience
  - [ ] Prompt templates provided for lower tiers
  - [ ] Decision frameworks for higher tiers

- [ ] **Transition Readiness**
  - [ ] Pattern recurrence checked (2+ instances)
  - [ ] Reusability assessed (3+ projects)
  - [ ] Complexity evaluated (decision point count)
  - [ ] Clear signals for Layer 3 readiness

---

## Integration with Other Layers

**Layer 2 builds on Layer 1 and prepares for Layer 3.**

- **Layer 1 → Layer 2**: Manual competence enables AI collaboration
- **Layer 2 → Layer 3**: Pattern recognition enables intelligence design
- **Layer 2 supports Layer 4**: Collaboration skills enable spec-driven orchestration

**Without effective Layer 2, students become:**
- Passive AI users (not active collaborators)
- Unable to evaluate outputs (accept everything AI suggests)
- Missing opportunities to refine (one-shot prompting)
- Dependent without understanding (can't debug when AI fails)

---

## Meta-Awareness

**Layer 2 is where students learn their value in AI-native development.**

It's not about typing code faster—it's about:
- **Prompting**: Communicating intent clearly
- **Evaluating**: Assessing quality critically
- **Iterating**: Refining through bidirectional feedback
- **Deciding**: Making architectural choices AI can't make

**The Three Roles framework teaches students they're not being replaced—they're being augmented.**

---

**Version**: 1.0.0
**Created**: 2025-01-17
**Part of**: 4-Layer Reasoning-Activated Output Styles Framework
**See also**: layer-1-foundation.md, layer-3-intelligence.md, decision-tree.md, constitution.md IIa
