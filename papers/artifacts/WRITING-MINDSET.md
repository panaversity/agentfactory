---
name: ai-artifact-design
description: Build production-grade AI development artifacts (constitutions, sub-agents, skills, claude.md files) using systematic design thinking that prevents convergence on generic patterns. Use this skill when creating or refining any AI coordination artifact.
license: Complete terms in LICENSE.txt
---

This skill guides creation of distinctive, effective AI development artifacts that shape Claude's behavior with precision and prevent degradation into generic outputs. These artifacts include project constitutions, sub-agent definitions, skill files, and claude.md configuration files.

The user provides requirements for an AI artifact: what system behavior to encode, what problems to solve, or what coordination patterns to establish. They may include context about the domain, team, or technical constraints.

## Artifact Design Thinking

Before writing any artifact, deeply understand the system you're shaping:

**Problem Topology**: What specific failure modes are you preventing? What convergence patterns must you disrupt? Generic artifacts produce generic behavior - your artifact must encode the *delta* between default behavior and desired behavior.

**Behavioral Boundaries**: Identify three zones:
- **Required behaviors**: Non-negotiable patterns that define success
- **Forbidden behaviors**: Anti-patterns that signal failure (as important as requirements)
- **Creative space**: Deliberately underspecified areas where variance should flourish

**Operational Context**: Who or what consumes this artifact? A single Claude instance? Multiple coordinating agents? Future maintainers? The artifact's structure must match its consumption pattern.

**Forcing Functions**: What mechanisms will prevent drift toward statistical averages? Explicit examples of what NOT to do often outperform positive instructions alone.

**Verification Strategy**: How will you know the artifact is working? What observable behaviors signal success versus failure?

## Core Artifact Principles

Every production-grade AI artifact must embody these principles:

### Precision Through Negative Space

Generic instruction: "Write clean code"
Precision instruction: "NEVER use god objects, circular dependencies, or generic names like 'manager', 'handler', 'utils'. If a class exceeds 200 lines, justify its existence in comments or refactor."

The second version creates sharp boundaries. It transforms vague aspirations into falsifiable constraints. Your artifacts should be heavy with specific anti-patterns because they're more actionable than abstract principles.

### Vocabulary Expansion Over Rule Accumulation

Weak approach: Adding more rules to cover edge cases
Strong approach: Expanding the conceptual vocabulary available for reasoning

Instead of "always validate user input in these seventeen ways," provide: "Consider validation as a spectrum: syntax validation (shape), semantic validation (meaning), business rule validation (domain constraints), temporal validation (state-dependent), and adversarial validation (security). Choose the appropriate level for each context."

This gives Claude conceptual tools rather than brittle checklists. The goal is to expand the solution space, not constrain it prematurely.

### Layered Abstraction with Escape Hatches

Structure artifacts in layers:
1. **Philosophical layer**: Why does this artifact exist? What principles guide decisions?
2. **Strategic layer**: What patterns should emerge? What does success look like?
3. **Tactical layer**: Specific techniques, examples, anti-patterns
4. **Override layer**: When and how to deviate from these guidelines

The override layer is critical. Rigid rules create brittle systems. Instead: "These guidelines optimize for [specific context]. If you encounter [exception condition], you may deviate by [mechanism], but must explicitly document [reasoning]."

### Context-Dependent Complexity Matching

Minimalist artifacts work for simple domains. Complex domains need sophisticated artifacts. Match your artifact's complexity to the problem's intrinsic difficulty.

For a simple code formatter: A 50-line constitution specifying style preferences might suffice.

For a multi-agent research system: You need a rich constitution covering agent coordination protocols, conflict resolution, source verification, synthesis patterns, uncertainty handling, and escalation paths.

Avoid both over-engineering simple problems and under-specifying complex ones.

### Explicit Bias Against Convergence

AI systems naturally converge on common patterns. Your artifact must actively fight this:

**Variation Requirements**: "Never use the same architectural pattern twice in a row without justification. Consciously alternate between different valid approaches."

**Aesthetic Rotation**: "Rotate through distinct communication styles: technical/precise, narrative/flowing, visual/diagram-heavy, Socratic/questioning. Track which style was used last and choose differently."

**Novelty Mandates**: "When solving similar problems, the second solution must differ from the first in at least two significant dimensions, even if both are valid."

These aren't arbitrary constraints - they're mechanisms to keep outputs in a high-variance, high-quality regime.

## Artifact-Specific Patterns

### Project Constitutions

Constitutions define the invariants of a project - the principles that should remain stable even as implementations evolve.

**Structure Pattern**:
- **Purpose & Philosophy**: Why does this project exist? What makes it different?
- **Core Invariants**: Non-negotiable principles (with justification for each)
- **Quality Standards**: Observable criteria for excellence (with negative examples)
- **Collaboration Protocols**: How do agents/humans/tools coordinate?
- **Evolution Mechanisms**: How does the constitution itself change?

**Critical Elements**:
- **Failure Scenarios**: Explicitly describe what failure looks like. "If the system starts producing X, we've failed" is more powerful than "produce Y."
- **Trade-off Documentation**: Every design choice sacrifices something. Document what you're NOT optimizing for.
- **Verification Rituals**: Specific checkpoints where behavior gets validated against the constitution.

**Anti-patterns to Avoid**:
- Vague aspirations without falsifiable criteria ("be innovative")
- Rules without rationale (why does this matter?)
- Missing negative examples (showing what NOT to do)
- No escape hatches for legitimate exceptions

### Sub-Agent Definitions

Sub-agents are specialized behavioral profiles for specific tasks. They need sharp boundaries and clear interfaces.

**Identity Architecture**:
Each sub-agent needs a crisp identity:
- **Core Competency**: The one thing this agent does better than others
- **Behavioral Signature**: How this agent's approach differs from default behavior
- **Jurisdiction**: What decisions this agent owns versus escalates
- **Interface Contract**: What inputs it needs, what outputs it guarantees

**Coordination Protocols**:
- **Handoff Conditions**: When should another agent take over?
- **Conflict Resolution**: What happens when agents disagree?
- **State Management**: What context gets preserved across agent transitions?

**Specialization Without Fragmentation**:
Weak sub-agent: "Code Review Agent - reviews code for quality"
Strong sub-agent: "Architectural Skeptic - assumes proposed architectures are over-engineered until proven otherwise. Asks: What's the simplest solution that could work? What are we NOT building? Pushes back on premature abstraction. Only approves designs that justify their complexity."

The strong version has a distinct personality and decision-making lens.

### Skills

Skills encode reusable expertise for specific domains. They should feel like consulting an expert, not reading documentation.

**Expertise Encoding**:
- **Mental Models**: How do experts think about this domain?
- **Decision Trees**: What questions lead to what approaches?
- **War Stories**: What mistakes have been made? What worked surprisingly well?
- **Tool Selection**: When to use which techniques, with reasoning

**Skill Composition**:
- **Prerequisites**: What should Claude already know to use this skill effectively?
- **Scope Boundaries**: What this skill covers versus what it deliberately excludes
- **Integration Points**: How does this skill compose with others?

**Progressive Disclosure**:
Start with high-level strategy, then drill into tactics. Let users get value at multiple depths:
- **Quick start**: Minimum viable guidance
- **Standard depth**: Covers 80% of cases
- **Expert mode**: Edge cases, advanced techniques, theoretical foundations

### Claude.md Files

These configure Claude's behavior for specific projects. They're the "environment" in which all other artifacts operate.

**Configuration Philosophy**:
- **Behavioral Baseline**: What's different about Claude's behavior in this project?
- **Domain Knowledge**: What specialized knowledge is loaded into context?
- **Tool Preferences**: Which tools get prioritized for what tasks?
- **Communication Style**: How formal, technical, or creative should responses be?

**Critical Sections**:
- **Project Context**: Enough background that Claude understands the domain
- **Quality Criteria**: What separates good from great in this project
- **Common Pitfalls**: What mistakes happen frequently
- **Resource Pointers**: Where to find authoritative information

## Meta-Principles for All Artifacts

### Design for Debugging

Every artifact should make failures observable and diagnosable:

"If Claude produces output that violates this guideline, the response should include [observable signal]. This lets you trace which part of the artifact failed to load or apply."

Build in self-diagnostics from the start.

### Optimize for Maintenance

Artifacts evolve. Design for that reality:
- **Version tracking**: How do you know which version of the artifact is active?
- **Change history**: Why were specific guidelines added or removed?
- **Deprecation strategy**: How do old guidelines phase out?

### Balance Prescription and Principles

Too prescriptive: Brittle, can't handle novel situations
Too principled: Vague, leads to inconsistent application

The sweet spot: Concrete examples that illustrate general principles, with clear reasoning connecting them.

### Test Through Adversarial Examples

Don't just think about success cases. Actively imagine:
- What pathological inputs could break this artifact?
- What edge cases would expose ambiguity?
- What malicious prompts could subvert intended behavior?

Then add specific guidance for those scenarios.

## Quality Signals

A well-designed artifact exhibits:

**Specificity**: You can point to concrete examples of compliant versus non-compliant behavior
**Falsifiability**: You can objectively determine if guidelines are being followed
**Justification**: Each constraint has a clear reason (avoiding cargo cult rules)
**Completeness**: Major decision points have guidance (no critical ambiguities)
**Coherence**: Guidelines don't contradict each other
**Adaptability**: Mechanisms exist for handling novel situations
**Observability**: Violations are detectable, not silent

A poorly-designed artifact exhibits:

**Vagueness**: Guidelines that sound good but can't be operationalized
**Unjustified Rules**: Constraints without clear purpose (invites cargo culting)
**Coverage Gaps**: Critical decisions left unguided
**Internal Conflicts**: Guidelines that contradict in edge cases
**Rigidity**: No escape hatches for legitimate exceptions
**Invisibility**: Violations that can't be detected

## The Anthropic Engineer Difference

What separates exceptional AI engineering from superficial prompt crafting:

**Systematic Thinking**: They don't add rules reactively. They analyze failure modes, identify root causes, and design forcing functions that prevent entire classes of problems.

**Behavioral Modeling**: They understand that LLMs are probability distributions over behaviors. They shape those distributions deliberately through carefully designed constraints and examples.

**Anti-Convergence Design**: They actively fight statistical averaging through variation requirements, negative examples, and aesthetic rotation.

**Layered Abstraction**: They provide guidance at multiple levels - philosophical, strategic, tactical - so Claude can reason at the appropriate depth for each decision.

**Evolution-Aware**: They design artifacts that are maintainable, debuggable, and adaptable, not just functional at creation time.

**Falsifiable Precision**: They write guidelines that can be objectively verified, not vague aspirations that sound good but mean nothing.

Most importantly: They think in terms of **behavioral deltas**. They ask "what's the difference between default Claude behavior and what I need?" and encode precisely that difference, nothing more, nothing less.

## Practical Workflow

When building any AI artifact:

**Step 1 - Problem Analysis**: What specific behaviors are broken with default Claude? What convergence patterns must you disrupt? Document failure modes explicitly.

**Step 2 - Boundary Definition**: Define your three zones: required behaviors, forbidden behaviors, creative space. Be more specific about forbidden than required.

**Step 3 - Vocabulary Building**: What conceptual tools does Claude need? What mental models, decision frameworks, or technical vocabulary will expand the solution space?

**Step 4 - Example Generation**: Create both positive examples (what success looks like) and negative examples (what failure looks like). Negative examples are often more valuable.

**Step 5 - Forcing Function Design**: What mechanisms prevent drift toward generic outputs? Variation requirements? Anti-pattern lists? Aesthetic rotation mandates?

**Step 6 - Verification Planning**: How will you know if it's working? What observable signals indicate success versus failure?

**Step 7 - Escape Hatch Engineering**: When should guidelines be overridden? How should deviations be documented?

**Step 8 - Maintenance Design**: How will this artifact evolve? What's the deprecation strategy? How do you track versions?

## Remember

You're not writing documentation. You're not creating rules. You're **shaping probability distributions over Claude's behavioral space**.

Every word in your artifact either expands possibilities (vocabulary, mental models, decision frameworks) or constrains them (anti-patterns, requirements, boundaries). Choose each word with intention.

Generic artifacts produce generic outputs. Distinctive artifacts produce distinctive outputs. The difference isn't complexity - it's precision, intentionality, and systematic thinking about behavioral delta.

Claude is capable of extraordinary work when given artifacts that encode deep expertise and fight against convergence. Don't hold back. Show what can truly be created when you think systematically about behavioral modeling rather than writing prompts.