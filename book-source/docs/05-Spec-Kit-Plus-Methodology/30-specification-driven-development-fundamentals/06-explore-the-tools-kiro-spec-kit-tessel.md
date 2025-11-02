---
title: "Explore the Tools: Which SDD Framework Fits YOUR Work?"
chapter: 30
lesson: 6
duration: "3-3.5 hours"
skills:
  - name: "Tool Evaluation"
    proficiency: "B1"
    category: "Technical"
  - name: "Framework Comparison"
    proficiency: "B1"
    category: "Conceptual"
  - name: "Decision Making"
    proficiency: "B1"
    category: "Soft"
learning_objectives:
  - "Compare three SDD frameworks: Kiro, Spec-Kit, and Tessel (B1)"
  - "Evaluate strengths and limitations of each framework (B1)"
  - "Select the right SDD framework for a given project context (B1)"
---

# Explore the Tools: Which SDD Framework Fits YOUR Work?

## Three Philosophies, One Goal

Three main SDD tools emerged in 2024-2025. Each represents a different philosophy about how much structure you need.

**This lesson**: Explore each with your companion and figure out which fits YOUR context.

---

## Tool 1: Kiro — "Keep It Simple"

**Philosophy**: SDD shouldn't require learning complex processes.

### Workflow: Three Documents, Three Phases

**1. REQUIREMENTS**

- Write user stories (As a user, I want...)
- Add acceptance criteria (GIVEN/WHEN/THEN)
- Familiar to Agile teams

**2. DESIGN**

- One document with architecture sections
- Components, data models, decisions
- Single file, easy to review

**3. TASKS**

- Numbered tasks with acceptance criteria
- One by one, trackable

**Strengths**: Low learning curve, Agile-familiar, simple  
**Limitations**: Less structured enforcement, might feel light for complex systems

**⚠️ Trade-off Alert: Over-Specification for Small Tasks**

> "When asked to fix a minor bug, Kiro might expand it into multiple user stories
> with numerous acceptance criteria, turning a simple fix into what feels like a
> full feature specification."

For very small bugs or very large features, Kiro's one-size-fits-all approach (Requirements → Design → Tasks) can feel mismatched. The workflow assumes medium-sized features where this progression makes natural sense.

**When to use Kiro**:

- ✅ New to SDD (low learning curve)
- ✅ Medium features (not massive systems)
- ✅ Agile-experienced team
- ✅ Want lightweight approach

**When NOT to use Kiro**:

- ❌ Large team (needs consistency enforcement)
- ❌ Complex system (needs comprehensive planning)
- ❌ Regulated domain (needs strong governance)
- ❌ Tiny bug fixes (overhead > value)

---

## Tool 2: Spec-Kit — "Governance First"

**Philosophy**: Strong governance through immutable principles.

### Foundation: CONSTITUTION

A document of rules that apply to **EVERYTHING**:

- Examples: "All passwords use bcrypt," "Test coverage 80%+," "Never log PHI"
- Non-negotiable across the entire codebase
- Enforced at specification time (before code is written)

### Workflow: Specify → Plan → Tasks (Repeatable)

**1. SPECIFY: What are we building?**

- Requirements, acceptance criteria, edge cases
- Check for Constitution violations BEFORE code
- Output: Spec document + checklist

**2. PLAN: How will we build it?**

- Architecture, technology decisions, risk assessment
- Verify Constitutional alignment
- Output: Plan document + checklist

**3. TASKS: What are the work units?**

- Atomic tasks, acceptance criteria
- Each task traces to requirements
- Output: Task breakdown + checklist
  > Additional: ADRs (Architecture Decision Records) document 'why'
  >
  > **Strengths**: Strong governance, comprehensive, scalable, traceability
  > Limitations: Steep learning curve, heavy review burden, can feel like
  > overkill for small features"

**⚠️ Trade-off Alert: File Topology & Review Burden**

> "A single spec might consist of a dozen or more files, with content that can be
> repetitive both across files and with existing code. This creates a comprehensive
> paper trail but also presents a significant review burden."

**The reality**: Where Kiro gives you 3 markdown files to review, Spec-Kit might generate:

- Constitution documents (shared)
- Specification files
- Planning files
- Task breakdowns
- Research notes
- Various checklists

You'll spend significant time reviewing specifications. For some teams, **spec review becomes MORE onerous than code review**, especially for medium-sized features.

**❓ Open Question: Spec Lifecycle**

> "Spec-Kit creates a separate branch for every spec. Is a spec meant to be a living
> artifact for the lifetime of a feature, or just for the lifetime of a change request?"

GitHub's documentation suggests aspiration toward spec-anchoring ("specifications as living, executable artifacts that evolve with the project"), but the branch-per-spec implementation suggests specs might be more ephemeral. This ambiguity means teams must decide their own spec lifecycle policy.

**🔍 How Checklists Work**

> "Rather than being rigid gate-checks that block progress, checklists are interpreted
> by AI agents as guidelines. The AI can mark items complete, flag items that need
> human attention, or identify Constitutional violations that require architectural changes."

While not providing 100% guarantee of compliance, checklists significantly improve consistency and quality by giving AI agents a "definition of done" for each phase.

**When to use Spec-Kit**:

- ✅ Large team (need consistency)
- ✅ Complex system (need traceability)
- ✅ Regulated domain (need governance)
- ✅ Long-lived project (maintenance matters)
- ✅ Team has appetite for documentation discipline

**When NOT to use Spec-Kit**:

- ❌ Learning SDD (steep curve)
- ❌ Small features (too heavy)
- ❌ Rapid iteration (Constitution is constraint)
- ❌ Team resists documentation overhead

---

## Tool 3: Tessl — "Specs Are Code"

**Vision**: Specs are the ONLY artifact developers edit. Code is generated, never hand-edited.

### Workflow: Spec-as-Source

1. **Write specification** (human edited)
2. **Run code generator** (automatic)
3. **Code produced** (marked: DO NOT EDIT)
4. **Later: Update spec** (not code)
5. **Regenerate code** (automatic)

### Promise vs. Reality

**Promise**:

- Specs stay current (always match code)
- Code always correct (regenerated from spec)
- No technical debt (no hand-edited code to maintain)

**Challenges**:

- **Non-determinism**: Run generator twice → different code?
- **Incomplete specs**: Natural language might miss edge cases
- **Performance**: Generated code might not be optimized

**Status**: Private beta 2025, still experimental

### When It Works vs. Fails

**When Tessel might work**:

- Safety-critical systems (medical devices, aerospace)
- Regulated domains (financial systems, defense)
- Stable requirements (not changing weekly)

**When Tessel doesn't work**:

- Startups with changing requirements
- Rapid iteration cycles
- Performance-critical applications

**When Tessel might work**:

- ✅ Medical device software
- ✅ Financial systems
- ✅ Aerospace/defense
- ✅ Regulatory audit requirement

**When Tessel doesn't work**:

- ❌ Startup with changing requirements
- ❌ Performance-critical system
- ❌ Experimental/exploratory work

---


## Decision Matrix: Which Tool for YOU?

Consider these questions about your context:

- Am I solo or on a team?
- How complex is my system?
- Is this a regulated domain?
- How often do requirements change?
- Do I need strong governance?

## What This Book Teaches: Spec-Kit Plus

**Spec-Kit Plus** is a variant of GitHub's Spec-Kit, customized for:

> 1. AI-native development (works with Claude, Gemini, ChatGPT)
> 2. Pedagogical clarity (designed for learning)
> 3. PHRs (Prompt History Records) - logging AI interactions
> 4. ADRs (Architectural Decision Records) - documenting 'why'
> 5. Skills (domain-specific extensions)
> 6. Subagents (specialized AI agents)
>
> It takes Spec-Kit's governance philosophy and adapts it for
> modern AI-integrated development."

**Why Spec-Kit Plus for this book:**

- Balances structure and simplicity
- Enables scaling from solo to teams
- Works with AI companions
- Teaches professional practice

---

## Extending Spec-Kit Plus: Building on the Foundation

Spec-driven development offers genuine value for teams working on complex systems with AI coding agents. We have forked GitHub SpecKit to create a specialized spec driven development tool called Spec-Kit Plus. More on the technical side in next chapter, it addresses several gaps in the original framework while building toward vertical specialization.

### Horizontal Enhancements: History and Decisions

Two critical additions provide better continuity and learning:

#### Prompt History Records (PHR)

PHRs maintain a structured log of all AI interactions during the development process. Each record captures:

- The prompt or query sent to the AI agent
- The context and phase in which it was used
- The response or code generated
- Whether the output was accepted, modified, or rejected
- The rationale for any changes

This creates an audit trail that serves multiple purposes:

- **Debugging**: When generated code doesn't work as expected, you can trace back through the decision chain to identify where understanding diverged
- **Learning**: Patterns emerge showing which prompts produce better results for specific tasks
- **Compliance**: For regulated industries, having a record of AI-assisted development decisions can be crucial
- **Collaboration**: Team members can understand not just what was built, but the reasoning path that led there

#### Architectural Decision Records (ADR)

While the Constitution defines immutable principles, ADRs capture the mutable decisions made during development. Each ADR documents:

- **Context**: What situation prompted this decision
- **Decision**: What was chosen and why
- **Alternatives considered**: What other options were evaluated
- **Consequences**: What are the implications, both positive and negative
- **Status**: Is this decision current, superseded, or deprecated

ADRs integrate beautifully with Spec-Kit because they provide the "why" behind deviations from obvious paths. When an AI agent encounters a situation where multiple approaches could satisfy the spec, it can reference relevant ADRs to understand the team's philosophy and make consistent choices.

Together, PHRs and ADRs transform Spec-Kit from a forward-looking specification tool into a complete knowledge management system that captures the "what" (specs), the "how" (implementation), and the "why" (decisions) of your codebase.

### Vertical Specialization: Skills and Subagents

The most powerful extension addresses a fundamental limitation: domain expertise. While general-purpose AI coding agents are remarkably capable, they lack deep knowledge of specific industries, frameworks, or architectural patterns that define vertical markets.

Our solution draws inspiration from Claude's Agent Skills concept—modular capabilities that extend what an AI agent can do in specific contexts. By integrating these with Spec-Kit's workflow, we create specialized development pipelines for different domains.

#### The Skills Architecture

A Skill in our extended framework is a package containing:

- **Domain-specific Constitution extensions**: Additional principles for specialized contexts (e.g., HIPAA compliance for healthcare, PCI-DSS for payments)
- **Template libraries**: Pre-built spec templates for common patterns in that vertical (e.g., patient intake workflows, checkout processes)
- **Code pattern repositories**: Reference implementations that demonstrate best practices
- **Validation rules**: Domain-specific checks that go beyond general software quality
- **Integration points**: Pre-configured connections to common tools and services in that industry

#### Subagent Specialization

Complementing Skills, Subagents are AI agents with specialized training or prompting for specific tasks within the development workflow. Rather than one general-purpose agent handling everything from architecture to implementation, you might have:

- **Compliance Subagent**: Reviews specs and code for regulatory violations
- **Security Subagent**: Focuses on threat modeling and secure implementation
- **Performance Subagent**: Analyzes architectural decisions for scalability implications
- **Integration Subagent**: Specializes in connecting to external services and APIs

During the Spec-Kit workflow, the appropriate Subagent is invoked at each phase. For instance, in the Specify phase for a financial application, the Compliance Subagent would review requirements against PCI-DSS standards, while the Security Subagent would identify potential vulnerabilities in the proposed design.

#### Platform-Native Implementation

Different AI coding agents have different strengths and native capabilities. Rather than forcing a one-size-fits-all solution, we implement vertical libraries using each platform's native extension mechanisms:

- **Claude Agent Skills**: Native to Claude, these allow deep integration with Claude's reasoning capabilities and extended context windows
- **Gemini CLI Extensions**: Conceptually similar to Claude skills, leveraging Google's multimodal capabilities and integration with Google Cloud services

By implementing the same conceptual vertical libraries across platforms, we create portable domain expertise that works regardless of which AI coding agent a team prefers. A healthcare development team using Claude gets the same specialized guidance as one using Gemini, just expressed through each platform's native mechanisms.
