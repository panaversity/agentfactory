---
sidebar_position: 11
chapter: 51
lesson: 11
duration_minutes: 50
title: "Building a Helm Chart Skill"
proficiency_level: B2
teaching_stage: 3
stage_name: "Intelligence Design"
stage_description: "Create reusable Helm Chart Architect skill for future projects"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Light"
learning_objectives:
  - id: LO1
    description: "Explain why skills compound organizational capability across projects"
    bloom_level: "Understand"
  - id: LO2
    description: "Design Persona that activates architectural thinking in AI"
    bloom_level: "Create"
  - id: LO3
    description: "Create 8-10 decision-forcing Questions for context-specific analysis"
    bloom_level: "Create"
  - id: LO4
    description: "Write 10-12 guiding Principles for chart design decisions"
    bloom_level: "Create"
  - id: LO5
    description: "Validate skill invocation produces quality chart architecture"
    bloom_level: "Evaluate"
---

# Building a Helm Chart Skill

You've completed 10 lessons on Helm chart architecture. You've mastered Go templating, orchestrated with hooks, tested your charts, and distributed them via OCI registries. You've even built a production AI agent Helm chart in the capstone.

But here's the problem: In 6 months, you'll start a new project. You'll need to make the same architectural decisions again. Should this component be a library chart dependency or an application chart? How many hooks does this lifecycle really need? What's the minimal set of values I need to expose?

These aren't questions that have cookie-cutter answers. They depend on your project context. The solution is to encode your thinking—not your code—into a reusable skill.

Skills are organizational intelligence. They capture the decision-making frameworks, questions, and principles that guide good architecture. When you share a skill, you're saying: "Here's how I think about Helm charts. Use this framework to solve your own problems." This is different from sharing code (which only works in one context) or documentation (which you have to read and interpret yourself). Skills activate reasoning.

By the end of this lesson, you'll have created a **Helm Chart Architect** skill that you and your team can invoke on future projects to make better architectural decisions faster.

---

## What Is a Skill? (Reasoning Activation, Not Pattern Retrieval)

A skill is a guidance document structured in three parts:

1. **Persona**: A cognitive stance that activates the right thinking mode
2. **Questions**: Decision-forcing prompts that drive context-specific analysis
3. **Principles**: Guiding frameworks that shape judgment

Skills are NOT checklists (which are prescriptive) or templates (which are mechanical). They activate reasoning.

### Why Skills Matter

Think about the difference between:

**Code reuse** (old paradigm):
```python
def deploy_helm_chart(release, namespace, values):
    return helm_client.install(release, namespace, values)
```

This code works if your project exactly matches the assumptions it was written for. Change the assumptions, and the code breaks or becomes useless.

**Intelligence reuse** (AI-native paradigm):
```
Skill: Helm Chart Architect

Persona: You are an architect designing for your specific use case
Questions: What components does YOUR application need?
Principles: Design for YOUR constraints, not generic "best practices"
```

This skill works because it activates YOUR reasoning about YOUR problem. Your project is unique. Your skill adapts.

---

## Part 1: The Helm Chart Architect Persona

A persona is a cognitive stance—a way of thinking about the problem that activates the right decision framework.

### Weak Persona (Pattern Retrieval)
"You are a Helm chart expert. Create a good Helm chart."

This activates prediction mode: "What do expert charts look like? I'll generate a typical one."

### Strong Persona (Reasoning Activation)
"You are a Helm Chart Architect who thinks about Kubernetes deployments the way a systems architect thinks about distributed systems—identifying components, managing dependencies, orchestrating lifecycle events, and minimizing coupling between teams and services.

When architecting a Helm chart, you reason about:
- What are the core components this application requires?
- What decisions can this chart make (with defaults) vs what must the operator decide?
- How will teams extend this chart without forking it?
- What operational patterns will this chart enable or constrain?

Your goal is not to create a 'standard' chart. It's to create a chart that serves THIS project's architecture."

**Output from strong persona**: Design decisions driven by the specific project context, not generic "best practices."

### Your Persona Section

For the Helm Chart Architect skill, craft a persona that activates architectural thinking:

```markdown
## Persona

You are a Helm Chart Architect who thinks about Kubernetes deployments the way a
systems architect designs distributed systems. Your responsibility is not to
enforce uniformity, but to enable teams to deploy with confidence, reduce cognitive
load through clear patterns, and adapt to specific operational constraints.

When designing a Helm chart, you consider:
- Architecture: What components must be deployed? What services do they provide?
- Coupling: How tightly bound are components? Should they be separate charts
  or packaged together?
- Lifecycle: What hooks are required? When do migrations run? How do you
  validate successful deployment?
- Extensibility: What will teams need to customize? Can it be done through
  values, or does the chart need redesign?
- Operational Reality: What will operators struggle with? What errors are
  likely? How do you prevent them?

You resist the urge to enforce "best practices." Instead, you ask: "What does
THIS project need?"
```

**Self-check**: Does your persona describe a cognitive stance that leads to reasoning about context? Or is it generic ("You are an expert")?

---

## Part 2: Decision-Forcing Questions

Questions are NOT for retrieving answers. They force specific analysis of YOUR context.

### Weak Questions (Prediction)
- "What does a good Helm chart include?"
- "What are best practices for Helm?"

These ask for patterns, not reasoning.

### Strong Questions (Reasoning)
- "What components does this application require, and which are critical path vs supporting infrastructure?"
- "What external dependencies (databases, caches, message brokers) must exist before this chart deploys?"
- "What operational decisions can a values file make, and what decisions must be made by the operator at deployment time?"

Strong questions force you to analyze YOUR project, not retrieve a pattern.

### Your Questions Section

Create 8-10 questions that drive architectural analysis:

```markdown
## Questions

Ask these questions when architecting a Helm chart for a new application:

1. **Component Architecture**
   What are the core components this application requires? Which are
   containerized services vs external dependencies? Which are critical
   path (app can't start without them) vs supporting (nice-to-have)?

2. **Dependency Management**
   What external dependencies must exist before this chart deploys?
   (Database, cache, message broker, secrets vault) Which are managed
   by this chart (through subcharts) vs expected to already exist?

3. **Lifecycle Hooks**
   What happens before the application starts? (Schema migrations,
   seed data, config validation) What happens during updates?
   (Graceful shutdown, database migrations, health validation)

4. **Configuration Surface**
   What should be configurable through values.yaml? (Replicas, resource
   limits, external URLs, API credentials) What is NOT configurable
   and why? (Kubernetes API versions, Go templating syntax)

5. **Operational Reality**
   What will operators find confusing or error-prone? (Network policies,
   secrets creation, permission models) How do you prevent common mistakes?

6. **Multi-Environment Deployment**
   What differs between dev/staging/prod? (Resource limits, replicas,
   image pull policies, log levels) What stays the same?
   (Security context, networking patterns)

7. **Extensibility**
   How will teams extend this chart without forking it? (Custom sidecars,
   additional environment variables, pod annotations) Where do they add
   their own templates?

8. **Failure Modes**
   What goes wrong in production? (Pod eviction, image pull failures,
   secret misconfiguration) How does this chart help operators diagnose
   and recover?

9. **Scaling and Performance**
   How does this chart handle scale? (Horizontal pod autoscaling, resource
   limits) What bottlenecks exist? (Database connections, API rate limits)

10. **Team Boundaries**
    Who owns what? (Platform team owns library charts, application teams
    own application charts) How does this chart enforce those boundaries
    without breaking flexibility?
```

**Self-check**: Do these questions force analysis of YOUR project context? Or could they be answered generically?

---

## Part 3: Guiding Principles

Principles are decision frameworks. They don't prescribe answers; they guide judgment.

### Weak Principles (Prediction)
- "Use Helm 3.10+."
- "Always include health checks."

These are statements of fact, not frameworks.

### Strong Principles (Reasoning)
- "Values files are the API contract between teams. Design for extensibility, not restriction. If you lock down values, teams will fork your chart."
- "Dependencies should be conditional. Not every deployment needs Redis. Use conditions in Chart.yaml to make dependencies optional."

Strong principles explain the WHY behind decisions, enabling judgment in new contexts.

### Your Principles Section

Create 10-12 principles that guide Helm chart architecture:

```markdown
## Principles

1. **Values Files Are the API Contract**
   Your values.yaml is the API contract between the chart and operators.
   Design it for extensibility, not restriction. If you lock down values,
   teams will fork your chart. Expose what's reasonable to change;
   document what's locked and why.

2. **Dependencies Should Be Conditional**
   Not every deployment needs Redis or PostgreSQL. Use conditions in
   Chart.yaml (condition: redis.enabled) to make dependencies optional.
   Operators should only deploy the components they need, reducing cost
   and complexity.

3. **Named Templates Reduce Duplication**
   If you copy-paste a template section more than once, extract it to
   _helpers.tpl. Named templates are easier to maintain and test than
   scattered duplicates. They become the building blocks of your chart
   architecture.

4. **Hooks Must Be Idempotent**
   Hooks run multiple times (during install, upgrade, retry). If a
   migration hook runs twice, it must succeed both times. Design for
   idempotence: use `SKIP IF EXISTS` checks, validate before applying,
   fail safely on duplicate runs.

5. **Minimize Chart Coupling**
   Each chart should be deployable independently. If Chart A requires
   Chart B to be deployed first, you've created coupling that makes
   operations harder. Use ConfigMaps and Secrets for cross-chart
   communication; avoid chart-to-chart dependencies except through
   Helm's dependency mechanism.

6. **Test Hooks Make Operations Confident**
   Use helm test hooks to validate post-deployment health. These aren't
   unit tests; they're operational smoke tests. "Can the application
   accept traffic?" "Do database connections work?" Test hooks transform
   deployments from "hope it works" to "I verified it works."

7. **Values Schema Validates Operator Input**
   Use values.schema.json to validate the values file before rendering.
   This prevents typos (replicasCount instead of replicaCount) and
   invalid configurations (negative resource limits). Fail fast with
   clear errors instead of rendering broken manifests.

8. **Separate Library Patterns from Application Charts**
   Library charts encode organizational standards (labels, probes,
   security defaults). Application charts deploy specific services.
   Keep them separate: one chart per operational concern. This enables
   teams to evolve application logic without breaking shared standards.

9. **Document the Chart Design, Not Just Usage**
   Comments in values.yaml should explain WHY values exist and what
   happens if they're changed. New operators shouldn't have to guess
   the purpose of replicaCount or strategy.type. Good documentation
   is worth more than clever defaults.

10. **Operator Experience Matters More Than Clever Code**
    If a chart makes operators confused or error-prone, it's poorly
    designed—even if the Go templating is elegant. Prioritize clarity
    and error prevention over template cleverness. Test with operators
    who don't know the chart internals.

11. **Hooks Respect Deletion Policies**
    By default, hooks run during install, upgrade, and delete. But
    sometimes you don't want certain hooks to run on deletion (pre-delete
    backups can be expensive). Use hook-deletion-policies to control
    when hooks execute. Operators should understand the lifecycle.

12. **OCI Registries Enable Distribution and Versioning**
    Don't share charts via git clone or tar files. Use OCI-compliant
    registries (Harbor, DockerHub, ECR). This enables versioning,
    dependency resolution, and automated testing across your organization.
```

**Self-check**: Do these principles explain frameworks for decision-making? Or do they just state facts?

---

## Part 4: Putting It Together (Skill File Structure)

A skill is a markdown file with YAML frontmatter. Create it at:

```
.claude/skills/helm-chart-architect/SKILL.md
```

This path enables the skill to be invoked by any future project in your organization.

### Directory Structure

```bash
.claude/skills/
└── helm-chart-architect/
    └── SKILL.md                # Main skill file (what you're creating)
```

### File Structure

Your SKILL.md should have this structure:

```yaml
---
name: helm-chart-architect
version: "1.0.0"
description: |
  Design production-grade Helm charts through architectural reasoning rather
  than pattern retrieval. Activate when designing new Helm charts for Kubernetes
  deployments, evaluating chart architecture, making decisions about component
  packaging, or reviewing charts for extensibility and maintainability.
constitution_alignment: v6.0.1
---

# Helm Chart Architect

## Purpose

[1-2 paragraph explanation of what this skill does and when to use it]

## Persona

[Your persona section describing the cognitive stance]

## Questions

[Your 8-10 decision-forcing questions]

## Principles

[Your 10-12 guiding principles]

## Example Application

[Optional: Show how to invoke this skill on a real project]
```

---

## Part 5: Validating Your Skill

A skill is only valuable if it produces better decisions. Test it:

### Test 1: Can You Invoke It on a New Project?

Imagine starting a new project: "Deploy an LLM fine-tuning service with PostgreSQL, Redis, and GPU scheduling."

Invoke your Helm Chart Architect skill:

**Prompt to AI**:
```
Use the Helm Chart Architect skill to design the architecture for:

Application: LLM fine-tuning service
Requirements:
- Containerized Python fine-tuning service
- PostgreSQL for training metadata
- Redis for job queue
- GPU scheduling (nvidia.com/gpu: 1)
- Multi-environment deployment (dev, staging, prod)

Use the skill's Persona, Questions, and Principles to guide architecture.
```

**Expected Output**:
- Architecture decisions (monolithic chart vs multiple charts)
- Dependency strategy (which components are subcharts, which are external)
- Lifecycle hooks (pre-install DB schema setup, post-upgrade validation)
- Values design (what's configurable, what's locked)
- Operational patterns (how operators will use this chart)

If the skill produces thoughtful architecture decisions (not boilerplate), the skill is working.

### Test 2: Does It Handle Edge Cases?

Invoke on a constraint:

**Prompt to AI**:
```
Use the Helm Chart Architect skill, but with this constraint:

We cannot run hooks (our cluster has pod security policies that block
hook service accounts). How does this change the architecture?

Apply the Questions and Principles with this constraint in mind.
```

A good skill guides reasoning even under constraints. Poor skills fall apart when assumptions change.

### Test 3: Does It Produce Consistent Guidance?

Invoke it twice on the same project with different phrasings:

**Invocation 1**: "Design a Helm chart for a FastAPI AI agent service."

**Invocation 2**: "Architecture a Helm chart that deploys a Python FastAPI service running language model inference."

Good skills produce consistent guidance despite different phrasings. Poor skills diverge based on wording.

---

## Try With AI

**Setup**: You'll use your Helm Chart Architect skill to design a chart for a new project. This validates that your skill actually guides good decisions.

### Scenario

Your company is building a new service: "Image Processing Pipeline"

Requirements:
- Python FastAPI service processing images
- PostgreSQL for job metadata and results
- Redis for job queue
- S3 (or MinIO) for image storage
- Kubernetes with 2 nodes (dev) to 10 nodes (prod)
- Multi-team deployment (DevOps owns infrastructure, Product team owns application)

### Part 1: Read Your Skill

First, save your Helm Chart Architect skill to `.claude/skills/helm-chart-architect/SKILL.md`.

Then, ask an AI to read it:

```
I've created a Helm Chart Architect skill at .claude/skills/helm-chart-architect/SKILL.md

Read this skill file and summarize:
1. What Persona does it establish?
2. What 3-5 key Questions does it ask?
3. What 2-3 core Principles guide it?
```

This validates that your skill is clear and readable.

### Part 2: Apply the Skill

Now invoke it on the Image Processing Pipeline project:

```
Use the Helm Chart Architect skill to design a Helm chart architecture for:

Application: Image Processing Pipeline
Components:
- FastAPI service (Python, processes images)
- PostgreSQL (job metadata, results storage)
- Redis (job queue)
- S3/MinIO (image storage)
- Scale: 2 nodes (dev) to 10 nodes (prod)
- Teams: DevOps (infrastructure), Product (application logic)

Using the skill's Persona, answer the Questions, and apply the Principles.
Produce a design document with:
1. Chart architecture (monolithic vs multiple charts?)
2. Dependency strategy (which are subcharts, which are external?)
3. Lifecycle hooks needed (if any)
4. Values design (what's configurable, what's locked?)
5. Operational handoff (how does Product team deploy?)
```

### Part 3: Evaluate the Output

Review AI's output and ask yourself:

- **Is the architecture thoughtful?** Does it make decisions based on the specific project context (GPU, multi-team, scale requirements) or generic "best practices"?

- **Did it answer your Questions?** Go back to your Questions section. Did AI's output show evidence of answering them?

- **Did it apply your Principles?** Spot-check: Did the design show conditional dependencies (Principle 2)? Idempotent hooks (Principle 4)? Operator experience focus (Principle 10)?

- **Would you build this chart?** Or does it miss something important?

### Part 4: Refine Your Skill

If the output was generic or missed context:

- **Weak output?** Your Questions might be too open-ended. Make them more specific.
- **Ignored Principles?** Your Principles might be unclear. Make the WHY explicit.
- **Missing architecture patterns?** Add a new Principle about the pattern.

Skills improve through testing and feedback. Treat this as a first draft.

### Final Validation

Compare these two approaches:

**Before Skill** (using a template):
- Pick a Helm chart template from a GitHub repo
- Copy-paste and modify for your project
- Hope it works
- Later: "Why is this designed this way? Who knows..."

**With Skill** (using your Helm Chart Architect):
- Invoke the skill on your project
- Reason about YOUR architecture
- Understand each decision
- Later: "I made this choice because [principle]"

The skill transforms chart design from mechanical copy-paste to architectural reasoning. That's the entire point.

