---
sidebar_position: 22
chapter: 50
lesson: 22
duration_minutes: 35
title: "Building the Kubernetes Deployment Skill"
proficiency_level: B1
teaching_stage: 3
stage_name: "Intelligence Design"
stage_description: "Transform deployment knowledge into reusable, cross-project skill"
cognitive_load:
  concepts_count: 6
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Extract deployment patterns from lessons 1-9 into reusable guidance framework"
    bloom_level: "Analyze"
  - id: LO2
    description: "Structure skill using Persona + Questions + Principles pattern"
    bloom_level: "Apply"
  - id: LO3
    description: "Articulate Kubernetes deployment principles as decision frameworks"
    bloom_level: "Apply"
  - id: LO4
    description: "Test skill on different application (not the Part 6 agent)"
    bloom_level: "Evaluate"
  - id: LO5
    description: "Understand cross-project value of accumulated intelligence"
    bloom_level: "Understand"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "2. Communication and Collaboration"
    competency: "2.3 Engagement in communities and networks"
  - objective_id: LO2
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools to solve problems"
  - objective_id: LO3
    competency_area: "5. Problem Solving"
    competency: "5.4 Identifying digital competence gaps"
  - objective_id: LO4
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO5
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
---

# Building the Kubernetes Deployment Skill

Over lessons 1-9, you've built mental models of Kubernetes concepts (Layers 1-2), then applied them in a capstone project that deployed your FastAPI agent to a cluster (Layer 4).

Now comes Layer 3: recognizing that the deployment pattern you just executed recurs across projects. Different applications. Different teams. Same core decisions: How do you balance resource requests against actual usage? When do you need liveness vs readiness probes? How do you structure labels for operational visibility?

This lesson guides you to extract that recurring knowledge into a reusable skill—a guidance framework that future projects can reference without rediscovering the same principles.

---

## From One-Time Knowledge to Reusable Intelligence

Your capstone deployment created a working application on Kubernetes. But the knowledge you built—the decision frameworks, the principles, the troubleshooting patterns—was embedded in one project.

If you deploy a second application next month, you'll face the same decisions:
- "Should this container have CPU limits?"
- "What probes do I actually need?"
- "How should I label things for monitoring?"

A skill captures these decisions in a format that next month's you can reference without rediscovering answers.

### What Makes a Pattern Worth Extracting?

Not every workflow becomes a skill. Consider three tests:

**1. Frequency**: Does this pattern recur across projects?
- Your Kubernetes deployment pattern: Yes. Any containerized workload on K8s follows similar decision paths.
- One-off emergency debugging: No. Skip skill creation.

**2. Complexity**: Does the pattern involve enough decision points to justify documentation?
- Kubernetes deployment: Yes. Resource limits, probes, labels, environment configuration, rollback strategy—each involves tradeoffs.
- "Run kubectl apply": No. Too simple.

**3. Organizational value**: Would multiple teams benefit from this guidance?
- Kubernetes deployment: Yes. Data teams, ML teams, web teams, API teams all deploy to K8s.
- Team-specific workaround: No. Document locally, don't package as reusable skill.

Your Kubernetes deployment pattern passes all three tests. Extract it.

---

## The Skill Structure: Persona + Questions + Principles

A reusable skill activates reasoning, not pattern retrieval. It does this through three components:

**Persona** defines the thinking stance that activates the right reasoning mode.

Instead of generic ("You are an expert in Kubernetes"), a strong persona forces specific analytical thinking:

```
Think like a DevOps engineer balancing deployment speed, resource costs,
and operational reliability for production clusters.
```

This persona activates multiple decision frameworks at once: speed vs cost tradeoffs, reliability requirements, cluster constraints.

**Questions** structure the analysis that forces context-specific reasoning.

Instead of asking "Is this secure?", strong questions force deeper analysis:

```
- What happens if this container's memory request is too high?
  (Cost and cluster scheduling consequences)
- What happens if it's too low?
  (OOMKill, crashes, customer impact)
- How do you discover the right value?
  (Load testing, monitoring, iteration)
```

**Principles** articulate the decision frameworks that guide application.

Instead of prescriptive rules ("Always set CPU limits"), strong principles explain the reasoning:

```
Resource requests drive scheduling. Resource limits prevent noisy neighbor problems.
But limits that are too tight cause crashes. Start with monitoring actual usage,
then set limits slightly above observed peak.
```

Together, these three components create a guidance framework. When someone deploys a new application, they:
1. Adopt the Persona (think like DevOps)
2. Work through the Questions (analyze their specific context)
3. Apply the Principles (make informed decisions)

---

## Building Your Skill: Step-by-Step

You'll create a skill file that captures deployment patterns from this chapter. The skill lives in:

```
.claude/skills/kubernetes-deployment/SKILL.md
```

### Step 1: Define Your Persona

Your persona should activate the decision-making mode you used throughout this chapter. Think about:

- **What role did you play?** You were an operator deploying applications to a cluster, balancing competing concerns.
- **What constraints shaped your thinking?** Production reliability, resource efficiency, operational visibility.
- **What expertise does this persona assume?** Understanding of containerization (Docker), basic cluster concepts, operations mindset.

Your persona might read:

```
Think like a cluster operator deploying containerized applications to
production Kubernetes. Your goal: reliable, observable applications that
don't waste resources. Your constraints: limited cluster capacity,
production service level agreements, operator time availability.
```

Write your persona in 2-3 sentences. Make it specific enough to activate thinking, not generic.

### Step 2: Identify Your Decision Points

Each decision point in the skill should answer a question that someone deploying a new application would ask.

From this chapter, your decision points include:

**Decision Point 1: Resource Management**
- Question: How much CPU and memory should my container request and limit?
- From lessons: 6, 7, 9
- Decision framework: Balance availability (high limits), cost (low requests), reliability (adequate resources)

**Decision Point 2: Health Checking**
- Question: What probes does my application need?
- From lessons: 3, 7, 8, 9
- Decision framework: Liveness catches hangs, readiness catches startup delays, together they enable self-healing

**Decision Point 3: Operational Visibility**
- Question: How should I structure labels and metadata?
- From lessons: 1, 4, 5, 7, 9
- Decision framework: Labels enable querying, monitoring, and operational debugging

**Decision Point 4: Configuration and Secrets**
- Question: How do I inject application configuration and protect sensitive data?
- From lessons: 6, 8, 9
- Decision framework: ConfigMaps for configuration, Secrets for sensitive data, environment variables for injection

**Decision Point 5: Deployment Strategy**
- Question: How do I update my application without downtime?
- From lessons: 4, 8, 9
- Decision framework: Rolling updates require proper health checks and resource padding

**Decision Point 6: Disaster Recovery**
- Question: What happens when things go wrong, and how do I recover?
- From lessons: 2, 7, 9
- Decision framework: Monitoring surfaces problems, rollbacks fix them, replicas provide availability

For your skill, extract 3-4 of these decision points. You'll create a question for each that guides someone through the decision.

### Step 3: Write Your Questions

Questions should force analysis of context, not retrieve memorized answers.

For each decision point, write 2-3 questions that guide someone through the reasoning:

**Resource Management Questions**:
```
- What does your application do during peak load?
  (Compute-intensive? I/O-bound? Long-running? Quick responses?)
- How do you currently know how much CPU and memory your app uses?
  (Monitoring data? Educated guess? Nothing?)
- What happens if your container runs out of memory?
  (How would your customers experience this? What's the impact?)
- How much extra headroom do you build in to handle traffic spikes?
  (1.2x peak? 1.5x? 2x?)
```

These questions don't have "right" answers. Instead, they guide someone through analyzing their specific context, then making informed decisions.

**Health Checking Questions**:
```
- What does it mean for your application to be "ready" to accept requests?
  (Fully booted? Database connected? Caches primed? Something else?)
- How long does your application take from start to ready?
  (Seconds? Minutes?)
- What failures would require restarting the container?
  (Deadlocks? Memory leaks? External service unavailable?)
- How do you test whether your probes work correctly?
```

Again, the questions guide analysis. Someone working through these questions will make better decisions than someone following prescriptive rules.

### Step 4: Articulate Your Principles

Principles explain the decision frameworks that guide the skill's application.

For resource management:

```
**Principle: Requests drive scheduling, limits prevent waste**

Container requests tell Kubernetes: "This container needs at least this much
CPU and memory to function." Kubernetes uses requests to decide which node
to place your Pod on.

Container limits tell Kubernetes: "Kill this container if it uses more than
this much memory" (memory is hard limit), or "throttle this container's CPU
if it exceeds this much" (CPU is soft limit).

If requests are too high, you're wasting cluster capacity. If requests are
too low, your application gets evicted when the node is full.

If limits are too low, your application crashes (memory) or performs poorly
(CPU throttling). If limits are absent, one misbehaving application can
starve the entire cluster.

Decision framework: Start with monitoring actual usage from development or
staging. Set requests to 75-80% of observed peak. Set limits to 1.2-1.5x
observed peak. Tune after observing production behavior.
```

For health checking:

```
**Principle: Probes enable self-healing**

Liveness probes answer: "Is this container still useful?" If the answer is
no, Kubernetes restarts it.

Readiness probes answer: "Can this container accept traffic?" If the answer
is no, the Service stops routing to it temporarily.

Together, they enable the self-healing cluster behavior: Bad container? Kill
it, start a new one. Container starting up? Don't route traffic to it yet.
Container temporarily overwhelmed? Stop routing, it catches up.

Decision framework: Every container needs both probes. Liveness detects
hangs and deadlocks. Readiness detects startup delays and dependency issues.
```

Your principles should explain the reasoning behind decisions, not prescribe specific values.

### Step 5: Create the Skill File

Create the directory and file:

```bash
mkdir -p .claude/skills/kubernetes-deployment
touch .claude/skills/kubernetes-deployment/SKILL.md
```

Inside, structure your skill with YAML frontmatter:

```yaml
---
name: "kubernetes-deployment"
description: |
  Deploy containerized applications to Kubernetes clusters with reliability and
  efficiency. Guides decisions on resource management, health checking,
  configuration injection, and deployment strategies. Use when deploying to
  Kubernetes, troubleshooting cluster issues, or designing cluster-aware applications.
version: "1.0.0"
---

# Kubernetes Deployment Guidance

## Your Persona

Think like [your persona here]

## Your Decision Points & Questions

### Decision 1: Resource Management

[Your questions here]

### Decision 2: Health Checking

[Your questions here]

### Decision 3: [Your Decision Point]

[Your questions here]

## Your Principles

### Principle 1: [Resource Management]

[Your principle explanation here]

### Principle 2: [Health Checking]

[Your principle explanation here]

### Principle 3: [Your Principle]

[Your principle explanation here]

## Common Patterns

[Patterns you noticed while deploying in lesson 9]

## Troubleshooting Checklist

[Issues and solutions you encountered]
```

Fill in each section with content from your experience in lessons 1-9.

---

## Testing Your Skill: Deploy a Different Application

Creating the skill is half the work. The other half is validating that it actually guides thinking about a NEW application.

Your capstone deployed your FastAPI agent. Now imagine deploying a completely different application—something you haven't worked with in this chapter.

### Choose Your Test Application

Pick one:

- **Python data processing job**: Runs for 1-2 hours, uses significant CPU and memory, processes a large file
- **Node.js web service**: Handles HTTP requests, connects to external APIs, should be highly available
- **Go batch processor**: Runs periodically, loads configuration from external sources, needs safe shutdown
- **Python API gateway**: Routes requests to multiple backends, high request volume, needs low latency

The application type doesn't matter. What matters: It's different enough from your FastAPI agent that you can't just copy manifests from lesson 9.

### Work Through Your Skill

Using only your skill (not lesson 9's code), answer these questions:

1. **Resource Planning**: What CPU and memory requests/limits would you set for this application? Why?
2. **Health Checking**: What probes would this application need? How would you implement them?
3. **Configuration**: What would need to be ConfigMaps vs Secrets for this application?
4. **Labels**: How would you structure labels to describe this application?
5. **Deployment Strategy**: What special considerations exist for updating this application?

Your skill should guide you through these decisions without prescribing specific answers.

### Reflection

After working through your skill on a different application:

- **What worked?** Which questions forced useful analysis?
- **What was missing?** What decisions did the skill not address?
- **What was too prescriptive?** Did any principles feel like rules instead of frameworks?
- **How would you improve it?** What would make it more useful?

This reflection becomes input for improving your skill for future use.

---

## The Broader Value: Cross-Project Accumulation

Why create this skill instead of just remembering how you deployed in lesson 9?

**Lesson 9 embedded the knowledge in one project.** Your capstone deployment worked, but the decision frameworks were context-specific.

**Your skill crystallizes the knowledge for reuse.** Three months from now, when you're deploying a different application to a different cluster, you reference this skill instead of rediscovering answers.

**And the skill compounds.** As you deploy more applications, you'll encounter variations, edge cases, and new decision points. You'll add them to the skill. Your teammates will reference it. The skill becomes a shared reference for deployment decisions across the organization.

This is intelligence accumulation. Not "I know how to deploy FastAPI agents" (too specific). Not "I'm good at Kubernetes" (vague). But "I have a guidance framework for deploying ANY containerized application to Kubernetes" (reusable, continuously improving).

That's the value of Layer 3 intelligence design.

---

## Try With AI

**Setup**: You have your Kubernetes deployment skill created. Now use AI to validate and improve it.

**Part 1: Skill Review**
Share your skill with Claude using this prompt:

```
I've created a Kubernetes deployment guidance skill for deploying
containerized applications to Kubernetes. Here's my skill:

[Paste your complete SKILL.md file]

Review my skill and identify:
- What decision points are well-guided?
- What decisions are missing or unclear?
- Where are principles too prescriptive (like rules) vs appropriately
  general (like frameworks)?
- How would someone deploying a new application use this skill?
```

Ask Claude to identify gaps in your skill's guidance. Use the feedback to strengthen the framework.

**Part 2: Apply Skill to New Context**
Ask Claude to work through your skill for a different application type:

```
Using my skill above, guide me through deploying a Python data processing
job that:
- Runs for 1-2 hours
- Uses significant CPU and memory
- Processes large files
- Needs to be restarted if it hangs

Walk through your Persona mindset, then work through each decision point in
my skill to design a deployment manifest for this application. Where does my
skill guide clearly? Where is it unclear?
```

Claude's response will reveal what parts of your skill actually guide decisions and what parts need refinement.

**Part 3: Validate Against Production Patterns**
Ask Claude for deployment patterns you might have missed:

```
I created a skill for Kubernetes deployment based on lessons in a course.
My skill addresses:
- Resource management
- Health checking
- Configuration injection
- Labeling
- Deployment strategy
- Disaster recovery

Looking at production Kubernetes deployments, what critical patterns am I
missing? What decisions would a production operator make that my skill
doesn't address?
```

Claude might identify security hardening, affinity rules, network policies, or observability concerns you hadn't considered. Add these to your skill if they're appropriate for your context.

**Part 4: Test Cross-Project Application**
Finally, validate that your skill truly guides cross-project thinking:

```
I'm deploying a Node.js API gateway to Kubernetes. It handles HTTP requests,
connects to external APIs, needs high availability. Using my skill, help me
design the deployment manifest. As you work through the skill, tell me:
- Which principles guide your decisions?
- Where do you need to adapt the skill for this specific context?
- What decisions does my skill not address for an API gateway?
```

This validates that your skill is truly reusable, not just a summary of lesson 9's specific agent deployment.

**Reflection**: Compare your skill guidance across these different applications. A well-designed skill should guide fundamentally similar decisions (resource balance, health checking) even when the specific answers differ.
