---
title: "Trace SDD Through Your Company"
chapter: 32
lesson: 3
duration: "2 hours"
skills:
  - name: "Organizational Analysis"
    proficiency: "B1"
    category: "Soft"
  - name: "Value Chain Mapping"
    proficiency: "B1"
    category: "Conceptual"
  - name: "SDD Adoption Assessment"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Map how specifications flow through organizational roles and processes (B1)"
  - "Identify opportunities for SDD adoption in your company (B1)"
  - "Assess business value of specification-driven practices (B1)"
---

# Trace SDD Through Your Company

## What You're About to Discover

In Lessons 1 and 2, you worked with small teams (2-5 people) using specifications. Now you're going to scale your thinking to the organizational level.

How do specifications enable a company with 100 engineers to avoid chaos? How do teams that don't even know each other still ship features that work together? How does a company like Stripe or Amazon avoid bottlenecks?

The answer: They use specifications at every level of organization.

In this lesson, you'll trace how specifications flow through a growing company, from your small grading system team (5 people) all the way to enterprise scale (100+ people). You'll discover that the same principles apply, but specifications take different forms at different scales.

---

## Part 1: From Team to Organization

Let's scale up progressively.

### Scale 1: Solo Developer (1 person)

**Your grading system**
- You write specs
- You implement
- You test
- You deploy

**Specs used for**: Thinking clearly about what to build

**Coordination needed**: None

---

### Scale 2: Small Team (5 people)

**Your grading system team** (from Lesson 2)
- 1 spec lead, 3 developers, 1 QA engineer
- All coordinate through specifications
- Synchronous communication: spec review, integration, deployment decisions

**Specs used for**:
- Feature definition (what each developer builds)
- Integration contracts (how features connect)
- Test criteria (how QA knows if it works)

**Coordination needed**: Specification review meetings, integration phase

---

### Scale 3: Department (15-20 people)

**Now the grading system team is part of a larger "Learning Management" department**

**Who's involved**:
- Grading system team (5 people)
- Student profile team (5 people)
- Assignment management team (5 people)
- Analytics team (3 people)

**New complexity**:
- These teams aren't building the same feature
- They're building different features that must integrate
- For example: Grading system needs to read from assignment management system
- Grading system needs to write to analytics database

**How specs handle this**:
- Assignment management team publishes: "Here's our API spec. Grading system can query assignments through this API."
- Grading system team trusts that spec
- Grading system developer builds against the published API spec
- When assignment management team implements, they commit to matching their spec
- Integration happens because both teams committed to specs

**Specs used for**:
- Service contracts (APIs between teams)
- Data formats (what data each service produces/consumes)
- Reliability contracts (SLAs: "This API will be 99.9% available")

**Coordination needed**: Cross-team specification review (happens once), then async development

---

### Scale 4: Company (50-100 people)

**Learning Management Platform at a mid-size company**

**Team structure**:
- 5 product teams (grading, assignments, student profiles, analytics, compliance)
- Each team has 5-10 engineers
- Product leadership (2-3 people) who set strategy
- Platform team (5-8 people) who manage shared infrastructure
- Data team (3-5 people) who manage databases and data pipelines

**How does this work?**

Think of it as a graph:
- Each team is a node
- Each specification is an edge connecting nodes
- Services communicate through published specs

**Example specification chain**:
1. **Student Profile Service** publishes: "Student schema (ID, name, email, etc.)"
2. **Grading Service** publishes: "Grade schema (student_id, score, rubric)"
3. **Analytics Service** publishes: "Analytics API: Get grades by cohort"
4. **Reporting Service** depends on both Grading and Analytics specs

**How coordination works**:
- Reporting team doesn't ask Grading team "Can we access grades?"
- Instead: Reporting team reads Grading team's published spec
- Reporting team builds against that spec
- When Grading team implements, they match their spec
- Integration is automatic

**This is the miracle of specifications at scale**: Teams work independently because specs define clear contracts.

**Specs used for**:
- Service contracts (APIs)
- Data models (schemas)
- Reliability contracts (SLAs)
- Behavioral contracts (how services respond to edge cases)
- Versioning strategy (how we change specs without breaking everything)

**Coordination needed**: Cross-team spec reviews (quarterly), async development (daily)

---

### Scale 5: Enterprise (200-500 people)

**At companies like Amazon, Google, Microsoft, Stripe**

**Team structure**:
- 20-50 product teams
- Hundreds of engineers
- Dedicated service teams
- Platform teams supporting infrastructure
- Data infrastructure teams
- DevOps teams

**How does this work at enterprise scale?**

Specifications become organizational infrastructure. It's not optional—it's how the company operates.

**Spec types**:
1. **Service specifications** (what each service does)
2. **API specifications** (how to call services)
3. **Data specifications** (schemas, formats)
4. **Infrastructure specifications** (how services run)
5. **Deployment specifications** (how code gets to production)
6. **Monitoring specifications** (what metrics to track)

**Governance**:
- Spec review committees (not individual reviews)
- Specification standards (all services follow the same spec format)
- Specification versioning (how specs change without breaking dependencies)
- Specification registries (catalog of all specs)

**The principle stays the same**: **Specifications replace synchronous communication.** Engineers don't ask managers what to do. Engineers read specifications. Engineers implement to specs. Managers review specs to guide strategy.

---

## Part 2: Trace Your Grading System at Different Scales

Now let's imagine your grading system growing through scales.

Ask your companion:

> **Prompt**: "I'm imagining my grading system at different organizational scales. At 1 person, it's just Python code. At 5 people, it's 2-3 services with integration contracts. At 50 people (part of a larger LMS), it's a service that publishes APIs. At 500 people (enterprise), it's one of 50 services. For each scale, show me: (1) How many teams depend on my specs? (2) What specs do I publish? (3) How many other teams' specs do I depend on? (4) What could go wrong if my specs are ambiguous?"

Your companion will help you trace something like:

**Your grading system at scale 1**:
- No one depends on your spec except you
- Risk: Low (only you pays cost of ambiguity)

**Your grading system at scale 5**:
- QA engineer and 3 developers depend on your spec
- Your spec must be clear (Integration test will fail if spec is ambiguous)
- Risk: Medium (4 people, 1 week of integration rework)

**Your grading system at scale 50**:
- 5 team members depend on your spec
- 10-15 people from other teams depend on your API spec
- Other teams can't ship features if your spec is wrong
- Risk: High (15 people blocked, cascading delays)

**Your grading system at scale 500**:
- 5 team members depend on your spec
- 50-100 people from other teams depend on your API spec
- Multiple critical services depend on your data format
- One spec change could require changes in 5 other services
- Risk: Very High (organizational impact)

This is why specification quality becomes non-negotiable at scale.

---

## Part 3: Discover Organizational Patterns

Let's extract the patterns that emerge as you scale.

### Pattern 1: Specifications Replace Meetings

At scale 1, you don't need meetings. You know what you're building.

At scale 5, you have spec review meetings (necessary).

At scale 50, you don't have daily meetings because specs define alignment. You have quarterly sync meetings to review architectural decisions, but daily work doesn't require coordination meetings.

At scale 500, the company has very few meetings. Specifications are the default communication medium. Meetings happen for exceptions, not for coordination.

**Why?** Because 500 people can't coordinate through meetings. Specs are the only scalable communication medium.

### Pattern 2: Hierarchy of Specs

At scale 1: One specification (your code).

At scale 5: Feature specs (what each developer builds) + integration contracts (how features connect).

At scale 50: Feature specs + service specs (what services offer) + API specs (how to call services).

At scale 500: Feature specs + service specs + API specs + schema specs + infrastructure specs + organizational specs (how different teams relate to each other).

Each level builds on the previous level.

### Pattern 3: Governance Centralizes Around Specs

At scale 1: No governance (just you).

At scale 5: Simple governance (spec lead approves changes).

At scale 50: Governance committee (architectural decisions require approval; affects multiple teams).

At scale 500: Specification infrastructure (spec formats, versioning standards, approval authority).

---

## Part 4: Design Your Company's Specification Strategy

Now let's make this real. Imagine your company at different scales and design how specifications would enable growth.

Ask your companion:

> **Prompt**: "I'm designing a specification strategy for a company that might grow from 5 people to 100+ people over 3 years. For each milestone (5 people, 15 people, 50 people, 100+ people), tell me: (1) What kinds of specs exist? (2) Who's responsible for specs? (3) How are specs reviewed/approved? (4) What breaks if specs are ambiguous? (5) How many other teams depend on each service's specs?"

You'll discover that specification strategy is organizational strategy.

### Design Your Specification Strategy

Create a document called "Our Growing Company's Specification Strategy":

**At 5 people**:
- Specs: Feature specs for each developer
- Owner: Product lead
- Review: 30 min spec review before implementation
- Risk if broken: Integration test fails
- Dependencies: Internal only

**At 15 people**:
- Specs: Feature specs + service specs + API specs
- Owners: Product lead + tech lead
- Review: Spec review + architecture review
- Risk if broken: Multiple teams blocked
- Dependencies: 2-3 other teams per service

**At 50 people**:
- Specs: All of above + schema specs + infrastructure specs
- Owners: Spec committee (3-5 senior engineers)
- Review: Formal architectural review process
- Risk if broken: Organization-wide delays
- Dependencies: 5-10 other teams per service

**At 100+ people**:
- Specs: All of above + organizational specs (how teams relate)
- Owners: Chief Architect + spec governance board
- Review: Formal governance with approval authority
- Risk if broken: Cascading failures across organization
- Dependencies: Many teams depend on each spec

---

## Part 5: Identify Critical Specs at Your Scale

Here's an important question: **What specs do you absolutely need at each scale?**

Ask your companion:

> **Prompt**: "If I'm building a company at these scales, what is the *minimum* set of specs I need to avoid chaos? What's the *ideal* set? What specs can I add later?"

Your companion might say:

**Minimum specs (absolute survival)**:
- Feature specifications (what you're building)
- API specifications (how services call each other)
- Acceptance criteria (how you know it works)

**Ideal specs** (at scale):
- All of above, plus:
- Service contracts (reliability, performance)
- Data schemas (how data is structured)
- Deployment specs (how services run)
- Monitoring specs (what to track)

**Nice-to-have specs** (but optional):
- Behavior specs (how services respond to edge cases)
- Performance specs (latency targets)
- Security specs (threat models)
- Compliance specs (regulatory requirements)

---

## Part 6: Create Your Scaling Diagram

Document what you've learned by creating a visual:

**Your diagram should show**:
1. Organization size (1 person → 5 → 15 → 50 → 100+)
2. Number of specs at each scale
3. Complexity of specs
4. Who manages specs
5. How many teams depend on each service

Example structure:

```
Scale 1: Solo (1 person)
  - 1 Feature Spec
  - Owner: You
  - Dependencies: 0
  - Time for spec: 1 hour

Scale 5: Small Team (5 people)
  - Feature Specs (3): for each developer
  - Integration Contracts (2): how features connect
  - Owner: Spec lead
  - Dependencies: Internal (4 other people)
  - Time for specs: 8 hours

Scale 15: Department (15 people)
  - Feature Specs (5)
  - Service Specs (3)
  - API Specs (3)
  - Owner: Tech lead + spec lead
  - Dependencies: 10-12 people from other teams
  - Time for specs: 24 hours (divided among spec team)

Scale 50: Company (50 people)
  - All previous specs, plus:
  - Schema Specs (5)
  - Infrastructure Specs (3)
  - Owners: Spec committee (3 people)
  - Dependencies: 30-40 people from other teams
  - Time for specs: Ongoing (spec team)

Scale 100+: Enterprise (100+ people)
  - All previous specs, plus:
  - Organizational Specs (governance)
  - Monitoring Specs
  - Security Specs
  - Owners: Chief Architect + governance board
  - Dependencies: Everyone depends on core specs
  - Time for specs: Dedicated spec infrastructure team
```

---

## Part 7: Identify Your Organization's Scaling Challenges

Every organization has unique challenges at each scale. What are yours?

Ask your companion:

> **Prompt**: "Here's my specification strategy for growing from 5 to 100+ people. For each scale, what's the biggest risk? Where might specifications become a bottleneck instead of enabling growth? How do I prevent that?"

Think about:
- **Spec review bottleneck**: Spec lead becomes overloaded. Solution: Spec review committee.
- **Spec governance chaos**: No clear approval process. Teams disagree. Solution: Define governance.
- **Spec version hell**: Multiple versions of specs live simultaneously. Solution: Versioning strategy.
- **Spec discovery**: Teams don't know what specs exist. Solution: Specification registry.
- **Spec quality variance**: Some specs are great, some are ambiguous. Solution: Specification standards.

---

## Part 8: Reflection

Answer these questions:

1. **What surprised you** about how specifications scale from small teams to enterprises?

2. **At what scale** does specification-driven development become non-optional? Why?

3. **What's the biggest risk** in your scaling plan? How will you address it?

4. **What new roles emerge** as your organization scales? (Spec lead, spec committee, Chief Architect, etc.)

5. **How does specification culture** change as you scale? At 5 people it's about clarity. At 500 people, it's about organizational alignment and preventing chaos.

---

## Try With AI

Solidify your learning by designing for a specific organizational context.

**Setup**: Use your AI companion to explore realistic scaling scenarios.

**Prompt Set**:

1. **Ground your thinking:**
   > "I work at a company of [X size]. How are specifications used (or not used) in my organization? What specs exist? What specs should exist but don't? Where are the coordination failures?"

2. **Identify your bottleneck:**
   > "As my company scales from [current size] to [future size], what will break in our current way of working? How would clear specifications help? What specs do we need to scale?"

3. **Design for your future:**
   > "Design a specification strategy for my team growing from [X] to [Y] people. What specs do we need at each stage? Who owns them? How do we review them?"

4. **Challenge yourself:**
   > "What kinds of problems can't be solved with specifications alone? Where do we still need synchronous communication, meetings, and direct collaboration? Why?"

**Expected Outcomes**:
- You understand how specifications enable organizational scaling
- You can identify where your organization's coordination failures come from (usually missing or ambiguous specs)
- You can design a specification strategy for your context
- You recognize that specifications are infrastructure—they require investment and governance

**Professional Reality**: Specification culture doesn't happen by accident. It requires leadership commitment, infrastructure investment, and organizational discipline. The companies that scale best (Stripe, Amazon, Google) have intentionally invested in specification practices. You're learning to be one of those engineers.
