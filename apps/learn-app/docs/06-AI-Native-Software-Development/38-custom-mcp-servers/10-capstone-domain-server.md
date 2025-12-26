---
sidebar_position: 10
title: "Capstone: Domain-Specific MCP Server"
description: "Build a production-ready MCP server for your domain using specification-first development. Compose skills from previous lessons into a Digital FTE."
keywords: ["MCP server", "spec-driven development", "capstone project", "domain-specific", "Digital FTE", "production deployment", "Layer 4"]
chapter: 38
lesson: 10
duration_minutes: 120

# HIDDEN SKILLS METADATA
skills:
  - name: "Specification-Driven MCP Server Design"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can write a complete, executable MCP server specification that drives AI implementation without additional guidance"

  - name: "Composing Multiple MCP Primitives"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can orchestrate tools, resources, and prompts into a coherent server architecture that solves a domain problem"

  - name: "Digital FTE Architecture Design"
    proficiency_level: "C1"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can design MCP servers that could function as autonomous Digital FTEs with clear monetization pathways"

  - name: "MCP Server Validation & Testing Strategy"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Quality Assurance"
    measurable_at_this_level: "Student can design acceptance tests that validate spec ↔ implementation alignment"

learning_objectives:
  - objective: "Write a comprehensive MCP server specification that captures intent, primitives, constraints, and success criteria"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student creates specification that could be handed to another developer (or AI) for implementation without clarification"

  - objective: "Design domain-specific tools, resources, and prompts that work together to solve a real-world problem"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Student articulates how each tool/resource/prompt contributes to the overall capability"

  - objective: "Apply specification-first methodology to guide AI implementation without iterative back-and-forth"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student's AI implementation requires minimal revision after initial prompt; spec was sufficient"

  - objective: "Validate that implemented server meets specification intent and could be packaged as a Digital FTE product"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Student articulates acceptance tests and can identify implementation gaps against spec"

cognitive_load:
  new_concepts: 4
  assessment: "4 concepts (spec-driven orchestration, composite architecture, validation strategy, Digital FTE design) at B2 tier balances integration with manageable load"

differentiation:
  extension_for_advanced: "Design a two-version architecture (MVP + enterprise); build a secondary resource type; implement auth flows for the server itself; design monetization model"
  remedial_for_struggling: "Start with simpler domain (fewer tools); focus on 2-3 core tools + 1 resource; use provided specification template without modification"
---

# Capstone: Domain-Specific MCP Server

You've built all the individual pieces of an MCP server. You understand tools, resources, prompts, authentication, testing, and deployment. Now comes the synthesis: building a complete, production-ready MCP server for a real-world domain.

This is a **Layer 4: Spec-Driven Integration** capstone. This means:

1. **You write the specification first** — capturing intent, constraints, and success criteria
2. **AI implements from your specification** — handling tactical details while you maintain strategic control
3. **You validate alignment** — confirming the implementation matches your original vision
4. **You package as a Digital FTE** — preparing a product that could generate recurring revenue

The outcome of this capstone is not just a working server. It's a **sellable Digital FTE component** — a piece of intelligence that encodes domain expertise and could be licensed, subscribed to, or deployed for a customer.

## Why Specification-First Matters for Capstones

In previous lessons, you built small tools or individual features. When scope is limited, vague specifications sometimes work. But when building an integrated system with multiple tools, resources, and prompts all working together, vagueness becomes expensive.

Consider two approaches:

**Approach 1: Vague Description**
> "Build an MCP server for project management. It should have tools to create and update tasks, show the status, and help with planning."

If you hand this to AI, you get back something, but does it match your vision?
- What data model do tasks use? (fields, required vs optional)
- What counts as "status"? (numeric codes? enum? string?)
- How does the planning tool work? (generates plans? analyzes existing tasks?)
- What resource format best serves agents? (JSON? hierarchical? queryable?)

Expect 3-5 iteration cycles to align the implementation with your intent.

**Approach 2: Clear Specification**
> [Your detailed specification covering intent, primitives, constraints, success criteria]

Now AI understands exactly what you need. The implementation comes back aligned with your vision the first time. One iteration for refinement, not five for clarification.

**The productivity equation from Chapter 1 holds here:**
```
Clear Specification + AI Execution = 1 iteration
Vague Idea + AI Execution = 5+ iterations
```

This capstone teaches you to write specifications that make Approach 2 possible.

## Choosing Your Domain

You have five options. Choose **ONE** that matches your interests or your future Digital FTE vision:

### Option 1: Project Management Server
**Domain Problem**: Teams need to organize work, track status, and generate insights

**Primitives**:
- **Tools**: `create_task`, `update_task_status`, `get_project_summary`, `suggest_milestones`
- **Resources**: `projects/{project_id}` (hierarchical task list), `team_capacity` (resource allocation)
- **Prompts**: "Project Analysis" (interpret trends), "Risk Identification" (flag overdue tasks)

**Success Criteria**:
- [ ] Create tasks with title, description, assigned owner, due date
- [ ] Update status (backlog → in_progress → review → done)
- [ ] Generate project summaries showing progress, blockers, capacity
- [ ] Suggest realistic milestones based on team velocity

---

### Option 2: Data Analysis Server
**Domain Problem**: Analysts need to query databases, generate reports, and create visualizations

**Primitives**:
- **Tools**: `run_sql_query`, `generate_report`, `detect_anomalies`, `export_results`
- **Resources**: `schema` (database structure), `queries/{query_id}` (saved queries), `results/{result_id}` (cached results)
- **Prompts**: "SQL Generation" (translate intent to queries), "Analysis" (interpret results)

**Success Criteria**:
- [ ] Execute parameterized SQL with input validation
- [ ] Generate formatted reports (CSV, JSON, markdown)
- [ ] Detect outliers/anomalies in datasets
- [ ] Cache results for reuse; include execution metadata

---

### Option 3: Customer CRM Server
**Domain Problem**: Sales teams need to track interactions, manage pipelines, and generate outreach

**Primitives**:
- **Tools**: `log_interaction`, `update_contact`, `get_pipeline_stage`, `generate_outreach`
- **Resources**: `contacts/{contact_id}`, `pipelines/{pipeline_name}`, `interaction_history`
- **Prompts**: "Relationship Analysis" (customer sentiment), "Outreach Strategy" (personalized messaging)

**Success Criteria**:
- [ ] Log calls, emails, meetings with context and sentiment
- [ ] Update contact info (company, role, needs)
- [ ] Visualize pipeline stages and conversion rates
- [ ] Generate personalized outreach with historical context

---

### Option 4: Code Quality Server
**Domain Problem**: Development teams need code review, style enforcement, and improvement suggestions

**Primitives**:
- **Tools**: `analyze_code`, `check_style_compliance`, `suggest_improvements`, `generate_test_cases`
- **Resources**: `style_guide` (project standards), `architecture` (pattern library), `review_history`
- **Prompts**: "Code Review" (identify issues), "Refactoring Suggestions" (improvement strategies)

**Success Criteria**:
- [ ] Analyze code for bugs, security issues, performance antipatterns
- [ ] Validate against style guide (naming, formatting, patterns)
- [ ] Suggest concrete improvements with examples
- [ ] Generate test cases for risky functions

---

### Option 5: Email Campaign Server
**Domain Problem**: Marketing teams need to design campaigns, track performance, and optimize messaging

**Primitives**:
- **Tools**: `schedule_campaign`, `track_opens`, `optimize_subject_line`, `segment_audience`
- **Resources**: `templates/{template_id}`, `campaign_metrics`, `audience_segments`
- **Prompts**: "Campaign Strategy" (targeting advice), "Copy Optimization" (message improvement)

**Success Criteria**:
- [ ] Schedule campaigns with audience, template, timing
- [ ] Track opens, clicks, conversions; calculate metrics
- [ ] Analyze A/B test results; recommend subject line changes
- [ ] Segment audiences by behavior, engagement, demographics

---

## Your Specification Template

Use this template to structure your thinking. Be as specific as possible. Vague specifications produce misaligned implementations.

```markdown
# MCP Server Specification: [Your Domain]

## 1. Intent (Why Does This Server Exist?)

Write 2-3 sentences explaining:
- What domain expertise does this server encode?
- What agents can do that they couldn't before?
- What business value does this create?

**Example**:
"This server encodes expert project management knowledge. It helps Claude Code plan complex projects by breaking them into phases, assigning work, and tracking progress automatically. Teams using this server can delegate planning and status tracking to AI, freeing human project managers for exception handling."

## 2. Primitives (What Can This Server Do?)

### Tools
List each tool with:
- **Name**: Simple, verb-noun format (e.g., `create_task`)
- **Intent**: What does this tool accomplish?
- **Input parameters**: Name, type, required/optional, constraints
- **Output format**: Structure of returned data
- **Success criterion**: How do we know it worked?

**Example**:
```
Tool: create_task
Intent: Create a new task in the project
Inputs:
  - project_id (string, required): ID of target project
  - title (string, required): Task title (1-100 chars)
  - description (string, optional): Detailed description
  - assignee (string, optional): User ID of responsible person
  - due_date (ISO 8601 date, optional): Task deadline
Outputs:
  - task_id (string): Unique task identifier
  - created_at (timestamp): When task was created
  - status (string): Initial status ("backlog")
Success Criterion: Task appears in project and can be queried immediately
```

### Resources
List each resource with:
- **URI scheme**: How is this resource identified? (e.g., `projects/{project_id}`)
- **Content type**: JSON? Text? Binary?
- **Querying**: Can it be filtered? Paginated?
- **Refresh rate**: How fresh is the data? (real-time, cached, hourly)

**Example**:
```
Resource: projects/{project_id}
URI: projects/{project_id}
Content: JSON
Format: {
  "id": string,
  "name": string,
  "owner": string,
  "tasks": [
    {
      "id": string,
      "title": string,
      "status": enum,
      "assignee": string,
      "due_date": ISO 8601 date
    }
  ],
  "metadata": {
    "total_tasks": number,
    "completed": number,
    "overdue": number
  }
}
Refresh: Real-time (queries current state)
```

### Prompts
List each prompt template with:
- **Name**: What is this prompt for?
- **Intent**: What expertise does it encode?
- **Inputs**: What data does it accept as context?
- **Outputs**: What kind of guidance does it provide?

**Example**:
```
Prompt: Project Analysis
Intent: Analyze project health and identify risks
Inputs: Project summary (name, tasks, status, dates, capacity)
Outputs: Risk assessment (3-5 risks ranked by severity) with mitigation suggestions
```

## 3. Constraints (What Are the Limits?)

### Functional Constraints
- Authentication: What APIs require keys? How are credentials secured?
- Performance: Response time targets? Max query sizes?
- Reliability: Retry logic? Timeout behavior?

### Non-Functional Constraints
- Scale: Concurrent connections? Data limits?
- Security: What data is sensitive? What validation is required?
- Compliance: Any regulatory requirements? (GDPR, PCI, HIPAA)

**Example**:
```
Functional:
  - API calls timeout at 30 seconds; retry up to 2 times with exponential backoff
  - Task titles limited to 100 characters; descriptions to 5000 characters
  - Project queries return paginated results (100 per page)

Non-Functional:
  - Support 10 concurrent API connections
  - Encrypt API keys in environment variables
  - Log all mutations (create/update/delete) for audit trail
```

## 4. Success Criteria (How Do We Validate?)

For each tool/resource/prompt, write a testable criterion:

- [ ] Tool criterion 1
- [ ] Tool criterion 2
- [ ] Resource criterion 1
- [ ] Prompt criterion 1

**Example**:
```
- [ ] create_task tool returns valid task_id immediately; task appears in project resource
- [ ] update_task_status tool changes status and updates resource; timestamp advances
- [ ] Project resource returns all tasks in correct hierarchy; no tasks are missing
- [ ] Project Analysis prompt identifies at least 1 real risk from provided data
```

## 5. Non-Goals (What Are We NOT Building?)

Explicitly state what's out of scope:

- Not building authentication system (assume agents provide API key)
- Not storing file attachments (only metadata)
- Not generating visualizations (text-based output only)
- Not supporting real-time collaboration (eventual consistency OK)

---

**Your turn**: Write your specification using this template. Be specific. Include examples. If you're uncertain about something, that's a signal that your specification isn't clear enough yet.

---

## Layer 4 Pattern: Spec → Prompt → Code → Validate

Before you move to "Try With AI," understand the full pattern you'll execute:

### Phase 1: Specification (You Complete)

You write the spec using the template above. This is the strategic phase. You define what the server should do and why.

### Phase 2: Prompt (You Write)

You craft a prompt to AI that references your specification. The prompt should say: "Here's my specification. Implement an MCP server that satisfies these criteria."

### Phase 3: Code (AI Generates)

AI generates `server.py` and test files based on your specification. AI doesn't guess. It follows the spec.

### Phase 4: Validate (You Assess)

You review the generated code:
- Does it implement all tools/resources/prompts from the spec?
- Do the tool signatures match your specification?
- Are success criteria testable with the generated code?

If implementation drifts from spec, you ask AI to fix specific deviations.

## Common Specification Mistakes to Avoid

### Mistake 1: Vague Tool Parameters
**Wrong**: "Tool: `analyze_code`. Input: 'code to analyze'"
**Right**: "Tool: `analyze_code`. Input: code_snippet (string, max 10,000 chars, required)"

### Mistake 2: Unclear Resource Structure
**Wrong**: "Resource: project status (shows status)"
**Right**: "Resource: `projects/{id}` returns JSON with fields: {name, tasks: [{id, status, assignee}], metadata: {total, completed}}"

### Mistake 3: Missing Constraints
**Wrong**: "Tool returns results"
**Right**: "Tool returns paginated results (max 100 per request); includes execution metadata (query time, rows affected)"

### Mistake 4: Ambiguous Success Criteria
**Wrong**: "Tool works correctly"
**Right**: "Tool creates task immediately; task appears in project resource within 100ms; assignee can view task"

### Mistake 5: Unstated Assumptions
**Wrong**: Assuming AI knows you want error handling for invalid project IDs
**Right**: "Tool validates project_id exists; returns error code 404 with message if not found"

## What Happens Next

In the "Try With AI" section, you will:

1. **Write your specification** — Choose a domain, complete the template
2. **Prompt AI to implement** — Reference your spec; ask for working server code
3. **Validate implementation** — Check if code matches spec; identify gaps
4. **Iterate on spec** — If implementation drifts, refine the spec and re-prompt

The goal is to reach **specification clarity** where AI's first implementation attempt requires minimal revision.

---

## Try With AI: Building Your Domain-Specific MCP Server

This capstone follows the specification-first pattern: You direct strategy, AI handles tactics.

### Part 1: Choose Domain & Write Specification

**Your task:**
1. Choose one of the five domains above (or use your own domain)
2. Complete the specification template for your chosen domain
   - Be specific about tool inputs/outputs
   - Include examples in your spec
   - Write concrete success criteria
3. Write your completed spec in a document (or in your prompt to AI)

**What you're learning**: Writing specifications that drive AI implementation is the primary skill of the agentic era. The clarity you achieve now determines how well AI executes later.

---

### Part 2: Generate Your MCP Server Implementation

**Your request to AI:**

```
You are an MCP server architect. Here is my specification for a domain-specific MCP server:

[PASTE YOUR COMPLETE SPECIFICATION HERE]

Generate a production-ready MCP server that:

1. Implements all tools specified above
   - Use FastMCP (@mcp.tool decorator)
   - Include proper type hints (they generate JSON schemas)
   - Add comprehensive docstrings

2. Implements all resources specified above
   - Use @mcp.resource for URI-based resources
   - Return data in the format specified

3. Implements all prompts specified above
   - Use @mcp.prompt to register prompt templates
   - Include the specification's intended parameters

4. Satisfies all constraints
   - Add input validation for tool parameters
   - Include error handling with appropriate messages
   - Add timeout/retry logic if specified

5. Includes test coverage
   - Unit tests for each tool
   - Integration tests validating spec ↔ implementation alignment
   - At least one test per success criterion

6. Follows the structure from Lesson 1 (Chapter 38)
   - Use FastMCP framework
   - Use uv for dependency management
   - Include proper pyproject.toml
   - Include type hints throughout

Output:
- server.py (complete implementation)
- test_server.py (comprehensive tests)
- pyproject.toml (with proper dependencies and entry point)
- README.md (including installation and usage)

Start with the server.py file.
```

**Copy the generated server code** into a local file. Do not modify yet.

**What you're learning**: This prompt demonstrates how precise specifications enable AI execution. Your specification drove the implementation. If the implementation matches your intent, your specification was clear.

---

### Part 3: Validate Spec ↔ Implementation Alignment

Review the generated code with this checklist:

**Tools Validation**:
- [ ] All tools from spec are implemented with @mcp.tool
- [ ] Tool names match specification exactly
- [ ] Input parameters match spec (types, required/optional, constraints)
- [ ] Output structure matches specification
- [ ] Error handling addresses spec constraints

**Resources Validation**:
- [ ] All resources from spec are implemented with @mcp.resource
- [ ] URI schemes match specification
- [ ] JSON structure matches specification format
- [ ] Data types match spec (strings, numbers, enums)

**Prompts Validation**:
- [ ] All prompts from spec are implemented
- [ ] Prompt parameters match specification
- [ ] Guidance aligns with intended expertise

**Tests Validation**:
- [ ] Tests exist for each tool (at least basic invocation)
- [ ] Tests verify outputs match spec format
- [ ] Tests check success criteria
- [ ] Test names are clear about what they validate

**Configuration**:
- [ ] pyproject.toml includes all dependencies
- [ ] Entry point is configured if needed
- [ ] README includes setup and usage examples

**Assessment Questions** (Answer these in your own words):

1. "Does the generated server implement my specification completely, or are there gaps?"
   - Name specific gaps if any exist

2. "If I gave this server to another developer, would they understand what it does?"
   - Would the code, docstrings, and tests make the domain clear?

3. "Could this server be packaged and deployed as a Digital FTE product?"
   - What's missing for production use? (metrics, logging, rate limiting?)

4. "What would I change in my specification if I could rewrite it?"
   - Did you discover missing details during implementation review?

---

### Part 4: Refine & Iterate (If Needed)

If the implementation drifts from your spec:

**Option A: Clarify Your Specification**
If you discover your spec was ambiguous:
> "I notice the tool returned [unexpected format]. Looking back at my specification, it should [clarification]. Can you update the tool implementation to match this clarification?"

This teaches you that vague specs cause rework.

**Option B: Fix the Implementation**
If the code has bugs but your spec was clear:
> "The `get_summary` tool is returning incomplete task lists. The specification requires all tasks to be included. Can you fix the implementation?"

This teaches you that even clear specs need validation.

**Option C: Recognize Spec Completeness**
If implementation matches specification exactly:
> "The implementation matches my specification perfectly. No changes needed."

This teaches you what specification clarity enables.

#### Digital FTE Packaging Validation

As part of Part 4, consider what makes your server a sellable Digital FTE:

- [ ] **Domain expertise encoded**: The server solves a specific domain problem, not generic automation
- [ ] **Clear value proposition**: A customer would pay for access to this capability
- [ ] **Reliable execution**: Tests pass; error handling is robust
- [ ] **Production-ready packaging**: Can be installed with `pip install`; runs with `my-server` command
- [ ] **Documented usage**: README explains what the server does and how agents use it
- [ ] **Extensibility**: New customers could extend it or customize for their variant of the domain

---

#### Final Reflection: The Spec-First Transformation

This capstone marks a fundamental shift in how you develop:

**Before (Vibe Coding):**
- Start with vague ideas → Code something → Debug → Iterate → Hope alignment happens

**After (Spec-Driven Development):**
- Write precise specification → Direct AI with clarity → AI executes correctly → Minimal iteration

The productivity difference is dramatic. The investment in specification upfront saves 3-5 iterations later.

**Assessment Questions** (Reflect on your capstone):

1. **Specification Clarity**: Was your specification clear enough that AI's implementation required minimal revision?
2. **Alignment**: Did the generated server match your original vision, or did you discover missing requirements during validation?
3. **Digital FTE Readiness**: Does your completed server meet the packaging checklist above? What's missing for production deployment?
4. **Process Improvement**: Where in previous projects would specification-first development have saved time? What prevented you from writing specs?

**Key Insight**: Every hour spent on specification clarity saves three hours of iteration and debugging. This becomes the foundation of building AI products at scale.

You've completed this capstone when all four assessment questions have concrete answers demonstrating specification clarity and implementation alignment.
