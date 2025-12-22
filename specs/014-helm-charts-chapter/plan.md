# Implementation Plan: Chapter 51 — Helm Charts for AI Services

**Branch**: `014-helm-charts-chapter` | **Date**: 2025-12-23 | **Spec**: `/specs/014-helm-charts-chapter/spec.md`

---

## Summary

Chapter 51 transforms students from Helm users into Helm chart architects who can design, template, test, and distribute production-grade charts for AI services. Building on Chapter 50's 45-minute introduction (Lesson 20), this chapter comprises 11 lessons across 4 pedagogical layers that systematically build mastery from advanced Go templating through AI-assisted development to specification-driven capstone projects.

The chapter progresses through Foundation (templating vocabulary) → Application (real-world patterns) → Distribution (enterprise scaling) → Synthesis (AI collaboration and capstone). All exercises execute on Minikube without cloud dependencies. The capstone produces a deployable AI agent chart with PostgreSQL/Redis dependencies, hooks, tests, and multi-environment configuration.

**Target Quality Tier**: Market-defining (comprehensive research, superior to official Helm docs)
**Pedagogical Innovation**: Layer 2 Helm charts via AI collaboration with invisible Three Roles framework

---

## Chapter Architecture

### Pedagogical Phases & Lesson Distribution

```
FOUNDATION PHASE (Lessons 1-3): Building Templating Vocabulary
├── Lesson 1: Advanced Go Templating (Layer 1 — Manual)
├── Lesson 2: Named Templates & Helpers (Layer 1 — Manual)
└── Lesson 3: Values Deep Dive (Layer 1 — Manual)
    Outcome: Students can create sophisticated, maintainable templates

APPLICATION PHASE (Lessons 4-6): Real-World Patterns
├── Lesson 4: Chart Dependencies (Layer 1 — Manual)
├── Lesson 5: Helm Hooks & Lifecycle Management (Layer 1 — Manual)
└── Lesson 6: Testing Your Charts (Layer 1 — Manual)
    Outcome: Students can architect multi-component deployments with CI/CD integration

DISTRIBUTION PHASE (Lessons 7-8): Enterprise Scaling
├── Lesson 7: OCI Registries & Distribution (Layer 1 — Manual)
└── Lesson 8: Library Charts & Standardization (Layer 1 — Manual)
    Outcome: Students can share charts across teams with organizational patterns

SYNTHESIS PHASE (Lessons 9-11): Intelligence & Mastery
├── Lesson 9: AI-Assisted Chart Development (Layer 2 — Collaboration)
├── Lesson 10: Capstone - Production AI Agent Chart (Layer 4 — Spec-Driven)
└── Lesson 11: Building a Helm Chart Skill (Layer 3 — Intelligence)
    Outcome: Students orchestrate complete AI deployment strategies with reusable intelligence
```

**Lesson Count Justification**: 11 lessons driven by concept density + proficiency tier + layer requirements:
- **Concept Density**: 20+ distinct concepts (Go templates, dependency management, hooks, testing, OCI, library charts, AI collaboration, skill design)
- **Proficiency Tier**: B1-B2 intermediate = 7-10 concepts per lesson sustainable
- **Layer Requirements**: Layers 1→2→3→4 progression requires 8 foundation lessons + 1 AI collaboration + 1 capstone + 1 intelligence design
- **Research depth**: Market-defining quality justifies 11 lessons (vs 8 minimum)

---

## Detailed Lesson Architecture

### FOUNDATION PHASE

---

#### **Lesson 1: Advanced Go Templating**

**Stage**: Layer 1 (Manual Foundation)
**Duration**: 45-60 minutes
**Proficiency**: B1 (building on Chapter 50 basic Helm)

**Learning Objectives**:
1. Apply Go template variables and pipeline syntax to create dynamic manifest values
2. Distinguish between template action and data context using `with` and `range` constructs
3. Analyze conditional template rendering with `if/else` logic for multi-environment support
4. Evaluate template output using `helm template --debug` to debug rendering errors
5. Create helper templates using variable assignments for code reuse

**Key Concepts** (count: 9, within B1 limit of 7-10):
1. Template variables (`{{.Values.image.tag}}`)
2. Pipelines and filters (`| upper | quote`)
3. Variable assignment (`{{$var := .Values.name}}`)
4. `with` block for context switching
5. `range` iteration over lists and maps
6. `if/else` conditional rendering
7. Comparison operators in conditionals (`eq`, `ne`, `gt`, `lt`)
8. Template functions (`quote`, `upper`, `lower`, `default`, `printf`)
9. Helm's built-in variables (`.Chart`, `.Release`, `.Namespace`)

**Cognitive Load Validation**: 9 concepts ≤ 10 limit → ✅ WITHIN B1 LIMIT

**Content Structure**:

1. **Manual Walkthrough** (15 min)
   - Explain Go templating vs static manifests
   - Show Helm template syntax with analogies (JavaScript template literals)
   - Demo: Simple variable substitution
   - Demo: Pipeline syntax with filters
   - Practice: Students manually edit templates, validate with `helm template`

2. **Hands-On Exercises** (25 min)
   - Exercise 1.1: Create Deployment with dynamic replica count (using `{{.Values.replicaCount}}`)
   - Exercise 1.2: Use pipelines to auto-case environment values (`{{ .Values.env | upper }}`)
   - Exercise 1.3: Implement conditional Ingress (include only if `ingress.enabled: true`)
   - Exercise 1.4: Use `range` to iterate over environment variables from values
   - Exercise 1.5: Debug template error with `helm template --debug` (intentional syntax error provided)

3. **Validation Checkpoints** (5 min)
   - Students verify templates render correctly without Helm errors
   - Compare rendered YAML to expected output structure

4. **Maps to Success Evals**:
   - SC-001: Students create sophisticated templates in <30 min (measured via capstone)
   - SC-004: Conceptual understanding validated through exercises

5. **Try With AI** (5 min)
   - Initial Request: "Ask AI to explain Go template syntax rules"
   - Critical Evaluation: "What assumptions did AI make about your project?"
   - Constraint Teaching: "I need templates that work with both dev (1 replica) and prod (5 replicas). How would you adjust?"
   - Refinement: "Show me a template that handles both"
   - Final Check: "What improved from the initial explanation to the final version?"

**Maps to Evals**: SC-001, SC-004 (Template sophistication, understanding)

---

#### **Lesson 2: Named Templates and Helpers**

**Stage**: Layer 1 (Manual Foundation)
**Duration**: 45 minutes
**Proficiency**: B1

**Learning Objectives**:
1. Design reusable template partials using `define` and `include` to reduce duplication
2. Compare deprecated `template` function vs modern `include` function for manifest consistency
3. Explain the difference between named templates (templates/\_helpers.tpl) and main templates
4. Apply helper templates for consistent labeling across all Kubernetes resources
5. Troubleshoot naming conflicts when multiple charts define identical template names

**Key Concepts** (count: 7, within B1 limit):
1. Named templates with `define` syntax
2. Helper template files (`_helpers.tpl`)
3. `include` function (modern approach)
4. `template` function (deprecated, why it's problematic)
5. Template scope and variable passing to helpers
6. Common patterns: labels, selectors, image pull secrets
7. Naming conventions to prevent conflicts

**Cognitive Load Validation**: 7 concepts = 7 limit → ✅ AT B1 BOUNDARY

**Content Structure**:

1. **Manual Walkthrough** (15 min)
   - Explain DRY principle for templates
   - Show `_helpers.tpl` file structure
   - Demonstrate `define`/`include` syntax
   - Compare to deprecated `template` function and why it breaks composition
   - Explain scope rules and variable passing

2. **Hands-On Exercises** (25 min)
   - Exercise 2.1: Create `_helpers.tpl` with common labels template
   - Exercise 2.2: Include labels helper in Deployment, Service, and ConfigMap
   - Exercise 2.3: Create image pull secrets helper (if registry requires auth)
   - Exercise 2.4: Verify `include` works vs show why `template` would fail
   - Exercise 2.5: Refactor 3 duplicate template sections into single include

3. **Validation Checkpoints** (5 min)
   - All resources have consistent labels via helpers
   - No template rendering errors
   - `helm template` output shows identical labels across all resources

4. **Maps to Success Evals**:
   - SC-004: Understand include vs template (conceptual test)
   - Foundation for SC-002 (multi-environment consistency)

5. **Try With AI**:
   - Initial Request: "Ask AI: What's the difference between include and template functions in Helm?"
   - Critical Evaluation: "Did AI explain WHY one is better? What was vague?"
   - Constraint Teaching: "I have 10 services needing identical labels. How would you structure helpers to DRY this up?"
   - Refinement: "Show me the exact helper structure you'd use"
   - Final Check: "What pattern emerged that you didn't anticipate?"

**Maps to Evals**: SC-004 (Template function understanding), Foundation for SC-005

---

#### **Lesson 3: Values Deep Dive**

**Stage**: Layer 1 (Manual Foundation)
**Duration**: 50 minutes
**Proficiency**: B1-B2

**Learning Objectives**:
1. Explain the Helm values hierarchy and override precedence (defaults < values.yaml < -f file < --set)
2. Design schema validation for values.yaml to catch configuration errors early
3. Apply global values to share configuration across parent and subchart namespaces
4. Evaluate environment-specific values files (dev/staging/prod) with schema validation
5. Troubleshoot values precedence issues when multiple override sources conflict

**Key Concepts** (count: 8, within B1 limit):
1. Values hierarchy and precedence rules
2. values.yaml structure and defaults
3. Environment-specific values files (values-dev.yaml, etc.)
4. Command-line overrides (`--set`, `-f`)
5. Global values (`.global` namespace)
6. Schema validation (JSONSchema for values)
7. Nested values and dotted path access
8. values.schema.json for IDE autocomplete and validation

**Cognitive Load Validation**: 8 concepts ≤ 10 limit → ✅ WITHIN B1 LIMIT

**Content Structure**:

1. **Manual Walkthrough** (20 min)
   - Explain values as "configuration parameters that customize chart behavior"
   - Show precedence hierarchy (defaults overridden by env-specific overrides)
   - Demonstrate command-line override precedence
   - Explain global values for subchart configuration
   - Introduce schema validation as "safety net for configuration errors"

2. **Hands-On Exercises** (25 min)
   - Exercise 3.1: Create values.yaml with realistic defaults (replicas, resources, image tags)
   - Exercise 3.2: Create values-dev.yaml, values-staging.yaml, values-prod.yaml with environment-specific overrides
   - Exercise 3.3: Verify override precedence: `helm template -f values-prod.yaml --set image.tag=v2.0`
   - Exercise 3.4: Create values.schema.json with validation rules
   - Exercise 3.5: Deliberately pass invalid value, verify schema catches error

3. **Validation Checkpoints** (5 min)
   - Each environment's values file renders correct manifest
   - Override precedence respected (CLI > file > defaults)
   - Schema validation catches invalid configurations

4. **Maps to Success Evals**:
   - SC-002: Multi-environment deployment (dev/staging/prod values files)
   - SC-004: Understanding values hierarchy

5. **Try With AI**:
   - Initial Request: "Ask AI: Design a values.schema.json for validating a chart's configuration"
   - Critical Evaluation: "Did AI include all required fields? What edge cases are missing?"
   - Constraint Teaching: "In production, we need 3-10 replicas max, 4Gi memory max, specific log levels. Add these constraints."
   - Refinement: "Show me a schema that enforces these production constraints"
   - Final Check: "What validation rules emerged that you wouldn't have thought of?"

**Maps to Evals**: SC-002, SC-004

---

### APPLICATION PHASE

---

#### **Lesson 4: Chart Dependencies**

**Stage**: Layer 1 (Manual Foundation)
**Duration**: 50 minutes
**Proficiency**: B1-B2

**Learning Objectives**:
1. Design Chart.yaml dependencies to compose parent and subchart relationships
2. Apply dependency conditions and tags to enable/disable subcharts per environment
3. Explain import-values to share configuration between parent and subchart namespaces
4. Execute `helm dependency update` to fetch and manage chart versions
5. Troubleshoot circular dependencies and version conflicts between subcharts

**Key Concepts** (count: 9, within B1 limit):
1. Chart.yaml dependency structure
2. Subchart resolution and versions
3. Dependency conditions (`condition: postgresql.enabled`)
4. Dependency tags (grouping related subcharts)
5. import-values for parent→subchart config passing
6. `helm dependency update` command
7. charts/ directory structure
8. Version constraints (ranges, exact, pre-releases)
9. Alias for multiple instances of same chart

**Cognitive Load Validation**: 9 concepts ≤ 10 limit → ✅ WITHIN B1 LIMIT

**Content Structure**:

1. **Manual Walkthrough** (15 min)
   - Explain subchart as "reusable application package" (PostgreSQL, Redis, messaging)
   - Show Chart.yaml dependency syntax
   - Demonstrate dependency conditions (enable/disable based on values)
   - Explain import-values for configuration sharing
   - Walk through `helm dependency update` workflow

2. **Hands-On Exercises** (30 min)
   - Exercise 4.1: Add Bitnami PostgreSQL as dependency with version constraint
   - Exercise 4.2: Add Redis dependency with condition (redis.enabled)
   - Exercise 4.3: Configure PostgreSQL via parent values: `postgresql.auth.database: mydb`
   - Exercise 4.4: Use import-values to expose PostgreSQL host to main chart
   - Exercise 4.5: Run `helm dependency update`, verify charts/ directory populated
   - Exercise 4.6: Create values-nodedb.yaml that disables Redis but enables PostgreSQL

3. **Validation Checkpoints** (5 min)
   - `helm dependency update` succeeds without version conflicts
   - Parent values correctly configure subchart behavior
   - Conditional dependencies enabled/disabled as expected

4. **Maps to Success Evals**:
   - SC-005: Add community chart as dependency and configure via parent values
   - Foundation for SC-002 (complete stack deployments)

5. **Try With AI**:
   - Initial Request: "Ask AI: Design Chart.yaml dependencies for an AI agent that needs PostgreSQL and Redis"
   - Critical Evaluation: "Did AI include version constraints? What happens if versions aren't specified?"
   - Constraint Teaching: "We want to disable Redis in dev (cost savings) but require it in prod. How do we structure this?"
   - Refinement: "Show me conditions and values structure that implements this"
   - Final Check: "What dependency patterns emerged that you'd reuse in future charts?"

**Maps to Evals**: SC-005, Foundation for SC-002

---

#### **Lesson 5: Helm Hooks and Lifecycle Management**

**Stage**: Layer 1 (Manual Foundation)
**Duration**: 60 minutes
**Proficiency**: B1-B2

**Learning Objectives**:
1. Analyze all 9 Helm hook types and their execution sequence in release lifecycle
2. Design pre-upgrade/post-upgrade hooks to implement zero-downtime database migrations
3. Apply hook weights and delete policies to control hook execution order and cleanup
4. Explain failure handling: what happens when hook job fails (deployment blocked)
5. Troubleshoot hook execution issues using pod logs and hook status

**Key Concepts** (count: 10, exceeds B1 limit — justification follows):
1. Hook types: pre-install, post-install, pre-upgrade, post-upgrade, pre-delete, post-delete, pre-rollback, post-rollback, test
2. Hook resource (Jobs, Pods, Services)
3. Hook weight for ordering multiple hooks
4. Hook delete policies (hook-succeeded, hook-failed, before-hook-creation)
5. Hook success/failure semantics (failure blocks deployment)
6. Annotation syntax (`helm.sh/hook:`, `helm.sh/hook-weight:`)
7. Hook execution phases relative to main deployment
8. Testing hooks (test hook differs from test command)
9. Rollback hooks for undo operations
10. Pod termination and cleanup after hook completion

**Cognitive Load Justification** (10 concepts): Hooks have intricate execution semantics (9 types × ordering × delete policies = high complexity). B1-B2 audience needs comprehensive coverage because hook errors cause production outages. Elevated limit justified.

**Content Structure**:

1. **Manual Walkthrough** (20 min)
   - Explain hooks as "lifecycle-aware Jobs that run at specific deployment phases"
   - Show execution sequence diagram (pre-install → install → post-install, etc.)
   - Demonstrate pre-upgrade hook for database migrations (critical pattern)
   - Explain hook weights for controlling order (lower weight = earlier execution)
   - Explain delete policies: when hooks auto-clean vs require manual cleanup

2. **Hands-On Exercises** (35 min)
   - Exercise 5.1: Create pre-install hook Job (DB schema initialization)
   - Exercise 5.2: Create pre-upgrade hook for migrations with weight ordering
   - Exercise 5.3: Create post-upgrade hook for cache invalidation
   - Exercise 5.4: Deliberately make hook fail, verify deployment blocks
   - Exercise 5.5: Test `helm.sh/hook-delete-policy: hook-succeeded` cleanup
   - Exercise 5.6: Use weights to ensure migration runs before rollback
   - Exercise 5.7: Check hook execution with `kubectl get job -l app.kubernetes.io/managed-by=Helm`

3. **Validation Checkpoints** (5 min)
   - Hooks execute in expected order (verified via pod logs)
   - Failed hook blocks deployment (safety check)
   - Delete policies cleanup hooks automatically
   - Deployment waits for hook completion

4. **Maps to Success Evals**:
   - SC-006: Write pre-upgrade hook that runs before deployment updates
   - Foundation for SC-001 (production-ready charts)

5. **Try With AI**:
   - Initial Request: "Ask AI: Design a pre-upgrade hook for database migrations that must complete before the app updates"
   - Critical Evaluation: "Did AI handle failure scenarios? What if the migration fails?"
   - Constraint Teaching: "We have 2 migration hooks: one for schema (must run first), one for data seeding (must run second). How do we enforce order?"
   - Refinement: "Show me weights and hook structure for this sequence"
   - Final Check: "What edge cases emerged (rollback handling, hook failures) that you'd test in prod?"

**Maps to Evals**: SC-006, Foundation for SC-001

---

#### **Lesson 6: Testing Your Charts**

**Stage**: Layer 1 (Manual Foundation)
**Duration**: 45 minutes
**Proficiency**: B1

**Learning Objectives**:
1. Apply `helm lint` to validate chart syntax and catch structural errors early
2. Design `helm template --debug` workflows for rendering verification without deploying
3. Create chart test pods (helm test) to verify post-deployment connectivity and health
4. Explain differences between chart tests (post-deployment validation) and unit tests (template syntax)
5. Troubleshoot chart issues before production deployment using template debugging

**Key Concepts** (count: 8, within B1 limit):
1. `helm lint` command and validation rules
2. Chart structure validation (Chart.yaml, values.yaml, templates/)
3. `helm template` for static rendering (no cluster required)
4. `helm template --debug` for detailed template variable inspection
5. Test pods with `helm.sh/hook: test` annotation
6. Test success/failure semantics (test pod exit code determines pass/fail)
7. `helm test` command execution (post-install verification)
8. Unit testing frameworks (optional: Helm unittest plugin)

**Cognitive Load Validation**: 8 concepts ≤ 10 limit → ✅ WITHIN B1 LIMIT

**Content Structure**:

1. **Manual Walkthrough** (15 min)
   - Explain testing as "safety net for chart validity before prod deployment"
   - Show `helm lint` output: structural validation
   - Demonstrate `helm template --debug`: inspect variable resolution
   - Explain test pods: post-deployment validation (connectivity, config)
   - Show test pod exit code determines pass/fail

2. **Hands-On Exercises** (25 min)
   - Exercise 6.1: Create chart with intentional error, run `helm lint` to catch it
   - Exercise 6.2: Fix error, verify `helm lint` passes
   - Exercise 6.3: Use `helm template --debug` to inspect variables resolving correctly
   - Exercise 6.4: Create test pod that connects to service and verifies it's running
   - Exercise 6.5: Deploy chart with test pod, run `helm test`, verify pod passes
   - Exercise 6.6: Make test pod fail intentionally, verify `helm test` reports failure

3. **Validation Checkpoints** (5 min)
   - `helm lint` catches syntax errors
   - `helm template --debug` renders variables correctly
   - Test pods verify post-deployment health
   - `helm test` reports pass/fail correctly

4. **Maps to Success Evals**:
   - SC-003: All code examples execute successfully on Minikube
   - SC-008: Foundation for validation-auditor checks

5. **Try With AI**:
   - Initial Request: "Ask AI: What should a test pod include to verify chart deployment succeeded?"
   - Critical Evaluation: "Did AI include all health checks you need (connectivity, configuration, performance)?"
   - Constraint Teaching: "Our AI agent needs to verify: (1) service is reachable, (2) database connection works, (3) auth endpoint responds. What test pod would validate all three?"
   - Refinement: "Show me a test pod that validates all three conditions"
   - Final Check: "What test scenarios emerged that would prevent prod issues?"

**Maps to Evals**: SC-003, SC-008

---

### DISTRIBUTION PHASE

---

#### **Lesson 7: OCI Registries and Distribution**

**Stage**: Layer 1 (Manual Foundation)
**Duration**: 50 minutes
**Proficiency**: B1-B2

**Learning Objectives**:
1. Apply OCI registry commands to push and pull Helm charts as container images
2. Design chart versioning and distribution strategy for team sharing
3. Explain differences between traditional HTTP chart repos (deprecated) vs OCI registries
4. Execute chart dependency updates from OCI registry URLs
5. Troubleshoot registry authentication and pull errors

**Key Concepts** (count: 9, within B1 limit):
1. OCI registry basics (treats charts as "container images")
2. `helm package` to create chart tarball
3. `helm push` to upload to OCI registry
4. `helm pull` to download from registry
5. OCI URL format (`oci://registry/path/to/chart`)
6. Registry authentication (credentials, tokens)
7. Chart versioning strategy (semantic versioning)
8. Dependency URLs using `oci://` in Chart.yaml
9. Registry access control (who can push/pull)

**Cognitive Load Validation**: 9 concepts ≤ 10 limit → ✅ WITHIN B1 LIMIT

**Content Structure**:

1. **Manual Walkthrough** (15 min)
   - Explain OCI registry as "unified distribution platform for charts and containers"
   - Show HTTP chart repos are deprecated, OCI is modern standard
   - Demonstrate `helm package` creating tarball
   - Demonstrate `helm push` to Docker Hub or Harbor
   - Explain versioning semantics and breaking changes

2. **Hands-On Exercises** (30 min)
   - Exercise 7.1: Package current chart with `helm package`
   - Exercise 7.2: Create public Docker Hub account (free tier)
   - Exercise 7.3: Push chart to `oci://docker.io/username/agent-chart` with `helm push`
   - Exercise 7.4: Pull chart from registry with `helm pull`
   - Exercise 7.5: Install chart directly from registry: `helm install my-app oci://docker.io/username/agent-chart`
   - Exercise 7.6: Add chart to another chart as dependency using OCI URL in Chart.yaml

3. **Validation Checkpoints** (5 min)
   - Chart successfully pushed to registry
   - Chart pull from registry succeeds
   - Installation from OCI URL works
   - Dependency resolution finds chart in registry

4. **Maps to Success Evals**:
   - SC-007: Push/pull charts from OCI registry
   - Foundation for SC-001 (production distribution)

5. **Try With AI**:
   - Initial Request: "Ask AI: What's the workflow for sharing Helm charts with a team using OCI registries?"
   - Critical Evaluation: "Did AI explain versioning strategy? What happens if you push same version twice?"
   - Constraint Teaching: "Our team has 10 microservices, each with its own chart. We want a central registry. How do we organize chart naming and versioning?"
   - Refinement: "Show me naming convention and versioning strategy"
   - Final Check: "What distribution patterns emerged that scale across team?"

**Maps to Evals**: SC-007, Foundation for SC-001

---

#### **Lesson 8: Library Charts and Organizational Standardization**

**Stage**: Layer 1 (Manual Foundation)
**Duration**: 45 minutes
**Proficiency**: B1-B2

**Learning Objectives**:
1. Design library charts (`type: library` in Chart.yaml) to enforce organizational patterns
2. Create reusable templates that other charts inherit (common labels, probes, resources)
3. Explain why library charts cannot be installed directly (safety mechanism)
4. Apply library charts across 3+ microservice charts to achieve standardization
5. Troubleshoot template conflicts when multiple library charts define same helper

**Key Concepts** (count: 8, within B1 limit):
1. Library chart type in Chart.yaml
2. Library chart structure (templates/ with helpers, no resources to install)
3. `include` to use library templates in application charts
4. Template naming conventions to prevent conflicts (prefix: `libname.labels`)
5. Common patterns: labels, probes, resource defaults, security contexts
6. Library chart versioning and compatibility
7. Multiple library chart composition (app using 2-3 libraries)
8. Namespace isolation for template names

**Cognitive Load Validation**: 8 concepts ≤ 10 limit → ✅ WITHIN B1 LIMIT

**Content Structure**:

1. **Manual Walkthrough** (15 min)
   - Explain library charts as "reusable template libraries for standardization"
   - Show Chart.yaml `type: library` prevents installation
   - Demonstrate helper templates in library chart
   - Explain naming conventions to avoid conflicts
   - Show how application charts inherit from library

2. **Hands-On Exercises** (25 min)
   - Exercise 8.1: Create library chart with `type: library`
   - Exercise 8.2: Add common labels helper template
   - Exercise 8.3: Add resource defaults helper (requests, limits)
   - Exercise 8.4: Add probes helper (liveness, readiness, startup)
   - Exercise 8.5: Create application chart that includes library helpers
   - Exercise 8.6: Verify library chart cannot be installed directly (fails as expected)

3. **Validation Checkpoints** (5 min)
   - Library chart has `type: library` in Chart.yaml
   - Application chart successfully includes library templates
   - Consistent labels/probes across multiple resources
   - Library chart installation properly fails

4. **Maps to Success Evals**:
   - Foundation for SC-001 (production patterns)
   - Foundation for Layer 3 skill creation

5. **Try With AI**:
   - Initial Request: "Ask AI: Design a library chart for organizational Kubernetes standards"
   - Critical Evaluation: "Did AI include all common patterns (labels, probes, resources, security)?"
   - Constraint Teaching: "Our org requires: (1) standard labels for cost tracking, (2) resource requests matching node capacity, (3) health probes with specific timeouts. Design a library enforcing all three."
   - Refinement: "Show me the library chart structure and helpers"
   - Final Check: "What standardization patterns would prevent common production issues?"

**Maps to Evals**: Foundation for SC-001, Layer 3 preparation

---

### SYNTHESIS PHASE

---

#### **Lesson 9: AI-Assisted Chart Development**

**Stage**: Layer 2 (AI Collaboration with Three Roles)
**Duration**: 60 minutes
**Proficiency**: B1-B2

**Learning Objectives**:
1. Prompt AI to generate chart scaffolding and templates based on requirements
2. Evaluate AI-generated templates for correctness, maintainability, and best practices
3. Refine AI output by teaching it organizational constraints and patterns
4. Iterate with AI to improve template quality through bidirectional collaboration
5. Validate that AI-assisted chart development produces production-ready output

**Layer 2 Critical Requirement**: Demonstrate all THREE ROLES (invisible framework):
- **AI as Teacher**: AI suggests templating patterns student didn't know
- **AI as Student**: Student teaches AI organizational constraints and requirements
- **AI as Co-Worker**: Iteration improves solution through collaborative refinement

**Key Concepts** (count: 7, within B1 limit):
1. Prompting strategy for chart generation (specify requirements, not implementation)
2. AI suggestions for Go template idioms and best practices
3. Constraint expression (how to communicate requirements to AI)
4. Template quality evaluation criteria (DRY, maintainability, performance)
5. Iteration workflows (prompt → evaluate → refine → validate)
6. AI limitations (when human judgment required)
7. Validation of AI-generated templates (lint, test, deploy)

**Cognitive Load Validation**: 7 concepts = 7 limit → ✅ AT B1 BOUNDARY

**Content Structure**:

1. **Manual Walkthrough** (10 min)
   - Explain how AI collaboration accelerates chart development
   - Show prompting strategy: "Generate chart for X with requirements Y"
   - Explain Three Roles (invisible to students): what they'll experience through action
   - Clarify: AI is not "correct by default"; students must evaluate and refine

2. **Three Roles Demonstrations** (40 min)

   **Role 1: AI as Teacher** (10 min)
   - Setup: Create basic Deployment template manually
   - Prompt: "Review my template. What best practices am I missing?"
   - What AI teaches: Templating idioms (quote function, sprig functions, proper defaults)
   - Student reflection: "What patterns did AI suggest that you didn't know?"

   **Role 2: AI as Student** (10 min)
   - Setup: AI generates basic chart
   - Student feedback: "This doesn't match our organization. We need: (1) specific labels for cost tracking, (2) resource limits matching our cluster capacity, (3) pre-upgrade hooks for migrations"
   - AI adapts: "Understood. I'll revise to include organizational patterns"
   - Student learning: "How does teaching AI constraints improve the output?"

   **Role 3: AI as Co-Worker** (15 min)
   - Setup: Iterative refinement loop
   - Iteration 1: AI generates chart, student reviews, identifies gaps
   - Iteration 2: AI refines based on feedback (adds hooks, adjusts templates)
   - Iteration 3: Student refines further (adds tests, improves error handling)
   - Iteration 4: Convergence on production-ready chart
   - Co-learning: "Neither of us had the final solution at the start. What emerged through iteration?"

3. **Hands-On Exercises** (10 min)
   - Exercise 9.1: Prompt AI to generate chart scaffold for AI agent with specific requirements
   - Exercise 9.2: Review AI output, identify 3 gaps or improvements
   - Exercise 9.3: Provide feedback to AI (constraint teaching)
   - Exercise 9.4: Iterate on AI output (3-4 refinement cycles)
   - Exercise 9.5: Deploy final chart, verify it matches requirements

4. **Maps to Success Evals**:
   - Layer 2 collaboration demonstrates AI value
   - SC-001: Students can create sophisticated charts (AI-assisted)
   - SC-008: Chapter passes constitution compliance (Three Roles invisible)

5. **Try With AI**: (Integrated into exercises, not separate section)
   - This lesson IS "Try With AI"—entire lesson demonstrates AI collaboration
   - No additional "Try With AI" section needed

**Meta-Commentary Prohibition Check**: ✅ NO framework labels in student text
- Students EXPERIENCE Three Roles through action (prompt → AI teaches → evaluate → refine)
- Students DON'T READ about roles ("This is AI as Teacher")
- Students DON'T SEE "What to notice: AI is teaching you"
- Students DO reflect on effects ("What improved through iteration?")

---

#### **Lesson 10: Capstone - Production AI Agent Chart**

**Stage**: Layer 4 (Spec-Driven Integration)
**Duration**: 90 minutes
**Proficiency**: B1-B2

**Learning Objectives**:
1. Write comprehensive specification for production AI agent Helm chart (intent-first)
2. Design chart architecture composing PostgreSQL, Redis, and application dependencies
3. Implement multi-environment values files with spec-driven requirements
4. Create pre-upgrade hooks for database migrations
5. Write chart tests validating connectivity and configuration
6. Execute complete deployment with `helm install` and `helm test`

**Layer 4 Critical Requirement**: Spec-driven approach (specification FIRST, before code)

**Key Concepts** (count: 8, within B1 limit):
1. Specification structure (intent, constraints, success criteria)
2. Chart architecture (main components, dependencies, hooks)
3. Multi-environment values (dev, staging, prod)
4. Dependency composition (PostgreSQL + Redis + app)
5. Hook design (pre-install, pre-upgrade, post-upgrade)
6. Test design (connectivity, health, configuration)
7. Success criteria validation (measurable, testable)
8. Production-readiness checklist

**Cognitive Load Validation**: 8 concepts ≤ 10 limit → ✅ WITHIN B1 LIMIT

**Content Structure**:

**Part 1: Specification Writing** (20 min)
   - Specification template with sections:
     - **Intent**: What are we building? (Production AI agent deployment)
     - **Constraints**: What limitations exist? (Minikube single-node, <2Gi memory)
     - **Success Criteria**: How do we know it works? (5 measurable criteria)
     - **Non-Goals**: What we're NOT doing (advanced scaling, multi-region)
   - Spec includes:
     - Chart must support dev (1 replica), staging (2 replicas), prod (5 replicas)
     - PostgreSQL for state storage
     - Redis for caching
     - Pre-upgrade migrations
     - Health checks and tests
     - Multi-environment values files

**Part 2: Chart Implementation** (50 min)
   - Exercise 10.1: Create Chart.yaml with PostgreSQL and Redis dependencies
   - Exercise 10.2: Design values.yaml with development defaults
   - Exercise 10.3: Create values-staging.yaml and values-prod.yaml with environment-specific overrides
   - Exercise 10.4: Implement templates: Deployment, Service, ConfigMap
   - Exercise 10.5: Add pre-upgrade migration hook
   - Exercise 10.6: Write post-upgrade notification hook
   - Exercise 10.7: Create test pods for connectivity and health
   - Exercise 10.8: Validate chart with `helm lint` and `helm template --debug`

**Part 3: Deployment and Validation** (20 min)
   - Exercise 10.9: Deploy to Minikube: `helm install my-agent ./agent-chart -f values-prod.yaml`
   - Exercise 10.10: Verify all resources created (pods, services, configmaps)
   - Exercise 10.11: Run `helm test my-agent` to execute test pods
   - Exercise 10.12: Validate deployment matches specification:
     - PostgreSQL pod running (database ready)
     - Redis pod running (cache ready)
     - Agent pod running with 5 replicas (production config)
     - Tests passing (connectivity validated)
     - Pre-upgrade hook executed (migrations logged)

**Part 4: Documentation** (5 min)
   - Add README.md with:
     - Chart purpose and components
     - Installation instructions
     - Configuration options
     - Troubleshooting guide

**Maps to Success Evals**:
   - SC-001: Students create production-ready chart in <90 min
   - SC-002: Deploy to 3 environments (dev/staging/prod) with single chart
   - SC-003: All examples execute on Minikube
   - SC-006: Pre-upgrade hooks functional
   - SC-008: Chapter passes constitution validation

**Layer 4 Validation**:
- ✅ Specification written FIRST (before code)
- ✅ Reusable intelligence from Lessons 1-9 applied (templates, hooks, tests, dependencies)
- ✅ Implementation aligns with specification (validated through acceptance tests)
- ✅ Student can articulate design decisions and tradeoffs

---

#### **Lesson 11: Building a Helm Chart Skill**

**Stage**: Layer 3 (Intelligence Design)
**Duration**: 45 minutes
**Proficiency**: B1-B2

**Learning Objectives**:
1. Extract reusable patterns from Lessons 1-10 into skill specification (Persona + Questions + Principles)
2. Design skill for architectural decision-making in Helm chart design
3. Create skill with decision frameworks for chart structure, dependency management, hook design
4. Document skill usage patterns and integration with future projects
5. Validate skill enables chart architects to make consistent, principled decisions

**Reusable Intelligence Format**: Persona + Questions + Principles (from Constitution)

**Key Concepts** (count: 7, within B1 limit):
1. Skill persona (chart architect thinking)
2. Analysis questions (what decisions does charter face?)
3. Decision frameworks (principles that guide choices)
4. Chart architecture patterns (single vs umbrella charts)
5. Dependency composition patterns (when to use subcharts)
6. Hook lifecycle patterns (pre-install, pre-upgrade, post-upgrade)
7. Multi-environment strategies (values file hierarchy)

**Cognitive Load Validation**: 7 concepts = 7 limit → ✅ AT B1 BOUNDARY

**Content Structure**:

1. **Intelligence Extraction** (15 min)
   - Analyze Lessons 1-10 for recurring decision points
   - Identify patterns that recur across projects:
     - "When should I use subcharts vs single-chart?"
     - "How do I structure values for multi-environment?"
     - "What hooks do I need for production deployments?"
     - "How do I test charts effectively?"

2. **Skill Specification** (25 min)

   **Skill Name**: `helm-chart-architect`

   **Persona Section**:
   ```
   Think like a production DevOps engineer designing Helm deployments.
   Your decisions balance team standardization (reduce drift) with project flexibility
   (accommodate unique requirements). You reason about operational burden:
   "Will future maintainers understand this architecture?"
   ```

   **Questions Section** (Decision Points):
   1. **Chart Scope**: Is this a single-app chart or umbrella (multi-component)?
      - Single: Deploy one application
      - Umbrella: Deploy coordinated services (app + DB + cache)
      - Implication: Affects structure, dependency count, maintenance burden

   2. **Dependencies**: What external services does this app require?
      - Database (PostgreSQL, MongoDB)? → Add as subchart
      - Cache (Redis, Memcached)? → Add as subchart
      - Message queue (Kafka, RabbitMQ)? → Add as subchart
      - Implication: More dependencies = more complexity, more testing, more failure points

   3. **Multi-Environment**: How many environments must this support?
      - Dev (low cost, minimal resources)
      - Staging (like prod but smaller)
      - Prod (full resources, high availability)
      - Implication: Requires values file hierarchy and careful override precedence

   4. **Lifecycle Management**: What pre/post-deployment operations are needed?
      - Database migrations? → pre-upgrade hook
      - Cache warming? → post-install hook
      - Notifications? → post-upgrade hook
      - Implication: Hook errors cause deployment failures—must be robust

   5. **Testing Strategy**: What must be validated post-deployment?
      - Service reachability (can other pods call this service)?
      - Configuration correctness (are values applied correctly)?
      - Database connectivity (does the app reach PostgreSQL)?
      - Implication: Good tests prevent silent failures

   **Principles Section** (Decision Frameworks):

   1. **DRY Template Design**
      - Use named templates for repeated patterns (labels, probes, resource defaults)
      - One truth source prevents drift—if label changes, it changes everywhere
      - Implication: Maintainability compounds with chart age

   2. **Progressive Complexity for Environments**
      - Dev values: Minimal resources, single replica, debugging enabled
      - Staging values: Prod-like but smaller, same structure for final validation
      - Prod values: Maximum resources, high availability, security hardened
      - Implication: Dev is cheap, prod is safe—no surprises at deployment time

   3. **Hook Safety First**
      - Hooks that fail MUST block deployment (safety mechanism)
      - Migration failures should prevent rollout (data integrity)
      - Verify hooks complete before marking chart "successful"
      - Implication: Slow deployment is better than broken data

   4. **Library Charts for Standardization**
      - Create library charts when 3+ apps need identical patterns
      - Library prevents drift and enforces organizational standards
      - Inheritance is more powerful than copy-paste
      - Implication: Standardization at scale requires shared templates

   3. **Test Coverage Proportional to Criticality**
      - Database-backed apps: Test connectivity, migrations, data integrity
      - Stateless services: Test reachability, configuration loading
      - Public APIs: Test endpoint responsiveness, auth
      - Implication: Test scope matches criticality of failures

3. **Skill Documentation** (5 min)
   - Document with examples for each decision point
   - Provide checklist for chart architects
   - Example: "Designing chart for PostgreSQL-backed AI agent"
     - Chart scope? → Umbrella (app + PostgreSQL)
     - Multi-environment? → Yes (dev/staging/prod)
     - Hooks? → pre-upgrade (migrations), post-upgrade (cache invalidation)
     - Tests? → Connectivity + health checks
     - Library? → Yes (organize 5+ similar services)

4. **Skill Validation**:
   - Use skill on new project (outside Chapter 51)
   - Verify decisions are consistent with framework
   - Check that skill accelerates decision-making
   - Refine skill based on real-world application

**Maps to Success Evals**:
   - Layer 3 intelligence creation
   - Reusable across future chart-related chapters
   - Foundation for Chapter 52+ (CI/CD, GitOps, observability)

**Skill File Structure**:
```
.claude/skills/engineering/helm-chart-architect/
├── SKILL.md                    # Persona + Questions + Principles
├── examples/
│   ├── single-app-chart.md
│   ├── umbrella-chart.md
│   └── multi-environment.md
└── checklist.md                # Decision checklist for architects
```

---

## Dependency Graph and Learning Progression

```
Lesson 1 (Advanced Go Templating)
    ↓ (foundation: manual template syntax)
    ↓
Lesson 2 (Named Templates & Helpers)
    ↓ (builds on: variable management, DRY patterns)
    ↓
Lesson 3 (Values Deep Dive)
    ↓ (enables: environment-specific configs)
    ↓
Lesson 4 (Chart Dependencies)
    ↓ (requires: template understanding + values mastery)
    ↓
Lesson 5 (Helm Hooks & Lifecycle)
    ↓ (optional dependency on 4, but common in real charts)
    ↓
Lesson 6 (Testing Your Charts)
    ↓ (applies to all previous: validates 1-5)
    ↓
Lesson 7 (OCI Registries)
    ↓ (optional: distribution of charts from 1-6)
    ↓
Lesson 8 (Library Charts)
    ↓ (optional: organizational patterns using 1-6)
    ↓
Lesson 9 (AI-Assisted Development) ← Layer 2 Collaboration
    ↓ (recaps 1-8, accelerates chart development with AI)
    ↓
Lesson 10 (Capstone - Production Chart) ← Layer 4 Spec-Driven
    ↓ (applies all skills 1-9: goes templates, deps, hooks, tests)
    ↓
Lesson 11 (Building a Helm Chart Skill) ← Layer 3 Intelligence
    ↓ (extracts reusable patterns from 1-10)
    ↓
Outcome: Reusable skill for future chart projects
```

**Critical Path Dependencies**:
- Lessons 1-3 MUST be completed before 4-6 (foundational concepts)
- Lessons 4-6 can be completed in any order (parallel concepts)
- Lesson 7-8 optional but recommended for production contexts
- Lesson 9 MUST come after 1-8 (requires competence before AI collaboration)
- Lesson 10 MUST come after 9 (capstone integrates all layers)
- Lesson 11 MUST come after 10 (extracts patterns from complete chapter)

---

## Pedagogical Arc Validation

### Foundation → Application → Distribution → Synthesis

**Foundation Phase (Lessons 1-3)**: Building templating vocabulary
- Lesson 1 teaches: "How do I make templates dynamic?"
- Lesson 2 teaches: "How do I make templates maintainable?"
- Lesson 3 teaches: "How do I configure templates for different contexts?"
- Outcome: Students can create sophisticated, maintainable, configurable templates

**Application Phase (Lessons 4-6)**: Real-world deployment patterns
- Lesson 4 teaches: "How do I manage multi-component deployments?"
- Lesson 5 teaches: "How do I orchestrate deployment lifecycle?"
- Lesson 6 teaches: "How do I validate deployments before production?"
- Outcome: Students can architect production-ready, tested, orchestrated systems

**Distribution Phase (Lessons 7-8)**: Enterprise scaling patterns
- Lesson 7 teaches: "How do I share charts across teams?"
- Lesson 8 teaches: "How do I enforce organizational standards?"
- Outcome: Students can distribute and standardize charts across organizations

**Synthesis Phase (Lessons 9-11)**: Intelligence and mastery
- Lesson 9 teaches: "How do I use AI to accelerate chart development?"
- Lesson 10 teaches: "How do I orchestrate complete AI deployment systems?"
- Lesson 11 teaches: "How do I capture and reuse this knowledge?"
- Outcome: Students are chart architects who think systematically, collaborate with AI, and share intelligence

---

## Cognitive Load Distribution

| Lesson | Concepts | Tier Limit | Load | Justification |
|--------|----------|-----------|------|---------------|
| 1 | 9 | 10 | FULL | Complex Go template syntax |
| 2 | 7 | 10 | MODERATE | Reusable patterns, manageable |
| 3 | 8 | 10 | MODERATE | Values hierarchy, lots of precedence rules |
| 4 | 9 | 10 | FULL | Dependency management complex |
| 5 | 10 | 10 | FULL+ | 9 hook types = intricate semantics (justified) |
| 6 | 8 | 10 | MODERATE | Testing patterns, straightforward |
| 7 | 9 | 10 | FULL | OCI registry operations |
| 8 | 8 | 10 | MODERATE | Library patterns, organizational |
| 9 | 7 | 10 | LIGHT | AI collaboration, fewer new concepts |
| 10 | 8 | 10 | MODERATE | Spec-driven integration, familiar patterns |
| 11 | 7 | 10 | LIGHT | Skill design, P+Q+P framework |

**Pattern**: Cognitive load varies across lessons, peaks at Lesson 5 (hooks), lightens at Lesson 9 (AI collaboration), and maintains sustainable load through capstone.

---

## Risk Mitigation

### Risk 1: Minikube Resource Constraints

**Problem**: Some exercises require significant resources (PostgreSQL + Redis + app = 2+ Gi memory). Minikube default: 2-4 Gi.

**Mitigation**:
- Document minimum Minikube setup: `minikube start --memory=4096 --cpus=2`
- Provide "fallback: local registry" for Docker Hub rate limits
- Lesson exercises include resource downsizing guidance
- Capstone explicitly states: "Run on Minikube with 4Gi minimum"

### Risk 2: OCI Registry Authentication

**Problem**: Lesson 7 requires Docker Hub account + login. Potential blockers: account creation, authentication tokens.

**Mitigation**:
- Lesson 7 provides step-by-step Docker Hub setup (free tier)
- Alternative: Local registry setup with `docker run registry:2`
- Provide pre-generated credentials as fallback
- Document common auth errors and solutions

### Risk 3: Go Template Learning Curve

**Problem**: Students from non-programming backgrounds may struggle with template syntax (pipelines, filters, context switching).

**Mitigation**:
- Lesson 1 includes MANY worked examples (manual walkthrough)
- Progressive complexity: variables → pipelines → with/range → conditionals
- `helm template --debug` provides immediate feedback for errors
- AI collaboration in Lesson 9 reduces template writing burden

### Risk 4: Hook Failure Blocking Deployment

**Problem**: Lesson 5 teaches that failed hooks block deployment (safety mechanism). Students may create failing hooks and become confused.

**Mitigation**:
- Lesson explicitly explains: "Hook failure = deployment failure (intentional safety)"
- Exercise 5.4: Deliberately create failing hook to demonstrate
- Document debugging: `kubectl logs [hook-pod-name]`
- Capstone includes working hooks (prevents confusion)

### Risk 5: Values Precedence Confusion

**Problem**: Helm values override precedence is counterintuitive (CLI > -f file > defaults). Students may expect different precedence.

**Mitigation**:
- Lesson 3 includes precedence diagram (visual explanation)
- Exercise 3.3: Explicitly test precedence ordering
- Include edge case: "What happens if both -f file and --set override same value?" (CLI wins)
- Constitution note: Values hierarchy is common misconception in Kubernetes

### Risk 6: Three Roles Framework Exposure (Constitutional Violation)

**Problem**: Lesson 9 must demonstrate Three Roles while keeping framework INVISIBLE to students. Risk: Exposition creeping into lesson text.

**Mitigation**:
- Lesson 9 uses action prompts + reflection questions (no role labels)
- Validation: Grep for "Three Roles", "AI as Teacher", "What to notice" → ZERO matches required
- Lesson 9 structure focuses on what students EXPERIENCE, not what framework labels it
- Constitution Section IIa provides exact validation patterns

### Risk 7: Capstone Scope Explosion

**Problem**: Lesson 10 asks for complete production chart (PostgreSQL + Redis + hooks + tests + multi-environment). Risk: 90 minutes insufficient, students get overwhelmed.

**Mitigation**:
- Lesson 10 clearly scopes: "We're creating ONE deployment, not enterprise system"
- Provide chart scaffold (Chart.yaml + values.yaml template structure)
- Break into 4 parts: Spec (20 min) + Implementation (50 min) + Deployment (20 min)
- Success criteria are specific, measurable, achievable in time

### Risk 8: Skill Design Feels Abstract

**Problem**: Lesson 11 asks students to design reusable skill using Persona + Questions + Principles. Risk: Students don't understand why this matters or how to apply it.

**Mitigation**:
- Lesson 11 explicitly shows: "This skill enables you to design charts for future AI services"
- Provide completed skill example (from Lessons 1-10)
- Validation: Use skill on new project (outside Chapter 51) to demonstrate value
- Layer 3 provides explicit structure (P+Q+P), not vague "capture what you learned"

---

## Success Validation Criteria

### Per-Lesson Validation Checkpoints

**Lesson 1 & 2 & 3** (Foundation Phase):
- ✅ Students can create template with variables, pipelines, conditionals
- ✅ Students can use `helm template --debug` to verify rendering
- ✅ `helm lint` passes all structural validation

**Lesson 4 & 5 & 6** (Application Phase):
- ✅ Chart dependencies resolve without conflicts
- ✅ Hooks execute in expected order (verified via logs)
- ✅ `helm test` passes all test pods

**Lesson 7 & 8** (Distribution Phase):
- ✅ Chart successfully pushes to OCI registry
- ✅ Chart successfully pulls from registry
- ✅ Library chart cannot be installed directly (fails as expected)

**Lesson 9** (AI Collaboration):
- ✅ Three Roles demonstrated but framework invisible (grep validation)
- ✅ Student can articulate what improved through AI iteration
- ✅ AI-assisted chart functionally equivalent to manual chart

**Lesson 10** (Capstone):
- ✅ Chart deploys successfully to Minikube with single `helm install` command
- ✅ All resources created (Pods, Services, ConfigMaps, etc.)
- ✅ `helm test` passes (connectivity and health verified)
- ✅ Pre-upgrade hook executes (migrations logged)
- ✅ Multi-environment values work (dev/staging/prod all deploy)

**Lesson 11** (Skill Creation):
- ✅ Skill file follows P+Q+P structure
- ✅ Skill enables chart design decisions for new project
- ✅ Skill captures patterns from Lessons 1-10

### Constitutional Compliance Validation

**Meta-Commentary Prohibition** (Constitution v6.0.1):
```bash
# Check student-facing lesson files
grep -i "What to notice\|What to expect\|AI.*teach\|AI.*learn\|AI as\|Layer [0-9]" *.md
# Expected: Zero matches
```

**Try With AI Section**:
```bash
# Check each lesson ends with "Try With AI" using action+reflection
grep "## Try With AI" *.md
# Expected: Present in Lessons 1-10 (Lesson 9 IS Try With AI, Lesson 11 optional)
```

**Code Examples Tested**:
- All code blocks accompanied by execution logs
- `helm lint`, `helm template --debug`, `helm test` all pass

**Cross-Chapter Differentiation**:
- No duplication of Chapter 50 Lesson 20 basic Helm content
- Chapter 51 assumes students have completed Chapter 50

---

## Research & Sourcing

### Primary Sources
1. **Official Helm Documentation** (context7-helm-docs)
   - Template syntax, Go packages, hook lifecycle
   - OCI registry support (Helm 3.7+)
   - Library chart patterns

2. **Kubernetes Documentation** (official)
   - Pod lifecycle hooks (different from Helm hooks—clarify distinction)
   - Deployment lifecycle, rolling updates, rollback

3. **Bitnami Charts** (real-world examples)
   - PostgreSQL chart structure
   - Redis chart dependencies
   - Common patterns from production charts

4. **Go Template Reference**
   - Pipeline operators, functions, conditional syntax
   - sprig package (Go template functions used by Helm)

### Secondary Sources
- Official Helm blog posts (Helm 3.13+ OCI improvements)
- Cloud-native blogs (production Helm patterns)
- GitHub examples (organization library charts)

### Knowledge Validation
- All code executed on Minikube 1.32+ with Helm 3.14+
- All claims verified against official docs
- Examples tested with student's Part 6 AI agent

---

## Quality Gates

### Pre-Implementation Gate (Spec Approval)
- ✅ Specification complete and unambiguous
- ✅ Chapter differentiation from Chapter 50 Lesson 20 clear
- ✅ 11 lessons justified by concept density analysis
- ✅ Layer progression (1→2→3→4) valid

### Phase 1: Lesson Implementation
- ✅ Each lesson written following plan structure
- ✅ All code examples tested on Minikube
- ✅ All claims verified against authoritative sources
- ✅ Cognitive load within B1 limits

### Phase 2: Constitutional Compliance
- ✅ Zero meta-commentary in student-facing text
- ✅ Three Roles invisible in Lesson 9 (grep validation)
- ✅ "Try With AI" sections follow action+reflection template
- ✅ No "Key Takeaways" or "What's Next" sections

### Phase 3: Capstone Validation
- ✅ Lesson 10 capstone produces deployable chart
- ✅ Chart deploys to Minikube without cloud dependencies
- ✅ Capstone demonstrates mastery across all layers

### Phase 4: Technical Review
- ✅ validation-auditor confirms constitutional compliance
- ✅ All code examples pass execution tests
- ✅ All technical claims verified
- ✅ Chapter superior to official Helm docs (qualitative assessment)

---

## Skill File Deliverable

**File**: `.claude/skills/engineering/helm-chart-architect/SKILL.md`

**Structure**:
```
---
name: "helm-chart-architect"
description: "Think systematically about Helm chart design for production AI services. Activates reasoning about chart structure (single vs umbrella), dependencies (what external services?), lifecycle (what hooks?), testing (what validates?), and standardization (library patterns?). Use when designing new charts or reviewing chart architecture."
version: "1.0.0"
domain: "engineering"
applies_to: "Helm chart design, Kubernetes deployment architecture"
---

# Helm Chart Architect Skill

## Persona
Think like a production DevOps engineer...

## Analysis Questions
1. Chart Scope: Single-app vs umbrella?
2. Dependencies: What external services?
...

## Principles
1. DRY Template Design
2. Progressive Complexity for Environments
...
```

---

## Chapter Success Metrics

**Quantitative**:
- 11 lessons implemented
- 40+ hands-on exercises
- 100% code examples tested
- 5+ success evals mapped to lessons
- Zero constitutional violations

**Qualitative**:
- Chapter superior to official Helm docs in pedagogical clarity
- Students can explain Helm design decisions
- Students can architect production charts
- Reusable skill enables future projects

**Mastery Outcomes**:
- SC-001: Students create production charts in <30 min (capstone timing)
- SC-002: Deploy to 3 environments with single chart
- SC-003: All examples execute on Minikube
- SC-005: Integrate community charts as dependencies
- SC-006: Implement lifecycle hooks
- SC-007: Distribute charts via OCI registries

---

## Next Steps (Integration with Future Chapters)

This chapter feeds forward to:
- **Chapter 52** (CI/CD & Infrastructure-as-Code): Helm charts + GitOps integration
- **Chapter 53** (Dapr): Helm charts for Dapr actor patterns
- **Chapter 54** (Observability): Helm charts with monitoring sidecars

Reusable Helm Chart Skill created in Lesson 11 applies to all downstream chapters.

---

**Plan Status**: Ready for lesson implementation
**Target Timeline**: 5-7 days for market-defining quality
**Quality Tier**: Market-defining (comprehensive research, 11 lessons, AI collaboration innovation)
