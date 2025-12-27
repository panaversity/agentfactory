---
sidebar_position: 5
chapter: 51
lesson: 5
duration_minutes: 50
title: "Chart Dependencies"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Compose charts with community subcharts like PostgreSQL and Redis"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Add dependencies to Chart.yaml with version constraints"
    bloom_level: "Apply"
  - id: LO2
    description: "Use conditions to enable/disable optional dependencies"
    bloom_level: "Apply"
  - id: LO3
    description: "Configure subcharts through parent values"
    bloom_level: "Apply"
  - id: LO4
    description: "Use import-values to expose subchart configuration"
    bloom_level: "Apply"
  - id: LO5
    description: "Run helm dependency update to fetch subcharts"
    bloom_level: "Apply"
---

# Chart Dependencies

In Lessons 1-3, you built sophisticated Helm charts from scratch: advanced templating, named helpers, and multi-environment values management. But professional AI deployments rarely start with nothing. Your AI agent needs PostgreSQL for state storage, Redis for caching, and perhaps RabbitMQ for message queues.

Instead of writing database charts yourself, you'll use community charts maintained by Bitnami and others. Helm dependency management is how you compose these public charts into your custom chart—a single `helm install` deploys your agent, its database, and all supporting services together.

This lesson teaches how to declare, configure, and manage chart dependencies—transforming your custom chart into an orchestration layer that composes production infrastructure.

---

## What You'll Learn

This lesson covers 9 concepts organized into three groups:

| Concept Group | Concepts | Focus |
|---------------|----------|-------|
| **Dependency Declaration** | 1-3 | Chart.yaml syntax, version constraints, fetching subcharts |
| **Conditional Management** | 4-5 | Enabling/disabling dependencies with conditions and tags |
| **Configuration Patterns** | 6-9 | Passing values, import-values, aliases, complete workflow |

**Prerequisites:**
- Lessons 1-3 completed (Chart.yaml structure, templates, values management)
- Understanding of semantic versioning (MAJOR.MINOR.PATCH)
- Familiarity with YAML nested structures

**Time Estimate:** 50 minutes
- Concepts: 25 minutes
- Exercises: 20 minutes
- Try With AI: 5 minutes

---

## Concept 1: Chart.yaml Dependencies Section

A Helm chart declares its dependencies in Chart.yaml using a `dependencies:` list. Each dependency specifies a repository, chart name, version, and optional metadata.

### Basic Dependency Declaration

```yaml
# task-api-chart/Chart.yaml
apiVersion: v2
name: task-api
version: 0.1.0
description: AI agent with PostgreSQL and Redis

dependencies:
  - name: postgresql
    version: "13.2.0"
    repository: "https://charts.bitnami.com/bitnami"
  - name: redis
    version: "18.5.0"
    repository: "https://charts.bitnami.com/bitnami"
```

**Output:**

When you run `helm dependency list task-api-chart`, Helm displays:
```
NAME            VERSION         REPOSITORY
postgresql      13.2.0          https://charts.bitnami.com/bitnami
redis           18.5.0          https://charts.bitnami.com/bitnami
```

Each dependency entry tells Helm: "Find this chart at that repository with that version."

---

## Concept 2: Subchart Resolution

Declaring a dependency is not enough—Helm must fetch the actual chart files. The `helm dependency update` command downloads subcharts and stores them in a `charts/` directory.

### Running Dependency Update

```bash
helm dependency update task-api-chart
```

**Output:**

```
Getting updates for unmanaged Helm repositories...
...Successfully got an update from the "bitnami" chart repository
Update Complete. ⎈Happy Helming!⎈
```

### The charts/ Directory

After running `helm dependency update`, your chart structure becomes:

```
task-api-chart/
├── Chart.yaml
├── values.yaml
├── charts/
│   ├── postgresql/
│   │   ├── Chart.yaml
│   │   ├── values.yaml
│   │   ├── templates/
│   │   └── ...
│   └── redis/
│       ├── Chart.yaml
│       ├── values.yaml
│       ├── templates/
│       └── ...
├── templates/
│   ├── deployment.yaml
│   └── ...
└── ...
```

The `charts/` directory contains complete subcharts. Helm treats them as part of your chart when rendering and installing.

---

## Concept 3: Version Constraints (Semantic Versioning)

Specifying exact versions (`"13.2.0"`) is restrictive. Version constraints allow you to accept compatible updates automatically.

### Semantic Versioning Rules

Helm uses standard semantic versioning constraints:

```yaml
dependencies:
  # Exact version only
  - name: postgresql
    version: "13.2.0"
    repository: "https://charts.bitnami.com/bitnami"

  # Caret: Compatible with this minor version (13.x.x)
  - name: redis
    version: "^18.5.0"
    repository: "https://charts.bitnami.com/bitnami"

  # Tilde: Compatible with this patch version (18.5.x)
  - name: rabbitmq
    version: "~11.1.0"
    repository: "https://charts.bitnami.com/bitnami"

  # Greater than or equal (allows any newer version)
  - name: mongodb
    version: ">=13.0.0"
    repository: "https://charts.bitnami.com/bitnami"

  # Range (version 12.x up to but not including 13.0.0)
  - name: elasticsearch
    version: ">=12.0.0,<13.0.0"
    repository: "https://charts.bitnami.com/bitnami"
```

**Output (helm dependency update behavior):**

- `"13.2.0"` → Always installs exactly 13.2.0 (no updates)
- `"^18.5.0"` → Installs 18.5.0, but accepts 18.10.0 or 18.99.0 (minor/patch updates)
- `"~11.1.0"` → Installs 11.1.0, but accepts 11.1.3 (patch updates only)
- `">=13.0.0"` → Installs 13.0.0 and accepts any newer version (risky in production)

For production AI services, use caret (`^`) constraints to get bug fixes without risking breaking changes.

---

## Concept 4: Conditions for Optional Dependencies

Not every deployment needs every dependency. A developer environment might skip PostgreSQL (using SQLite instead), while production requires it. The `condition:` field gates whether a subchart is installed.

### Conditional Dependency Declaration

```yaml
# task-api-chart/Chart.yaml
dependencies:
  - name: postgresql
    version: "^13.0.0"
    repository: "https://charts.bitnami.com/bitnami"
    condition: postgresql.enabled

  - name: redis
    version: "^18.0.0"
    repository: "https://charts.bitnami.com/bitnami"
    condition: redis.enabled
```

### Controlling with Values

```yaml
# task-api-chart/values.yaml
postgresql:
  enabled: true
  auth:
    username: agent
    password: secretpassword
    database: agent_db

redis:
  enabled: false  # Skip Redis in default config
```

**Output (helm install with this values.yaml):**

- PostgreSQL subchart IS rendered and installed (condition: `postgresql.enabled = true`)
- Redis subchart is NOT rendered (condition: `redis.enabled = false`)

To enable Redis:
```bash
helm install task-api ./task-api-chart --set redis.enabled=true
```

---

## Concept 5: Tags for Grouping Dependencies

When you have many optional dependencies, managing them individually is tedious. The `tags:` field groups dependencies, so you can enable/disable groups with a single condition.

### Tagging Dependencies

```yaml
# task-api-chart/Chart.yaml
dependencies:
  # Database tier
  - name: postgresql
    version: "^13.0.0"
    repository: "https://charts.bitnami.com/bitnami"
    condition: postgresql.enabled
    tags:
      - database

  # Caching tier
  - name: redis
    version: "^18.0.0"
    repository: "https://charts.bitnami.com/bitnami"
    condition: redis.enabled
    tags:
      - cache

  # Message queue tier
  - name: rabbitmq
    version: "^12.0.0"
    repository: "https://charts.bitnami.com/bitnami"
    condition: rabbitmq.enabled
    tags:
      - messaging
```

### Controlling with Tag Values

```yaml
# task-api-chart/values.yaml
tags:
  database: true    # Enable all dependencies tagged with "database"
  cache: false      # Disable all dependencies tagged with "cache"
  messaging: false  # Disable all dependencies tagged with "messaging"

postgresql:
  enabled: null     # Controlled by "database" tag
redis:
  enabled: null
rabbitmq:
  enabled: null
```

**Output:**

- PostgreSQL is installed (because `tags.database=true`)
- Redis and RabbitMQ are skipped (their tags are false)

To enable all data services:
```bash
helm install task-api ./task-api-chart --set tags.cache=true --set tags.messaging=true
```

---

### Checkpoint: Dependency Declaration & Control

You've covered the fundamentals of dependency management. Here's a quick reference:

| Feature | Syntax | Purpose | Example |
|---------|--------|---------|---------|
| **Basic dependency** | `name:`, `version:`, `repository:` | Declare subchart requirement | `postgresql 13.2.0` |
| **Version constraint** | `^`, `~`, `>=`, ranges | Allow compatible updates | `^13.0.0` (minor/patch OK) |
| **Condition** | `condition: subchart.enabled` | Optional dependency | Enable/disable per environment |
| **Tags** | `tags: [database]` | Group dependencies | Enable all databases together |
| **Fetch subcharts** | `helm dependency update` | Download to `charts/` | Creates local copies |

**Self-Check Questions:**

1. **Version Constraints**: If you declare `version: "^18.5.0"` for Redis, will Helm accept version 19.0.0?
   <details><summary>Answer</summary>No. Caret (`^`) allows minor and patch updates within the same major version (18.x.x), but not major version changes.</details>

2. **Conditions vs Tags**: When would you use `condition:` instead of `tags:`?
   <details><summary>Answer</summary>Use `condition:` for individual dependency control. Use `tags:` when grouping related dependencies (like all caching services) for bulk enable/disable.</details>

3. **Missing helm dependency update**: What happens if you add a dependency to Chart.yaml but skip `helm dependency update`?
   <details><summary>Answer</summary>Helm install will fail with a "dependency not found" error because the subchart files don't exist in `charts/` yet.</details>

---

## Concept 6: import-values for Subchart Configuration

Subcharts have their own `values.yaml` files with configurations. You configure them through your parent values, but exposing subchart settings directly clutters your values file. The `import-values:` field re-exports selected subchart values to your parent namespace.

### Standard Approach (No import-values)

Without import-values, subchart configuration is deeply nested:

```yaml
# task-api-chart/values.yaml
postgresql:
  auth:
    username: agent
    password: secretpassword
    database: agent_db
  primary:
    persistence:
      size: 10Gi
  metrics:
    enabled: true
```

In your templates, you reference the full path: `{{ .Values.postgresql.auth.username }}`.

### Simplifying with import-values

```yaml
# task-api-chart/Chart.yaml
dependencies:
  - name: postgresql
    version: "^13.0.0"
    repository: "https://charts.bitnami.com/bitnami"
    condition: postgresql.enabled
    import-values:
      - child: auth
        parent: postgresqlAuth
      - child: primary.persistence
        parent: postgresqlStorage
```

With this mapping, your values become:

```yaml
# task-api-chart/values.yaml
postgresqlAuth:
  username: agent
  password: secretpassword
  database: agent_db

postgresqlStorage:
  size: 10Gi
```

In templates: `{{ .Values.postgresqlAuth.username }}` (cleaner namespace).

**Output:**

When Helm renders, the import-values mapping translates `postgresqlAuth` back to the subchart's `auth` namespace, so PostgreSQL receives the correct configuration.

---

## Concept 7: Passing Configuration to Subcharts

Subcharts inherit parent values using a specific naming convention. The subchart name becomes a key in the parent values.

### Configuration Hierarchy

```yaml
# task-api-chart/values.yaml
# Direct subchart configuration
postgresql:
  auth:
    username: agent
    password: secretpassword
  primary:
    persistence:
      enabled: true
      size: 20Gi

redis:
  auth:
    enabled: true
    password: cachepassword
  replica:
    replicaCount: 2
```

**Output (how subcharts receive values):**

When Helm renders the postgresql subchart, it has access to `.Values.postgresql.*` configuration. The subchart's templates reference its own `.Values.auth.username`, but the parent passes it via `.Values.postgresql.auth.username`.

Helm automatically translates the hierarchy: parent's `postgresql:` key becomes subchart's root values.

---

## Concept 8: Aliases for Multiple Instances

Sometimes you need the same chart multiple times with different configurations (two PostgreSQL instances for different databases, or two Redis caches for different use cases). The `alias:` field enables this.

### Multiple Instance Declaration

```yaml
# task-api-chart/Chart.yaml
dependencies:
  # Primary database
  - name: postgresql
    version: "^13.0.0"
    repository: "https://charts.bitnami.com/bitnami"
    alias: postgresql-primary

  # Secondary database for analytics
  - name: postgresql
    version: "^13.0.0"
    repository: "https://charts.bitnami.com/bitnami"
    alias: postgresql-analytics
```

### Configuring Each Instance

```yaml
# task-api-chart/values.yaml
postgresql-primary:
  auth:
    database: agent_primary
  primary:
    persistence:
      size: 50Gi

postgresql-analytics:
  auth:
    database: agent_analytics
  primary:
    persistence:
      size: 100Gi  # Larger for analytics data
```

**Output:**

Two independent PostgreSQL instances are deployed, each receiving its own configuration. The alias key determines which subchart configuration applies.

---

## Concept 9: The Complete Dependency Workflow

Here's how all these pieces work together:

### Step 1: Declare Dependencies in Chart.yaml

```yaml
# task-api-chart/Chart.yaml
apiVersion: v2
name: task-api-chart
version: 1.0.0

dependencies:
  - name: postgresql
    version: "^13.0.0"
    repository: "https://charts.bitnami.com/bitnami"
    condition: postgresql.enabled
    tags:
      - database
```

### Step 2: Configure in values.yaml

```yaml
# task-api-chart/values.yaml
tags:
  database: true

postgresql:
  enabled: null  # Controlled by tag
  auth:
    username: agent
    password: mypassword
    database: agentdb
  primary:
    persistence:
      size: 10Gi
```

### Step 3: Update Dependencies

```bash
cd task-api-chart
helm dependency update
```

**Output:**

```
Getting updates for unmanaged Helm repositories...
...Successfully got an update from the "bitnami" chart repository
Update Complete. ⎈Happy Helming!⎈
```

The `charts/postgresql/` directory now exists.

### Step 4: Reference Subchart Values in Templates

```yaml
# task-api-chart/templates/deployment.yaml
spec:
  containers:
  - name: agent
    env:
    - name: DATABASE_HOST
      value: "{{ .Release.Name }}-postgresql"
    - name: DATABASE_USER
      value: "{{ .Values.postgresql.auth.username }}"
    - name: DATABASE_NAME
      value: "{{ .Values.postgresql.auth.database }}"
```

### Step 5: Install Combined Chart

```bash
helm install task-api ./task-api-chart
```

**Output:**

```
NAME: task-api
STATUS: deployed
REVISION: 1

Resources created:
  - Deployment (task-api)
  - Service (task-api)
  - StatefulSet (task-api-postgresql)
  - Service (task-api-postgresql)
  - ConfigMap (task-api-postgresql-config)
  - Secret (task-api-postgresql-password)
  ...
```

A single `helm install` command deployed your agent, its database, and all supporting infrastructure.

---

## Common Mistakes

**1. Forgetting `helm dependency update` After Changing Chart.yaml**

```yaml
# Added new dependency to Chart.yaml
dependencies:
  - name: redis
    version: "^18.0.0"
    repository: "https://charts.bitnami.com/bitnami"
```

```bash
# Immediately run helm install without updating
helm install task-api ./task-api-chart
# ERROR: dependency "redis" not found
```

**Fix:** Always run `helm dependency update` after modifying the `dependencies:` section.

**2. Using Exact Versions in Production**

```yaml
dependencies:
  - name: postgresql
    version: "13.2.0"  # ❌ Locked to exact version, no security patches
```

**Fix:** Use semantic versioning constraints like `^13.2.0` to automatically receive patch and minor updates.

**3. Conflicting Condition and Tag Settings**

```yaml
# Chart.yaml
dependencies:
  - name: postgresql
    condition: postgresql.enabled
    tags:
      - database
```

```yaml
# values.yaml - CONFLICT
tags:
  database: false  # Tag says disable

postgresql:
  enabled: true    # Condition says enable
```

**What happens:** The `condition:` field takes precedence over `tags:`, so PostgreSQL WILL be installed (enabled=true wins).

**Fix:** When using both conditions and tags, set individual `enabled:` to `null` and control via tags only.

**4. Incorrect import-values Mapping**

```yaml
# Chart.yaml - WRONG
import-values:
  - child: auth
    parent: postgresql.auth  # ❌ Don't nest under subchart name
```

**Fix:** The `parent:` field creates a NEW top-level key in parent values:
```yaml
import-values:
  - child: auth
    parent: postgresAuth  # ✅ Top-level key
```

**5. Forgetting to Configure Subchart Service Names**

```yaml
# Parent deployment.yaml - WRONG
env:
  - name: DATABASE_HOST
    value: "postgresql"  # ❌ Missing release name prefix
```

**Fix:** Subchart service names are prefixed with the release name:
```yaml
env:
  - name: DATABASE_HOST
    value: "{{ .Release.Name }}-postgresql"  # ✅ Full service name
```

---

## Exercise 4.1: Add PostgreSQL Dependency

Add Bitnami PostgreSQL to your chart with version constraint `^13.0.0`.

**Instructions:**

1. Open your `task-api-chart/Chart.yaml`
2. Add a `dependencies:` section after `description:`
3. Add PostgreSQL with repository `https://charts.bitnami.com/bitnami`

**Expected outcome:** You should be able to run `helm dependency list task-api-chart` and see PostgreSQL listed.

---

## Exercise 4.2: Add Condition for Optional PostgreSQL

Make PostgreSQL optional using the `condition: postgresql.enabled` field, then set the condition in values.yaml.

**Instructions:**

1. Update `Chart.yaml` to add `condition: postgresql.enabled` to the PostgreSQL dependency
2. Update `values.yaml` to add:
   ```yaml
   postgresql:
     enabled: true
   ```

**Expected outcome:** PostgreSQL should only render when `postgresql.enabled: true`.

---

## Exercise 4.3: Configure PostgreSQL via Parent Values

Set PostgreSQL username, password, and database through parent values.

**Instructions:**

1. Update `values.yaml`:
   ```yaml
   postgresql:
     enabled: true
     auth:
       username: agent
       password: secretpassword123
       database: myagentdb
     primary:
       persistence:
         enabled: true
         size: 10Gi
   ```

2. Verify the structure matches PostgreSQL subchart expectations

**Expected outcome:** When you run `helm template task-api-chart`, the PostgreSQL StatefulSet should have these auth values rendered.

---

## Exercise 4.4: Use import-values to Expose PostgreSQL Configuration

Simplify parent values using import-values mapping.

**Instructions:**

1. Update `Chart.yaml`:
   ```yaml
   dependencies:
     - name: postgresql
       version: "^13.0.0"
       repository: "https://charts.bitnami.com/bitnami"
       condition: postgresql.enabled
       import-values:
         - child: auth
           parent: postgresAuth
   ```

2. Refactor `values.yaml`:
   ```yaml
   postgresAuth:
     username: agent
     password: secretpassword123
     database: myagentdb

   postgresql:
     enabled: true
     primary:
       persistence:
         enabled: true
         size: 10Gi
   ```

**Expected outcome:** The import-values mapping automatically translates `postgresAuth` to PostgreSQL's `auth` namespace.

---

## Exercise 4.5: Run helm dependency update and Verify charts/ Directory

Download subcharts and verify they're stored in `charts/`.

**Instructions:**

1. Run: `helm dependency update task-api-chart`
2. List the contents: `ls -la task-api-chart/charts/`
3. Check PostgreSQL was downloaded: `ls task-api-chart/charts/postgresql/`

**Expected outcome:** The `charts/postgresql/` directory exists with a complete subchart.

---

## Exercise 4.6: Create Environment-Specific Values File

Create a `values-nodedb.yaml` that disables PostgreSQL but enables a Redis cache for development.

**Instructions:**

1. Create `task-api-chart/values-nodedb.yaml`:
   ```yaml
   postgresql:
     enabled: false

   redis:
     enabled: true
     auth:
       enabled: false  # No password in dev
   ```

2. Test rendering: `helm template task-api-chart -f task-api-chart/values-nodedb.yaml`

**Expected outcome:** PostgreSQL StatefulSet should NOT appear, but Redis Deployment should.

---

## Try With AI

**Part 1: Challenge Description**

You're building a production chart for an AI model serving service that needs:
- PostgreSQL for inference result caching
- Redis for request deduplication
- Optional RabbitMQ for async task queues

Set up a chart with these three dependencies, but make RabbitMQ optional (disabled by default).

**Part 2: Initial Setup**

Ask AI to generate the `Chart.yaml` with proper dependency declarations using the patterns you learned:

"I'm building a Helm chart for an AI model server. Create Chart.yaml with three dependencies: PostgreSQL (^13.0.0), Redis (^18.0.0), and RabbitMQ (^12.0.0) from Bitnami. Make RabbitMQ optional with condition: rabbitmq.enabled. Use semantic versioning constraints to allow patch updates."

**Part 3: Configuration Review**

Review the generated Chart.yaml and ask:
- Are the repository URLs correct?
- Are the version constraints appropriate for production?
- Is the condition field properly formatted?

If the output is missing the `condition:` field for RabbitMQ or uses exact versions, request corrections.

**Part 4: Values Configuration**

Ask AI to generate matching `values.yaml`:

"Generate values.yaml for this chart with: postgresql.enabled=true, redis.enabled=true, rabbitmq.enabled=false. Include basic auth configuration for PostgreSQL (username: agent, database: agentdb)."

**Part 5: Verification**

Compare your AI-generated files against the requirements:
- Does the `condition:` field on RabbitMQ prevent installation when not explicitly enabled?
- Can you override these conditions with `--set rabbitmq.enabled=true`?
- Does the values structure match the subchart expectations?

If all match → You've successfully composed a production multi-dependency chart.

