---
sidebar_position: 5
chapter: 51
lesson: 5
duration_minutes: 55
title: "Helm Hooks and Lifecycle Management"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Orchestrate zero-downtime deployments with pre/post lifecycle hooks"
cognitive_load:
  concepts_count: 10
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Identify all 9 Helm hook types and their execution timing"
    bloom_level: "Remember"
  - id: LO2
    description: "Create pre-upgrade hooks for database migrations"
    bloom_level: "Create"
  - id: LO3
    description: "Use hook weights to control execution order"
    bloom_level: "Apply"
  - id: LO4
    description: "Configure hook-delete-policy for resource cleanup"
    bloom_level: "Apply"
  - id: LO5
    description: "Debug hook execution with kubectl"
    bloom_level: "Apply"
---

# Helm Hooks and Lifecycle Management

When you deploy a Helm chart with `helm install`, Kubernetes doesn't just create resources and declare victory. Before the deployment runs, you might need to initialize a database schema. After the deployment succeeds, you might need to invalidate cached data. Before rolling back, you might need to run data migration reversals. If something fails catastrophically, you might want to trigger an alert or notification.

This is where Helm hooks come in. Hooks are Kubernetes resources (usually Jobs) that execute at specific points in the release lifecycle—before installation, after upgrade, before rollback—giving you fine-grained control over what happens when.

Think of hooks as lifecycle-aware automation. Instead of manually running `kubectl exec` to initialize databases, you declare what should happen using Helm hooks, and your chart orchestrates the entire sequence automatically.

---

## Concept 1: What Are Helm Hooks?

A Helm hook is a Kubernetes resource (Pod, Job, Deployment, etc.) that Helm executes at a specific point in the release lifecycle. Hooks are declared using annotations in your chart templates.

### The Hook Annotation

Every hook requires this annotation:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: db-init
  annotations:
    "helm.sh/hook": "pre-install"
spec:
  # Job specification
```

**Output:** When you run `helm install my-app ./my-chart`, Helm detects this annotation and executes the Job BEFORE creating other resources.

The key insight: Hooks are regular Kubernetes resources with special timing. You write them like any other resource, then add the annotation to control WHEN they execute.

---

## Concept 2: The 9 Hook Types

Helm defines 9 hook types that correspond to points in the release lifecycle. Understanding when each executes is critical to using hooks correctly.

### Hook Execution Timeline

Here's a visual representation of when each hook runs:

```
INSTALL Release:
  1. pre-install hooks
  2. Create resources (Deployment, Service, ConfigMap, etc.)
  3. post-install hooks
  4. Release marked as deployed

UPGRADE Release:
  1. pre-upgrade hooks
  2. Update resources
  3. post-upgrade hooks
  4. Release marked as upgraded

TEST Release:
  1. test hooks run independently
  2. Result reported (pass/fail)

DELETE Release:
  1. pre-delete hooks
  2. Remove resources
  3. post-delete hooks
  4. Release removed from Helm history

ROLLBACK Release:
  1. pre-rollback hooks
  2. Revert to previous version
  3. post-rollback hooks
  4. Previous version restored
```

### The 9 Hook Types Explained

**1. `pre-install`**

Runs BEFORE any resources are created during `helm install`. Use this for initialization work that must complete before the main deployment:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: db-schema-init
  annotations:
    "helm.sh/hook": "pre-install"
    "helm.sh/hook-weight": "-1"
spec:
  template:
    spec:
      containers:
      - name: init
        image: postgres:15
        command:
        - /bin/sh
        - -c
        - |
          psql -h ${DB_HOST} -U ${DB_USER} -d ${DB_NAME} <<EOF
          CREATE TABLE IF NOT EXISTS agents (
            id SERIAL PRIMARY KEY,
            name VARCHAR(255),
            created_at TIMESTAMP DEFAULT NOW()
          );
          EOF
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** When you run `helm install my-app ./chart`, this Job runs first, initializing the database schema. Only after it succeeds do other resources deploy.

---

**2. `post-install`**

Runs AFTER all resources are created during `helm install`. Use this for post-deployment configuration, registrations, or notifications:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: register-service
  annotations:
    "helm.sh/hook": "post-install"
spec:
  template:
    spec:
      containers:
      - name: register
        image: curl:latest
        command:
        - /bin/sh
        - -c
        - |
          sleep 10  # Wait for service to stabilize
          curl -X POST http://service-registry:8080/register \
            -H "Content-Type: application/json" \
            -d '{"name":"my-agent","port":8000}'
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** This Job runs after the Deployment and Service are ready, registering the service in a service registry. The 10-second delay ensures the service has time to start.

---

**3. `pre-upgrade`**

Runs BEFORE resources are updated during `helm upgrade`. Use this for migrations that must happen before the new code runs:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: db-migrate-v2
  annotations:
    "helm.sh/hook": "pre-upgrade"
    "helm.sh/hook-weight": "-5"
spec:
  template:
    spec:
      containers:
      - name: migrate
        image: postgres:15
        command:
        - /bin/sh
        - -c
        - |
          psql -h ${DB_HOST} -U ${DB_USER} -d ${DB_NAME} <<EOF
          ALTER TABLE agents ADD COLUMN status VARCHAR(50) DEFAULT 'active';
          CREATE INDEX idx_agents_status ON agents(status);
          EOF
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** When you run `helm upgrade my-app ./chart`, this migration Job runs first. The new code version expects the `status` column, which this hook creates. If migration fails, the upgrade is blocked (preventing incompatible code from running against old schema).

---

**4. `post-upgrade`**

Runs AFTER resources are updated during `helm upgrade`. Use this for cache invalidation, notifications, or health checks:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: invalidate-cache
  annotations:
    "helm.sh/hook": "post-upgrade"
spec:
  template:
    spec:
      containers:
      - name: cache-bust
        image: redis:7
        command:
        - redis-cli
        - -h
        - redis-cache
        - FLUSHALL
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** After the new Deployment is running, this hook connects to Redis and flushes all cached data. The new code version might have different caching logic, so stale cache entries would cause bugs. This ensures a clean cache.

---

**5. `pre-delete`**

Runs BEFORE resources are deleted during `helm uninstall`. Use this for data export, backup, or graceful shutdown sequences:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: export-data
  annotations:
    "helm.sh/hook": "pre-delete"
    "helm.sh/hook-weight": "1"
spec:
  template:
    spec:
      containers:
      - name: backup
        image: postgres:15
        command:
        - /bin/sh
        - -c
        - |
          pg_dump -h ${DB_HOST} -U ${DB_USER} -d ${DB_NAME} \
            > /backup/agents_backup_$(date +%s).sql
      restartPolicy: Never
  backoffLimit: 1
  volumes:
  - name: backup-vol
    persistentVolumeClaim:
      claimName: backup-pvc
```

**Output:** When you run `helm uninstall my-app`, this Job backs up the database to a PVC before any resources are removed.

---

**6. `post-delete`**

Runs AFTER resources are deleted during `helm uninstall`. Use this for cleanup tasks or notifications:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: cleanup-registry
  annotations:
    "helm.sh/hook": "post-delete"
spec:
  template:
    spec:
      containers:
      - name: cleanup
        image: curl:latest
        command:
        - /bin/sh
        - -c
        - |
          curl -X DELETE http://service-registry:8080/agents/my-app
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** After the entire release is deleted, this Job deregisters the service from the service registry.

---

**7. `pre-rollback`**

Runs BEFORE rolling back to a previous release. Use this for data state checks or backup creation:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: backup-before-rollback
  annotations:
    "helm.sh/hook": "pre-rollback"
spec:
  template:
    spec:
      containers:
      - name: backup
        image: postgres:15
        command:
        - /bin/sh
        - -c
        - |
          pg_dump -h ${DB_HOST} -U ${DB_USER} -d ${DB_NAME} \
            > /backup/rollback_backup_$(date +%s).sql
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** If a deployment fails and you need to `helm rollback`, this hook runs first, preserving the current database state as a backup.

---

**8. `post-rollback`**

Runs AFTER rolling back to a previous release. Use this for recovery validation or notifications:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: verify-rollback
  annotations:
    "helm.sh/hook": "post-rollback"
spec:
  template:
    spec:
      containers:
      - name: health-check
        image: curl:latest
        command:
        - /bin/sh
        - -c
        - |
          for i in {1..10}; do
            if curl -f http://my-app:8000/health; then
              echo "Rollback successful - service healthy"
              exit 0
            fi
            sleep 5
          done
          echo "Rollback failed - service not responding"
          exit 1
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** After rollback completes, this hook verifies the previous version is actually running and healthy before declaring the rollback successful.

---

**9. `test`**

A special hook that runs during `helm test` (not during install/upgrade/delete). Use this for post-deployment testing:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: smoke-test
  annotations:
    "helm.sh/hook": "test"
spec:
  template:
    spec:
      containers:
      - name: test
        image: curlimages/curl:latest
        command:
        - /bin/sh
        - -c
        - |
          # Test 1: Service responds
          curl -f http://my-app:8000/health || exit 1

          # Test 2: Database is accessible
          curl -f http://my-app:8000/api/agents || exit 1

          # Test 3: API returns expected format
          RESPONSE=$(curl -s http://my-app:8000/api/agents)
          echo "$RESPONSE" | grep -q '"agents"' || exit 1

          echo "All tests passed"
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** When you run `helm test my-app`, Helm executes this Job and reports the result. If the test succeeds (exit 0), Helm shows "TEST SUITE: PASSED". If it fails (non-zero exit), Helm shows "TEST SUITE: FAILED" and you know there's a problem.

---

## Concept 3: Hook Resources (Usually Jobs)

While Hooks can be any Kubernetes resource, the most common choice is Jobs because:

1. **Jobs have explicit completion semantics**: A Job succeeds when it completes (exit 0) and fails when it doesn't.
2. **Jobs are idempotent**: Running a Job multiple times should be safe.
3. **Jobs timeout naturally**: If a Job hangs, Kubernetes will eventually kill it.

You CAN use other resources as hooks:

```yaml
# Pod as a hook (less common - Pods don't have "success" semantics)
apiVersion: v1
kind: Pod
metadata:
  name: setup-pod
  annotations:
    "helm.sh/hook": "pre-install"

# Deployment as a hook (unusual - normally you use Deployment as main resource)
apiVersion: apps/v1
kind: Deployment
metadata:
  name: setup-deployment
  annotations:
    "helm.sh/hook": "pre-install"
```

But Jobs are strongly preferred. **Use Jobs for all lifecycle hooks.**

---

## Concept 4: Hook Weights (Execution Order)

When multiple hooks run at the same phase (e.g., two `pre-install` hooks), Helm needs to know which runs first. You control this with hook weights:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: schema-init
  annotations:
    "helm.sh/hook": "pre-install"
    "helm.sh/hook-weight": "-5"  # Runs first
spec:
  # ...
---
apiVersion: batch/v1
kind: Job
metadata:
  name: seed-data
  annotations:
    "helm.sh/hook": "pre-install"
    "helm.sh/hook-weight": "0"   # Runs second
spec:
  # ...
---
apiVersion: batch/v1
kind: Job
metadata:
  name: verify-init
  annotations:
    "helm.sh/hook": "pre-install"
    "helm.sh/hook-weight": "5"   # Runs last
spec:
  # ...
```

**Output:** When you run `helm install my-app ./chart`, Helm executes:
1. schema-init (weight -5)
2. seed-data (weight 0)
3. verify-init (weight 5)

**Rule**: Lower weight = earlier execution. Think of weights as "sort order":
- Negative weights run first
- Zero weights run in the middle
- Positive weights run last

Use gaps in weights (e.g., -5, 0, 5) to allow future hooks to insert themselves without renumbering.

---

## Concept 5: Hook Delete Policies (Resource Cleanup)

After a hook completes, you probably want to clean up its Pod/Job. You control this with the `hook-delete-policy` annotation:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: db-init
  annotations:
    "helm.sh/hook": "pre-install"
    "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"
spec:
  # ...
```

### The 3 Delete Policies

**1. `hook-succeeded`**

Delete the Job ONLY if it succeeds (exit code 0). If it fails, leave it for debugging:

```yaml
annotations:
  "helm.sh/hook": "pre-install"
  "helm.sh/hook-delete-policy": "hook-succeeded"
```

**Output:** If the Job succeeds, Helm deletes it after completion. If it fails, the Job remains in the cluster so you can inspect its logs with `kubectl logs job/db-init`.

---

**2. `hook-failed`**

Delete the Job ONLY if it fails. If it succeeds, keep it:

```yaml
annotations:
  "helm.sh/hook": "pre-install"
  "helm.sh/hook-delete-policy": "hook-failed"
```

**Output:** Useful if you want to keep evidence of successful migrations for audit purposes. If the Job fails, delete it to avoid cluster clutter.

---

**3. `before-hook-creation`**

Delete ANY previous hook with the same name BEFORE creating a new one. Useful for upgrades where the same hook runs multiple times:

```yaml
annotations:
  "helm.sh/hook": "pre-upgrade"
  "helm.sh/hook-delete-policy": "before-hook-creation"
```

**Output:** On first `helm upgrade`, this creates the Job. On second `helm upgrade`, Helm deletes the old Job, then creates a new one. This prevents accumulation of old Jobs in the cluster.

---

### Combining Policies

You can combine multiple policies:

```yaml
annotations:
  "helm.sh/hook": "post-upgrade"
  "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"
```

**Output:** This means:
- Delete the Job if it succeeds
- AND delete any previous Job with the same name before creating this one

This is the most common combination for upgrade hooks—clean up old jobs and remove successful ones to keep the cluster tidy.

---

## Concept 6: Hook Success/Failure Semantics

What happens when a hook fails? Understanding this is critical to using hooks reliably.

### When a Hook Fails

If a hook Job exits with non-zero code (indicating failure), Helm's behavior depends on the hook type:

**Pre-Install/Pre-Upgrade Hooks**: Release BLOCKED
```yaml
annotations:
  "helm.sh/hook": "pre-upgrade"
```

**Example:** Your migration hook fails. Helm immediately stops the upgrade and rolls back. The new code never runs. This prevents deploying incompatible code to incompatible database schema.

**Output:** `helm upgrade` returns error, release stays at previous version:
```
Error: release failed: pre-upgrade hook failed
Release "my-app" rolled back
```

---

**Post-Install/Post-Upgrade Hooks**: Release COMPLETES (but marked degraded)
```yaml
annotations:
  "helm.sh/hook": "post-upgrade"
```

**Example:** Your cache-bust hook fails. The upgrade already succeeded (your new code is running). The hook failure is reported but doesn't block the release.

**Output:** `helm upgrade` completes, but with warning:
```
Release "my-app" has been upgraded. Status: DEPLOYED
WARNING: post-upgrade hook failed
```

---

**Delete Hooks**: Release BLOCKING

If a `pre-delete` hook fails, the release is NOT uninstalled:

```yaml
annotations:
  "helm.sh/hook": "pre-delete"
```

**Output:** `helm uninstall` returns error, resources remain in cluster:
```
Error: release failed: pre-delete hook failed
Release "my-app" is still deployed
```

---

## Concept 7: Hook Annotations (The Complete Set)

When declaring a hook, you use these annotations. Here's the full set you need to know:

```yaml
metadata:
  annotations:
    "helm.sh/hook": "pre-install"              # Required: Which lifecycle phase
    "helm.sh/hook-weight": "0"                 # Optional: Execution order (default 0)
    "helm.sh/hook-delete-policy": "hook-succeeded"  # Optional: Cleanup strategy
```

**Common Combinations:**

```yaml
# Migration hook: Runs early, blocks on failure, cleaned up on success
annotations:
  "helm.sh/hook": "pre-upgrade"
  "helm.sh/hook-weight": "-10"
  "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"

# Post-deployment notification: Runs after, doesn't block, cleaned up always
annotations:
  "helm.sh/hook": "post-install"
  "helm.sh/hook-weight": "0"
  "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"

# Test: No delete policy (test hooks are handled specially)
annotations:
  "helm.sh/hook": "test"
```

---

## Concept 8: Testing Hooks with `helm test`

The `test` hook is special—it doesn't run during install/upgrade/delete. Instead, it runs when you explicitly call:

```bash
helm test my-app
```

**Example test hook:**

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: smoke-tests
  annotations:
    "helm.sh/hook": "test"
spec:
  template:
    spec:
      containers:
      - name: test
        image: myregistry/test-runner:v1
        command:
        - /bin/sh
        - -c
        - |
          set -e

          echo "Testing API endpoint..."
          curl -f http://my-app:8000/health || exit 1

          echo "Testing database connectivity..."
          curl -f http://my-app:8000/api/agents || exit 1

          echo "All tests passed!"
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** Running `helm test my-app` executes this Job and reports the result:

```
TEST SUITE: my-app
```

If the test succeeds (exit 0):
```
PASSED
```

If the test fails (non-zero exit):
```
FAILED
```

---

## Concept 9: Debugging Hooks with kubectl

When a hook fails, you need to see what went wrong. Hooks are just Kubernetes Jobs, so you debug them like any Job:

### List all hook Jobs in the release

```bash
kubectl get jobs -l release=my-app
```

**Output:**
```
NAME                    COMPLETIONS   DURATION   AGE
db-init                 1/1           10s        2m
register-service        1/1           5s         1m
```

### View hook Job logs

```bash
kubectl logs job/db-init
```

**Output:**
```
Initializing database schema...
CREATE TABLE IF NOT EXISTS agents...
Schema initialized successfully.
```

### Describe a failed Job to see why it failed

```bash
kubectl describe job db-init
```

**Output:**
```
Name:           db-init
Status:         Failed
Reason:         BackoffLimitExceeded
Start Time:     2025-12-23T10:00:00Z
Completion Time: 2025-12-23T10:00:30Z
Pods Statuses:  0 Running / 0 Succeeded / 1 Failed

Events:
  Type     Reason              Message
  ----     ------              -------
  Normal   SuccessfulCreate    Created pod: db-init-abc12
  Warning  BackoffLimitExceeded Job has reached the specified backoff limit
```

This tells you the Job ran, the Pod failed, and Helm gave up retrying. Next, check the Pod logs:

```bash
kubectl logs pod/db-init-abc12
```

**Output:**
```
Error: Unable to connect to database host 'postgres:5432'
Connection refused
```

Now you see the problem: The database isn't running or the hostname is wrong.

---

## Concept 10: Rollback and Re-execution

One critical thing to understand: When you upgrade a chart with hooks, and the pre-upgrade hook fails, what happens?

```bash
helm upgrade my-app ./chart
# Error: pre-upgrade hook failed
```

The release **rolls back** to the previous version. The Pods are reverted, and the cluster state is restored.

But hooks themselves are NOT rolled back. If your `pre-upgrade` migration hook ran and PARTIALLY modified the database (created a column, then failed), that partial state is NOT automatically reverted.

This is why idempotency is critical:

```sql
-- WRONG: Assumes column doesn't exist
ALTER TABLE agents ADD COLUMN status VARCHAR(50);

-- RIGHT: Only adds if it doesn't already exist
ALTER TABLE agents ADD COLUMN IF NOT EXISTS status VARCHAR(50);
```

By using idempotent migrations (IF NOT EXISTS, CREATE INDEX IF NOT EXISTS, etc.), you ensure that re-running a failed hook is safe.

---

## Exercises

### Exercise 1: Pre-Install Hook for Database Schema

Create a pre-install hook that initializes a PostgreSQL database schema:

**File: `my-agent-chart/templates/hooks/db-init.yaml`**

```yaml
{{ if .Values.postgres.enabled }}
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "my-agent-chart.fullname" . }}-db-init
  annotations:
    "helm.sh/hook": "pre-install"
    "helm.sh/hook-weight": "-10"
    "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"
spec:
  template:
    spec:
      containers:
      - name: init
        image: postgres:15
        env:
        - name: PGHOST
          value: {{ .Values.postgres.host }}
        - name: PGUSER
          value: {{ .Values.postgres.user }}
        - name: PGDATABASE
          value: {{ .Values.postgres.database }}
        - name: PGPASSWORD
          valueFrom:
            secretKeyRef:
              name: {{ .Values.postgres.secretName }}
              key: password
        command:
        - /bin/sh
        - -c
        - |
          psql <<EOF
          CREATE TABLE IF NOT EXISTS agents (
            id SERIAL PRIMARY KEY,
            name VARCHAR(255) NOT NULL,
            model_id VARCHAR(50),
            status VARCHAR(50) DEFAULT 'initialized',
            created_at TIMESTAMP DEFAULT NOW(),
            updated_at TIMESTAMP DEFAULT NOW()
          );
          CREATE INDEX IF NOT EXISTS idx_agents_status ON agents(status);
          CREATE INDEX IF NOT EXISTS idx_agents_created ON agents(created_at);
          EOF
      restartPolicy: Never
  backoffLimit: 1
{{ end }}
```

**Output:** Running `helm install my-app ./my-agent-chart` executes this Job first. The database schema is initialized idempotently (IF NOT EXISTS prevents errors if run multiple times). Only after successful schema creation do other resources deploy.

---

### Exercise 2: Pre-Upgrade Hook with Weight Ordering

Create two pre-upgrade hooks where the migration runs before validation:

**File: `my-agent-chart/templates/hooks/pre-upgrade.yaml`**

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "my-agent-chart.fullname" . }}-migrate
  annotations:
    "helm.sh/hook": "pre-upgrade"
    "helm.sh/hook-weight": "-5"
    "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"
spec:
  template:
    spec:
      containers:
      - name: migrate
        image: postgres:15
        env:
        - name: PGHOST
          value: {{ .Values.postgres.host }}
        - name: PGUSER
          value: {{ .Values.postgres.user }}
        - name: PGDATABASE
          value: {{ .Values.postgres.database }}
        - name: PGPASSWORD
          valueFrom:
            secretKeyRef:
              name: {{ .Values.postgres.secretName }}
              key: password
        command:
        - /bin/sh
        - -c
        - |
          psql <<EOF
          ALTER TABLE agents ADD COLUMN IF NOT EXISTS embedding_model VARCHAR(100);
          ALTER TABLE agents ADD COLUMN IF NOT EXISTS parameters JSONB DEFAULT '{}';
          EOF
      restartPolicy: Never
  backoffLimit: 1
---
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "my-agent-chart.fullname" . }}-validate-migration
  annotations:
    "helm.sh/hook": "pre-upgrade"
    "helm.sh/hook-weight": "0"
    "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"
spec:
  template:
    spec:
      containers:
      - name: validate
        image: postgres:15
        env:
        - name: PGHOST
          value: {{ .Values.postgres.host }}
        - name: PGUSER
          value: {{ .Values.postgres.user }}
        - name: PGDATABASE
          value: {{ .Values.postgres.database }}
        - name: PGPASSWORD
          valueFrom:
            secretKeyRef:
              name: {{ .Values.postgres.secretName }}
              key: password
        command:
        - /bin/sh
        - -c
        - |
          # Check that migration columns exist
          psql -c "SELECT embedding_model, parameters FROM agents LIMIT 1;" || exit 1
          echo "Migration validated successfully"
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** When you run `helm upgrade my-app ./chart`:
1. Migrate hook runs (weight -5)
2. Validate hook runs (weight 0)

If validation fails, the upgrade is blocked and rolled back. The deployment doesn't update.

---

### Exercise 3: Post-Upgrade Hook for Cache Invalidation

Create a hook that clears Redis cache after a successful upgrade:

**File: `my-agent-chart/templates/hooks/post-upgrade.yaml`**

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "my-agent-chart.fullname" . }}-cache-bust
  annotations:
    "helm.sh/hook": "post-upgrade"
    "helm.sh/hook-weight": "0"
    "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"
spec:
  template:
    spec:
      containers:
      - name: cache-bust
        image: redis:7
        command:
        - /bin/sh
        - -c
        - |
          redis-cli -h {{ .Values.redis.host }} -p {{ .Values.redis.port }} FLUSHALL
          echo "Cache cleared"
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** After `helm upgrade my-app ./chart` completes, this Job runs. It connects to Redis and clears all cached data. Clients that request data will miss cache, refill from source, and have fresh data for the new deployment version.

---

### Exercise 4: Intentional Hook Failure

Create a pre-upgrade hook that intentionally fails to see how Helm handles it:

**File: `my-agent-chart/templates/hooks/debug-failure.yaml`**

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "my-agent-chart.fullname" . }}-debug-fail
  annotations:
    "helm.sh/hook": "pre-upgrade"
    "helm.sh/hook-weight": "-20"
    "helm.sh/hook-delete-policy": "before-hook-creation"
spec:
  template:
    spec:
      containers:
      - name: fail
        image: busybox:latest
        command:
        - /bin/sh
        - -c
        - |
          echo "This hook intentionally fails"
          exit 1
      restartPolicy: Never
  backoffLimit: 1
```

**Steps:**
1. Run `helm upgrade my-app ./chart`
2. Watch the error: `Error: release failed: pre-upgrade hook failed`
3. Check hook status: `kubectl get jobs | grep debug-fail`
4. View logs: `kubectl logs job/my-app-debug-fail`
5. Check release status: `helm status my-app`

**Output:** The upgrade is blocked. The release remains at its previous version. The Job shows 0 completions because it failed. When you remove this hook and upgrade again, the release proceeds normally.

---

### Exercise 5: Hook Delete Policy (hook-succeeded)

Deploy a hook, let it succeed, and verify it's cleaned up:

**File: `my-agent-chart/templates/hooks/cleanup-test.yaml`**

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "my-agent-chart.fullname" . }}-cleanup-test
  annotations:
    "helm.sh/hook": "post-install"
    "helm.sh/hook-delete-policy": "hook-succeeded"
spec:
  template:
    spec:
      containers:
      - name: work
        image: busybox:latest
        command:
        - /bin/sh
        - -c
        - |
          echo "Working..."
          sleep 5
          echo "Done"
      restartPolicy: Never
  backoffLimit: 1
```

**Steps:**
1. Run `helm install my-app ./chart`
2. Wait 10 seconds
3. Check Jobs: `kubectl get jobs`
4. Notice the Job is GONE (deleted because it succeeded)

**Output:** The Job appears briefly, runs to completion, and Helm automatically deletes it. Your cluster stays clean.

---

### Exercise 6: Hook Weights with Three Jobs

Create three hooks that run in specific order using different weights:

**File: `my-agent-chart/templates/hooks/sequence.yaml`**

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "my-agent-chart.fullname" . }}-step-1-backup
  annotations:
    "helm.sh/hook": "pre-upgrade"
    "helm.sh/hook-weight": "-10"
    "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"
spec:
  template:
    spec:
      containers:
      - name: backup
        image: busybox:latest
        command:
        - sh
        - -c
        - echo "Step 1: Backing up data" && sleep 2
      restartPolicy: Never
  backoffLimit: 1
---
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "my-agent-chart.fullname" . }}-step-2-migrate
  annotations:
    "helm.sh/hook": "pre-upgrade"
    "helm.sh/hook-weight": "0"
    "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"
spec:
  template:
    spec:
      containers:
      - name: migrate
        image: busybox:latest
        command:
        - sh
        - -c
        - echo "Step 2: Running migration" && sleep 2
      restartPolicy: Never
  backoffLimit: 1
---
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "my-agent-chart.fullname" . }}-step-3-verify
  annotations:
    "helm.sh/hook": "pre-upgrade"
    "helm.sh/hook-weight": "10"
    "helm.sh/hook-delete-policy": "hook-succeeded,before-hook-creation"
spec:
  template:
    spec:
      containers:
      - name: verify
        image: busybox:latest
        command:
        - sh
        - -c
        - echo "Step 3: Verifying migration" && sleep 2
      restartPolicy: Never
  backoffLimit: 1
```

**Output:** When you run `helm upgrade`, the hooks execute in order:
```
Step 1: Backing up data
Step 2: Running migration
Step 3: Verifying migration
```

Use `kubectl get jobs` to see all three Jobs created in sequence.

---

### Exercise 7: Checking Hook Execution with kubectl

Examine the hooks deployed in your chart:

**Steps:**
1. Deploy the chart: `helm install my-app ./my-agent-chart`
2. List all Jobs: `kubectl get jobs`
3. Get detailed Job info: `kubectl get jobs -o wide`
4. View a Job's completion: `kubectl describe job my-app-db-init`
5. Check logs: `kubectl logs job/my-app-db-init`
6. List hook Events: `kubectl get events --sort-by=.metadata.creationTimestamp`

**Output:** You'll see:
- Jobs created for each hook
- Completion status (1/1 means succeeded, 0/1 means failed)
- Logs showing what each hook did
- Events showing the timeline of hook execution

---

## Try With AI

### Part 1: Exploring Hook Patterns

Ask AI to explain a specific hook use case:

**Prompt:**
"I'm deploying an AI agent to Kubernetes using Helm. Before each upgrade, I need to:
1. Back up the current database
2. Run schema migrations
3. Validate the migration succeeded

Show me a complete pre-upgrade hook sequence using weights and delete policies."

**What to evaluate:**
- Does the response use three separate Jobs with weights (-10, 0, 10)?
- Does it use idempotent SQL (IF NOT EXISTS, IF NOT PRESENT)?
- Does it set appropriate delete-policy (hook-succeeded,before-hook-creation)?
- Are the Job definitions complete and runnable?

### Part 2: Refining for Your Use Case

Based on AI's response, customize it:

**Prompt:**
"I'm using PostgreSQL version 13 and the database name is 'agent_db'. The credentials are in a Secret called 'postgres-creds'. Update the migration hook to match my environment."

**What to evaluate:**
- Does it reference the correct Secret name?
- Are environment variables set from Secret values?
- Is the SQL appropriate for PostgreSQL 13?

### Part 3: Debugging Hook Failures

Ask AI to help debug a failing hook:

**Prompt:**
"My pre-upgrade hook is failing with 'connection refused'. The hook is trying to connect to 'postgres' but the actual service hostname is 'postgres-svc.data-layer'. Show me:
1. What the problem is
2. How to fix the hook
3. How to verify it's fixed"

**What to evaluate:**
- Does AI identify the hostname mismatch?
- Does it suggest updating Values.yaml for environment-specific hostnames?
- Does it recommend debugging commands (kubectl logs, kubectl describe)?

### Part 4: Validating Hook Safety

Ask AI to review a hook you've written:

**Prompt:**
"Review this hook for production safety:
[paste your hook YAML]

Check for:
- Idempotency issues
- Missing error handling
- Delete policy appropriateness
- Timeout concerns"

**What to evaluate:**
- Does AI catch non-idempotent operations (operations that fail if run twice)?
- Does it suggest adding set -e (fail on first error)?
- Does it recommend reasonable timeouts?

### Part 5: Integration Testing

Ask AI for a test strategy:

**Prompt:**
"How do I test that my pre-upgrade hook works correctly without actually running helm upgrade in production? Show me a test plan using kind or Minikube."

**What to evaluate:**
- Does it suggest spinning up a test cluster?
- Does it recommend testing both success and failure cases?
- Does it mention checking hook order with weights?

