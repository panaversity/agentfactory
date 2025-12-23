---
sidebar_position: 9
title: "Sync Waves and Resource Hooks"
description: "Control deployment ordering with waves and lifecycle hooks"
keywords: [argocd, sync waves, hooks, presync, postsync, migration, deployment order]
---

# Sync Waves and Resource Hooks

You've learned how to manage sync policies in Lesson 8. Now you face a critical challenge: **deployment order matters**. Your PostgreSQL database migration must run before your FastAPI application deploys. Your health checks must pass before marking the deployment as healthy. Your rollback must run cleanup jobs when deployment fails.

This is where sync waves and resource hooks solve real problems. Waves order resource creation sequentially. Hooks define what happens at critical sync points: before, during, and after deployment.

## The Order Problem

Consider this scenario. You're deploying version 2 of your agent with a database schema change:

```
1. Your CI pipeline builds the new container
2. ArgoCD syncs the manifests
3. ArgoCD creates: ConfigMap, Secret, Deployment, Service all at once
4. The Deployment starts immediately without migrations
5. The container crashes (no database columns it expects)
```

Without hooks, your application fails because migrations never ran. With hooks:

```
1. CI pipeline builds the new container
2. ArgoCD syncs the manifests
3. ArgoCD creates Secret, ConfigMap (wave 0)
4. ArgoCD runs migration Job as PreSync hook (before sync)
5. ArgoCD creates Deployment, Service (wave 1)
6. ArgoCD verifies readiness (hook with HookSucceeded deletion policy)
```

The difference is simple: **waves group resources by creation order. Hooks define executable steps around those waves.**

## Sync Waves Explained

A **sync wave** is an annotation that tells ArgoCD: "Create this resource in this order, then move to the next wave."

### Wave Numbers and Ordering

Waves execute in ascending numerical order:

```
Wave -1  → Created first (infrastructure preparation)
Wave 0   → Created second (main infrastructure: config, secrets, volumes)
Wave 1   → Created third (applications: deployments, services)
Wave 2   → Created fourth (post-deployment: monitoring, cleanup)
```

**Key principle**: ArgoCD waits for each wave to fully sync and stabilize before moving to the next wave. A resource in wave 1 cannot start until all wave 0 resources are healthy.

### Annotating Resources for Waves

The annotation syntax is simple:

```yaml
metadata:
  annotations:
    argocd.argoproj.io/sync-wave: "0"
```

**Example: ConfigMap and Secret in Wave 0**

```yaml
# Config must exist before app starts
apiVersion: v1
kind: ConfigMap
metadata:
  name: agent-config
  annotations:
    argocd.argoproj.io/sync-wave: "0"
data:
  agent_model: gpt-4
  max_tokens: "4096"
---
# Secrets in same wave as config
apiVersion: v1
kind: Secret
metadata:
  name: agent-secrets
  annotations:
    argocd.argoproj.io/sync-wave: "0"
type: Opaque
data:
  api_key: YWJjMTIz  # base64 encoded
```

**Output:**
```
Wave 0 resources created and stabilized
- ConfigMap agent-config: Ready
- Secret agent-secrets: Ready
```

**Example: Deployment in Wave 1**

```yaml
# App deployment waits for wave 0
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-api
  annotations:
    argocd.argoproj.io/sync-wave: "1"
spec:
  replicas: 1
  selector:
    matchLabels:
      app: agent-api
  template:
    metadata:
      labels:
        app: agent-api
    spec:
      containers:
      - name: api
        image: myregistry.azurecr.io/agent-api:v2.1.0
        ports:
        - containerPort: 8000
        envFrom:
        - configMapRef:
            name: agent-config
        - secretRef:
            name: agent-secrets
```

**Output:**
```
Wave 0 complete. Proceeding to Wave 1.
- Deployment agent-api: Creating replicas (waiting for wave 0)
- Service agent-api: Created
Pod: agent-api-xxxxx ready
```

## Resource Hooks: PreSync, PostSync, SyncFail

A **resource hook** is a Kubernetes Job or Pod that runs at specific points in the sync lifecycle. Unlike regular resources, hooks don't stay running—they execute and exit.

### Hook Types and When to Use Them

| Hook Type | When | Example |
|-----------|------|---------|
| **PreSync** | Before any resources sync | Database migrations, data preparation |
| **Sync** | During normal resource creation | Custom initialization (rarely used) |
| **PostSync** | After sync completes successfully | Notifications, smoke tests, warmup |
| **SyncFail** | When sync fails | Alerting, rollback notifications |
| **PostDelete** | When ArgoCD deletes resources | Cleanup, deregistration |

### Hook Deletion Policy

After a hook finishes, what happens to the Job/Pod?

```yaml
argocd.argoproj.io/hook-deletion-policy: [HookSucceeded|HookFailed|BeforeHookCreation]
```

| Policy | Deletes When | Use Case |
|--------|--------------|----------|
| **HookSucceeded** | Hook succeeds | One-time jobs (migrations) |
| **HookFailed** | Hook fails | Keep failed jobs for debugging |
| **BeforeHookCreation** | Before hook runs (cleanup old ones) | Prevent multiple job instances |

## PreSync Hooks: Running Database Migrations

A **PreSync hook** runs before any resources sync. This is where you run database migrations.

### Complete Database Migration Example

```yaml
# Service account for migration job
apiVersion: v1
kind: ServiceAccount
metadata:
  name: db-migrate
  namespace: agents
---
# ClusterRole for migration job (if needed)
apiVersion: rbac.authorization.k8s.io/v1
kind: Role
metadata:
  name: db-migrate
  namespace: agents
rules:
- apiGroups: [""]
  resources: ["secrets"]
  verbs: ["get"]
---
# Bind role to service account
apiVersion: rbac.authorization.k8s.io/v1
kind: RoleBinding
metadata:
  name: db-migrate
  namespace: agents
subjects:
- kind: ServiceAccount
  name: db-migrate
  namespace: agents
roleRef:
  kind: Role
  name: db-migrate
  apiGroup: rbac.authorization.k8s.io
---
# PreSync hook: Database migration job
apiVersion: batch/v1
kind: Job
metadata:
  name: db-migration
  namespace: agents
  annotations:
    # This is a hook, not a regular resource
    argocd.argoproj.io/hook: PreSync
    # Run before sync, then delete on success
    argocd.argoproj.io/hook-deletion-policy: HookSucceeded
    # Run in wave -1 to execute before everything else
    argocd.argoproj.io/sync-wave: "-1"
spec:
  backoffLimit: 1
  template:
    spec:
      serviceAccountName: db-migrate
      restartPolicy: Never
      containers:
      - name: migrate
        image: myregistry.azurecr.io/agent-api:v2.1.0
        command:
        - sh
        - -c
        - |
          # Wait for database to be ready (with timeout)
          timeout 60 bash -c 'until python -c "import psycopg2; psycopg2.connect(host='$DB_HOST',user='$DB_USER',password='$DB_PASSWORD',dbname='$DB_NAME')" 2>/dev/null; do sleep 2; done'

          # Run migrations with alembic
          cd /app
          pip install -q alembic psycopg2-binary
          alembic upgrade head

          # Verify migration succeeded
          python -c "
          import psycopg2
          conn = psycopg2.connect(host='$DB_HOST',user='$DB_USER',password='$DB_PASSWORD',dbname='$DB_NAME')
          cursor = conn.cursor()
          cursor.execute(\"SELECT version FROM alembic_version;\")
          version = cursor.fetchone()
          print(f'✓ Migration completed. Version: {version[0]}')
          cursor.close()
          conn.close()
          "
        env:
        - name: DB_HOST
          valueFrom:
            secretKeyRef:
              name: db-secrets
              key: host
        - name: DB_USER
          valueFrom:
            secretKeyRef:
              name: db-secrets
              key: username
        - name: DB_PASSWORD
          valueFrom:
            secretKeyRef:
              name: db-secrets
              key: password
        - name: DB_NAME
          valueFrom:
            secretKeyRef:
              name: db-secrets
              key: database
        resources:
          requests:
            memory: "256Mi"
            cpu: "100m"
          limits:
            memory: "512Mi"
            cpu: "500m"
```

**Output (on successful migration):**
```
PreSync: Running database migration
Job: db-migration created
- Waiting for migration to complete...
- Migration command: alembic upgrade head
- ✓ Migration completed. Version: 5

Job deleted (HookSucceeded policy)
PreSync complete. Proceeding to sync.
```

**Output (on migration failure):**
```
PreSync: Running database migration
Job: db-migration created
- Waiting for migration to complete...
- ERROR: Cannot find column 'agent_id' in agents table
Job NOT deleted (job failed, waiting for manual investigation)
Sync FAILED - PreSync hook failed
```

## PostSync Hooks: Notifications and Smoke Tests

A **PostSync hook** runs after sync completes successfully. Use this for:

- Notifications (Slack, email, webhook)
- Smoke tests (basic health checks)
- Warming up caches
- Database reseeding

### Example: Slack Notification on Deployment

```yaml
# Secret containing webhook URL
apiVersion: v1
kind: Secret
metadata:
  name: slack-webhook
  namespace: agents
type: Opaque
stringData:
  webhook-url: https://hooks.slack.com/services/YOUR/WEBHOOK/URL
---
# PostSync hook: Send Slack notification
apiVersion: batch/v1
kind: Job
metadata:
  name: notify-deployment
  namespace: agents
  annotations:
    argocd.argoproj.io/hook: PostSync
    argocd.argoproj.io/hook-deletion-policy: HookSucceeded
    argocd.argoproj.io/sync-wave: "2"
spec:
  backoffLimit: 1
  template:
    spec:
      restartPolicy: Never
      containers:
      - name: notify
        image: curlimages/curl:latest
        command:
        - sh
        - -c
        - |
          WEBHOOK_URL=$(cat /var/run/secrets/slack/webhook-url)
          DEPLOYMENT_IMAGE=$(kubectl get deployment agent-api -n agents -o jsonpath='{.spec.template.spec.containers[0].image}')
          REPLICA_READY=$(kubectl get deployment agent-api -n agents -o jsonpath='{.status.readyReplicas}')

          curl -X POST "$WEBHOOK_URL" \
            -H 'Content-Type: application/json' \
            -d "{
              \"text\": \"Agent API Deployment Complete\",
              \"blocks\": [
                {
                  \"type\": \"section\",
                  \"text\": {
                    \"type\": \"mrkdwn\",
                    \"text\": \"*Agent API Deployed Successfully*\n*Image:* \`$DEPLOYMENT_IMAGE\`\n*Ready Replicas:* $REPLICA_READY\"
                  }
                }
              ]
            }"
        volumeMounts:
        - name: slack-secret
          mountPath: /var/run/secrets/slack
          readOnly: true
        resources:
          requests:
            memory: "64Mi"
            cpu: "10m"
          limits:
            memory: "128Mi"
            cpu: "100m"
      volumes:
      - name: slack-secret
        secret:
          secretName: slack-webhook
          items:
          - key: webhook-url
            path: webhook-url
```

**Output:**
```
PostSync: Running notifications
Job: notify-deployment created
- Sending Slack notification...
- curl: Slack API responded with 200 OK
✓ Deployment notification sent

Job deleted (HookSucceeded policy)
Sync complete.
```

## SyncFail Hooks: Alerting on Failure

A **SyncFail hook** runs when the sync process fails. Use this for alerting and rollback notifications.

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: sync-failed-alert
  namespace: agents
  annotations:
    argocd.argoproj.io/hook: SyncFail
    argocd.argoproj.io/hook-deletion-policy: HookFailed
    argocd.argoproj.io/sync-wave: "3"
spec:
  backoffLimit: 0
  template:
    spec:
      restartPolicy: Never
      containers:
      - name: alert
        image: curlimages/curl:latest
        command:
        - sh
        - -c
        - |
          WEBHOOK_URL=$(cat /var/run/secrets/slack/webhook-url)

          curl -X POST "$WEBHOOK_URL" \
            -H 'Content-Type: application/json' \
            -d "{
              \"text\": \"ALERT: Agent Deployment Failed\",
              \"blocks\": [
                {
                  \"type\": \"section\",
                  \"text\": {
                    \"type\": \"mrkdwn\",
                    \"text\": \":warning: *Deployment Failed*\nCheck ArgoCD UI for details. Cluster state may be inconsistent.\"
                  }
                }
              ]
            }"
        volumeMounts:
        - name: slack-secret
          mountPath: /var/run/secrets/slack
          readOnly: true
      volumes:
      - name: slack-secret
        secret:
          secretName: slack-webhook
```

**Output (when sync fails):**
```
Sync FAILED: Invalid image reference
SyncFail: Running alert hook
Job: sync-failed-alert created
- Sending failure alert to Slack...
- curl: Slack API responded with 200 OK
✓ Failure alert sent

Cluster is in UNKNOWN health state (check manually)
```

## Hook Deletion Policies in Detail

### HookSucceeded (Default for One-Time Jobs)

```yaml
argocd.argoproj.io/hook-deletion-policy: HookSucceeded
```

Deletes the Job immediately after it succeeds. Use this for:
- Database migrations
- Data initialization
- One-time setup

**Example**: Migration job should not persist after running.

### HookFailed (Keep Failed Jobs for Debugging)

```yaml
argocd.argoproj.io/hook-deletion-policy: HookFailed
```

Deletes the Job only if it fails. If it succeeds, the Job persists. Use this when:
- You want to debug what happened after success
- You want audit logs of hook execution

**Example**: Keep successful notification jobs for audit trail.

### BeforeHookCreation (Cleanup Old Instances)

```yaml
argocd.argoproj.io/hook-deletion-policy: BeforeHookCreation
```

Deletes the previous hook instance before creating a new one. Use this when:
- Hook might run multiple times
- You want only the latest instance around

**Example**: Notification hooks might run on each sync; delete the old one before creating a new one.

## Complete Example: Multi-Wave Deployment with Hooks

Here's a realistic example combining waves and hooks:

```yaml
# Wave -1: Prepare (migrations)
---
apiVersion: batch/v1
kind: Job
metadata:
  name: db-migration
  namespace: agents
  annotations:
    argocd.argoproj.io/hook: PreSync
    argocd.argoproj.io/hook-deletion-policy: HookSucceeded
    argocd.argoproj.io/sync-wave: "-1"
spec:
  backoffLimit: 1
  template:
    spec:
      restartPolicy: Never
      containers:
      - name: migrate
        image: myregistry.azurecr.io/agent-api:v2.1.0
        command: ["alembic", "upgrade", "head"]
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: db-secrets
              key: url
        resources:
          requests:
            memory: "256Mi"
            cpu: "100m"
          limits:
            memory: "512Mi"
            cpu: "500m"

# Wave 0: Infrastructure (config, secrets, volumes)
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: agent-config
  annotations:
    argocd.argoproj.io/sync-wave: "0"
data:
  log_level: INFO
  max_workers: "4"
---
apiVersion: v1
kind: Secret
metadata:
  name: agent-secrets
  annotations:
    argocd.argoproj.io/sync-wave: "0"
type: Opaque
data:
  api_key: YWJjMTIz

# Wave 1: Application
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-api
  annotations:
    argocd.argoproj.io/sync-wave: "1"
spec:
  replicas: 2
  selector:
    matchLabels:
      app: agent-api
  template:
    metadata:
      labels:
        app: agent-api
    spec:
      containers:
      - name: api
        image: myregistry.azurecr.io/agent-api:v2.1.0
        ports:
        - containerPort: 8000
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 10
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 5
---
apiVersion: v1
kind: Service
metadata:
  name: agent-api
  annotations:
    argocd.argoproj.io/sync-wave: "1"
spec:
  type: ClusterIP
  ports:
  - port: 8000
    targetPort: 8000
  selector:
    app: agent-api

# Wave 2: Post-deployment (checks, notifications)
---
apiVersion: batch/v1
kind: Job
metadata:
  name: smoke-tests
  namespace: agents
  annotations:
    argocd.argoproj.io/hook: PostSync
    argocd.argoproj.io/hook-deletion-policy: HookSucceeded
    argocd.argoproj.io/sync-wave: "2"
spec:
  backoffLimit: 1
  template:
    spec:
      restartPolicy: Never
      containers:
      - name: test
        image: curlimages/curl:latest
        command:
        - sh
        - -c
        - |
          # Wait for service to be ready
          sleep 5

          # Test health endpoint
          curl -f http://agent-api:8000/health || exit 1

          # Test readiness endpoint
          curl -f http://agent-api:8000/ready || exit 1

          echo "✓ All smoke tests passed"
        resources:
          requests:
            memory: "64Mi"
            cpu: "10m"
          limits:
            memory: "128Mi"
            cpu: "100m"
```

**Output (complete sync with waves):**
```
ArgoCD Sync Started

Wave -1: PreSync Preparation
  Job: db-migration running
  - Database migration in progress...
  - ✓ Migration completed
  Job: db-migration deleted (HookSucceeded)
  Wave -1 complete ✓

Wave 0: Infrastructure
  ConfigMap: agent-config created
  Secret: agent-secrets created
  Waiting for ConfigMap/Secret to stabilize...
  Wave 0 stable ✓

Wave 1: Application
  Deployment: agent-api created (requesting 2 replicas)
  Service: agent-api created
  Waiting for Deployment to stabilize...
  Pod 1: agent-api-5f8c9b... Ready
  Pod 2: agent-api-5f8c9b... Ready
  Deployment stable (2/2 ready) ✓

Wave 2: Post-Sync Verification
  Job: smoke-tests running
  - Testing http://agent-api:8000/health
  - Testing http://agent-api:8000/ready
  - ✓ All smoke tests passed
  Job: smoke-tests deleted (HookSucceeded)
  Wave 2 complete ✓

Sync SUCCESSFUL
Application Status: Healthy
```

## Troubleshooting Failed Hooks

When hooks fail, ArgoCD stops the sync. Here's how to diagnose:

### Check Hook Job Logs

```bash
# List jobs in the namespace
kubectl get jobs -n agents

# View logs from failed migration
kubectl logs -n agents job/db-migration

# Describe the job to see event messages
kubectl describe job -n agents db-migration
```

**Output:**
```
NAME            COMPLETIONS   DURATION   AGE
db-migration    0/1           5s         5s

---

LOGS:
  ERROR: Cannot connect to database at 192.168.1.10:5432
  Connection refused - check DATABASE_URL environment variable

---

EVENTS:
  Type    Reason    Age   From      Message
  ----    ------    ----  ----      -------
  Normal  Created   5s    batch     Created pod: db-migration-xxxxx
  Normal  Started   3s    batch     Started pod: db-migration-xxxxx
  Warning Failed    1s    batch     Pod failed (exit code 1)
```

### Common Issues

| Issue | Cause | Fix |
|-------|-------|-----|
| Hook never runs | Wave number too high | Check `sync-wave` annotation; PreSync should be -1 |
| Hook runs but fails | Missing environment variables | Verify `valueFrom` references exist (secrets, configmaps) |
| Job persists when it shouldn't | Wrong deletion policy | Change to `HookSucceeded` for one-time jobs |
| Multiple jobs running | `BeforeHookCreation` not set | Add deletion policy to cleanup old instances |
| Sync proceeds despite hook failure | Hook is not a PreSync | Change `argocd.argoproj.io/hook: PreSync` for critical jobs |

## Key Concepts Review

**Sync Waves**:
- Annotate resources with `argocd.argoproj.io/sync-wave: "N"`
- ArgoCD creates waves in ascending order (-1, 0, 1, 2, ...)
- Waits for each wave to stabilize before proceeding

**Resource Hooks**:
- Execute at sync lifecycle points: PreSync, Sync, PostSync, SyncFail, PostDelete
- Not regular resources—they run and exit
- Use `argocd.argoproj.io/hook: [HookType]` annotation

**Hook Deletion Policies**:
- `HookSucceeded`: Delete on success (one-time jobs like migrations)
- `HookFailed`: Delete on failure (keep successful runs for audit)
- `BeforeHookCreation`: Delete previous instance before creating new one

**Wave -1 PreSync Hooks**:
- Run before any resources sync
- Perfect for database migrations
- Ensure prerequisite data exists before application starts

**Wave 2+ PostSync Hooks**:
- Run after all resources are healthy
- Use for notifications, smoke tests, warmup
- Verify deployment success before marking complete

## Try With AI

Ask Claude: "I need my PostgreSQL database migration to run before my FastAPI agent deploys. Create a PreSync hook Job that runs alembic upgrade head. The database connection details are in a secret named 'db-secrets' with keys 'host', 'username', 'password', and 'database'."

Review Claude's response for these key aspects:

- Does the Job use the `argocd.argoproj.io/hook: PreSync` annotation?
- Does it set `argocd.argoproj.io/sync-wave: "-1"` to run before everything?
- Does it use `argocd.argoproj.io/hook-deletion-policy: HookSucceeded`?
- Does it reference the secret using `valueFrom`?
- Does it include error handling (timeout for database readiness)?
- Does it have proper resource requests and limits?

Iterate with Claude: "Now add a PostSync hook that sends a Slack notification when the deployment succeeds. The hook should include the container image tag that was deployed. Reference the Slack webhook URL from a secret named 'slack-webhook' with key 'url'."

Verify the PostSync hook:

- Uses `argocd.argoproj.io/hook: PostSync` annotation?
- Has `argocd.argoproj.io/sync-wave: "2"` to run after the app starts?
- Retrieves the deployed image using `kubectl get deployment`?
- Formats a proper Slack message payload?
- Handles the webhook URL securely from the secret?

One more refinement: "If the migration fails, I want the hook job to stay in the cluster so I can debug why. Update the deletion policy appropriately."

Check if Claude:
- Changed deletion policy to NOT delete failed jobs?
- OR left migration as `HookSucceeded` but added a separate debug/audit hook?
- Explained the tradeoff between cleanup and debugging?
