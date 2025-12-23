---
sidebar_position: 12
title: "Health Status and Notifications"
description: "Monitor application health and configure alerts for deployment events"
keywords: [argocd, health, notifications, alerts, slack, webhook, monitoring]
---

# Health Status and Notifications

You've deployed applications with ArgoCD and automated their sync strategies. But deployments fail silently. A Pod crashes, the sync stalls, and you don't know until a customer reports the issue. This lesson teaches you to monitor health status and get alerts when things go wrong.

ArgoCD continuously watches your running applications. It assesses whether each resource is healthy, degraded, or unknown. When status changes, it can notify Slack, send webhooks, or trigger other integrations. By the end of this lesson, you'll understand how ArgoCD evaluates health and how to configure notifications for critical events.

## Understanding Health Status

Every resource in ArgoCD has a health status. ArgoCD evaluates health by examining resource state, checking conditions, and running custom health checks.

### Built-in Health Checks by Resource Type

ArgoCD includes out-of-the-box health logic for standard Kubernetes resources. Let's see how health is determined for each type:

**Deployment Health**: ArgoCD checks if desired replicas match ready replicas

```yaml
# Deploy a simple web application
apiVersion: apps/v1
kind: Deployment
metadata:
  name: web-app
spec:
  replicas: 3
  selector:
    matchLabels:
      app: web
  template:
    metadata:
      labels:
        app: web
    spec:
      containers:
      - name: nginx
        image: nginx:1.25
        ports:
        - containerPort: 80
```

**Output**: Deployment health status
```
$ argocd app get my-app
Resources:
NAME           KIND         VERSION  HEALTH       STATUS
web-app        Deployment   apps/v1  Healthy      Synced
```

When 3 of 3 replicas are ready, status is Healthy. If only 1 of 3 is ready (others starting or crashed), it's Progressing. If 0 of 3 are ready after 10 minutes, it's Degraded.

**Pod Health**: ArgoCD checks if all containers are running and conditions are true

```bash
# View a Pod's conditions
kubectl describe pod web-app-abc123 -n agent-app
```

**Output**: Pod with all conditions healthy
```
Name:         web-app-abc123
Status:       Running

Conditions:
  Type              Status
  Initialized       True
  Ready             True
  ContainersReady   True
  PodScheduled      True

Containers:
  nginx:
    Container ID:  docker://abc123...
    State:         Running
      Started:     2025-12-23T14:30:00Z
```

**StatefulSet Health**: ArgoCD verifies all replicas are ready and updated

```yaml
# Deploy a PostgreSQL database with StatefulSet
apiVersion: apps/v1
kind: StatefulSet
metadata:
  name: postgres
spec:
  replicas: 3
  selector:
    matchLabels:
      app: postgres
  template:
    metadata:
      labels:
        app: postgres
    spec:
      containers:
      - name: postgres
        image: postgres:15
        env:
        - name: POSTGRES_PASSWORD
          value: "secret"
```

**Output**: StatefulSet health when rolling out
```
$ argocd app get db-app
Resources:
NAME           KIND         VERSION  HEALTH         STATUS
postgres       StatefulSet  apps/v1  Progressing    Synced

# Reason: 2 of 3 replicas ready (deployment in progress)
```

**Service Health**: Services are always considered healthy (they don't fail)

```yaml
apiVersion: v1
kind: Service
metadata:
  name: api-service
spec:
  selector:
    app: api
  ports:
  - port: 80
    targetPort: 8000
```

**Output**: Service health status
```
$ argocd app get api-app
Resources:
NAME           KIND         VERSION  HEALTH       STATUS
api-service    Service      v1       Healthy      Synced

# Note: Always Healthy, regardless of endpoints
```

**Job Health**: ArgoCD checks if the job completed successfully

```yaml
# A migration job that runs once
apiVersion: batch/v1
kind: Job
metadata:
  name: db-migrate
spec:
  template:
    spec:
      restartPolicy: Never
      containers:
      - name: migrate
        image: migrate:latest
        command: ["migrate", "-path", "/migrations", "-database", "postgresql://..."]
```

**Output**: Job health during execution
```
$ argocd app get migration-app
Resources:
NAME           KIND         VERSION  HEALTH         STATUS
db-migrate     Job          batch/v1 Progressing    Synced

# Once job completes, status becomes Healthy
$ kubectl get job db-migrate
NAME         COMPLETIONS   DURATION   AGE
db-migrate   1/1           15s        45s

$ argocd app get migration-app
Resources:
NAME           KIND         VERSION  HEALTH       STATUS
db-migrate     Job          batch/v1 Healthy      Synced
```

**PersistentVolumeClaim Health**: ArgoCD checks if the claim is bound to actual storage

```yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: data-storage
spec:
  accessModes:
  - ReadWriteOnce
  resources:
    requests:
      storage: 10Gi
  storageClassName: standard
```

**Output**: PVC health when storage unavailable
```
$ kubectl get pvc data-storage
NAME            STATUS    VOLUME   CAPACITY   AGE
data-storage    Pending   <none>   <none>     2m

$ argocd app get storage-app
Resources:
NAME           KIND         VERSION  HEALTH       STATUS
data-storage   PVC          v1       Degraded     Synced

# Reason: No storage class available, volume cannot bind
```

### The Five Health States

ArgoCD assigns every resource one of these states:

| State | Meaning | Example | Recovery |
|-------|---------|---------|----------|
| **Healthy** | Resource is working as designed | Deployment ready, Pod running | N/A—already good |
| **Progressing** | Resource is changing toward healthy state | Pod starting up, rolling update in progress | Automatic—wait for completion |
| **Degraded** | Resource failed and needs intervention | Pod CrashLoopBackOff, Job failed | Fix underlying issue, redeploy |
| **Unknown** | ArgoCD cannot determine health | Custom resource with no health rule | Define custom health check |
| **Missing** | Resource should exist but doesn't | Deployment not created yet | Wait for sync to complete |

### Application-Level Health Aggregation

ArgoCD doesn't just report resource health—it aggregates across all resources in an Application:

```
Application Health = Worst health of all contained resources
```

Example: You deploy a web app (Deployment) and database (StatefulSet). If the database Pod is Degraded but the web app Deployment is Healthy, the **whole Application is Degraded**. This forces you to fix the root cause.

```bash
# Check aggregated application health
argocd app get agent-app
```

**Output**: Application marked Degraded due to one failed resource
```
$ argocd app get agent-app
Name:               agent-app
Project:            default
Server:             https://kubernetes.default.svc
Namespace:          agent-app
Status:             Synced
Health Status:      Degraded

Resources:
NAME                             KIND         VERSION  HEALTH       STATUS
agent-app-deployment             Deployment   apps/v1  Healthy      Synced
agent-app-db                      StatefulSet  apps/v1  Degraded     Synced
agent-app-service                 Service      v1       Healthy      Synced

# Application is Degraded because agent-app-db StatefulSet is Degraded
```

## Custom Health Checks with Lua

Built-in health checks work for standard Kubernetes resources. But if you use CustomResources, Operators, or non-standard resources, you need custom health logic. ArgoCD allows you to define health checks in Lua—a lightweight scripting language.

### When to Use Custom Health Checks

**Scenario 1: Operator-Managed Resources**

You install an ArgoCD Operator that manages ArgoCD instances. The CRD looks like:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ArgoCD
metadata:
  name: argocd
status:
  phase: Running
  conditions:
    - type: Reconciled
      status: "True"
```

**Output**: Custom resource without built-in health understanding
```
$ argocd app get argocd-instance
Resources:
NAME      KIND     VERSION  HEALTH       STATUS
argocd    ArgoCD   v1alpha1 Unknown      Synced

# ArgoCD doesn't understand this CRD, so health is Unknown
```

To fix this, you define a custom Lua health check.

### Health Check ConfigMap Structure

Custom health checks live in a ConfigMap that ArgoCD reads at startup:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-cm
  namespace: argocd
data:
  resource.customizations.health.argoproj.io_ArgoCD: |
    hs = {}
    if obj.status.phase == "Running" then
      hs.status = "Healthy"
    else
      hs.status = "Progressing"
      hs.message = "Phase: " .. obj.status.phase
    end
    return hs
```

**Output**: Custom health check applied to ArgoCD CRD
```
$ kubectl get cm -n argocd argocd-cm -o jsonpath='{.data.resource\.customizations\.health\.argoproj\.io_ArgoCD}'
hs = {}
if obj.status.phase == "Running" then
  hs.status = "Healthy"
else
  hs.status = "Progressing"
  hs.message = "Phase: " .. obj.status.phase
end
return hs

$ argocd app get argocd-instance
Resources:
NAME      KIND     VERSION  HEALTH       STATUS
argocd    ArgoCD   v1alpha1 Healthy      Synced
```

**Key pattern**: `resource.customizations.health.{GROUP}_{KIND}`

Where:
- `GROUP` = resource's API group (e.g., `argoproj.io`, `mycompany.io`)
- `KIND` = resource kind (e.g., `ArgoCD`, `Database`)
- value = Lua script that returns health status

### Example: Custom Health Check for PostgreSQL

Imagine you use a PostgreSQL Operator that creates database instances:

```yaml
apiVersion: postgresql.cnpg.io/v1
kind: Cluster
metadata:
  name: postgres
status:
  phase: "healthy"
  conditions:
    - type: Ready
      status: "True"
    - type: BackupReady
      status: "False"
```

Define health as "Ready condition true AND last backup within 24 hours":

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-cm
  namespace: argocd
data:
  resource.customizations.health.postgresql.cnpg.io_Cluster: |
    hs = {}
    local ready = false
    local backup_ok = true

    if obj.status.conditions then
      for i, condition in ipairs(obj.status.conditions) do
        if condition.type == "Ready" and condition.status == "True" then
          ready = true
        end
        if condition.type == "BackupReady" and condition.status == "False" then
          backup_ok = false
        end
      end
    end

    if not ready then
      hs.status = "Degraded"
      hs.message = "Ready condition false"
    elseif not backup_ok then
      hs.status = "Degraded"
      hs.message = "No backup in 24 hours"
    else
      hs.status = "Healthy"
    end

    return hs
```

**Output**: Custom health check for PostgreSQL cluster
```
$ kubectl apply -f argocd-cm.yaml
configmap/argocd-cm configured

$ argocd app get db-app
Resources:
NAME         KIND     HEALTH        STATUS
postgres     Cluster  Healthy       Synced

# Custom Lua health check is now active
```

## Notification Triggers: When ArgoCD Alerts

ArgoCD fires notifications when certain events occur. Understanding these triggers helps you configure the right notifications for your needs.

### Sync Triggers

**On Sync Success**: When a sync completes without errors

```bash
# Example: You merge a PR with new Deployment
# GitHub Actions builds → pushes image → commits to GitOps repo
# ArgoCD detects change and syncs
# Trigger fires: on-sync-succeeded
```

**Output**: ArgoCD logs sync success
```
$ kubectl logs -f -n argocd deployment/argocd-application-controller | grep sync
INFO ... application-controller ... agent-app: sync succeeded
```

**On Sync Failure**: When a sync fails (manifest invalid, image pull fails, etc.)

```bash
# Example: You commit invalid YAML
# ArgoCD attempts sync but manifest validation fails
# Trigger fires: on-sync-failed
```

**Output**: ArgoCD logs sync failure
```
$ kubectl logs -f -n argocd deployment/argocd-application-controller | grep sync
ERROR ... application-controller ... agent-app: sync failed, unknown field 'imagePull'
```

**On Sync Started**: When a sync begins (triggered manually or by auto-sync)

```bash
# Example: ArgoCD detects drift → auto-sync enabled → sync initiates
# Trigger fires: on-sync-started
```

**Output**: ArgoCD logs sync initiation
```
$ kubectl logs -f -n argocd deployment/argocd-application-controller | grep sync
INFO ... application-controller ... agent-app: initiating sync operation
```

### Health Triggers

**On Health Change to Degraded**: When a resource health changes from Healthy to Degraded

```bash
# Example: Pod running → Container crashes → Pod CrashLoopBackOff → health = Degraded
# Trigger fires: on-health-degraded
```

**Output**: Health status change in ArgoCD
```
$ argocd app get agent-app
Resources:
NAME                             KIND         VERSION  HEALTH       STATUS
agent-app-deployment             Deployment   apps/v1  Degraded     Synced

# Pod failed, health cascaded to Deployment, then to Application
```

**On Application Health Degraded**: When overall Application health becomes Degraded

```bash
# Example: One resource fails → Application cascades to Degraded
```

**Output**: ArgoCD shows degraded application
```
$ argocd app get agent-app --refresh
Name:               agent-app
Status:             Synced
Health Status:      Degraded
```

## Configuring Slack Notifications

Slack is the most common notification destination. ArgoCD integrates with Slack using the `argocd-notifications` application.

### Install argocd-notifications

ArgoCD doesn't include notifications by default. Install the controller:

```bash
kubectl create namespace argocd-notifications

kubectl apply -n argocd-notifications -f \
  https://raw.githubusercontent.com/argoproj-labs/argocd-notifications/release-1.3/manifests/install.yaml
```

**Output**: Notification controller running
```
$ kubectl get pods -n argocd-notifications
NAME                                  READY   STATUS    RESTARTS   AGE
argocd-notifications-controller       1/1     Running   0          2m
```

### Create Slack Webhook

Before configuring ArgoCD, create an Incoming Webhook in your Slack workspace:

1. Go to https://api.slack.com/apps
2. Click "Create New App" → "From scratch"
3. Name: "ArgoCD Notifications"
4. Select your workspace
5. Go to "Incoming Webhooks" → enable it
6. Click "Add New Webhook to Workspace"
7. Select target channel (e.g., #deployments)
8. Authorize
9. Copy the webhook URL

**Example webhook URL format**:
```
https://hooks.slack.com/services/T00000000/B00000000/XXXXXXXXXXXXXXXXXXXX
```

### Configure argocd-notifications-cm

Create a ConfigMap in the argocd-notifications namespace:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-notifications-cm
  namespace: argocd-notifications
data:
  service.slack: |
    token: $slack-token
  trigger.on-sync-succeeded: |
    - when: app.status.operationState.phase in ['Succeeded'] and app.status.health.status == 'Healthy'
      send: [app-sync-success]
  trigger.on-sync-failed: |
    - when: app.status.operationState.phase in ['Failed']
      send: [app-sync-failed]
  trigger.on-health-degraded: |
    - when: app.status.health.status == 'Degraded'
      send: [app-health-degraded]
  template.app-sync-success: |
    message: |
      Deployment Succeeded ✅
      Application: {{ .app.metadata.name }}
      Timestamp: {{ .app.status.operationState.finishedAt }}
    slack:
      attachments: |
        [{
          "color": "#18be52",
          "fields": [
            {"title": "Sync Status", "value": "{{ .app.status.operationState.phase }}", "short": true},
            {"title": "Repository", "value": "{{ .app.spec.source.repoURL }}", "short": true}
          ]
        }]
  template.app-sync-failed: |
    message: |
      Deployment Failed ❌
      Application: {{ .app.metadata.name }}
    slack:
      attachments: |
        [{
          "color": "#e96d76",
          "fields": [
            {"title": "Sync Status", "value": "{{ .app.status.operationState.phase }}", "short": true},
            {"title": "Error", "value": "{{ .app.status.operationState.syncResult.errors[0].message }}", "short": false}
          ]
        }]
  template.app-health-degraded: |
    message: |
      Application Health Degraded ⚠️
      Application: {{ .app.metadata.name }}
    slack:
      attachments: |
        [{
          "color": "#f4e04d",
          "fields": [
            {"title": "Health Status", "value": "{{ .app.status.health.status }}", "short": true},
            {"title": "Namespace", "value": "{{ .app.metadata.namespace }}", "short": true}
          ]
        }]
```

**Output**: ConfigMap created and applied
```
$ kubectl apply -f argocd-notifications-cm.yaml
configmap/argocd-notifications-cm created

$ kubectl get cm -n argocd-notifications argocd-notifications-cm
NAME                         DATA   AGE
argocd-notifications-cm       6      30s
```

### Create Secret for Slack Token

The Slack webhook URL must be in a Secret:

```bash
SLACK_WEBHOOK="https://hooks.slack.com/services/YOUR/WEBHOOK/URL"

kubectl create secret generic argocd-notifications-secret \
  --from-literal=slack-token="$SLACK_WEBHOOK" \
  -n argocd-notifications
```

**Output**: Secret created
```
$ kubectl get secret -n argocd-notifications argocd-notifications-secret
NAME                              TYPE     DATA   AGE
argocd-notifications-secret       Opaque   1      30s
```

### Subscribe Application to Notifications

Add labels to your Application to subscribe to notifications:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: agent-app
  namespace: argocd
  labels:
    notifications.argoproj.io/subscribe.on-sync-succeeded.slack: deployments
    notifications.argoproj.io/subscribe.on-sync-failed.slack: critical-alerts
    notifications.argoproj.io/subscribe.on-health-degraded.slack: critical-alerts
spec:
  project: default
  source:
    repoURL: https://github.com/myteam/agent-deployment
    targetRevision: HEAD
    path: helm/agent
  destination:
    server: https://kubernetes.default.svc
    namespace: agent-app
  syncPolicy:
    automated:
      prune: true
      selfHeal: true
```

**Output**: Application receives notifications
```
$ kubectl apply -f agent-app.yaml
application.argoproj.io/agent-app created

# When sync succeeds, Slack message arrives in #deployments:
Deployment Succeeded ✅
Application: agent-app
Timestamp: 2025-12-23T14:32:15Z

Sync Status: Succeeded
Repository: https://github.com/myteam/agent-deployment
```

## Webhook Notifications for Custom Integrations

Slack is common, but some teams use custom systems: PagerDuty, Opsgenie, DataDog, or internal webhooks. Webhooks let you send HTTP POST requests to any endpoint.

### Configure Webhook Notification

In `argocd-notifications-cm`, add webhook service and template:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-notifications-cm
  namespace: argocd-notifications
data:
  service.webhook: |
    url: $webhook-url
  trigger.on-sync-failed: |
    - when: app.status.operationState.phase in ['Failed']
      send: [app-sync-failed-webhook]
  template.app-sync-failed-webhook: |
    webhook:
      path: /argocd/sync-failed
      method: POST
      body: |
        {
          "app_name": "{{ .app.metadata.name }}",
          "namespace": "{{ .app.metadata.namespace }}",
          "sync_status": "{{ .app.status.operationState.phase }}",
          "error": "{{ .app.status.operationState.syncResult.errors[0].message }}",
          "timestamp": "{{ .app.status.operationState.finishedAt }}"
        }
```

**Output**: Webhook request sent to custom endpoint
```
POST /argocd/sync-failed HTTP/1.1
Host: webhook.example.com
Content-Type: application/json

{
  "app_name": "agent-app",
  "namespace": "argocd",
  "sync_status": "Failed",
  "error": "resource not found: ConfigMap agent-config",
  "timestamp": "2025-12-23T14:32:15Z"
}
```

### Subscribe Application to Webhook

Add webhook subscription to your Application:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: agent-app
  namespace: argocd
  labels:
    notifications.argoproj.io/subscribe.on-sync-failed.webhook: agent-webhook
spec:
  # ... rest of Application spec
```

**Output**: Notification sent to webhook endpoint
```
$ kubectl logs -f -n argocd-notifications argocd-notifications-controller | grep webhook
INFO ... webhook notification sent to webhook.example.com/argocd/sync-failed
```

## Complete Notification Setup Example

Let's deploy a real example: an agent application with health checks and Slack notifications.

### Step 1: Create Custom Health Check ConfigMap

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-cm
  namespace: argocd
data:
  resource.customizations.health.apps_Deployment: |
    hs = {}
    if obj.spec.replicas and obj.status.replicas and obj.spec.replicas == obj.status.replicas then
      hs.status = "Healthy"
    else
      hs.status = "Degraded"
      hs.message = "Replicas not ready: desired=" .. obj.spec.replicas .. ", ready=" .. (obj.status.replicas or 0)
    end
    return hs
```

**Output**: Custom health check registered
```
$ kubectl get cm -n argocd argocd-cm -o yaml | grep -A8 "resource.customizations"
resource.customizations.health.apps_Deployment: |
  hs = {}
  if obj.spec.replicas and obj.status.replicas...
```

### Step 2: Install argocd-notifications

```bash
kubectl apply -n argocd-notifications -f \
  https://raw.githubusercontent.com/argoproj-labs/argocd-notifications/release-1.3/manifests/install.yaml

kubectl wait --for=condition=ready pod \
  -l app.kubernetes.io/name=argocd-notifications-controller \
  -n argocd-notifications --timeout=60s
```

**Output**: Notification controller ready
```
condition met
pod/argocd-notifications-controller-5c8d6c44f5-7xk9m condition met
```

### Step 3: Create Slack Webhook Secret

```bash
SLACK_WEBHOOK="https://hooks.slack.com/services/YOUR/WEBHOOK/URL"

kubectl create secret generic argocd-notifications-secret \
  --from-literal=slack-token="$SLACK_WEBHOOK" \
  -n argocd-notifications
```

**Output**: Secret stored
```
secret/argocd-notifications-secret created
```

### Step 4: Configure Notifications

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-notifications-cm
  namespace: argocd-notifications
data:
  service.slack: |
    token: $slack-token
  trigger.on-sync-succeeded: |
    - when: app.status.operationState.phase in ['Succeeded']
      send: [app-sync-success]
  trigger.on-sync-failed: |
    - when: app.status.operationState.phase in ['Failed']
      send: [app-sync-failed]
  trigger.on-health-degraded: |
    - when: app.status.health.status == 'Degraded'
      send: [app-health-degraded]
  template.app-sync-success: |
    message: |
      ✅ Deployment Succeeded
      App: {{ .app.metadata.name }}
    slack:
      attachments: |
        [{
          "color": "#18be52",
          "fields": [
            {"title": "Namespace", "value": "{{ .app.metadata.namespace }}", "short": true}
          ]
        }]
  template.app-sync-failed: |
    message: |
      ❌ Deployment Failed
      App: {{ .app.metadata.name }}
    slack:
      attachments: |
        [{
          "color": "#e96d76",
          "fields": [
            {"title": "Namespace", "value": "{{ .app.metadata.namespace }}", "short": true}
          ]
        }]
  template.app-health-degraded: |
    message: |
      ⚠️ Application Unhealthy
      App: {{ .app.metadata.name }}
    slack:
      attachments: |
        [{
          "color": "#f4e04d",
          "fields": [
            {"title": "Health", "value": "{{ .app.status.health.status }}", "short": true}
          ]
        }]
```

**Output**: Notifications configured
```
$ kubectl apply -f argocd-notifications-cm.yaml -n argocd-notifications
configmap/argocd-notifications-cm configured
```

### Step 5: Deploy Application with Notifications

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: agent-app
  namespace: argocd
  labels:
    notifications.argoproj.io/subscribe.on-sync-succeeded.slack: deployments
    notifications.argoproj.io/subscribe.on-sync-failed.slack: critical-alerts
    notifications.argoproj.io/subscribe.on-health-degraded.slack: critical-alerts
spec:
  project: default
  source:
    repoURL: https://github.com/myteam/agent-deployment
    targetRevision: HEAD
    path: helm/agent
  destination:
    server: https://kubernetes.default.svc
    namespace: agent-app
  syncPolicy:
    automated:
      prune: true
      selfHeal: true
```

**Output**: Application created with notification subscriptions
```
$ kubectl apply -f agent-app.yaml
application.argoproj.io/agent-app created

# When sync succeeds, Slack receives in #deployments:
✅ Deployment Succeeded
App: agent-app
Namespace: agent-app
```

### Step 6: Verify Health Status and Notifications

```bash
# Check application health
argocd app get agent-app

# Check notification controller logs
kubectl logs -f -n argocd-notifications \
  -l app.kubernetes.io/name=argocd-notifications-controller
```

**Output**: Application healthy and notifications active
```
$ argocd app get agent-app
Name:               agent-app
Status:             Synced
Health Status:      Healthy

$ kubectl logs -n argocd-notifications argocd-notifications-controller | grep agent-app
INFO ... sent notification for app=agent-app, trigger=on-sync-succeeded
```

## Try With AI

Now you understand how ArgoCD monitors application health and sends notifications when deployment events occur. In Lesson 16, you'll collaborate with Claude to design sophisticated notification systems and troubleshoot health issues.

For now, explore your understanding with these prompts:

**Setup**: If you have an ArgoCD Application deployed from Lesson 7, you can test these scenarios:
- Deploy a broken image to see health change to Degraded
- Configure a custom health check for a resource you're using
- Set up Slack notifications to see real deployment alerts

**Prompt 1: Diagnose Current Health**

Ask Claude to analyze your application's health status:

```
Here's my agent application in ArgoCD:

[paste output of: argocd app get agent-app]

Which resources are unhealthy? Why might my Deployment be Degraded?
```

**Prompt 2: Design Custom Health Check**

If you use a custom resource, ask Claude to generate a health check:

```
I use a CustomResource called "AgentService" with this status structure:

[paste CRD structure]

Define a Lua health check that marks it Degraded if the API endpoint
is unreachable.
```

**Prompt 3: Configure Notifications**

Ask Claude to build your notification ConfigMap:

```
I want notifications when:
1. Deployment succeeds (send to #deployments Slack)
2. Health becomes degraded (alert #critical with @channel)
3. Sync fails (webhook to incident management system)

Generate the complete argocd-notifications-cm ConfigMap.
```

