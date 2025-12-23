---
sidebar_position: 15
title: "Multi-Cluster Deployments"
description: "Deploy to multiple Kubernetes clusters with ArgoCD hub-spoke architecture"
keywords: [argocd, multi-cluster, hub-spoke, cluster registration, applicationset, gitops]
---

# Multi-Cluster Deployments

So far you've deployed your FastAPI agent to a single Kubernetes cluster. That works for development. But production systems need redundancy: if one cluster fails, your agent keeps running on another. If you need to test a new version before rolling out to all users, you deploy to a staging cluster first. This lesson teaches you to manage **multiple clusters from one ArgoCD instance** using a hub-spoke architecture.

In hub-spoke, ArgoCD (the hub) manages deployment to many Kubernetes clusters (the spokes). You define your application once in Git. ArgoCD syncs that same application to cluster 1, cluster 2, cluster 3—each with different configurations. One Git repository becomes the source of truth for your entire infrastructure.

## The Hub-Spoke Architecture

A **hub-spoke topology** has one control point (ArgoCD hub) managing many execution points (Kubernetes clusters as spokes). This is different from decentralized approaches where each cluster runs its own ArgoCD instance.

### Why Hub-Spoke?

**Single pane of glass**: One ArgoCD UI/CLI shows status across all clusters

```
ArgoCD Hub                 Kubernetes Clusters
┌──────────────┐
│  ArgoCD      │          ┌──────────────┐
│  Server      │──────────│ Prod Cluster │
│              │          │   (us-east)  │
│              │          └──────────────┘
│ Git Repo     │
│ (source of   │          ┌──────────────┐
│  truth)      │──────────│ Staging      │
│              │          │  (us-west)   │
│              │          └──────────────┘
└──────────────┘
                          ┌──────────────┐
                    ──────│ DR Cluster   │
                          │  (eu-west)   │
                          └──────────────┘
```

**Cost of a unified approach**: Secrets containing cluster credentials must be stored securely in ArgoCD, not in Git. We'll address this in Lesson 14 (Secrets Management).

**Alternative: cluster-local ArgoCD** (not hub-spoke):

```
Git Repo              Kubernetes Clusters

Prod Cluster          ┌──────────────┐
  └─ ArgoCD ────────────│ Prod Cluster │
                        └──────────────┘

Staging Cluster       ┌──────────────┐
  └─ ArgoCD ────────────│ Staging      │
                        └──────────────┘
```

This approach works for teams with separate infra teams per cluster but loses the unified deployment view. We'll focus on hub-spoke because it's more common for AI agents.

## Registering External Clusters

ArgoCD starts with one cluster: the one it's installed in (the hub). To deploy to other clusters (spokes), you must **register** those clusters with ArgoCD first.

### Local Cluster Registration (Hub Cluster)

When you install ArgoCD on a cluster, it automatically registers itself:

```yaml
apiVersion: cluster.argoproj.io/v1alpha1
kind: Cluster
metadata:
  name: in-cluster
spec:
  server: https://kubernetes.default.svc
  config:
    bearerToken: <token>
    tlsClientConfig:
      insecure: false
      caData: <ca-cert>
```

**Output:**
```
Cluster registered successfully
Name:  in-cluster
URL:   https://kubernetes.default.svc
Status: Healthy
```

### Registering External Clusters

To register an external cluster (e.g., your staging environment), you need:

1. **Access to the external cluster's API server** (kubeconfig context)
2. **A service account with cluster-admin** permissions (or appropriate RBAC)
3. **The `argocd` CLI** to register the cluster

**Step 1: Create a service account on the external cluster**

```bash
# On the external cluster, create a namespace and service account
kubectl create namespace argocd
kubectl create serviceaccount argocd-manager -n argocd

# Grant cluster-admin permissions
kubectl create clusterrolebinding argocd-manager-cluster-admin \
  --clusterrole=cluster-admin \
  --serviceaccount=argocd:argocd-manager
```

**Output:**
```
namespace/argocd created
serviceaccount/argocd-manager created
clusterrolebinding.rbac.authorization.k8s.io/argocd-manager-cluster-admin created
```

**Step 2: Get the external cluster's kubeconfig**

```bash
# Generate a kubeconfig for the service account
kubectl config get-contexts

# Current context should be your external cluster
# If not, switch to it:
kubectl config use-context <external-cluster-context>
```

**Output:**
```
CURRENT   NAME           CLUSTER      AUTHINFO     NAMESPACE
*         staging-us-west-1  us-west-1      admin
          prod-us-east-1    us-east-1      admin
```

**Step 3: Register the cluster with ArgoCD**

```bash
# Switch back to your HUB cluster where ArgoCD is installed
kubectl config use-context in-cluster

# Port-forward to ArgoCD (if it's not exposed)
kubectl port-forward -n argocd svc/argocd-server 8080:443 &

# Register the external cluster
argocd cluster add staging-us-west-1 \
  --name staging \
  --in-cluster=false

# If you want to give it a custom name:
argocd cluster add staging-us-west-1 \
  --name staging-deployment \
  --namespace argocd
```

**Output:**
```
INFO[0003] ServiceAccount "argocd-manager" created in namespace "argocd"
INFO[0004] ClusterRole "argocd-manager-role" created
INFO[0005] ClusterRoleBinding "argocd-manager-rolebinding" created
Cluster 'staging' has been added to Argo CD. An RBAC ClusterRole 'argocd-manager-role' and ClusterRoleBinding 'argocd-manager-rolebinding' have been created on cluster 'staging' to manage cluster credentials.

You can now deploy applications to this cluster by setting the destination cluster of an Application to 'staging' (e.g. destination.name=staging)
```

## Cluster Secrets and Authentication

When you register an external cluster, ArgoCD stores the cluster's API server URL and authentication credentials as a **Kubernetes Secret** in the hub cluster.

### Viewing Registered Clusters

```bash
# List all registered clusters
argocd cluster list

# Get details of a specific cluster
argocd cluster get staging

# View the cluster secret directly
kubectl get secret -n argocd | grep cluster

# Inspect a cluster secret
kubectl get secret -n argocd \
  -l argocd.argoproj.io/secret-type=cluster \
  -o yaml
```

**Output:**
```
NAME           CLUSTER             TLS
in-cluster     https://kubernetes.default.svc   false
staging        https://staging-api.example.com  true
prod           https://prod-api.example.com     true

---

apiVersion: v1
kind: Secret
metadata:
  name: cluster-staging-0123456789abcdef
  namespace: argocd
  labels:
    argocd.argoproj.io/secret-type: cluster
type: Opaque
data:
  server: aHR0cHM6Ly9zdGFnaW5nLWFwaS5leGFtcGxlLmNvbQ==  # base64 encoded
  name: c3RhZ2luZw==  # base64 encoded
  config: eyJiZWFyZXJUb2tlbiI6Ijc4OXB4eVl6ZUZRSXdVMkZrVUhGcGJISmhiblJsIn0=
```

### Cluster Credentials: Bearer Token

The `config` field in the secret contains authentication details. For external clusters, it typically includes:

```json
{
  "bearerToken": "<service-account-token>",
  "tlsClientConfig": {
    "insecure": false,
    "caData": "<base64-encoded-ca-cert>"
  }
}
```

The bearer token comes from the `argocd-manager` service account on the external cluster:

```bash
# Get the token from the external cluster
kubectl get secret -n argocd \
  $(kubectl get secret -n argocd | grep argocd-manager-token | awk '{print $1}') \
  -o jsonpath='{.data.token}' | base64 -d
```

**Output:**
```
eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJrdWJlcm5ldGVzL3NlcnZpY2VhY2NvdW50Iiwia3ViZXJuZXRlcy5pby9zZXJ2aWNlYWNjb3VudC9uYW1lc3BhY2UiOiJhcmdvY2QiLCJrdWJlcm5ldGVzLmlvL3NlcnZpY2VhY2NvdW50L3NlY3JldC5uYW1lIjoiYXJnb2NkLW1hbmFnZXItdG9rZW4tOXA0ZGwiLCJrdWJlcm5ldGVzLmlvL3NlcnZpY2VhY2NvdW50L3NlcnZpY2VhY2NvdW50Lm5hbWUiOiJhcmdvY2QtbWFuYWdlciIsImt1YmVybmV0ZXMuaW8vc2VydmljZWFjY291bnQvc2VydmljZWFjY291bnQudWlkIjoiOWQ1YTc1YzItZjM0ZS00YjQ3LWJhYmUtODJmMmI4N2RhMjI0In0.4bGl...
```

### Cluster Health Check

ArgoCD periodically verifies cluster connectivity:

```bash
# Check cluster health
kubectl describe secret -n argocd cluster-staging-0123456789abcdef

# Or via CLI
argocd cluster get staging
```

**Output:**
```
Name:           staging
Server:         https://staging-api.example.com
Connection Status: Successful
```

If a cluster becomes unreachable, ArgoCD marks it as unhealthy but continues managing other clusters.

## ApplicationSet with Cluster Generator

You've already learned ApplicationSets in Lesson 10 (List and Matrix generators). Now you'll use the **Cluster generator** to deploy an application to multiple registered clusters with cluster-specific configurations.

### The Cluster Generator Concept

Instead of creating separate Applications for prod, staging, and DR:

```yaml
# ❌ Old way: Three separate Applications
---
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: agent-prod
spec:
  destination:
    server: https://prod-api.example.com
---
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: agent-staging
spec:
  destination:
    server: https://staging-api.example.com
---
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: agent-dr
spec:
  destination:
    server: https://dr-api.example.com
```

Use a **Cluster generator** to create one Application per registered cluster:

```yaml
# ✅ New way: One ApplicationSet generates three Applications
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-multi-cluster
spec:
  generators:
    - clusters: {}  # Generates one Application per registered cluster
  template:
    metadata:
      name: 'agent-{{name}}'
    spec:
      project: default
      destination:
        server: '{{server}}'
        namespace: agent
      source:
        repoURL: https://github.com/example/agent
        path: manifests/
        targetRevision: main
```

The `clusters: {}` generator creates template variables for every registered cluster:
- `{{name}}`: Cluster name (e.g., "staging", "prod")
- `{{server}}`: Cluster API server URL (e.g., "https://staging-api.example.com")
- `{{metadata.labels}}`: Cluster labels (if you've added them)

### Cluster-Specific Configurations

Real deployments need different configs per cluster. You might want:
- **Prod**: 3 replicas, resource limits, strict security policies
- **Staging**: 1 replica, minimal resources, relaxed policies
- **DR**: 3 replicas, same as prod but in different region

Use **Helm values overrides** to customize per cluster:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-multi-cluster
spec:
  generators:
    - clusters:
        selector:
          matchLabels:
            deploy: "true"  # Only deploy to clusters with this label
  template:
    metadata:
      name: 'agent-{{name}}'
    spec:
      project: default
      destination:
        server: '{{server}}'
        namespace: agent
      source:
        repoURL: https://github.com/example/agent
        path: helm/
        targetRevision: main
        helm:
          releaseName: agent
          values: |
            replicas: "{{replicas}}"
            environment: "{{name}}"
```

**Step 1: Add labels to clusters**

```bash
# Label the staging cluster
argocd cluster patch staging \
  -p '{"metadata":{"labels":{"env":"staging","deploy":"true"}}}'

# Label the prod cluster
argocd cluster patch prod \
  -p '{"metadata":{"labels":{"env":"prod","deploy":"true"}}}'

# Label the DR cluster
argocd cluster patch dr \
  -p '{"metadata":{"labels":{"env":"dr","deploy":"true"}}}'
```

**Output:**
```
cluster 'staging' patched
cluster 'prod' patched
cluster 'dr' patched
```

**Step 2: Create values-per-cluster in your Git repository**

Create these files in your agent repository:

**helm/values.yaml** (default values for all clusters):
```yaml
replicas: 1
resources:
  requests:
    memory: "64Mi"
    cpu: "100m"
  limits:
    memory: "128Mi"
    cpu: "500m"
securityContext:
  runAsNonRoot: false
```

**helm/values-prod.yaml** (prod-specific overrides):
```yaml
replicas: 3
resources:
  requests:
    memory: "512Mi"
    cpu: "500m"
  limits:
    memory: "1Gi"
    cpu: "1000m"
securityContext:
  runAsNonRoot: true
```

**helm/values-staging.yaml** (staging-specific overrides):
```yaml
replicas: 1
resources:
  requests:
    memory: "64Mi"
    cpu: "100m"
  limits:
    memory: "128Mi"
    cpu: "500m"
```

**helm/values-dr.yaml** (DR cluster same as prod):
```yaml
replicas: 3
resources:
  requests:
    memory: "512Mi"
    cpu: "500m"
  limits:
    memory: "1Gi"
    cpu: "1000m"
securityContext:
  runAsNonRoot: true
```

Verify the files exist:

```bash
ls -la helm/values*.yaml
```

**Output:**
```
-rw-r--r--  1 user  group  298 Dec 23 10:15 helm/values.yaml
-rw-r--r--  1 user  group  156 Dec 23 10:15 helm/values-staging.yaml
-rw-r--r--  1 user  group  298 Dec 23 10:15 helm/values-prod.yaml
-rw-r--r--  1 user  group  298 Dec 23 10:15 helm/values-dr.yaml
```

**Step 3: Create ApplicationSet with per-cluster values**

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-multi-cluster
spec:
  generators:
    - clusters:
        selector:
          matchLabels:
            deploy: "true"
  template:
    metadata:
      name: 'agent-{{name}}'
    spec:
      project: default
      syncPolicy:
        automated:
          prune: true
          selfHeal: true
      destination:
        server: '{{server}}'
        namespace: agent
      source:
        repoURL: https://github.com/example/agent
        path: helm/
        targetRevision: main
        helm:
          releaseName: agent
          valueFiles:
            - values.yaml
            - values-{{name}}.yaml  # Cluster-specific overrides
```

**Apply the ApplicationSet**:

```bash
kubectl apply -f applicationset.yaml

# Watch ArgoCD generate Applications for each cluster
argocd app list

# Wait for sync to complete
argocd app wait agent-staging --sync
argocd app wait agent-prod --sync
argocd app wait agent-dr --sync
```

**Output:**
```
NAME                CLUSTER            NAMESPACE  PROJECT  STATUS    HEALTH
agent-staging       staging            agent      default  Synced    Healthy
agent-prod          prod               agent      default  Synced    Healthy
agent-dr            dr                 agent      default  Synced    Healthy
```

## Cross-Cluster Networking Considerations

Multi-cluster deployments raise networking questions:

### Service Discovery Between Clusters

If your agent in cluster A needs to call a service in cluster B, you have options:

**Option 1: Direct IP/DNS (not recommended)**
```
Agent Pod (Cluster A) → Service IP of Cluster B
Problem: Cluster-local service IPs don't route between clusters
```

**Option 2: Ingress/Load Balancer**
```
Agent Pod (Cluster A) → Load Balancer IP (external address of Cluster B)
Cluster B's Ingress routes to the service
Problem: Extra hops, more latency
```

**Option 3: Service Mesh (advanced)**
```
Istio/Linkerd manages cross-cluster networking automatically
Problem: Adds complexity, requires multiple control planes
```

For your AI agent, if each cluster is independent (data doesn't flow between clusters), you don't need cross-cluster communication. Each cluster runs a complete copy of your agent with its own database.

### DNS Across Clusters

Each Kubernetes cluster has its own DNS domain:
- **In Cluster A**: `agent-service.agent.svc.cluster.local` resolves only within Cluster A
- **In Cluster B**: Same `agent-service.agent.svc.cluster.local` is different from Cluster A

To expose a service to other clusters, use an external DNS name:

```bash
# Get the external endpoint
kubectl get svc -n agent agent-service -o jsonpath='{.status.loadBalancer.ingress[0].hostname}'
```

**Output:**
```
agent-staging.example.com
agent-prod.example.com
agent-dr.example.com
```

## Disaster Recovery: ArgoCD HA and Cluster Failover

With multiple clusters, you need resilience at two levels: **ArgoCD itself must be HA**, and **your clusters must be capable of failover**.

### ArgoCD High Availability (Hub Cluster)

If your ArgoCD hub cluster goes down, you cannot deploy to spoke clusters. Make ArgoCD highly available:

```yaml
# Install ArgoCD with HA enabled
helm install argocd argo/argo-cd \
  --namespace argocd \
  --set server.replicas=3 \
  --set controller.replicas=3 \
  --set repo.replicas=3 \
  --set redis.replicas=3
```

**Output:**
```
Release "argocd" has been installed.
Deployment argocd-application-controller: 3 replicas
Deployment argocd-server: 3 replicas
Deployment argocd-repo-server: 3 replicas
StatefulSet redis: 3 replicas
```

Each component is fault-tolerant:
- **Controller**: Reconciles Applications across all clusters
- **Server**: Serves the UI/API
- **Repo Server**: Clones Git repositories and renders manifests
- **Redis**: Stores application state

If one component pod crashes, others take over.

### Cluster Failover: Traffic Shifting

Your agent runs on three clusters (staging, prod, DR). If the prod cluster fails:

**Scenario 1: Users access through a load balancer**
```
User Traffic → AWS NLB (Network Load Balancer)
                ├─→ Prod cluster (prod.example.com)  [FAILED]
                ├─→ DR cluster (dr.example.com)      [HEALTHY]
                └─→ Staging (staging.example.com)    [BACKUP]

NLB detects prod is unhealthy (health checks fail)
NLB routes traffic to DR cluster
```

**Scenario 2: Users access through DNS (geo-routing)**
```
User in US → Prod cluster (us-east)     [FAILED]
             → DR cluster (us-west)     [HEALTHY]

User in EU → Prod EU cluster (eu-west) [FAILED]
            → No DR fallback in EU
            → User degraded or served from US
```

For your agent, implement:

1. **Health checks** on all clusters
2. **DNS failover** (Route53, Google Cloud DNS, Cloudflare) to shift traffic
3. **ArgoCD monitoring** to detect when clusters become unhealthy

```bash
# Check if a cluster is healthy
argocd cluster get prod

# Check application health on prod cluster
argocd app get agent-prod

# If unhealthy, manually shift traffic (or automate in DNS)
# Update DNS to point to DR cluster
# Verify traffic is reaching DR
```

**Output:**
```
Application: agent-prod
Status: Degraded
Server: https://prod-api.example.com (UNREACHABLE)

---

Cluster: prod
Connection Status: Failed (connection timeout)
```

## Complete Multi-Cluster ApplicationSet Example

Here's a production-ready example:

**Directory structure**:
```
repo/
├── argocd/
│   └── agent-multi-cluster-appset.yaml
├── helm/
│   ├── Chart.yaml
│   ├── values.yaml
│   ├── values-staging.yaml
│   ├── values-prod.yaml
│   └── values-dr.yaml
└── manifests/
    ├── configmap.yaml
    └── secrets.yaml  # NOTE: NEVER commit secrets here (use External Secrets)
```

**argocd/agent-multi-cluster-appset.yaml**:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-multi-cluster
  namespace: argocd
spec:
  syncPolicy:
    preserveResourcesOnDeletion: true
  generators:
    # Generate one Application per cluster with deploy label
    - clusters:
        selector:
          matchLabels:
            deploy: "true"
  template:
    metadata:
      name: 'agent-{{name}}'
      finalizers:
        - resources-finalizer.argocd.argoproj.io  # Cleanup resources on deletion
    spec:
      project: default  # Use RBAC project to restrict deployments
      syncPolicy:
        automated:
          prune: true        # Delete resources removed from Git
          selfHeal: true     # Revert manual kubectl changes
          allowEmpty: false  # Prevent accidental empty syncs
        syncOptions:
          - CreateNamespace=true
        retry:
          limit: 5
          backoff:
            duration: 5s
            factor: 2
            maxDuration: 3m
      destination:
        server: '{{server}}'
        namespace: agent
      source:
        repoURL: https://github.com/example/agent
        path: helm/
        targetRevision: main
        helm:
          releaseName: agent-{{name}}
          values: |
            cluster: "{{name}}"
            environment: "{{metadata.labels.env}}"
          valueFiles:
            - values.yaml
            - values-{{metadata.labels.env}}.yaml
```

**Deploy the ApplicationSet**:

```bash
# Ensure clusters are registered and labeled
argocd cluster list
argocd cluster patch staging --labels 'env=staging,deploy=true'
argocd cluster patch prod --labels 'env=prod,deploy=true'
argocd cluster patch dr --labels 'env=dr,deploy=true'

# Apply the ApplicationSet
kubectl apply -f argocd/agent-multi-cluster-appset.yaml

# Watch Applications get generated
watch argocd app list

# Check sync status
argocd app get agent-staging --refresh
argocd app get agent-prod --refresh
argocd app get agent-dr --refresh
```

**Output**:
```
NAME            CLUSTER    STATUS     HEALTH
agent-staging   staging    Syncing    Progressing
agent-prod      prod       Synced     Healthy
agent-dr        dr         Synced     Healthy

ApplicationSet: agent-multi-cluster
Generated Applications: 3
Total Resources Deployed: 15
```

## Try With AI

**Setup**: Use the same FastAPI agent from previous lessons. You now have three Kubernetes clusters available (or can simulate with three Minikube instances).

**Part 1: Design Your Multi-Cluster Strategy**

Before writing any YAML, clarify your deployment strategy:

Ask AI:

"I have a FastAPI agent that I want to deploy to three clusters: staging (for testing), prod (for users), and DR (disaster recovery backup). Each cluster should have different resource allocations: staging gets 1 replica and minimal resources, prod gets 3 replicas with high resource limits. DR gets the same as prod but in a different region. I also want to store sensitive configuration in a vault, not in Git. Design a multi-cluster deployment strategy using ArgoCD that supports: (1) Separate configurations per cluster, (2) Secrets management outside of Git, (3) Automatic failover if one cluster becomes unhealthy. What components do I need?"

Review AI's recommendation. Ask yourself:
- Does this strategy use hub-spoke (one ArgoCD managing many clusters)?
- Are configurations truly separate per cluster (values files)?
- How does the design prevent secrets from entering Git?

**Part 2: Refine Secret Handling**

Based on AI's answer, refine the approach:

"The strategy mentions External Secrets Operator for secrets. How would I configure External Secrets to pull database passwords from HashiCorp Vault for my prod cluster, while the staging cluster gets test credentials from a different secret location? Show me the ExternalSecret CRD format."

Evaluate AI's response:
- Does it show the correct ExternalSecret CRD structure?
- Are the Vault paths different for staging vs prod?
- Would this actually work, or are there missing prerequisites?

**Part 3: Test with One Cluster First**

Before deploying to three clusters, test with one:

"I want to set up a test ApplicationSet with just my staging cluster to verify the approach works before adding prod and DR. Give me a minimal ApplicationSet that deploys to a single cluster with custom values. How do I verify it synced successfully?"

Check AI's answer against what you learned in Lesson 10 (ApplicationSets).

**Part 4: Scaling to Three Clusters**

Once staging works, expand to three:

"Now add the prod and dr clusters to the ApplicationSet. How do I ensure the cluster selector only deploys to clusters with the deploy=true label? Show me the updated ApplicationSet and the commands to label each cluster."

Validate that:
- The Cluster generator uses the correct selector
- Each cluster gets labeled appropriately
- The valueFiles reference per-cluster overrides

**Part 5: Design Failover**

Finally, address resilience:

"If my prod cluster becomes unreachable, how does ArgoCD detect this and how would my users be notified? What monitoring should I add to alert when a cluster is unhealthy? Should I automate failover to the DR cluster or handle it manually?"

Think about:
- How quickly ArgoCD detects cluster failures
- Whether DNS-based failover is better than application-level failover
- What operational runbooks you'd need for actual failure scenarios

