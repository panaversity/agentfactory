---
sidebar_position: 10
chapter: 50
lesson: 10
duration_minutes: 40
title: "Service Discovery Deep Dive"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual DNS debugging builds understanding of service networking"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain Kubernetes DNS architecture (CoreDNS)"
    bloom_level: "Understand"
  - id: LO2
    description: "Understand DNS naming convention: service.namespace.svc.cluster.local"
    bloom_level: "Understand"
  - id: LO3
    description: "Debug DNS resolution failures with nslookup and dig"
    bloom_level: "Apply"
  - id: LO4
    description: "Configure headless Services for direct Pod discovery"
    bloom_level: "Apply"
  - id: LO5
    description: "Troubleshoot endpoint mismatches and selector issues"
    bloom_level: "Apply"
  - id: LO6
    description: "Understand cross-namespace service discovery"
    bloom_level: "Understand"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
  - objective_id: LO2
    competency_area: "1. Information and Data Literacy"
    competency: "1.3 Managing data, information and digital content"
  - objective_id: LO3
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO6
    competency_area: "2. Communication and Collaboration"
    competency: "2.1 Interacting through digital technologies"
---

# Service Discovery Deep Dive

You've learned how Services provide stable IP addresses to dynamic Pods. But what happens when Agent A in `agents` namespace tries calling Agent B's API in `services` namespace and gets "host not found"? The answer lies in Kubernetes DNS.

Service discovery is the mechanism that translates service names (like `api.services.svc.cluster.local`) into IP addresses. When DNS resolution fails, entire microservices architectures collapse. In this lesson, you'll build mental models of how Kubernetes DNS works, then debug common connectivity failures systematically.

---

## The Service Discovery Problem: "Host Not Found"

Imagine this scenario:

Your agent in `agents` namespace calls the API service in `services` namespace:

```bash
kubectl run -it agent-debug --image=curlimages/curl --rm -- \
  curl http://api.services/health
```

**Output:**
```
curl: (6) Could not resolve host: api.services
```

Why? The agent can't find the service. The full name exists—`api.services.svc.cluster.local`—but without understanding Kubernetes DNS architecture, you're debugging blind.

---

## How Kubernetes DNS Works: CoreDNS Architecture

Kubernetes doesn't use external DNS for internal service discovery. Instead, every cluster runs CoreDNS—a DNS server that understands Kubernetes services and translates names to IP addresses.

### The DNS Chain

When a Pod requests `api.services`:

```
1. Pod queries localhost:53 (DNS resolver in container)
2. Resolver forwards to kube-dns service (Cluster IP: 10.96.0.10)
3. CoreDNS pod answers using cluster's service records
4. IP address returned to Pod
```

**Key insight**: CoreDNS is itself deployed as a Pod (or Pods) in the `kube-system` namespace, managed by a Deployment:

```bash
kubectl get pods -n kube-system -l k8s-app=kube-dns
```

**Output:**
```
NAME                    READY   STATUS    RESTARTS   AGE
coredns-7db6d8ff4c      1/1     Running   0          3d
coredns-7db6d8ff4d      1/1     Running   0          3d
```

CoreDNS runs as multiple replicas for redundancy. Each CoreDNS pod watches the Kubernetes API for Service changes and automatically updates DNS records.

---

## DNS Naming Hierarchy: FQDN Explained

Every service in Kubernetes has a Fully Qualified Domain Name (FQDN). Understanding this hierarchy is critical for cross-namespace discovery.

### The FQDN Structure

```
service.namespace.svc.cluster.local
```

**Example breakdown** for a service named `api` in `services` namespace:

```
api                          ← Service name
  .services                  ← Namespace
    .svc                     ← Service subdomain (always literal)
      .cluster.local         ← Cluster domain (configurable, usually "cluster.local")
```

Complete FQDN: `api.services.svc.cluster.local`

### Short vs Long Names (Context Matters)

A Pod in the **same namespace** can use a **short name**:

```bash
# From a Pod in "services" namespace
curl http://api:8080/health
```

**Output:**
```
HTTP/1.1 200 OK
{"status": "healthy"}
```

A Pod in **different namespace** must use the **full FQDN**:

```bash
# From a Pod in "agents" namespace
curl http://api.services.svc.cluster.local:8080/health
```

**Output:**
```
HTTP/1.1 200 OK
{"status": "healthy"}
```

### Why the FQDN Matters

Short names only work because CoreDNS automatically appends the Pod's namespace. When you request `api` from namespace `agents`:

1. Pod searches for `api` locally (not found—doesn't exist in `agents`)
2. Pod searches for `api.agents.svc.cluster.local` (not found—service is in `services`)
3. Failure: "host not found"

The FQDN bypasses this search chain entirely.

---

## Debugging DNS: nslookup and dig

When service discovery fails, manual DNS debugging reveals the root cause. You'll use two tools: `nslookup` (simpler) and `dig` (more detailed).

### Setting Up a Debug Pod

Create a debug Pod with DNS tools:

```bash
kubectl run -it debug-dns --image=busybox:1.35 --rm --restart=Never -- \
  /bin/sh
```

Once inside the Pod:

```bash
/ #
```

### Using nslookup: DNS Lookup Basics

Query a service name:

```bash
nslookup api.services.svc.cluster.local
```

**Output (Success):**
```
Server:         10.96.0.10
Address:        10.96.0.10#53

Name:   api.services.svc.cluster.local
Address: 10.97.45.123
```

The server `10.96.0.10` is the kube-dns service. The resolved address `10.97.45.123` is the Service's ClusterIP.

**Output (Failure - Wrong Namespace):**
```bash
nslookup api.agents.svc.cluster.local
```

```
Server:         10.96.0.10
Address:        10.96.0.10#53

** server can't find api.agents.svc.cluster.local: NXDOMAIN
```

`NXDOMAIN` means "Non-Existent Domain"—the service doesn't exist in that namespace.

### Using dig: Detailed DNS Information

`dig` provides deeper insights into DNS records:

```bash
dig api.services.svc.cluster.local
```

**Output:**
```
; <<>> DiG 9.16.1-Ubuntu <<>> api.services.svc.cluster.local
;; ->>HEADER<<- opcode: QUERY, status: NOERROR, id: 12345
;; flags: qr rd; QUERY: 1, ANSWER: 1, AUTHORITY: 0, ADDITIONAL: 0

;; QUESTION SECTION:
;api.services.svc.cluster.local. IN    A

;; ANSWER SECTION:
api.services.svc.cluster.local. 30 IN  A       10.97.45.123

;; Query time: 2 msec
```

Key sections:

- **QUESTION**: What are we looking up? (A record for IP address)
- **ANSWER**: The IP address (10.97.45.123)
- **Query time**: DNS resolution took 2 milliseconds (fast = healthy)

### SRV Records: Service Records for Load Balancing

For headless services, CoreDNS creates SRV records that list all Pod IPs:

```bash
dig SRV api.services.svc.cluster.local
```

**Output:**
```
;; ANSWER SECTION:
api.services.svc.cluster.local. 30 IN  SRV     0 100 8080 api-5f7d8c4f9z.api.services.svc.cluster.local.
api.services.svc.cluster.local. 30 IN  SRV     0 100 8080 api-5f7d8c4f9a.api.services.svc.cluster.local.

;; ADDITIONAL SECTION:
api-5f7d8c4f9z.api.services.svc.cluster.local. 30 IN A 10.244.0.5
api-5f7d8c4f9a.api.services.svc.cluster.local. 30 IN A 10.244.0.6
```

SRV records include:

- **Port number** (8080)
- **Pod-specific FQDNs** (api-5f7d8c4f9z.api.services.svc.cluster.local)
- **Pod IPs** (10.244.0.5, 10.244.0.6)

This allows clients to connect directly to Pods instead of going through the Service's load balancer.

---

## Headless Services: Direct Pod Discovery

Some applications need to connect directly to Pods, not through a load-balanced Service. Database replication, peer discovery, and stateful applications all require this. That's where headless services come in.

### ClusterIP vs Headless

A **regular Service** (ClusterIP):

```yaml
apiVersion: v1
kind: Service
metadata:
  name: api
  namespace: services
spec:
  selector:
    app: api
  ports:
    - port: 8080
      targetPort: 8080
```

Returns a **single virtual IP**:

```bash
kubectl get svc -n services api
```

**Output:**
```
NAME   TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)
api    ClusterIP   10.97.45.123    <none>        8080/TCP
```

A **headless Service** (no ClusterIP):

```yaml
apiVersion: v1
kind: Service
metadata:
  name: db
  namespace: databases
spec:
  clusterIP: None  # This makes it headless
  selector:
    app: postgres
  ports:
    - port: 5432
      targetPort: 5432
```

Returns **Pod IPs directly**:

```bash
kubectl get svc -n databases db
```

**Output:**
```
NAME   TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)
db     ClusterIP   None         <none>        5432/TCP
```

### DNS Resolution: Headless Service

Query a headless service:

```bash
dig db.databases.svc.cluster.local
```

**Output:**
```
;; ANSWER SECTION:
db.databases.svc.cluster.local. 30 IN  A       10.244.1.5
db.databases.svc.cluster.local. 30 IN  A       10.244.1.6
db.databases.svc.cluster.local. 30 IN  A       10.244.1.7
```

Instead of one virtual IP, DNS returns **all Pod IPs** that match the selector. Clients can connect to any Pod directly.

---

## Troubleshooting Endpoint Mismatches

When DNS resolves but connections fail, the problem is usually **endpoint mismatch**: the Service selector doesn't match the Pod labels.

### Discovering Endpoints

List endpoints for a service:

```bash
kubectl get endpoints api -n services
```

**Output (Healthy):**
```
NAME   ENDPOINTS           AGE
api    10.244.0.5:8080     5m
```

The Service knows about one Pod at IP 10.244.0.5.

**Output (No Endpoints):**
```
NAME   ENDPOINTS   AGE
api    <none>      5m
```

The Service found zero Pods. This means the selector isn't matching any Pods.

### Debugging Selector Mismatch

Check the Service's selector:

```bash
kubectl get svc api -n services -o jsonpath='{.spec.selector}'
```

**Output:**
```
{"app":"api"}
```

The Service looks for Pods with label `app: api`.

Check Pods in the namespace:

```bash
kubectl get pods -n services --show-labels
```

**Output (Mismatch):**
```
NAME            STATUS   LABELS
api-5f7d8c      Running  app=backend,version=v1
api-deployment  Running  app=api-service,version=v1
```

Neither Pod has `app: api`. The selector `app: api` matches zero Pods.

**Output (Match):**
```
NAME            STATUS   LABELS
api-5f7d8c      Running  app=api,version=v1
api-deployment  Running  app=api,version=v1
```

Both Pods have `app: api`. Now check endpoints again:

```bash
kubectl get endpoints api -n services
```

**Output:**
```
NAME   ENDPOINTS                    AGE
api    10.244.0.5:8080,10.244.0.6:8080
```

Both Pods are now listed as endpoints.

### Manual Label Verification

Use `kubectl get pods` with label selector syntax:

```bash
kubectl get pods -n services -l app=api
```

**Output (No Results):**
```
No resources found in services namespace.
```

The selector matches zero Pods.

**Output (Match):**
```
NAME            READY   STATUS    RESTARTS   AGE
api-5f7d8c      1/1     Running   0          5m
api-deployment  1/1     Running   0          3m
```

The selector matches Pods successfully. Now endpoints should be populated.

---

## Cross-Namespace Service Discovery

Services in different namespaces don't see each other by default. You must use the FQDN to reach across namespaces.

### Accessing Services Across Namespaces

From a Pod in `agents` namespace accessing `api` service in `services` namespace:

```bash
# Short name (fails - wrong namespace)
curl http://api:8080/health
# Error: Could not resolve host: api

# Full FQDN (succeeds)
curl http://api.services.svc.cluster.local:8080/health
# HTTP/1.1 200 OK
```

CoreDNS includes the full FQDN in all service discovery lookups. The FQDN is always correct.

### Default Namespace Assumption

Pods assume their own namespace when resolving short names. CoreDNS tries:

1. `servicename` (fails immediately—not a valid FQDN)
2. `servicename.namespace.svc.cluster.local` (finds the service in its own namespace)
3. Falls back through search domains if configured

This is why short names only work in-namespace.

---

## Connectivity Test: From Debugging to Validation

Let's walk through a complete debugging scenario, from failure to root cause to fix.

### Scenario: Agent A Calls Agent B

**Setup:**

- Agent A Pod in `agents` namespace
- Agent B Service in `services` namespace
- Agent A can't reach Agent B

**Step 1: Verify Service Exists**

```bash
kubectl get svc -n services
```

**Output:**
```
NAME    TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)
agent-b ClusterIP   10.97.123.45     <none>        8080/TCP
```

Service exists. Step 2: Check endpoints.

**Step 2: Verify Endpoints**

```bash
kubectl get endpoints agent-b -n services
```

**Output:**
```
NAME      ENDPOINTS   AGE
agent-b   <none>      10m
```

No endpoints. The Service has no Pods. Step 3: Check selectors.

**Step 3: Check Service Selector**

```bash
kubectl get svc agent-b -n services -o jsonpath='{.spec.selector}'
```

**Output:**
```
{"app":"agent-b","version":"v1"}
```

Step 4: Check if Pods match.

**Step 4: List Pods with Required Labels**

```bash
kubectl get pods -n services -l app=agent-b,version=v1
```

**Output:**
```
No resources found in services namespace.
```

No Pods match. Step 5: Check what Pods exist.

**Step 5: List All Pods in Namespace**

```bash
kubectl get pods -n services --show-labels
```

**Output:**
```
NAME             READY   STATUS    LABELS
agent-b-5f7d8c   1/1     Running   app=agent-b,version=v2
```

The Pod has label `version=v2` but the Service requires `version=v1`.

**Root Cause:** Label mismatch.

**Fix:** Update Deployment to match Service selector:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-b
  namespace: services
spec:
  selector:
    matchLabels:
      app: agent-b
      version: v1  # Match Service selector
  template:
    metadata:
      labels:
        app: agent-b
        version: v1  # Match Service selector
    spec:
      containers:
      - name: agent-b
        image: myregistry.azurecr.io/agent-b:latest
        ports:
        - containerPort: 8080
```

Apply and verify:

```bash
kubectl apply -f agent-b-deployment.yaml

kubectl get endpoints agent-b -n services
```

**Output:**
```
NAME      ENDPOINTS              AGE
agent-b   10.244.0.10:8080       2m
```

Endpoints now populated. Test from Agent A:

```bash
kubectl run -it agent-a-test --image=curlimages/curl --rm -- \
  curl http://agent-b.services.svc.cluster.local:8080/health
```

**Output:**
```
HTTP/1.1 200 OK
{"status": "healthy"}
```

Success.

---

## Try With AI

**Setup**: You have three microservices in Kubernetes:

- `auth-service` in `auth` namespace (handles authentication)
- `api-gateway` in `api` namespace (routes requests)
- `user-db` in `databases` namespace (PostgreSQL database)

Both `api-gateway` and `auth-service` are running, but `api-gateway` cannot reach `auth-service`. Connections time out.

**Part 1: Manual Diagnosis**

Before asking AI, diagnose manually:

1. Verify `auth-service` exists:
   ```bash
   kubectl get svc -n auth auth-service
   ```

2. Check endpoints:
   ```bash
   kubectl get endpoints auth-service -n auth
   ```

3. Query DNS from a debug Pod:
   ```bash
   kubectl run -it debug --image=busybox:1.35 --rm --restart=Never -- \
     nslookup auth-service.auth.svc.cluster.local
   ```

4. Check the Service selector:
   ```bash
   kubectl get svc auth-service -n auth -o jsonpath='{.spec.selector}'
   ```

5. List Pods in the namespace and compare labels:
   ```bash
   kubectl get pods -n auth --show-labels
   ```

**Part 2: Gathering Evidence**

Before consulting AI, collect:

- Service definition (YAML)
- Endpoint status
- Pod labels
- DNS query results
- Error message from API Gateway logs

This evidence determines whether the issue is DNS, endpoints, or labels.

**Part 3: Collaboration with AI**

Once you've gathered evidence, ask AI:

"I have `auth-service` in the `auth` namespace. DNS resolves `auth-service.auth.svc.cluster.local` to 10.97.50.100, but `api-gateway` in the `api` namespace gets connection timeouts. Here's the Service definition [paste YAML], the endpoints output shows `<none>`, and the Pods in the namespace have labels [paste labels]. What's the likely root cause?"

AI will identify the mismatch between Service selector and Pod labels, or point out that endpoints are missing entirely.

**Part 4: Validation**

Test your fix:

```bash
kubectl run -it api-test --image=curlimages/curl --rm -- \
  curl -v http://auth-service.auth.svc.cluster.local:8080/health
```

Connection should succeed (HTTP 200 or appropriate response).

Compare your manual diagnosis to AI's analysis. What did you miss? What did AI catch that you didn't?
