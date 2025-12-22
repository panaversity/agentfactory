---
sidebar_position: 9
chapter: 50
lesson: 9
duration_minutes: 45
title: "Ingress: Exposing Your Agent to the World"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual Ingress configuration builds understanding of external access patterns"
cognitive_load:
  concepts_count: 10
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain difference between Ingress and LoadBalancer Services"
    bloom_level: "Understand"
  - id: LO2
    description: "Install an Ingress controller (nginx-ingress on Minikube)"
    bloom_level: "Apply"
  - id: LO3
    description: "Create Ingress resources with path-based routing"
    bloom_level: "Apply"
  - id: LO4
    description: "Create Ingress resources with host-based routing"
    bloom_level: "Apply"
  - id: LO5
    description: "Configure TLS termination for HTTPS"
    bloom_level: "Apply"
  - id: LO6
    description: "Route traffic to multiple agent versions for A/B testing"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO3
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "2. Communication and Collaboration"
    competency: "2.6 Managing digital identity"
  - objective_id: LO6
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools creatively"
---

# Ingress: Exposing Your Agent to the World

Your agent running in Kubernetes handles thousands of requests. But how does traffic reach it? In Lesson 5, you learned about Services—the stable interfaces to your Pods. LoadBalancer Services work well for simple cases, but production systems need something more powerful: **Ingress**.

Ingress lets you expose HTTP and HTTPS routes from outside the cluster to services within it. Unlike LoadBalancer, which creates a cloud load balancer for every service (expensive), Ingress shares one load balancer across multiple services. You can route requests based on hostname, URL path, or both. You can terminate TLS for HTTPS. You can even implement A/B testing by routing different traffic percentages to different versions of your agent.

Think of it this way: LoadBalancer is a direct tunnel to one service. Ingress is an intelligent receptionist—it looks at your request, reads the address and directions, and routes you to the right department.

## Why LoadBalancer Isn't Enough

**The Cost Problem**

Every LoadBalancer Service you create provisions a cloud load balancer. On AWS, that's $16/month per load balancer. With 10 services, you're paying $160/month just for load balancers. Ingress shares one load balancer across all services—one $16/month bill instead of ten.

**The Feature Gap**

LoadBalancer gives you Layer 4 routing (TCP/UDP based on port). It knows nothing about HTTP. Ingress gives you Layer 7 routing:

- Route `/api/v1/` requests to the stable agent version
- Route `/api/v2/beta/` requests to the experimental agent
- Route `dashboard.example.com` to monitoring, `api.example.com` to your agent
- Terminate TLS for HTTPS
- Implement request rewriting
- Control access with rate limiting

**The Management Problem**

With LoadBalancer, every service is exposed separately. You're managing multiple external IPs, each with different security rules. With Ingress, you have one entrypoint—one place to manage TLS certificates, one place to enforce security policies.

## Ingress Architecture: Controller and Resource

Ingress has two parts: a **resource** (declarative specification) and a **controller** (the process that implements it).

### The Ingress Resource

The Ingress resource is a Kubernetes API object—like Deployment or Service. It specifies your desired routing rules:

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: agent-ingress
spec:
  ingressClassName: nginx
  rules:
  - host: api.example.com
    http:
      paths:
      - path: /api/v1/
        backend:
          service:
            name: agent-stable
            port:
              number: 8000
      - path: /api/v2/beta/
        backend:
          service:
            name: agent-experimental
            port:
              number: 8000
```

This says: "Listen on api.example.com, route requests to `/api/v1/` to the agent-stable service, route requests to `/api/v2/beta/` to the agent-experimental service."

The resource is just a specification. It does nothing by itself.

### The Ingress Controller

The **Ingress controller** reads Ingress resources and implements them. It's a Pod running in your cluster that watches for Ingress resources, then configures a real load balancer (nginx, HAProxy, cloud provider's LB) to actually route traffic.

Popular controllers:

- **nginx-ingress** (open source, works everywhere)
- **AWS ALB** (AWS-native, integrates with ELBv2)
- **GCP Cloud Armor** (GCP-native)
- **Kong** (commercial-grade API gateway)

For this chapter, we'll use nginx-ingress because it works on Minikube, cloud clusters, and on-premises Kubernetes.

## Installing nginx-ingress on Minikube

Minikube provides an addon to install nginx-ingress automatically. Let's enable it:

```bash
minikube addons enable ingress
```

**Output:**
```
    Verifying ingress addon...
    The 'ingress' addon is enabled
```

Verify the controller is running:

```bash
kubectl get pods -n ingress-nginx
```

**Output:**
```
NAMESPACE       NAME                                       READY   STATUS    RESTARTS   AGE
ingress-nginx   ingress-nginx-admission-patch-xxxxx        0/1     Completed   0          2m
ingress-nginx   ingress-nginx-admission-create-xxxxx       0/1     Completed   0          2m
ingress-nginx   ingress-nginx-controller-yyyyy             1/1     Running     0          2m
```

The `ingress-nginx-controller` is the daemon watching your cluster for Ingress resources. When you create an Ingress, this controller reads it and configures nginx to route traffic accordingly.

Check which IngressClass is available:

```bash
kubectl get ingressclasses
```

**Output:**
```
NAME    CONTROLLER                       AGE
nginx   k8s.io/ingress-nginx/nginx       2m
```

The `nginx` IngressClass is your controller. When you create an Ingress with `ingressClassName: nginx`, this controller takes responsibility for implementing it.

## Path-Based Routing: Versioned APIs

Your agent has evolved. You have a stable `/api/v1/` endpoint that clients rely on, and a new `/api/v2/beta/` endpoint with experimental features. Different Deployments run each version:

```bash
kubectl create deployment agent-v1 --image=your-registry/agent:v1 --replicas=2
kubectl create deployment agent-v2 --image=your-registry/agent:v2 --replicas=1
```

**Output:**
```
deployment.apps/agent-v1 created
deployment.apps/agent-v2 created
```

Expose each as a Service:

```bash
kubectl expose deployment agent-v1 --port=8000 --target-port=8000 --name=agent-v1-service
kubectl expose deployment agent-v2 --port=8000 --target-port=8000 --name=agent-v2-service
```

**Output:**
```
service/agent-v1-service exposed
service/agent-v2-service exposed
```

Verify both services exist:

```bash
kubectl get svc | grep agent
```

**Output:**
```
NAME                TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)
agent-v1-service   ClusterIP   10.96.200.50    <none>        8000/TCP
agent-v2-service   ClusterIP   10.96.201.100   <none>        8000/TCP
```

Now create an Ingress to route traffic to both:

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: agent-path-routing
spec:
  ingressClassName: nginx
  rules:
  - host: localhost  # For Minikube testing
    http:
      paths:
      - path: /api/v1
        pathType: Prefix
        backend:
          service:
            name: agent-v1-service
            port:
              number: 8000
      - path: /api/v2/beta
        pathType: Prefix
        backend:
          service:
            name: agent-v2-service
            port:
              number: 8000
```

Save as `agent-path-routing.yaml` and apply:

```bash
kubectl apply -f agent-path-routing.yaml
```

**Output:**
```
ingress.networking.k8s.io/agent-path-routing created
```

Verify the Ingress is configured:

```bash
kubectl get ingress
```

**Output:**
```
NAME                   CLASS   HOSTS      ADDRESS        PORTS   AGE
agent-path-routing     nginx   localhost  192.168.49.2   80      10s
```

The ADDRESS is your Minikube node's IP. Wait a moment for nginx to configure, then test:

```bash
minikube service --url ingress-nginx -n ingress-nginx
```

**Output:**
```
http://192.168.49.2:80
```

```bash
curl http://192.168.49.2/api/v1/health
```

**Output:**
```
{"status": "healthy", "version": "1.0"}
```

```bash
curl http://192.168.49.2/api/v2/beta/health
```

**Output:**
```
{"status": "healthy", "version": "2.0-beta", "experimental": true}
```

The same Ingress gateway routes `/api/v1` and `/api/v2/beta` to different backend services. This is the power of path-based routing—one IP, multiple APIs, each backed by independent Deployments.

## Host-Based Routing: Multiple Domains

Your team operates multiple services from one cluster:

- `agent.example.com` — Your AI agent API
- `dashboard.example.com` — Monitoring dashboard
- `webhook.example.com` — Event receiver

Each has its own Service. Host-based Ingress routes traffic to the right Service based on the hostname:

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: agent-host-routing
spec:
  ingressClassName: nginx
  rules:
  - host: agent.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: agent-service
            port:
              number: 8000
  - host: dashboard.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: dashboard-service
            port:
              number: 3000
  - host: webhook.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: webhook-service
            port:
              number: 5000
```

Apply this configuration:

```bash
kubectl apply -f agent-host-routing.yaml
```

**Output:**
```
ingress.networking.k8s.io/agent-host-routing created
```

Verify the Ingress configuration:

```bash
kubectl get ingress agent-host-routing -o wide
```

**Output:**
```
NAME                   CLASS   HOSTS                                          ADDRESS        PORTS   AGE
agent-host-routing     nginx   agent.example.com,dashboard.example.com,...   192.168.49.2   80      30s
```

Test locally by modifying `/etc/hosts` (or adding entries to your DNS):

```bash
echo "192.168.49.2 agent.example.com dashboard.example.com webhook.example.com" >> /etc/hosts
cat /etc/hosts | tail -1
```

**Output:**
```
192.168.49.2 agent.example.com dashboard.example.com webhook.example.com
```

Then test each hostname:

```bash
curl http://agent.example.com/health
```

**Output:**
```
{"status": "healthy", "service": "agent", "uptime_seconds": 3600}
```

```bash
curl http://dashboard.example.com/
```

**Output:**
```
<html>Dashboard - 12 agents online, 0 errors</html>
```

The same Ingress controller routes three different hostnames to three different services. This is the foundation of multi-tenant deployments—one gateway, many applications.

## TLS Termination for HTTPS

Internet traffic should be encrypted. Kubernetes TLS termination means the Ingress handles encryption/decryption, so backend services communicate in plain HTTP internally (they're protected by the network).

### Create a Self-Signed Certificate (for testing)

```bash
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
  -keyout /tmp/tls.key \
  -out /tmp/tls.crt \
  -subj "/CN=agent.example.com"
```

**Output:**
```
Generating a RSA private key
...................................................+++++
Generating a self signed certificate
...
```

### Create a TLS Secret

Kubernetes stores certificates in Secrets. Create one with your TLS key and certificate:

```bash
kubectl create secret tls agent-tls \
  --cert=/tmp/tls.crt \
  --key=/tmp/tls.key
```

**Output:**
```
secret/agent-tls created
```

Verify the secret:

```bash
kubectl get secret agent-tls
```

**Output:**
```
NAME         TYPE                DATA   AGE
agent-tls    kubernetes.io/tls   2      5s
```

### Update Ingress with TLS

Modify your Ingress to reference the TLS secret:

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: agent-tls-ingress
spec:
  ingressClassName: nginx
  tls:
  - hosts:
    - agent.example.com
    secretName: agent-tls
  rules:
  - host: agent.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: agent-service
            port:
              number: 8000
```

Apply:

```bash
kubectl apply -f agent-tls-ingress.yaml
```

**Output:**
```
ingress.networking.k8s.io/agent-tls-ingress created
```

Test HTTPS:

```bash
curl --insecure https://agent.example.com/health
```

**Output:**
```
{"status": "healthy", "tls": "yes"}
```

(We use `--insecure` because the self-signed certificate isn't in your system's trust store. In production, you'd use a certificate from a trusted CA like Let's Encrypt.)

The Ingress controller (nginx) terminates TLS—it decrypts incoming HTTPS traffic and routes requests to your services over plain HTTP. This simplifies certificate management (one place to update certs) and reduces computational burden on your agent services.

Verify the TLS secret is mounted correctly:

```bash
kubectl describe ingress agent-tls-ingress
```

**Output:**
```
Name:             agent-tls-ingress
Namespace:        default
Address:          192.168.49.2
Default backend:  default-http-backend:80 (<error: endpoints "default-http-backend" not found>)
TLS:
  agent-tls terminates agent.example.com
Rules:
  Host                   Path  Backends
  ----                   ----  --------
  agent.example.com
                         /     agent-service:8000 (10.244.0.10:8000,10.244.0.11:8000)
Annotations:
  <none>
Events:                  <none>
```

The TLS configuration is active. The nginx controller has loaded your certificate and key from the `agent-tls` secret.

## A/B Testing with Traffic Splitting

Your team wants to validate a new agent version with 10% of traffic while keeping 90% on the stable version. You can't do this with basic routing—you need **weighted traffic splitting**.

Create two Services:

```bash
kubectl create deployment agent-stable --image=agent:v1.0 --replicas=9
kubectl create deployment agent-test --image=agent:v2.0-rc1 --replicas=1
kubectl expose deployment agent-stable --port=8000 --name=agent-stable
kubectl expose deployment agent-test --port=8000 --name=agent-test
```

**Output:**
```
deployment.apps/agent-stable created
deployment.apps/agent-test created
service/agent-stable exposed
service/agent-test exposed
```

With nginx-ingress, you can use the `nginx.ingress.kubernetes.io/service-weights` annotation:

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: agent-canary
  annotations:
    nginx.ingress.kubernetes.io/service-weights: |
      agent-stable: 90
      agent-test: 10
spec:
  ingressClassName: nginx
  rules:
  - host: api.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: agent-stable
            port:
              number: 8000
```

Apply:

```bash
kubectl apply -f agent-canary.yaml
```

**Output:**
```
ingress.networking.k8s.io/agent-canary created
```

Send 100 requests and observe traffic distribution:

```bash
for i in {1..100}; do curl http://api.example.com/version; done | sort | uniq -c
```

**Output:**
```
      90 {"version": "1.0", "stable": true}
      10 {"version": "2.0-rc1", "canary": true}
```

Roughly 90% reach the stable version, 10% reach the test version. This lets you validate new code with real traffic before full rollout.

## Ingress Annotations: Advanced Configuration

Annotations let you customize Ingress behavior without changing the core specification. Common annotations for nginx-ingress:

### Rate Limiting

Protect your agent from being overwhelmed by limiting requests per client:

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: agent-rate-limited
  annotations:
    nginx.ingress.kubernetes.io/limit-rps: "100"
    nginx.ingress.kubernetes.io/limit-connections: "10"
spec:
  ingressClassName: nginx
  rules:
  - host: api.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: agent-service
            port:
              number: 8000
```

Apply and test:

```bash
kubectl apply -f agent-rate-limited.yaml
```

**Output:**
```
ingress.networking.k8s.io/agent-rate-limited created
```

Verify the annotations are applied:

```bash
kubectl get ingress agent-rate-limited -o jsonpath='{.metadata.annotations}'
```

**Output:**
```
{"nginx.ingress.kubernetes.io/limit-connections":"10","nginx.ingress.kubernetes.io/limit-rps":"100"}
```

### CORS Headers

Allow browser clients from specific origins to call your agent:

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: agent-cors
  annotations:
    nginx.ingress.kubernetes.io/enable-cors: "true"
    nginx.ingress.kubernetes.io/cors-allow-origin: "https://dashboard.example.com"
    nginx.ingress.kubernetes.io/cors-allow-methods: "GET, POST, PUT"
spec:
  ingressClassName: nginx
  rules:
  - host: api.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: agent-service
            port:
              number: 8000
```

Apply:

```bash
kubectl apply -f agent-cors.yaml
```

**Output:**
```
ingress.networking.k8s.io/agent-cors created
```

## Debugging Ingress Issues

When routing fails, use these kubectl commands to diagnose:

### Check Ingress Status

```bash
kubectl get ingress
```

**Output:**
```
NAME              CLASS   HOSTS              ADDRESS        PORTS   AGE
agent-ingress     nginx   api.example.com    192.168.49.2   80      5m
```

If ADDRESS is `<none>`, the ingress controller hasn't assigned an IP yet (usually means services don't exist).

### Describe Ingress Details

```bash
kubectl describe ingress agent-ingress
```

**Output:**
```
Name:             agent-ingress
Namespace:        default
Address:          192.168.49.2
Ingress Class:    nginx
Host(s)           Path  Backends
----              ----  --------
api.example.com
                  /api/v1    agent-v1-service:8000
                  /api/v2    agent-v2-service:8000
Annotations:      <none>
Events:
  Type    Reason  Age   From                      Message
  ----    ------  ----  ----                      -------
  Normal  Sync    30s   nginx-ingress-controller  Scheduled for sync
```

This shows exactly what rules are configured and which backends they target.

### Check Controller Logs

```bash
kubectl logs -n ingress-nginx -l app.kubernetes.io/name=ingress-nginx -f
```

**Output (example):**
```
I1222 10:45:23.123456   12345 main.go:104] Starting NGINX Ingress Controller
I1222 10:45:24.234567   12345 store.go:123] Found Ingress api.example.com
I1222 10:45:25.345678   12345 controller.go:456] Sync: agent-ingress
I1222 10:45:26.456789   12345 template.go:789] Generating NGINX config
```

Logs show when the controller detects new Ingress resources and updates its configuration.

### Test Connectivity

If the Ingress won't route traffic, verify the backend Service:

```bash
kubectl port-forward svc/agent-v1-service 8000:8000
```

In another terminal:

```bash
curl http://localhost:8000/health
```

**Output:**
```
{"status": "healthy", "version": "1.0"}
```

If this works but Ingress routing doesn't, the problem is in the Ingress controller configuration, not the Service.

### Common Issues and Solutions

**Issue: "Service not found" errors in Ingress**

Check the Service exists in the correct namespace:

```bash
kubectl get svc agent-v1-service
```

**Output:**
```
NAME                TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)
agent-v1-service    ClusterIP   10.96.200.50    <none>        8000/TCP
```

**Issue: 503 Service Unavailable**

The Ingress exists but backends are unhealthy:

```bash
kubectl describe svc agent-v1-service
```

**Output:**
```
Name:              agent-v1-service
Endpoints:         10.244.0.10:8000,10.244.0.11:8000
```

If Endpoints is empty, Pods aren't running or labels don't match.

## Try With AI

**Setup**: You have two agent services running in your cluster: `chat-agent` and `tool-agent`. Both listen on port 8000. You want to expose them via Ingress so:

- `http://api.example.com/chat/*` routes to `chat-agent`
- `http://api.example.com/tools/*` routes to `tool-agent`
- Both should be accessible over HTTPS using a self-signed certificate

**Part 1: Ask AI for the Ingress design**

Prompt AI:

```
I have two Kubernetes services:
- chat-agent (port 8000)
- tool-agent (port 8000)

I need an Ingress that:
1. Routes /chat/* to chat-agent
2. Routes /tools/* to tool-agent
3. Uses HTTPS with a TLS secret named "api-tls"
4. Uses nginx-ingress controller

Design the Ingress YAML that accomplishes this. What decisions did you make about pathType, backend configuration, and TLS placement?
```

**Part 2: Evaluate the design**

Review AI's response. Ask yourself:

- Does each path rule specify correct service and port?
- Is the TLS configuration in the spec.tls section?
- Did AI explain why certain pathTypes (Exact vs Prefix) were chosen?
- Are both hosts unified under one Ingress or separate Ingresses?

**Part 3: Test the design**

Create the services (if not already running):

```bash
kubectl create deployment chat-agent --image=your-registry/chat-agent:latest
kubectl create deployment tool-agent --image=your-registry/tool-agent:latest
kubectl expose deployment chat-agent --port=8000 --name=chat-agent
kubectl expose deployment tool-agent --port=8000 --name=tool-agent
```

Create the TLS secret:

```bash
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
  -keyout /tmp/api.key -out /tmp/api.crt -subj "/CN=api.example.com"
kubectl create secret tls api-tls --cert=/tmp/api.crt --key=/tmp/api.key
```

Apply AI's Ingress:

```bash
kubectl apply -f ingress.yaml  # The YAML AI generated
```

Test routing:

```bash
curl --insecure https://api.example.com/chat/hello
curl --insecure https://api.example.com/tools/list
```

**Part 4: Refinement**

If routing works:

- What annotations might improve this Ingress (rate limiting, CORS handling)?
- How would you add health checks to ensure failed backends are removed?

If routing fails:

- Check logs: `kubectl logs -n ingress-nginx <ingress-controller-pod>`
- Verify services exist: `kubectl get svc`
- Test Service connectivity directly: `kubectl port-forward svc/chat-agent 8000:8000`
- Verify TLS secret: `kubectl get secret api-tls`

**Part 5: Compare to your design**

When you started, you might have created separate Ingresses per service. Look at AI's consolidated design:

- Why is one Ingress cleaner than two?
- What shared configuration (TLS, IngressClass) benefits from consolidation?
- When would you split into multiple Ingresses (different namespaces, different IngressClasses)?
