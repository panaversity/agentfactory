### Core Concept
Horizontal Pod Autoscaler (HPA) automatically scales Pod replicas up and down based on metrics (CPU, memory, or custom). Scales up fast to respond to spikes, scales down slowly to avoid thrashing—a feedback loop that keeps your cluster right-sized to current demand.

### Key Mental Models
- **Metrics Flow**: kubelet measures actual usage → metrics-server aggregates → HPA reads metrics → adjusts replica count → Deployment creates/removes Pods
- **Stabilization Windows**: scaleUp has short/no window (respond immediately to spikes), scaleDown has long window (prevent reactive scaling on brief dips)
- **Scaling Math**: desired_replicas = ceil(current_replicas × (current_usage / target)), e.g., 2 Pods at 80% CPU with 50% target → need 4 Pods

### Critical Patterns
- Configure metrics-server first (on Minikube: minikube addons enable metrics-server) before creating HPAs—HPA needs monitoring data
- Set target utilization 50-80% (too low = over-provisioning costs, too high = slow spike response)
- Use longer scaleDown stabilization (300-600s) for bursty workloads; shorter (60-300s) for steady-state services
- Monitor HPA status with kubectl describe hpa to see scaling decisions and events

### Common Mistakes
- Creating HPA without metrics-server—HPA shows <unknown> metrics and can't scale
- Setting minReplicas too high or maxReplicas too low—constrains HPA's ability to balance demand
- Not adjusting initialDelaySeconds for slow-starting containers—causes scaling thrashing during startup
- Setting CPU target too tight without accounting for request/limit ratio

### Connections
- **Builds on**: Resource requests (Lesson 7) which HPA uses for calculations, Deployments (Lesson 4)
- **Leads to**: Custom metrics scaling for non-CPU signals (queue depth, inference latency), production cost optimization
