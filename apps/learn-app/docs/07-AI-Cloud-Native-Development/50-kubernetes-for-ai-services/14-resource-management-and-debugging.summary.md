### Core Concept
Resource requests and limits drive Kubernetes scheduling and eviction decisions. Requests determine where Pods can run (scheduling), limits prevent resource monopolization. QoS classes (Guaranteed, Burstable, BestEffort) prioritize which Pods get evicted under resource pressure.

### Key Mental Models
- **Requests vs Limits**: Requests are minimum guaranteed resources (scheduling constraint), limits are maximum allowed (enforcement boundary)
- **QoS Hierarchy**: Guaranteed (requests=limits, evicted last) > Burstable (requests<limits, evicted second) > BestEffort (no requests/limits, evicted first)
- **Debugging Signal Flow**: Status → Events → Logs → Interactive access reveals progressively deeper information about Pod failures

### Critical Patterns
- Set requests based on observed baseline usage from monitoring; set limits to 1.2-1.5x peak observed usage
- Use kubectl top to monitor actual CPU/memory consumption before calculating requests
- Debug using four-signal pattern: get pods (what state), describe (why that state), logs (what did app say), exec (investigate interactively)
- Create Pod with failureThreshold and periodSeconds together: total wait time = failureThreshold × periodSeconds

### Common Mistakes
- Setting limits equal to requests for non-critical workloads (wastes cluster capacity) or leaving limits unset (enables noisy neighbor problems)
- Over-requesting resources based on speculation instead of monitoring (kills cluster utilization)
- Ignoring OOMKilled and CrashLoopBackOff signals—they reveal specific root causes, not mysterious failures
- Setting readiness/liveness probe timeoutSeconds >= periodSeconds (causes overlapping probes and confusion)

### Connections
- **Builds on**: Pod resource configuration (Lesson 4), health checks (Lesson 8)
- **Leads to**: Autoscaling (Lesson 9) which uses requests/limits to calculate desired replicas
