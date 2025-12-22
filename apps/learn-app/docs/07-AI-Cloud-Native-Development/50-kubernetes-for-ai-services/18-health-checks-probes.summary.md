### Core Concept
Three health probe types answer different questions: Readiness (is this Pod ready for traffic?), Liveness (is this Pod still alive?), Startup (is initialization complete?). Together they enable Kubernetes to remove unready Pods from services and restart broken ones without operator intervention.

### Key Mental Models
- **Probe Independence**: Each probe has separate timing (initialDelaySeconds, periodSeconds, failureThreshold)—readiness checks frequently and reacts quickly; liveness checks slowly and tolerates transience
- **Timing Trade-off**: fast initialDelaySeconds catches problems quickly but risks false positives on slow-starting apps; slow values hide real problems
- **Probe Types**: HTTP (app-specific endpoint), TCP (port availability), Exec (shell commands) enable custom health checks for any workload

### Critical Patterns
- Implement three endpoints: /health/startup (busy-wait until model loads), /health/ready (returns 200 only when full dependencies ready), /health/live (always 200 if process alive)
- Set initialDelaySeconds to 1.5x actual startup time; set periodSeconds 5-10s for readiness (quick reaction), 10-30s for liveness (less frequent restarts)
- Set timeoutSeconds low (1-2s) for fast endpoints; set failureThreshold high for liveness (3-5) to tolerate transient failures
- Test probes by port-forwarding to the Pod and curling endpoints manually—discover timeout/response issues before deployment

### Common Mistakes
- Setting initialDelaySeconds too low for slow-starting apps (readiness fails while model loads, Pod removed from service prematurely)
- Using same probe endpoint for liveness and readiness (confused signals—readiness should be more conservative)
- Setting timeoutSeconds >= periodSeconds (causes overlapping probe executions and unpredictable behavior)
- Not configuring probes at all—running containers that are broken but never restarted

### Connections
- **Builds on**: Container health (Lesson 3), Service discovery (Lesson 5)
- **Leads to**: Production reliability (self-healing clusters), Deployment strategies (rolling updates require probes to verify readiness)
