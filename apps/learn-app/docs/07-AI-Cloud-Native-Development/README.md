---
sidebar_position: 7
title: "Part 7: AI Cloud Native Development"
---

# Part 7: AI Cloud Native Development

You've built AI agents locally in Parts 1-6—designing agent architectures, implementing integrations, and mastering spec-driven development. Part 7 bridges local development to production deployment at scale. You'll learn containerization strategies, orchestration patterns, and operational excellence practices that transform agents from learning projects into production systems serving real users.

**Prerequisites**: Parts 4-5 (SDD-RI fundamentals and Python fundamentals) are required. Part 6 (AI Native Software Development) is strongly recommended for agent building experience before tackling deployment.

---

## Goals

By completing Part 7, you will:

- **Understand agent deployment architectures**: Learn how containerized agents, orchestration platforms (Kubernetes, Dapr), and infrastructure patterns enable production-scale systems
- **Implement containerization strategies**: Package agents with Docker, manage multi-stage builds, and optimize images for deployment efficiency
- **Apply orchestration patterns**: Deploy agents to Kubernetes clusters, manage state with Dapr Actors, and orchestrate workflows with Dapr's distributed application runtime
- **Design operational excellence**: Build observability into agent systems (metrics, logs, traces), engineer for cost optimization, and implement security governance for production deployments

---

## Chapter Progression

Part 7's chapters build deployment capability through four stages:

### Foundational Infrastructure

Start with production API patterns and containerization fundamentals.

- **FastAPI for AI Cloud-Native Services (Deep Dive)** — Advanced API patterns, async processing, dependency injection, and deployment configurations
- **Docker for AI Services** — Building container images, multi-stage builds, optimization strategies, and shipping agents as containers
- **Apache Kafka for Event-Driven AI Systems** — Event streaming for agent communication, message patterns, and asynchronous architectures

### Orchestration

Scale from single containers to orchestrated clusters with deployment automation.

- **Kubernetes for AI Services** — Container orchestration, deployments, services, config management, and running agents in K8s clusters
- **CI/CD & Infrastructure-as-Code for AI Services** — Automated pipelines, infrastructure provisioning, GitOps patterns, and deployment workflows

### Dapr Framework

Apply Dapr's distributed application runtime for microservices, state, workflows, and agent-specific patterns.

- **Dapr for AI Microservices** — Sidecar building blocks, service invocation, pub/sub messaging, and state management
- **Dapr Actors for Agentic State and Concurrency** — Virtual actor pattern for agent state, concurrency control, and distributed coordination
- **Dapr Workflows for Long-Running Orchestration** — Durable workflows, saga patterns, and long-running agent processes
- **Dapr Agents** — Designing agentic services on Dapr, combining actors + workflows + AI for production agent architectures

### Operations Excellence

Operate agent systems with observability, security, and cost engineering.

- **Observability, Cost & Performance Engineering** — Metrics, logs, traces, cost optimization strategies, and performance tuning for agent workloads
- **API Edge & Gateway for AI Services** — Ingress patterns, Kong gateway configuration, rate limiting, and API management
- **Security, Safety & Governance for Agentic Systems** — Authentication, authorization, secrets management, safety guardrails, and compliance

**Why this sequence?** You can't orchestrate what you can't containerize. You can't operate what you haven't deployed. Each stage builds deployment capability: Foundation (can containerize agents) → Orchestration (can deploy to clusters) → Dapr Framework (can build distributed agent systems) → Operations (can run production systems reliably and securely).