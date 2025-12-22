---
sidebar_position: 50
title: "Chapter 50: Kubernetes for AI Services"
description: "Deploy and orchestrate AI agents on Kubernetes clusters"
---

# Chapter 50: Kubernetes for AI Services

Docker gives you portable containers. Kubernetes orchestrates them—handling deployment, scaling, networking, and self-healing automatically. When your agent container crashes, Kubernetes restarts it. When traffic spikes, Kubernetes scales it. When you push a new version, Kubernetes rolls it out safely.

This chapter teaches Kubernetes comprehensively, covering everything from core primitives to production-ready patterns for AI services. You'll learn the declarative model that makes Kubernetes powerful, understand how the control plane manages desired state, and master the kubectl commands that make operations automated and repeatable.

By the end, your containerized FastAPI agent from Chapter 49 will be running on a Kubernetes cluster with proper health checks, resource management, security policies, and automated scaling—all without manual intervention.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand Kubernetes architecture**: Control plane, worker nodes, and the declarative model
- **Deploy containers on Kubernetes**: Pods, Deployments, ReplicaSets, and lifecycle management
- **Use advanced Pod patterns**: Init containers for setup, sidecars for logging and monitoring
- **Organize workloads**: Namespaces for isolation, resource quotas for multi-tenancy
- **Expose services**: ClusterIP, NodePort, LoadBalancer, Ingress for HTTP routing
- **Configure applications**: ConfigMaps for configuration, Secrets for sensitive data
- **Persist data**: Persistent Volumes, Claims, StorageClasses, StatefulSets for stateful workloads
- **Manage resources**: CPU/memory requests and limits, Horizontal Pod Autoscaler
- **Secure deployments**: RBAC, SecurityContext, NetworkPolicy, Pod Security Standards
- **Monitor health**: Liveness, readiness, and startup probes
- **Use kubectl-ai**: AI-assisted manifest generation and cluster operations
- **Package with Helm**: Charts, templates, and release management
- **Deploy agents at scale**: Rolling updates, horizontal scaling, self-healing patterns

## Chapter Structure

| Lesson | Title | Layer | Focus |
|--------|-------|-------|-------|
| 1 | Kubernetes Architecture & the Declarative Model | L1 | Control plane, workers, desired vs observed state |
| 2 | Setting Up Minikube | L1 | Installation, cluster creation, kubectl context |
| 3 | Pods: The Atomic Unit | L1 | Pod anatomy, YAML, lifecycle, multi-container |
| 4 | Deployments: Self-Healing at Scale | L1 | ReplicaSets, rolling updates, rollbacks |
| 5 | Services & Networking | L1 | ClusterIP, NodePort, LoadBalancer, DNS |
| 6 | Init Containers: Preparing Your Environment | L1 | Initialization patterns, dependency setup |
| 7 | Sidecar Containers: Your Agent's Best Friend | L1 | Native sidecars (K8s 1.28+), logging, metrics |
| 8 | Namespaces: Virtual Clusters | L1 | Isolation, ResourceQuotas, LimitRanges |
| 9 | Ingress: Exposing Your Agent to the World | L1 | Path/host routing, TLS, annotations |
| 10 | Service Discovery Deep Dive | L1 | CoreDNS, FQDN, headless services |
| 11 | ConfigMaps & Secrets | L1 | Configuration injection, security notes |
| 12 | Persistent Storage: PV and PVC | L1 | Storage lifecycle, access modes, StorageClass |
| 13 | StatefulSets: When Your Agent Needs Identity | L1 | Stable identity, volumeClaimTemplates |
| 14 | Resource Management & Debugging | L1 | Requests/limits, QoS, kubectl debug |
| 15 | Horizontal Pod Autoscaler | L1 | Metrics-server, CPU/memory scaling, behavior |
| 16 | RBAC: Securing Agent Deployments | L1 | ServiceAccount, Role, RoleBinding |
| 17 | Kubernetes Security for AI Services | L1 | SecurityContext, NetworkPolicy, Pod Security |
| 18 | Health Checks & Probes | L1 | Liveness, readiness, startup probes |
| 19 | AI-Assisted K8s with kubectl-ai | L2 | Natural language to manifests, debugging |
| 20 | Helm Charts for AI Agent Packaging | L1 | Charts, templates, releases |
| 21 | Capstone: Deploy Your Agent to Kubernetes | L4 | Spec-driven deployment, full validation |
| 22 | Building the Kubernetes Deployment Skill | L3 | Persona + Questions + Principles |

## 4-Layer Teaching Progression

This chapter follows the **4-Layer Teaching Method**:

- **Lessons 1-18, 20 (Layer 1)**: Build mental models of Kubernetes concepts manually before AI assistance. This includes core primitives (Pods, Deployments, Services), advanced patterns (init containers, sidecars, StatefulSets), networking (Ingress, service discovery), storage, security (RBAC, NetworkPolicy), autoscaling, and Helm packaging.

- **Lesson 19 (Layer 2)**: Collaborate with kubectl-ai using Three Roles to translate natural language into cluster operations. By this point, you have deep Kubernetes knowledge to evaluate AI-generated manifests critically.

- **Lesson 21 (Layer 4)**: Apply all lessons in a spec-driven capstone project. Deploy your Part 6 FastAPI agent with proper configuration, security, health checks, and resource management.

- **Lesson 22 (Layer 3)**: Create reusable intelligence—a Kubernetes deployment skill that compounds across cloud-native projects.

## Prerequisites

- **Chapter 49 completion**: A containerized FastAPI agent pushed to a registry—this is the container you'll deploy to Kubernetes
- **Docker familiarity**: You should understand images, containers, and how to run containers locally
- **Basic command-line comfort**: You should be comfortable running commands in a terminal and navigating file systems
- **No Kubernetes experience required**: Lesson 1 explains cluster architecture and the declarative model from scratch

## Your Part 6 Agent: The Thread Through This Chapter

Throughout this chapter, we deploy your Part 6 FastAPI agent to Kubernetes:

- **Lessons 1-18**: Learn Kubernetes concepts using your agent as the running example—from basic Pod deployment to production-ready configurations with security, autoscaling, and health checks
- **Lesson 19**: Use kubectl-ai to generate deployment manifests and troubleshoot cluster issues collaboratively
- **Lesson 20**: Package your agent as a Helm chart for repeatable, environment-specific deployments
- **Lesson 21 (Capstone)**: Deploy your containerized Part 6 agent to a Kubernetes cluster with all production patterns applied
- **Lesson 22**: Create a reusable skill for future Kubernetes deployment work

By the end, your agent runs on an orchestrated cluster with automatic scaling, self-healing, proper security, and rolling updates.

## Looking Ahead

This chapter produces a deployed agent and a reusable Kubernetes deployment skill. Chapter 51 (Kafka for AI Events) builds on this foundation with event-driven architectures for AI agent communication.
