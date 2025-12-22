---
sidebar_position: 50
title: "Chapter 50: Kubernetes for AI Services"
description: "Deploy and orchestrate AI agents on Kubernetes clusters"
---

# Chapter 50: Kubernetes for AI Services

Docker gives you portable containers. Kubernetes orchestrates them—handling deployment, scaling, networking, and self-healing automatically. When your agent container crashes, Kubernetes restarts it. When traffic spikes, Kubernetes scales it. When you push a new version, Kubernetes rolls it out safely.

This chapter teaches Kubernetes from first principles, with a focus on deploying AI services at scale. You'll learn the declarative model that makes Kubernetes powerful, understand how the control plane manages desired state, and master the kubectl commands that make operations automated and repeatable.

By the end, your containerized FastAPI agent from Chapter 49 will be running on a Kubernetes cluster, handling rolling updates, resource constraints, and automated recovery—all without manual intervention.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand Kubernetes architecture**: Control plane, worker nodes, and the declarative model
- **Deploy containers on Kubernetes**: Pods, Deployments, ReplicaSets, and lifecycle management
- **Expose services**: ClusterIP for internal communication, NodePort for external access, LoadBalancer for production
- **Configure applications**: ConfigMaps for configuration, Secrets for sensitive data, environment variables
- **Manage resources**: CPU and memory requests and limits, quality of service (QoS)
- **Debug clusters**: kubectl commands for inspecting logs, describing resources, and troubleshooting
- **Use kubectl-ai**: AI-assisted manifest generation and cluster operations
- **Set up Minikube**: Local Kubernetes cluster for development and testing
- **Deploy agents at scale**: Rolling updates, horizontal scaling, self-healing patterns

## Chapter Structure

| Lesson | Title | Layer | Focus |
|--------|-------|-------|-------|
| 1 | Kubernetes Architecture & the Declarative Model | L1 | Control plane, workers, desired vs observed state |
| 2 | Setting Up Minikube | L1 | Installation, cluster creation, kubectl context |
| 3 | Pods: The Atomic Unit | L1 | Pod anatomy, YAML, lifecycle, multi-container |
| 4 | Deployments: Self-Healing at Scale | L1 | ReplicaSets, rolling updates, rollbacks |
| 5 | Services & Networking | L1 | ClusterIP, NodePort, LoadBalancer, DNS |
| 6 | ConfigMaps & Secrets | L1 | Configuration injection, security notes |
| 7 | Resource Management & Debugging | L1 | Requests/limits, kubectl debug commands |
| 8 | AI-Assisted K8s with kubectl-ai | L2 | Natural language to manifests, debugging |
| 9 | Capstone: Deploy Your Agent to Kubernetes | L4 | Spec-driven deployment, full validation |
| 10 | Building the Kubernetes Deployment Skill | L3 | Persona + Questions + Principles |

## 4-Layer Teaching Progression

This chapter follows the **4-Layer Teaching Method**:

- **Lessons 1-7 (Layer 1)**: Build mental models of Kubernetes concepts manually before AI assistance
- **Lesson 8 (Layer 2)**: Collaborate with kubectl-ai using Three Roles (invisible framework) to translate natural language into cluster operations
- **Lesson 9 (Layer 4)**: Apply all lessons in spec-driven capstone project, validating deployment and resource management
- **Lesson 10 (Layer 3)**: Create reusable intelligence that compounds across cloud-native projects

## Prerequisites

- **Chapter 49 completion**: A containerized FastAPI agent pushed to a registry—this is the container you'll deploy to Kubernetes
- **Docker familiarity**: You should understand images, containers, and how to run containers locally
- **Basic command-line comfort**: You should be comfortable running commands in a terminal and navigating file systems
- **No Kubernetes experience required**: Lesson 1 explains cluster architecture and the declarative model from scratch

## Your Part 6 Agent: The Thread Through This Chapter

Throughout this chapter, we deploy your Part 6 FastAPI agent to Kubernetes:

- **Lessons 1-7**: Learn Kubernetes concepts using simplified examples and local development patterns
- **Lesson 8**: Use kubectl-ai to generate deployment manifests and troubleshoot cluster issues
- **Lesson 9 (Capstone)**: Deploy your containerized Part 6 agent to a Kubernetes cluster (Minikube for learning, cloud cluster for production)
- **Lesson 10**: Create a reusable skill for future Kubernetes deployment work

By the end, your agent runs on an orchestrated cluster with automatic scaling, self-healing, and rolling updates.

## Looking Ahead

This chapter produces a deployed agent and a reusable Kubernetes deployment skill. Chapter 51 (Helm) packages your deployments into reusable, versioned charts for repeatable releases across environments.
