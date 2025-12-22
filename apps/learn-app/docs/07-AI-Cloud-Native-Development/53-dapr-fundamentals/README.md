---
sidebar_position: 53
title: "Chapter 53: Dapr Fundamentals"
description: "Simplify distributed agent systems with Dapr's portable building blocks"
---

# Chapter 53: Dapr Fundamentals

You've learned Kafka directly. But what if you need to swap Kafka for RabbitMQ, or Redis, or AWS SNS? With raw clients, that's a code rewrite. Dapr (Distributed Application Runtime) abstracts these infrastructure dependencies behind portable HTTP/gRPC APIs. Your code talks to Dapr; Dapr talks to the infrastructure.

This chapter introduces Dapr's core building blocks: pub/sub, state management, service invocation, and secrets. You'll deploy Dapr on your Minikube cluster and refactor your agent to use Dapr's APIs instead of direct infrastructure clients.

The result: simpler code that's portable across infrastructures. Change Kafka to Redis by updating a YAML file, not your application code.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand Dapr architecture**: Sidecars, building blocks, and components
- **Deploy Dapr on Kubernetes**: Installation, configuration, and verification
- **Use Pub/Sub**: Publish and subscribe to events through Dapr
- **Use State Management**: Store and retrieve state without direct database code
- **Use Service Invocation**: Call services with built-in retries and mTLS
- **Use Secrets Management**: Access secrets from Kubernetes or external stores
- **Configure components**: Connect Dapr to Kafka, PostgreSQL, Redis, etc.
- **Swap backends**: Change infrastructure by updating YAML, not code

## Chapter Structure

1. **Dapr Architecture** — Sidecars, building blocks, and the component model
2. **Installing Dapr on Minikube** — CLI setup, Kubernetes deployment, and dashboard
3. **Pub/Sub Building Block** — Publishing and subscribing through Dapr
4. **State Management** — Storing conversation state and agent memory
5. **Service Invocation** — Calling between services with resilience
6. **Secrets Management** — Accessing API keys and credentials securely
7. **Component Configuration** — Connecting to Kafka, PostgreSQL, and Redis
8. **Capstone: Dapr-Enabled Agent** — Refactor your agent to use Dapr building blocks

## Prerequisites

- Chapter 52: Kafka fundamentals (understand what Dapr abstracts)
- Chapter 50-51: Kubernetes and Helm
- Your agent service from Part 6

## Looking Ahead

This chapter covers Dapr's stateless building blocks. Chapter 54 adds stateful patterns—actors for agent state and workflows for long-running orchestration.
