"""
Course: Cloud Native AI: Docker, Kubernetes & DAPR for Agent Systems
Code: AI-400
Generated: 2025-11-10T00:00:00Z

Database-ready course definition.
Constitution: v3.1.2
"""

from datetime import datetime
from typing import Dict, List, Any


# Course metadata
COURSE_CODE = "AI-400"
COURSE_NAME = "Cloud Native AI: Docker, Kubernetes & DAPR for Agent Systems"
COURSE_INITIALS = "CNAI"


# Course definition (database-ready)
course: Dict[str, Any] = {
    "course_code": "AI-400",
    "course_name": "Cloud Native AI: Docker, Kubernetes & DAPR for Agent Systems",
    "course_initials": "CNAI",
    "course_description": """Master cloud-native infrastructure for AI agent systems using specification-driven development. Learn Docker containerization, Kubernetes orchestration, and DAPR abstractions to deploy production-ready agent applications with observability, scaling, and cloud-agnostic patterns. Build the foundation for deploying autonomous AI systems at scale.""",
    "created_by": "db_admin",
    "updated_by": "db_admin",
    "course_outcomes": [
        "Understand cloud-native thinking for AI systems: containerization, orchestration, and cloud-agnostic abstractions",
        "Master Docker for AI applications using specification-driven development and AI-generated Dockerfiles",
        "Deploy and scale agent systems on Kubernetes with production-grade configurations",
        "Implement DAPR Core abstractions for state management, pub/sub messaging, and service invocation",
        "Build observable AI systems with OpenTelemetry for logs, metrics, and distributed tracing",
        "Design CI/CD pipelines for automated agent deployment with testing and validation",
        "Apply AI-Driven Development (AIDD) to generate and validate cloud infrastructure code"
    ],
    "long_description": """The era of local-only AI development is over. The future isn't about building agents on your laptop; it's about deploying autonomous systems to production. Cloud Native AI represents the infrastructure foundation, the essential shift from "developer" to "cloud architect." It's a methodology that moves you from "running code locally" to "deploying systems that scale"—where you'll master containerization, orchestration, and cloud-agnostic patterns that power production AI applications.

This course introduces Specification-Driven Infrastructure, the professional approach where you write infrastructure specifications and AI generates Docker, Kubernetes, and DAPR configurations. You'll learn the three pillars of cloud-native AI: Docker for packaging applications into portable containers, Kubernetes for orchestrating containers at scale, and DAPR for cloud-agnostic abstractions that work everywhere. Using AI-Driven Development throughout, you'll master container patterns for AI applications including multi-stage builds for Python dependencies, layer optimization for faster deployments, and agent-friendly networking.

You'll start with Docker Fundamentals, learning to containerize AI applications with Dockerfiles generated from specifications. Then Kubernetes Basics, deploying agent systems with pods, deployments, and services while AI handles YAML complexity. You'll master DAPR Core abstractions—state stores that work with any database, pub/sub that works with any message broker, and service invocation that works across any platform. Finally, Production Kubernetes teaches you observability with OpenTelemetry, horizontal pod autoscaling for traffic spikes, and CI/CD pipelines for automated deployments.

This isn't just learning tools; it's learning professional infrastructure thinking. By the end, you won't just run agents locally—you'll deploy production systems that scale, monitor themselves, and adapt to demand. You'll understand how to package Python agents into containers, orchestrate them on Kubernetes, and abstract cloud services with DAPR so your code runs anywhere. This is the foundation for everything that follows: distributed agents, stateful systems, and autonomous architectures.""",
    "learning_modules": [
        {
            "module_id": 1,
            "module_name": "Foundations: Cloud Native Infrastructure for AI",
            "module_description": "Understand why cloud-native patterns matter for AI systems and how specification-driven infrastructure accelerates deployment. Learn the architecture of containerization (Docker), orchestration (Kubernetes), and cloud-agnostic abstractions (DAPR). Establish mental models for thinking in production infrastructure, not just local development."
        },
        {
            "module_id": 2,
            "module_name": "Docker Fundamentals: Containerizing AI Applications",
            "module_description": "Master Docker through specification-first development where AI generates Dockerfiles from your requirements. Learn multi-stage builds for Python dependencies, layer optimization for faster builds, container networking for agent communication, and Docker Compose for multi-container applications. Package AI agents into portable, production-ready containers."
        },
        {
            "module_id": 3,
            "module_name": "Kubernetes Basics: Orchestrating Agent Systems",
            "module_description": "Deploy AI applications to Kubernetes using AI-generated manifests. Master pods (smallest deployable units), deployments (managing replicas), services (networking), ConfigMaps and Secrets (configuration), and StatefulSets (persistent agents). Learn to think in Kubernetes primitives while AI handles YAML complexity."
        },
        {
            "module_id": 4,
            "module_name": "DAPR Core: Cloud-Agnostic Abstractions",
            "module_description": "Build cloud-agnostic AI systems using DAPR abstractions. Master state management (works with PostgreSQL, Redis, or any store), pub/sub messaging (works with Kafka, RabbitMQ, or any broker), and service invocation (works across any platform). Write agent code once, deploy anywhere—AWS, Azure, GCP, or on-premises."
        },
        {
            "module_id": 5,
            "module_name": "Production Operations: Observability, Scaling & CI/CD",
            "module_description": "Operate production AI systems with professional DevOps practices. Implement OpenTelemetry for observability (logs, metrics, distributed tracing), horizontal pod autoscaling for traffic spikes, health checks and graceful shutdown, and CI/CD pipelines for automated testing and deployment. Monitor costs, track performance, and ensure reliability at scale."
        }
    ],
    "pre_requisite": [
        "AI-300 or equivalent (AI-Driven Development with Python and Agentic AI)",
        "Database fundamentals (Part 10: PostgreSQL, Graph, Vector databases)",
        "Basic command-line and terminal proficiency",
        "Understanding of client-server architecture"
    ],
    "media_link": "https://i.postimg.cc/XYLz3tSB/course-2.webp"
}


def get_course_dict() -> Dict[str, Any]:
    """Return complete course dictionary for database insertion."""
    return course.copy()


def get_course_code() -> str:
    """Return course code."""
    return course["course_code"]


def get_course_name() -> str:
    """Return course name."""
    return course["course_name"]


def get_course_outcomes() -> List[str]:
    """Return list of course learning outcomes."""
    return course["course_outcomes"].copy()


def get_learning_modules() -> List[Dict[str, Any]]:
    """Return list of learning modules."""
    return [m.copy() for m in course["learning_modules"]]


def get_prerequisites() -> List[str]:
    """Return list of prerequisites."""
    return course["pre_requisite"].copy()


def validate_course() -> Dict[str, bool]:
    """
    Validate course data structure.

    Returns:
        Dictionary with validation results
    """
    return {
        "has_code": bool(course.get("course_code")),
        "has_name": bool(course.get("course_name")),
        "has_outcomes": len(course.get("course_outcomes", [])) >= 4,
        "has_modules": len(course.get("learning_modules", [])) >= 4,
        "outcomes_are_list": isinstance(course.get("course_outcomes"), list),
        "modules_are_list": isinstance(course.get("learning_modules"), list),
    }


if __name__ == "__main__":
    print(f"Course: {COURSE_NAME}")
    print(f"Code: {COURSE_CODE}")
    print(f"Outcomes: {len(get_course_outcomes())} learning outcomes")
    print(f"Modules: {len(get_learning_modules())} learning modules")
    print(f"Prerequisites: {len(get_prerequisites())} items")
    print("\nValidation:", validate_course())
    print("\n✅ Course data ready for database insertion")
