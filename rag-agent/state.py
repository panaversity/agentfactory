"""Define the state structures for the RoboLearn agent."""

from __future__ import annotations

from chatkit.agents import AgentContext

from chatkit_store import RequestContext


class RoboLearnAgentContext(AgentContext[RequestContext]):
    """Represents the complete state of the RoboLearn agent, extending AgentContext.

    This class can be used to store any information needed throughout the agent's lifecycle.
    """

    pass


