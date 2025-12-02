"""ChatKit server integration for RoboLearn RAG agent."""

from __future__ import annotations

import base64
import logging
from collections.abc import AsyncIterator
from datetime import datetime

import httpx

from chatkit.agents import AgentContext, stream_agent_response, ThreadItemConverter
from chatkit.server import ChatKitServer
from chatkit.types import (
    Attachment,
    AssistantMessageContent,
    AssistantMessageItem,
    ImageAttachment,
    ThreadItemDoneEvent,
    ThreadMetadata,
    ThreadStreamEvent,
    UserMessageItem,
    UserMessageTextContent,
)
from chatkit.store import AttachmentStore
from agents import Agent, Runner
from agents.items import (
    ResponseInputItemParam,
    ResponseInputFileContentParam,
    ResponseInputImageContentParam,
)

from chatkit_store import PostgresStore, RequestContext
from rag.tools import search_tool, SEARCH_TOOL_SCHEMA
from state import RoboLearnAgentContext
from prompts import ROBOLEARN_SYSTEM

logger = logging.getLogger(__name__)


class RoboLearnThreadItemConverter(ThreadItemConverter):
    """Custom converter to handle file attachments for RoboLearn agent."""

    async def attachment_to_message_content(
        self, attachment: Attachment
    ) -> ResponseInputItemParam:
        """
        Convert ChatKit attachment to Agent SDK format.

        Downloads the file from storage and converts to base64 data URL.
        """
        try:
            # Get the download URL from the attachment
            download_url = str(attachment.upload_url)

            # Download the file content
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.get(download_url)
                response.raise_for_status()
                file_bytes = response.content

            # Convert to base64 data URL
            base64_data = base64.b64encode(file_bytes).decode("utf-8")
            data_url = f"data:{attachment.mime_type};base64,{base64_data}"

            # Return appropriate format based on attachment type
            if isinstance(attachment, ImageAttachment):
                logger.info(
                    f"Converting image attachment: {attachment.name} ({len(file_bytes)} bytes)"
                )
                return ResponseInputImageContentParam(
                    type="input_image",
                    detail="auto",
                    image_url=data_url,
                )
            else:
                # For files (PDFs, etc.)
                logger.info(
                    f"Converting file attachment: {attachment.name} ({len(file_bytes)} bytes)"
                )
                return ResponseInputFileContentParam(
                    type="input_file",
                    file_data=data_url,
                    filename=attachment.name or "unknown",
                )
        except Exception as e:
            logger.error(f"Failed to convert attachment {attachment.id}: {e}")
            raise


def _user_message_text(item: UserMessageItem) -> str:
    """Extract text from user message item."""
    parts: list[str] = []
    for part in item.content:
        if isinstance(part, UserMessageTextContent):
            parts.append(part.text)
    return " ".join(parts).strip()


class RoboLearnChatKitServer(ChatKitServer[RequestContext]):
    """
    ChatKit server for RoboLearn RAG agent.

    Integrates with the existing RAG search tool while providing
    ChatKit conversation management and persistence.

    This server handles read-only operations (items.list, threads.list, etc.)
    automatically via the base ChatKit server, and only uses custom logic
    for agent-triggering operations (threads.create, threads.run).
    """

    def __init__(
        self, store: PostgresStore, attachment_store: AttachmentStore | None = None
    ):
        """Initialize the ChatKit server with PostgreSQL store."""
        super().__init__(store, attachment_store)
        logger.info("RoboLearnChatKitServer initialized")

    async def respond(
        self,
        thread: ThreadMetadata,
        input_user_message: UserMessageItem | None,
        context: RequestContext,
    ) -> AsyncIterator[ThreadStreamEvent]:
        """
        Generate response for user message using RoboLearn RAG agent.

        Args:
            thread: Thread metadata
            input_user_message: User's message (None for retry scenarios)
            context: Request context with user_id

        Yields:
            ThreadStreamEvent: Stream of chat events
        """
        # Only handle user message operations that should trigger the agent
        # For read-only operations (items.list, threads.list, etc.),
        # let the base ChatKit server handle them
        if not input_user_message:
            logger.info(
                "No user message provided - this is likely a read-only operation"
            )
            return

        try:
            # Extract user message
            user_text = _user_message_text(input_user_message)
            if not user_text:
                logger.warning("Empty user message")
                return

            # Extract user info and page context from metadata
            user_info = None
            page_context = None
            if hasattr(context, 'metadata') and context.metadata:
                user_info = context.metadata.get('userInfo')
                page_context = context.metadata.get('pageContext')
            
            # Build user context string for agent awareness
            user_context_str = ""
            if user_info:
                user_context_str = f"\n\nUser Information:\n"
                user_context_str += f"- Name: {user_info.get('name', 'Unknown')}\n"
                if user_info.get('email'):
                    user_context_str += f"- Email: {user_info.get('email')}\n"
                if user_info.get('role'):
                    user_context_str += f"- Role: {user_info.get('role')}\n"
                if user_info.get('softwareBackground'):
                    user_context_str += f"- Software Background: {user_info.get('softwareBackground')}\n"
                if user_info.get('hardwareTier'):
                    user_context_str += f"- Hardware Tier: {user_info.get('hardwareTier')}\n"
            
            # Build page context string for agent awareness
            page_context_str = ""
            if page_context:
                page_context_str = f"\n\nCurrent Page Context:\n"
                page_context_str += f"- Page Title: {page_context.get('title', 'Unknown')}\n"
                page_context_str += f"- Page URL: {page_context.get('url', 'Unknown')}\n"
                page_context_str += f"- Page Path: {page_context.get('path', 'Unknown')}\n"
                if page_context.get('headings'):
                    page_context_str += f"- Page Topics: {page_context.get('headings')}\n"
                if page_context.get('description'):
                    page_context_str += f"- Description: {page_context.get('description')}\n"

            # Get previous messages from thread for context
            previous_items = await self.store.load_thread_items(
                thread.id,
                after=None,
                limit=10,
                order="desc",
                context=context,
            )

            # Build message history for agent
            messages = []
            for item in reversed(previous_items.data):
                if isinstance(item, UserMessageItem):
                    messages.append(
                        {"role": "user", "content": _user_message_text(item)}
                    )
                elif isinstance(item, AssistantMessageItem):
                    messages.append(
                        {
                            "role": "assistant",
                            "content": item.content[0].text if item.content else "",
                        }
                    )

            # Add current message
            messages.append({"role": "user", "content": user_text})

            # Create history string to be included in the prompt (like CarFixer does)
            # This is the key: include conversation history in the system prompt
            history_str = "\n".join([f"{m['role']}: {m['content']}" for m in messages])

            # Create RoboLearnAgentContext with all required AgentContext fields
            agent_context = RoboLearnAgentContext(
                # Required AgentContext fields
                thread=thread,
                store=self.store,
                request_context=context,
            )

            # Create agent with RAG search tool
            # Include conversation history, user info, and page context in the instructions so agent is aware of everything
            agent = Agent(
                name="RoboLearnAssistant",
                tools=[search_tool],
                instructions=f"Conversation history:\n{history_str}\n{user_context_str}{page_context_str}\n\n{ROBOLEARN_SYSTEM}",
            )

            # Convert user message with attachments to Agent SDK format
            converter = RoboLearnThreadItemConverter()
            agent_input = await converter.to_agent_input(input_user_message)

            # Run agent with streaming
            # Note: We pass only the current input, history is in the prompt
            logger.info(
                f"Running RoboLearn agent for user {context.user_id} with {len(input_user_message.attachments)} attachments and {len(messages)-1} previous messages in history"
            )
            result = Runner.run_streamed(agent, agent_input, context=agent_context)
            async for event in stream_agent_response(agent_context, result):
                yield event
            logger.info(f"RoboLearn agent response completed for user {context.user_id}")
            return

        except Exception as e:
            logger.exception(f"Error in RoboLearn agent: {e}")

            # Send error message
            error_message = AssistantMessageItem(
                id=self.store.generate_item_id("message", thread, context),
                thread_id=thread.id,
                created_at=datetime.now(),
                content=[
                    AssistantMessageContent(
                        text="I apologize, but I encountered an error processing your request. Please try again.",
                        annotations=[],
                    )
                ],
            )
            yield ThreadItemDoneEvent(item=error_message)


def create_chatkit_server(
    store: PostgresStore, attachment_store: AttachmentStore | None = None
) -> RoboLearnChatKitServer:
    """Create a configured RoboLearn ChatKit server instance."""
    return RoboLearnChatKitServer(store, attachment_store)

