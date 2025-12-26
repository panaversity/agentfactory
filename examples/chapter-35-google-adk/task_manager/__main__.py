"""
Entry point for running TaskManager as a module.

Usage:
    python -m task_manager
"""

from task_manager.agent import demo
import asyncio

if __name__ == "__main__":
    # Import and run the demo from agent.py
    from task_manager.agent import __name__ as agent_name

    # The demo function is defined in agent.py
    import asyncio
    from google.adk import Runner
    from google.genai import types
    from task_manager.agent import root_agent
    from task_manager.session import get_session_service

    async def main():
        """Run the TaskManager agent in interactive mode."""
        print("=" * 60)
        print("TaskManager Agent - Interactive Mode")
        print("=" * 60)
        print("\nCommands:")
        print("  - Type your task management requests")
        print("  - Type 'quit' or 'exit' to stop")
        print("  - Type 'help' for available operations")
        print("-" * 60)

        # Create runner with in-memory session
        session_service = get_session_service("development")
        runner = Runner(
            app_name="task_manager",
            agent=root_agent,
            session_service=session_service
        )

        # Create a session
        session = await session_service.create_session(
            app_name="task_manager",
            user_id="interactive_user"
        )

        print(f"\nSession started: {session.id}\n")

        while True:
            try:
                user_input = input("You: ").strip()

                if not user_input:
                    continue

                if user_input.lower() in ('quit', 'exit', 'q'):
                    print("\nGoodbye!")
                    break

                user_content = types.Content(
                    role='user',
                    parts=[types.Part(text=user_input)]
                )

                print()  # Blank line before response

                async for event in runner.run_async(
                    user_id=session.user_id,
                    session_id=session.id,
                    new_message=user_content
                ):
                    if event.content and event.content.parts:
                        for part in event.content.parts:
                            if part.text:
                                print(f"TaskManager: {part.text}")

                print()  # Blank line after response

            except KeyboardInterrupt:
                print("\n\nInterrupted. Goodbye!")
                break
            except Exception as e:
                print(f"\nError: {e}\n")

    asyncio.run(main())
