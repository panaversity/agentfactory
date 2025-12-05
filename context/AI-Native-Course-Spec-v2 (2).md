# **AI Native Software Development**

Course Specification & Lesson Guide

*Building DocuBot: A Production RAG Chatbot Step-by-Step*

Version 2.0 | 12/5/2025 | Pedagogically Optimized

# **Pedagogical Design Principles**

This course follows three core principles to minimize cognitive load and maximize learning:

* Progressive Depth: Concepts introduced simply first, full depth only when formally taught. No forward references to code not yet learned.

* One Concept at a Time: Each lesson focuses on ONE main idea. Multiple small lessons beat one overwhelming lesson.

* Concept → Apply Separation: Each lesson teaches the concept FIRST, then has a dedicated 'Apply to DocuBot' section for project work.

## **Chapter 1: Introduction to AI APIs & OpenAI Agents**

**Duration:** 2-3 hours  •  **Lessons:** 4  •  **Depth:** Conceptual (No Code Yet)

**Chapter Overview**

This chapter builds foundational understanding of APIs and AI agents through analogies and visual concepts. Students learn WHAT these technologies are and WHY they matter—without writing code. The focus is on mental models that will support all future learning.

**⚡ Cognitive Load:** LOW \- Pure concepts with real-world analogies. No code complexity. One idea at a time.

**Learning Objectives**

* Explain what an API is using a simple analogy

* Describe the difference between an LLM and an Agent

* Set up a working development environment

* Visualize the RAG chatbot architecture you'll build

**🤖 DocuBot State After This Chapter**

*Project folder created, environment configured, architecture diagram documented. No code written yet—students have a clear mental map of what they're building.*

**Lessons**

**Lesson 1.1: What Are APIs? The Bridge to AI Services**

**Duration:** 30-40 minutes

**🎯 Learning Goal**

*Understand what APIs are and verify your OpenAI API key works.*

**📖 Concept (What You'll Learn)**

An API (Application Programming Interface) is how your code communicates with external services like OpenAI. You send a request, the service processes it, and sends back a response. It's like placing an order at a restaurant.

**Key Points:**

* APIs let different software 'talk' to each other

* You send requests and receive responses (like texting)

* API keys are like passwords that identify you

* JSON is the common 'language' APIs use

**💡 Simple Analogy**

*Think of a restaurant: You (your code) tell the waiter (API) what you want. The waiter takes your order to the kitchen (OpenAI's servers), and brings back your food (the AI response). You never go into the kitchen—the waiter handles everything.*

**💻 Code Level:** MINIMAL CODE \- Only API key verification (3-5 lines). No agent code yet.

**🤖 Apply to DocuBot Project**

**Task:** Test your OpenAI API key with a simple API call. Create test\_api.py with just: 1\) Import OpenAI, 2\) Create client, 3\) Make a simple chat completion call, 4\) Print the response. This verifies your key works before we build agents.

**Outcome:** *Confirmed working API key. You see a response from OpenAI, proving your setup is correct.*

**💡 Hints (if you get stuck):**

1\. First, make sure you've set OPENAI\_API\_KEY in your .env file or environment

2\. Import with: from openai import OpenAI

3\. Create client with: client \= OpenAI() — it automatically reads your API key from environment

4\. Use client.chat.completions.create() to make a request

5\. The response text is in: response.choices\[0\].message.content

**📝 Starter Code:**

from openai import OpenAI\\n\\nclient \= OpenAI()\\n\\n\# Your code here: make a chat completion call

**Lesson 1.2: Agents vs LLMs: From Chatbots to Action-Takers**

**Duration:** 30-40 minutes

**🎯 Learning Goal**

*Understand the fundamental difference between a static LLM and a dynamic agent.*

**📖 Concept (What You'll Learn)**

An LLM (like ChatGPT) can only respond with text. An Agent is an LLM with superpowers—it can use tools, remember things, and take real actions. Agents don't just answer questions; they DO things.

**Key Points:**

* LLM \= Language model that generates text responses

* Agent \= LLM \+ ability to take actions \+ memory \+ tools

* Agents can search databases, call APIs, and perform tasks

* The 'agent loop' means it keeps working until the task is done

**💡 Simple Analogy**

*ChatGPT is like a very smart person trapped in a room who can only talk through a slot in the door. An Agent is that same smart person, but now they have a phone to make calls, a computer to search things, and can actually leave the room to complete tasks for you.*

**💻 Code Level:** NO CODE \- We're building mental models. Code starts in Chapter 2\.

**🤖 Apply to DocuBot Project**

**Task:** Make a comparison table with two columns: 'What ChatGPT Can Do' vs 'What DocuBot Agent Will Do'. List at least 5 items in each column.

**Outcome:** *A clear comparison showing why DocuBot needs to be an agent (search documents, remember context, format citations) not just a chatbot.*

**💡 Hints (if you get stuck):**

1\. Think about ChatGPT's limitations: it can only respond with text, has no memory between sessions

2\. Consider what DocuBot needs: search through documents, remember what you asked before, cite sources

3\. Examples for ChatGPT column: 'Answer general questions', 'Write text', 'Explain concepts'

4\. Examples for DocuBot column: 'Search my uploaded documents', 'Remember our conversation', 'Cite which document the answer came from'

**Lesson 1.3: Environment Setup: Your Development Workspace**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Set up a complete development environment ready for agent development.*

**📖 Concept (What You'll Learn)**

Before writing code, you need the right tools installed. We'll set up Python, the UV package manager, VS Code, and create the project folder structure. This is like setting up your kitchen before cooking.

**Key Points:**

* UV is a fast Python package manager (like npm for Python)

* Virtual environments keep project dependencies separate

* Environment variables (like API keys) stay in .env files

* A consistent folder structure makes projects manageable

**💡 Simple Analogy**

*A chef doesn't start cooking without organizing their station first. Ingredients go in specific places, tools are laid out, and everything is clean. Your development environment is your 'coding station'—organized and ready.*

**💻 Code Level:** TERMINAL COMMANDS ONLY \- Installing tools, creating folders. No Python code yet.

**🤖 Apply to DocuBot Project**

**Task:** Follow the setup guide to: 1\) Install UV, 2\) Create docubot project folder, 3\) Initialize UV project, 4\) Create .env file with your OpenAI API key, 5\) Verify everything works with a simple test.

**Outcome:** *A fully configured project folder with UV, virtual environment, and .env file ready.*

**💡 Hints (if you get stuck):**

1\. Install UV on Mac/Linux: curl \-LsSf https://astral.sh/uv/install.sh | sh

2\. Create folder: mkdir docubot && cd docubot

3\. Initialize project: uv init

4\. Create .env file: touch .env then add OPENAI\_API\_KEY=your-key-here

5\. Install OpenAI package: uv add openai

6\. Test by running your test\_api.py from Lesson 1.1

**Lesson 1.4: Course Project Architecture: The DocuBot Blueprint**

**Duration:** 25-35 minutes

**🎯 Learning Goal**

*Understand the complete system you'll build and how each chapter contributes.*

**📖 Concept (What You'll Learn)**

DocuBot is a RAG (Retrieval-Augmented Generation) chatbot that answers questions about documents. Before building, you need to see the whole picture—like having blueprints before constructing a house.

**Key Points:**

* DocuBot has 4 main parts: Agent, RAG Pipeline, Backend API, Frontend UI

* Each chapter adds one piece to this system

* By the end, you'll have a production-ready application

* Understanding the end goal makes each step meaningful

**💡 Simple Analogy**

*Building a house: You need to see the architect's drawings before laying the first brick. Otherwise, you're just putting bricks somewhere random. The architecture diagram is your 'house blueprint' for DocuBot.*

**💻 Code Level:** NO CODE \- Architecture documentation only.

**🤖 Apply to DocuBot Project**

**Task:** Create an architecture.md file in your project that includes: 1\) System overview diagram, 2\) List of components with brief descriptions, 3\) Which chapter builds which component.

**Outcome:** *A documented architecture that serves as your roadmap for the entire course.*

**💡 Hints (if you get stuck):**

1\. Start with a simple box diagram: User → Frontend → Backend → Agent → Vector DB

2\. Components to list: Agent (brain), Tools (abilities), RAG Pipeline (document search), FastAPI (backend), ChatKit (frontend)

3\. Use markdown headers: \#\# System Overview, \#\# Components, \#\# Chapter Roadmap

4\. For the roadmap, just list: Ch 1-3: Agent basics, Ch 4-9: Advanced agent features, Ch 10-11: RAG pipeline, Ch 12-13: Frontend, Ch 14-16: Production

5\. You can draw the diagram on paper first, then describe it in text

**📋 Chapter Summary & Recap**

You now understand that APIs are bridges between your code and AI services, agents are LLMs that can take actions (not just chat), and you have a clear vision of the DocuBot project. Your development environment is ready.

## **Chapter 2: Your First Agent**

**Duration:** 3-4 hours  •  **Lessons:** 4  •  **Depth:** Beginner Code (Agent basics only)

**Chapter Overview**

Now we write our first code\! This chapter focuses ONLY on creating a basic agent—no tools, no complex features. Students learn the Agent class, the Runner, and how to debug. We keep it simple: one agent, one response.

**⚡ Cognitive Load:** MODERATE \- First code chapter. Small, complete examples. One new concept per lesson. Lots of practice.

**Learning Objectives**

* Create a working AI agent using three lines of code

* Understand what the Agent class does

* Run an agent using Runner.run\_sync()

* Debug basic agent issues with print statements

**🤖 DocuBot State After This Chapter**

*A simple agent named DocuBot that responds to questions. It can't search documents yet (that comes later), but it works and responds intelligently based on its instructions.*

**Lessons**

**Lesson 2.1: Agent Fundamentals: What Makes an Agent**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Understand the core components of an agent before writing any code.*

**📖 Concept (What You'll Learn)**

An agent in the OpenAI SDK is defined by just a few things: a name (identity), instructions (personality/behavior), and a model (which AI brain to use). That's it for the basics.

**Key Points:**

* name: What you call your agent (like 'DocuBot')

* instructions: Rules for how the agent behaves (its 'personality')

* model: Which OpenAI model powers it (gpt-4o-mini is good for learning)

* These three properties are enough to create a working agent

**💡 Simple Analogy**

*Hiring an employee: You give them a name badge (name), a job description (instructions), and assign their role level (model). With these three things, they can start working.*

**💻 Code Level:** READING CODE \- We'll look at simple examples, but focus on understanding, not writing yet.

**🤖 Apply to DocuBot Project**

**Task:** Write down (in plain English, not code): 1\) What should DocuBot be named? 2\) What instructions would make DocuBot helpful for answering document questions? 3\) Which model should it use and why?

**Outcome:** *A written specification for DocuBot's name, instructions, and model choice.*

**💡 Hints (if you get stuck):**

1\. For the name: Keep it simple and descriptive—'DocuBot' works great\!

2\. For instructions, think about: What is DocuBot's job? How should it respond? What should it NOT do?

3\. Example instruction start: 'You are DocuBot, a helpful assistant that answers questions about documents...'

4\. For model: gpt-4o-mini is cheaper and fast (good for learning), gpt-4o is smarter but costs more

5\. Write 3-5 sentences for instructions covering: role, behavior, limitations

**Lesson 2.2: Your First Agent in Python**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Write and run your first working agent with just 4 lines of code.*

**📖 Concept (What You'll Learn)**

Creating an agent is surprisingly simple. You import the library, create an Agent with a name and instructions, run it with Runner.run\_sync(), and print the result. Four lines of code \= working agent.

**Key Points:**

* from agents import Agent, Runner (import what you need)

* Agent(name='...', instructions='...') creates the agent

* Runner.run\_sync(agent, 'your question') runs the agent

* result.final\_output contains the agent's response

**💡 Simple Analogy**

*It's like using a microwave: 1\) Get the microwave (import), 2\) Put food in with settings (create Agent), 3\) Press start (Runner.run\_sync), 4\) Take out food (get final\_output). Simple steps, powerful result.*

**💻 Code Level:** WRITING CODE \- Your first agent\! Simple, complete, working code.

**🤖 Apply to DocuBot Project**

**Task:** Create a file called docubot\_v1.py with code that: 1\) Creates an agent named 'DocuBot' with helpful instructions, 2\) Asks it 'What can you help me with?', 3\) Prints the response. Run it and verify it works.

**Outcome:** *A working docubot\_v1.py that responds when you run it.*

**💡 Hints (if you get stuck):**

1\. Start with: from agents import Agent, Runner

2\. Create agent: agent \= Agent(name='DocuBot', instructions='You are a helpful document assistant...')

3\. Run it: result \= Runner.run\_sync(agent, 'What can you help me with?')

4\. Print: print(result.final\_output)

5\. Make sure you have openai-agents installed: uv add openai-agents

**📝 Starter Code:**

from agents import Agent, Runner\\n\\n\# Create your agent here\\nagent \= Agent(\\n    name='DocuBot',\\n    instructions='...'  \# Add your instructions\\n)\\n\\n\# Run the agent and print result

**Lesson 2.3: The Agent Loop: How Agents Think and Respond**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Understand what happens 'behind the scenes' when an agent runs.*

**📖 Concept (What You'll Learn)**

When you call Runner.run\_sync(), the agent enters a 'loop': it receives your input, processes it through the LLM, checks if it needs to do anything else, and eventually returns a final response. Understanding this loop helps with debugging.

**Key Points:**

* The agent receives your input as a 'message'

* It sends the message to the LLM (OpenAI's servers)

* The LLM generates a response

* If more work is needed, the loop continues; otherwise, it exits

* Runner.run\_sync() is the 'synchronous' (blocking) version

**💡 Simple Analogy**

*A chef preparing your order: They read the ticket (input), cook the dish (process), check if it's complete (loop check), and either serve it (return) or keep cooking (continue loop).*

**💻 Code Level:** READING & MODIFYING CODE \- Understanding execution flow, adding print statements.

**🤖 Apply to DocuBot Project**

**Task:** Modify docubot\_v1.py to: 1\) Print 'Starting DocuBot...' before running, 2\) Print the type of result (type(result)), 3\) Print 'Response received\!' after. This helps you see the execution flow.

**Outcome:** *Understanding that the agent runs synchronously and returns a result object.*

**💡 Hints (if you get stuck):**

1\. Add print('Starting DocuBot...') before the Runner.run\_sync() line

2\. After getting result, add: print(type(result)) to see what kind of object it is

3\. Add print('Response received\!') after printing the output

4\. Run the code and observe the order of print statements

5\. Notice how the code waits at Runner.run\_sync() until the response is ready

**Lesson 2.4: Debugging Agents: When Things Don't Work**

**Duration:** 30-40 minutes

**🎯 Learning Goal**

*Learn practical debugging techniques for agent development.*

**📖 Concept (What You'll Learn)**

Things will go wrong—that's normal\! The key is knowing how to investigate. Print statements, checking result properties, and reading error messages are your debugging tools.

**Key Points:**

* print() is your best friend for debugging

* result.final\_output shows what the agent said

* result.new\_items shows the conversation messages

* Error messages usually tell you exactly what's wrong

* If the agent doesn't follow instructions, your instructions might be unclear

**💡 Simple Analogy**

*Debugging is like being a detective. Something's not working \= a mystery to solve. Print statements are your magnifying glass—they show you what's happening at each step.*

**💻 Code Level:** WRITING & DEBUGGING CODE \- Adding debug output, handling common errors.

**🤖 Apply to DocuBot Project**

**Task:** Intentionally break docubot\_v1.py in 3 ways (wrong import, missing API key, typo in code), run it each time, and document the error message you see. Then fix each error. This teaches you to read error messages.

**Outcome:** *Experience with common errors and how to fix them. A 'troubleshooting notes' section in your project.*

**💡 Hints (if you get stuck):**

1\. Error 1 \- Wrong import: Try 'from agent import Agent' (missing 's') and note the error

2\. Error 2 \- Missing API key: Temporarily rename your .env file and run the code

3\. Error 3 \- Typo: Change 'instructions' to 'instruction' (missing 's')

4\. For each error, copy the error message to your notes

5\. Common fix patterns: ModuleNotFoundError \= check package name, AuthenticationError \= check API key

**📋 Chapter Summary & Recap**

You've created your first working agent\! You understand that Agent() creates the agent, Runner.run\_sync() executes it, and result.final\_output contains the response. DocuBot can now have basic conversations.

## **Chapter 3: Configuring & Extending Your Agent**

**Duration:** 6-7 hours  •  **Lessons:** 6  •  **Depth:** Full Depth (Configuration \+ Tools taught here)

**Chapter Overview**

This is where agents get powerful. We learn to customize agent behavior through configuration AND give agents abilities through tools. This chapter has full depth because tools are formally introduced here—not before.

**⚡ Cognitive Load:** MODERATE-HIGH \- Multiple new concepts, but introduced one at a time. Each lesson builds on the previous. Plenty of practice time.

**Learning Objectives**

* Configure agent behavior with model settings and parameters

* Write effective agent instructions that control behavior

* Create custom tools using the @function\_tool decorator

* Connect tools to agents and understand tool selection

**🤖 DocuBot State After This Chapter**

*A fully configured agent with custom instructions, appropriate model settings, and 2-3 working tools. DocuBot can now perform actions like getting the current date or searching a simple knowledge base.*

**Lessons**

**Lesson 3.1: Agent Configuration Options**

**Duration:** 30-40 minutes

**🎯 Learning Goal**

*Understand all the ways you can customize an agent's behavior.*

**📖 Concept (What You'll Learn)**

Beyond name and instructions, agents have many configuration options: which model to use, how creative to be (temperature), and more. Configuration is how you tune the agent for your specific needs.

**Key Points:**

* Model choice affects capability and cost (gpt-4o vs gpt-4o-mini)

* Temperature controls creativity (0 \= focused, 1 \= creative)

* ModelSettings groups model-related options

* Configuration should match your use case

**💡 Simple Analogy**

*Configuring an agent is like adjusting a camera before taking a photo. You set the mode (model), adjust focus/creativity (temperature), and tune other settings. The same camera can take very different photos with different settings.*

**💻 Code Level:** WRITING CODE \- ModelSettings, different configurations.

**🤖 Apply to DocuBot Project**

**Task:** Create docubot\_v2.py with: 1\) Explicit model selection (gpt-4o-mini), 2\) Temperature set to 0.3 (more focused for document Q\&A), 3\) Test how responses change with temperature 0.1 vs 0.9.

**Outcome:** *Configured DocuBot with intentional model settings. Notes on how temperature affects responses.*

**💡 Hints (if you get stuck):**

1\. Import ModelSettings: from agents import Agent, Runner, ModelSettings

2\. Create settings: model\_settings \= ModelSettings(temperature=0.3)

3\. Apply to agent: Agent(name='DocuBot', instructions='...', model='gpt-4o-mini', model\_settings=model\_settings)

4\. Test with temperature=0.1 (very focused) and temperature=0.9 (more creative)

5\. Ask the same question with different temperatures and compare responses

**📝 Starter Code:**

from agents import Agent, Runner, ModelSettings\\n\\nmodel\_settings \= ModelSettings(temperature=0.3)\\n\\nagent \= Agent(\\n    name='DocuBot',\\n    instructions='...',\\n    model='gpt-4o-mini',\\n    model\_settings=model\_settings\\n)

**Lesson 3.2: Writing Effective Instructions**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Master the art of writing instructions that make agents behave exactly as intended.*

**📖 Concept (What You'll Learn)**

Instructions are the most powerful way to control agent behavior. Good instructions are clear, specific, and include examples of what TO do and what NOT to do. Think of instructions as writing a job description for your agent.

**Key Points:**

* Be specific: 'Always cite sources' not 'be helpful'

* Include role definition: 'You are a document assistant that...'

* Add constraints: 'Never make up information'

* Give format guidance: 'Respond in bullet points when listing'

* Instructions can be dynamic (change based on context)

**💡 Simple Analogy**

*Writing agent instructions is like training a new employee. Vague instructions ('do good work') fail. Specific instructions ('answer customer calls within 3 rings, greet with company name, always offer a callback') succeed.*

**💻 Code Level:** WRITING CODE \- Comprehensive instruction crafting.

**🤖 Apply to DocuBot Project**

**Task:** Rewrite DocuBot's instructions to include: 1\) Clear role definition, 2\) What it should do when answering, 3\) What it should NEVER do, 4\) How to handle unknown information. Make instructions at least 5-6 sentences.

**Outcome:** *DocuBot with professional, comprehensive instructions that control its behavior.*

**💡 Hints (if you get stuck):**

1\. Start with role: 'You are DocuBot, a helpful assistant specialized in answering questions about documents.'

2\. Add behavior: 'When answering questions, be concise and accurate. Always cite which document your answer comes from.'

3\. Add constraints: 'Never make up information. If you don\\'t know something, say so clearly.'

4\. Add format guidance: 'When listing multiple items, use bullet points for clarity.'

5\. Add fallback: 'If the user asks about something not in the documents, politely explain you can only help with document-related questions.'

**Lesson 3.3: What Are Tools? Giving Agents Superpowers**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Understand what tools are and why they transform agents from chatbots to action-takers.*

**📖 Concept (What You'll Learn)**

Tools are functions that agents can call to perform actions beyond generating text. With tools, an agent can search databases, perform calculations, call APIs, or do anything a Python function can do. Tools are what make agents truly useful.

**Key Points:**

* A tool is a Python function the agent can call

* The agent decides WHEN to use a tool based on user input

* Tools have descriptions that tell the agent what they do

* Agents can use multiple tools in sequence

* The agent loop handles tool execution automatically

**💡 Simple Analogy**

*Tools are like smartphone apps for your agent. The phone (agent) is useful on its own, but apps (tools) let it do specific things: maps for directions, calculator for math, camera for photos. Each tool adds a specific capability.*

**💻 Code Level:** CONCEPTUAL \+ READING CODE \- Understanding tools before writing them.

**🤖 Apply to DocuBot Project**

**Task:** List 5 tools that DocuBot will eventually need to be a good document assistant. For each tool, write: 1\) What it does, 2\) What inputs it needs, 3\) What it returns. Don't write code yet—just design.

**Outcome:** *A tool design document listing the tools DocuBot will use.*

**💡 Hints (if you get stuck):**

1\. Tool 1: search\_documents \- searches for relevant documents, needs a query string, returns list of matching docs

2\. Tool 2: get\_document\_content \- gets full content of a specific doc, needs document ID, returns text content

3\. Tool 3: get\_current\_date \- gets today's date, needs no input, returns formatted date string

4\. Think about: What actions would a human assistant need to do to answer document questions?

5\. Format as a simple table or list in a markdown file: tools\_design.md

**Lesson 3.4: Creating Your First Tool**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Write a working tool using the @function\_tool decorator.*

**📖 Concept (What You'll Learn)**

The @function\_tool decorator turns any Python function into a tool. The function's docstring becomes the tool description, and type hints tell the agent what inputs are expected. It's surprisingly simple.

**Key Points:**

* @function\_tool decorator marks a function as a tool

* The docstring is critical—it tells the agent when to use the tool

* Type hints (str, int, etc.) define expected inputs

* Return value goes back to the agent for processing

* Simple tools first—complex tools later

**💡 Simple Analogy**

*Creating a tool is like teaching someone a new skill with clear instructions. The decorator says 'this is a skill', the docstring explains 'use this skill when...', and the function body is 'how to perform the skill'.*

**💻 Code Level:** WRITING CODE \- Creating @function\_tool decorated functions.

**🤖 Apply to DocuBot Project**

**Task:** Create a simple tool called get\_current\_date() that returns today's date formatted nicely. Decorate it with @function\_tool. Test that the agent actually calls it when asked 'What's today's date?'

**Outcome:** *DocuBot with its first working tool. It can now answer date questions dynamically.*

**💡 Hints (if you get stuck):**

1\. Import: from agents import Agent, Runner, function\_tool

2\. Import datetime: from datetime import datetime

3\. Decorator goes above function: @function\_tool

4\. Add a docstring: '''Returns the current date in a readable format.'''

5\. Function body: return datetime.now().strftime('%B %d, %Y')

6\. Don't forget to add the tool to your agent using tools=\[get\_current\_date\]

**📝 Starter Code:**

from agents import Agent, Runner, function\_tool\\nfrom datetime import datetime\\n\\n@function\_tool\\ndef get\_current\_date() \-\> str:\\n    '''Returns the current date in a readable format.'''\\n    \# Your code here: return formatted date

**Lesson 3.5: Connecting Tools to Agents**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Register tools with agents and control tool selection behavior.*

**📖 Concept (What You'll Learn)**

Tools don't work until you connect them to an agent. You pass tools in a list to the Agent constructor. The agent then decides when to call each tool based on user input and tool descriptions.

**Key Points:**

* tools=\[tool1, tool2\] adds tools to an agent

* The agent reads tool descriptions to decide which to use

* tool\_choice='auto' lets the agent decide (default)

* tool\_choice='required' forces tool use

* Multiple tools can be called in one response

**💡 Simple Analogy**

*It's like giving a worker access to a toolbox. You put tools in the box (tools=\[...\]), they can see what's available (descriptions), and they pick the right tool for each job (automatic selection).*

**💻 Code Level:** WRITING CODE \- Registering tools, testing tool selection.

**🤖 Apply to DocuBot Project**

**Task:** Create docubot\_v3.py with: 1\) At least 2 tools (get\_current\_date and a simple calculate tool), 2\) Both tools registered with the agent, 3\) Test that the agent correctly chooses which tool to use for different questions.

**Outcome:** *DocuBot with multiple tools demonstrating correct tool selection.*

**💡 Hints (if you get stuck):**

1\. Create second tool: @function\_tool\\ndef calculate(expression: str) \-\> str:\\n    '''Calculates a math expression like 2+2 or 10\*5.'''

2\. For calculate body, use: return str(eval(expression)) — note: eval is simple but not safe for production

3\. Register both tools: Agent(name='DocuBot', instructions='...', tools=\[get\_current\_date, calculate\])

4\. Test: 'What is today?' should trigger get\_current\_date

5\. Test: 'What is 15 \* 7?' should trigger calculate

**📝 Starter Code:**

from agents import Agent, Runner, function\_tool\\nfrom datetime import datetime\\n\\n@function\_tool\\ndef get\_current\_date() \-\> str:\\n    '''Returns the current date.'''\\n    return datetime.now().strftime('%B %d, %Y')\\n\\n@function\_tool\\ndef calculate(expression: str) \-\> str:\\n    '''Calculates a math expression like 2+2 or 10\*5.'''\\n    return str(eval(expression))\\n\\nagent \= Agent(\\n    name='DocuBot',\\n    instructions='You are a helpful assistant. Use tools when needed.',\\n    tools=\[get\_current\_date, calculate\]\\n)

**Lesson 3.6: Tool Design Best Practices**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Learn patterns for writing reliable, well-designed tools.*

**📖 Concept (What You'll Learn)**

Good tools are focused (do one thing well), well-documented (clear descriptions), and handle errors gracefully. Poorly designed tools confuse agents and cause failures.

**Key Points:**

* One tool \= one purpose (don't make Swiss Army knife tools)

* Tool descriptions should be crystal clear

* Handle errors and return helpful error messages

* Use Pydantic models for complex inputs (optional for now)

* Test tools independently before connecting to agents

**💡 Simple Analogy**

*A good tool is like a good kitchen utensil: the whisk whisks, the spatula flips, the peeler peels. A bad tool tries to do everything and does nothing well.*

**💻 Code Level:** WRITING CODE \- Refactoring tools to follow best practices.

**🤖 Apply to DocuBot Project**

**Task:** Review your existing tools. Improve their docstrings, add error handling (try/except), and make sure each does exactly one thing. Create one new tool: get\_greeting(name) that returns a personalized greeting.

**Outcome:** *Well-designed tools with clear descriptions and error handling.*

**💡 Hints (if you get stuck):**

1\. Better docstring example: '''Returns the current date. Use this when the user asks about today's date or what day it is.'''

2\. Add error handling: try:\\n    return str(eval(expression))\\nexcept Exception as e:\\n    return f'Error: {str(e)}'

3\. For get\_greeting: @function\_tool\\ndef get\_greeting(name: str) \-\> str:\\n    '''Creates a personalized greeting. Use when user introduces themselves.'''\\n    return f'Hello, {name}\! How can I help you today?'

4\. Test each tool individually before adding to agent

5\. Check: Does the docstring clearly explain WHEN to use this tool?

**📋 Chapter Summary & Recap**

Your agent is now customizable and capable. You can tune its behavior with configuration, shape its personality with instructions, and give it abilities with tools. DocuBot is becoming useful\!

## **Chapter 4: Agent Memory, Sessions & Context**

**Duration:** 3-4 hours  •  **Lessons:** 4  •  **Depth:** Full Depth (Memory/Sessions taught here)

**Chapter Overview**

Agents need to remember\! This chapter teaches the SDK's Session system for automatic conversation history, plus custom context for passing data to agents and tools. Students learn to build agents that feel continuous.

**⚡ Cognitive Load:** MODERATE \- One core concept (memory/state), explored through different mechanisms. Each lesson is self-contained.

**Learning Objectives**

* Understand why agents need memory and context

* Use built-in Sessions for automatic conversation history

* Create custom context objects for dependency injection

* Design context structures for RAG applications

**🤖 DocuBot State After This Chapter**

*DocuBot now remembers the conversation—you can have multi-turn dialogues where it references what was said earlier. It also has a context object ready to hold retrieved documents.*

**Lessons**

**Lesson 4.1: Why Agents Need Memory**

**Duration:** 25-35 minutes

**🎯 Learning Goal**

*Understand the memory problem and why sessions are necessary.*

**📖 Concept (What You'll Learn)**

By default, each agent run is independent—the agent has no memory of previous conversations. This is fine for single questions, but for real applications, you need continuity. That's where Sessions come in.

**Key Points:**

* Without memory, every question is treated as the first

* Users expect 'remember what I just said'

* Memory enables multi-turn conversations

* The SDK provides built-in Session management

**💡 Simple Analogy**

*Imagine calling customer service and having to re-explain your entire issue every time they pass you to someone new. That's an agent without memory. Sessions are like a shared case file that follows you.*

**💻 Code Level:** READING CODE \- Understanding the problem before the solution.

**🤖 Apply to DocuBot Project**

**Task:** Test current DocuBot: Ask a question, then ask a follow-up that requires remembering the first answer. Document how it fails. This shows why we need Sessions.

**Outcome:** *Documentation of the memory problem in your current DocuBot.*

**💡 Hints (if you get stuck):**

1\. Run DocuBot twice: First ask 'My name is Wania' then ask 'What is my name?'

2\. Notice how the agent says it doesn't know your name—it forgot\!

3\. Document this in memory\_problem.md with: What you asked, What it answered, Why this is a problem

4\. Think about: How would a real assistant remember this?

5\. This is the exact problem Sessions will solve in the next lesson

**Lesson 4.2: Sessions: Built-in Conversation Memory**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Implement Sessions to give your agent persistent conversation memory.*

**📖 Concept (What You'll Learn)**

The SDK's Session system automatically stores and retrieves conversation history. SQLiteSession stores locally, RedisSession for production. Just pass a session to Runner.run() and memory is handled for you.

**Key Points:**

* Session is the SDK's built-in memory system

* SQLiteSession stores conversations in a local file

* Pass session=my\_session to Runner.run()

* The same session across runs \= continuous conversation

* Sessions store messages automatically

**💡 Simple Analogy**

*A Session is like a notebook that automatically records the conversation. When you talk again, the agent reads the notebook first, then responds with full context.*

**💻 Code Level:** WRITING CODE \- Implementing Sessions with SQLite.

**🤖 Apply to DocuBot Project**

**Task:** Update DocuBot to use SQLiteSession: 1\) Create a session with a unique ID, 2\) Use the same session across multiple runs, 3\) Verify the agent now remembers previous messages.

**Outcome:** *DocuBot that maintains conversation history between runs.*

**💡 Hints (if you get stuck):**

1\. Import: from agents import SQLiteSession

2\. Create session: session \= SQLiteSession('docubot\_memory.db', 'user\_123')

3\. Use in Runner: result \= await Runner.run(agent, 'message', session=session)

4\. Note: Runner.run() is async, use Runner.run\_sync() for synchronous or await Runner.run()

5\. Test: Run twice with same session ID—'My name is Wania' then 'What is my name?'—it should remember\!

**📝 Starter Code:**

from agents import Agent, Runner, SQLiteSession\\n\\nagent \= Agent(name='DocuBot', instructions='...')\\n\\n\# Create a session that persists\\nsession \= SQLiteSession('docubot\_memory.db', 'user\_123')\\n\\n\# Now use this session in your runs

**Lesson 4.3: Custom Context: Dependency Injection**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Create custom context objects to pass data to agents and tools.*

**📖 Concept (What You'll Learn)**

Sometimes you need to pass custom data to your agent—user preferences, configuration, retrieved documents. Custom context objects let you inject any data you need, accessible in tools and dynamic instructions.

**Key Points:**

* Agent\[TContext\] makes agents generic over a context type

* Create a dataclass or Pydantic model for your context

* Pass context to Runner.run(context=my\_context)

* Access context in tools via ctx.context

* Context is NOT conversation history (that's Sessions)

**💡 Simple Analogy**

*Context is like a backpack the agent carries. You can put anything in it—user info, settings, documents. The agent and its tools can reach into the backpack whenever they need something.*

**💻 Code Level:** WRITING CODE \- Creating and using custom context classes.

**🤖 Apply to DocuBot Project**

**Task:** Create a DocuBotContext dataclass with: user\_name (str), preferred\_format ('bullet' or 'prose'), max\_response\_length (int). Pass this to DocuBot and modify instructions to use these preferences.

**Outcome:** *DocuBot that personalizes responses based on context preferences.*

**💡 Hints (if you get stuck):**

1\. Import dataclass: from dataclasses import dataclass

2\. Define context: @dataclass\\nclass DocuBotContext:\\n    user\_name: str\\n    preferred\_format: str \= 'prose'\\n    max\_response\_length: int \= 200

3\. Make agent generic: agent: Agent\[DocuBotContext\] \= Agent(...)

4\. Create context instance: ctx \= DocuBotContext(user\_name='Wania', preferred\_format='bullet')

5\. Pass to runner: Runner.run(agent, message, context=ctx)

**📝 Starter Code:**

from dataclasses import dataclass\\nfrom agents import Agent, Runner\\n\\n@dataclass\\nclass DocuBotContext:\\n    user\_name: str\\n    preferred\_format: str \= 'prose'  \# 'bullet' or 'prose'\\n    max\_response\_length: int \= 200\\n\\n\# Create your agent with Agent\[DocuBotContext\]

**Lesson 4.4: Context Design for RAG**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Design the context structure that will support document retrieval.*

**📖 Concept (What You'll Learn)**

For RAG applications, context will hold retrieved documents, search queries, and confidence scores. Designing this now—even with placeholder data—prepares your agent for real retrieval later.

**Key Points:**

* RAG context needs: retrieved\_documents, search\_query, confidence

* Start with placeholder/mock data

* Structure should match what retrieval will provide

* Think about what the agent needs to generate good answers

**💡 Simple Analogy**

*Designing RAG context is like designing a filing system before you have files. You create the folders (structure) first, so when documents arrive, you know exactly where they go.*

**💻 Code Level:** WRITING CODE \- Designing context structure with placeholders.

**🤖 Apply to DocuBot Project**

**Task:** Create RAGContext class with: retrieved\_docs (list), search\_query (str), num\_results (int), retrieval\_confidence (float). For now, populate with mock data. Update tools to access this context.

**Outcome:** *RAG-ready context structure that will connect to real retrieval in Chapter 10\.*

**💡 Hints (if you get stuck):**

1\. Define Document type: @dataclass\\nclass Document:\\n    id: str\\n    content: str\\n    source: str\\n    score: float

2\. RAG context structure: @dataclass\\nclass RAGContext:\\n    retrieved\_docs: list\[Document\]\\n    search\_query: str\\n    num\_results: int \= 5\\n    retrieval\_confidence: float \= 0.0

3\. Create mock data: mock\_docs \= \[Document(id='1', content='Sample content...', source='doc1.pdf', score=0.95)\]

4\. Access in tool: def search\_docs(ctx: RunContextWrapper\[RAGContext\], query: str):\\n    return ctx.context.retrieved\_docs

5\. Later (Ch 10), we'll replace mock data with real vector search

**📝 Starter Code:**

from dataclasses import dataclass\\n\\n@dataclass\\nclass Document:\\n    id: str\\n    content: str\\n    source: str\\n    score: float\\n\\n@dataclass\\nclass RAGContext:\\n    retrieved\_docs: list\[Document\]\\n    search\_query: str\\n    num\_results: int \= 5\\n    retrieval\_confidence: float \= 0.0

**📋 Chapter Summary & Recap**

Your agent now has memory\! Sessions handle conversation history automatically, and custom context lets you inject any data you need. DocuBot can have natural, continuous conversations.

## **Chapter 5: Streaming Responses**

**Duration:** 3-4 hours  •  **Lessons:** 4  •  **Depth:** Full Depth (Streaming taught here)

**Chapter Overview**

Users hate waiting. Streaming lets agents deliver responses word-by-word as they're generated, creating a much better user experience. This chapter covers the SDK's streaming capabilities.

**⚡ Cognitive Load:** MODERATE \- One core concept (streaming) with practical implementation. Async patterns introduced gently.

**Learning Objectives**

* Explain why streaming matters for user experience

* Implement streaming using Runner.run\_streamed()

* Process different types of stream events

* Handle streaming with tools and complex operations

**🤖 DocuBot State After This Chapter**

*DocuBot now streams responses in real-time—users see words appearing as they're generated, not a long wait followed by a wall of text.*

**Lessons**

**Lesson 5.1: Why Streaming Matters**

**Duration:** 25-35 minutes

**🎯 Learning Goal**

*Understand the UX impact of streaming vs. waiting.*

**📖 Concept (What You'll Learn)**

Without streaming, users wait for the entire response to generate before seeing anything. With streaming, they see words appear in real-time. This feels faster (even if total time is the same) and keeps users engaged.

**Key Points:**

* Perceived speed matters as much as actual speed

* Streaming reduces 'time to first token' to near-zero

* Users can start reading immediately

* Streaming also lets users stop generation early if needed

**💡 Simple Analogy**

*It's the difference between watching a movie buffer for 5 minutes then play, versus watching it start immediately even if quality adjusts. Same content, very different experience.*

**💻 Code Level:** CONCEPTUAL \- Understanding the why before the how.

**🤖 Apply to DocuBot Project**

**Task:** Ask DocuBot a question that requires a long response. Time how long you wait before seeing anything. Write down how this feels as a user.

**Outcome:** *Documented user experience problem that streaming will solve.*

**💡 Hints (if you get stuck):**

1\. Ask a complex question: 'Explain the history of artificial intelligence in detail'

2\. Use a stopwatch or your phone to time from pressing Enter to seeing the first word

3\. Notice how you're staring at a blank screen—this is the 'time to first token' problem

4\. Write in streaming\_ux.md: 1\) How long you waited, 2\) How it felt, 3\) What would be better

5\. Compare to ChatGPT or other AI chat apps that stream—feels much more responsive\!

**Lesson 5.2: Implementing Streaming**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Use Runner.run\_streamed() to enable real-time responses.*

**📖 Concept (What You'll Learn)**

Runner.run\_streamed() returns a StreamedRunResult that you iterate over. Each iteration yields an event with content. You process events as they arrive, printing or displaying incrementally.

**Key Points:**

* Runner.run\_streamed() returns immediately with a result object

* Use 'async for event in result.stream\_events()' to process

* Events arrive as content is generated

* This requires async/await pattern

**💡 Simple Analogy**

*Streaming is like a live news ticker vs. waiting for the complete news article. The ticker updates constantly; you read as it goes.*

**💻 Code Level:** WRITING CODE \- Implementing streaming with async patterns.

**🤖 Apply to DocuBot Project**

**Task:** Create docubot\_streaming.py: 1\) Use Runner.run\_streamed() instead of run\_sync(), 2\) Loop over stream\_events() and print each chunk, 3\) Compare the experience to the non-streaming version.

**Outcome:** *DocuBot with streaming responses showing in real-time.*

**💡 Hints (if you get stuck):**

1\. You need async/await: async def main(): ...

2\. Use Runner.run\_streamed(): result \= Runner.run\_streamed(agent, 'message')

3\. Iterate with: async for event in result.stream\_events(): ...

4\. Print without newline: print(chunk, end='', flush=True)

5\. Run async code with: asyncio.run(main())

6\. Remember to import asyncio at the top

**📝 Starter Code:**

import asyncio\\nfrom agents import Agent, Runner\\n\\nasync def main():\\n    agent \= Agent(name='DocuBot', instructions='...')\\n    result \= Runner.run\_streamed(agent, 'Tell me about AI')\\n    \\n    async for event in result.stream\_events():\\n        \# Process event here\\n        pass\\n\\nasyncio.run(main())

**Lesson 5.3: Processing Stream Events**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Handle different event types during streaming.*

**📖 Concept (What You'll Learn)**

Streaming produces different types of events: text content, tool calls, tool outputs, and more. You check event.item.type to know what you're receiving and handle each appropriately.

**Key Points:**

* Events have different types (message\_output\_item, tool\_call\_output\_item, etc.)

* Use ItemHelpers.text\_message\_output() for text content

* Tool events let you show 'DocuBot is searching...'

* Clean separation of event handling makes code maintainable

**💡 Simple Analogy**

*Processing events is like sorting mail. You look at what type each piece is (letter, package, bill) and handle each differently.*

**💻 Code Level:** WRITING CODE \- Event type handling and display logic.

**🤖 Apply to DocuBot Project**

**Task:** Enhance streaming to: 1\) Show tool calls with 'Using tool: \[name\]...', 2\) Display tool outputs, 3\) Show final text response. This gives users visibility into what's happening.

**Outcome:** *Streaming that shows both tool activity and final responses.*

**💡 Hints (if you get stuck):**

1\. Import ItemHelpers: from agents import ItemHelpers

2\. Check event type: if event.item.type \== 'message\_output\_item': ...

3\. Get text content: text \= ItemHelpers.text\_message\_output(event.item)

4\. For tool calls, check: if event.item.type \== 'tool\_call\_item': print(f'Using tool: {event.item.name}')

5\. Use if/elif chain to handle different event types separately

**Lesson 5.4: Streaming with Tools**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Handle streaming when agents use tools.*

**📖 Concept (What You'll Learn)**

When an agent uses tools during streaming, you get tool-related events mixed with content events. Properly handling these creates a smooth experience where users see progress indicators during tool execution.

**Key Points:**

* Tool calls and outputs appear as events in the stream

* You can show progress: 'Searching documents...'

* Multiple tools \= multiple tool events

* Error events also stream—handle gracefully

**💡 Simple Analogy**

*It's like a cooking show where the chef explains each step as they do it, not just showing the finished dish.*

**💻 Code Level:** WRITING CODE \- Integrating tool feedback with streaming.

**🤖 Apply to DocuBot Project**

**Task:** Add visual feedback for tool execution: When a tool starts, print a spinner or 'Working...'. When it completes, show 'Done\!'. This makes DocuBot feel responsive even during long operations.

**Outcome:** *Polished streaming experience with clear progress indication.*

**💡 Hints (if you get stuck):**

1\. Track tool state: when you see tool\_call\_item, print '⏳ Searching...'

2\. When you see tool\_call\_output\_item, print '✅ Done\!'

3\. Simple spinner: print('⏳', end='', flush=True) then print('\\r✅ Done\!')

4\. Test with your date or calculate tools to see tool events

5\. For multiple tools, you might see several tool\_call events before output

**📝 Starter Code:**

async for event in result.stream\_events():\\n    if event.item.type \== 'tool\_call\_item':\\n        print(f'⏳ Using {event.item.name}...', end='', flush=True)\\n    elif event.item.type \== 'tool\_call\_output\_item':\\n        print(' ✅')\\n    elif event.item.type \== 'message\_output\_item':\\n        text \= ItemHelpers.text\_message\_output(event.item)\\n        if text:\\n            print(text, end='', flush=True)

**📋 Chapter Summary & Recap**

Streaming transforms user experience. Runner.run\_streamed() returns events as they happen, and you process them to show real-time output. DocuBot now feels responsive and modern.

## **Chapter 6: Multi-Agent Systems & Handoffs**

**Duration:** 4-5 hours  •  **Lessons:** 4  •  **Depth:** Full Depth (Handoffs taught here)

**Chapter Overview**

Complex tasks often require specialized agents working together. This chapter teaches how to create multiple agents and use handoffs to delegate between them. The multi-agent pattern is essential for sophisticated applications.

**⚡ Cognitive Load:** MODERATE-HIGH \- New architectural concept. Built step-by-step with clear examples. Each lesson adds one aspect.

**Learning Objectives**

* Design multi-agent architectures for complex tasks

* Create specialized agents with focused capabilities

* Implement handoffs for agent-to-agent delegation

* Choose between handoffs and agents-as-tools patterns

**🤖 DocuBot State After This Chapter**

*DocuBot is now a system of 3+ specialized agents: a triage agent that routes questions, a document search agent, and a summarization agent. They work together seamlessly.*

**Lessons**

**Lesson 6.1: Multi-Agent Architecture Patterns**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Understand when and how to split work across multiple agents.*

**📖 Concept (What You'll Learn)**

A single agent trying to do everything becomes unreliable. Multi-agent systems use specialized agents—one for searching, one for summarizing, one for formatting—that work together. The 'triage' pattern routes requests to the right specialist.

**Key Points:**

* Specialization improves reliability and quality

* Triage agent routes to specialists

* Each specialist is simpler and more focused

* Agents can be developed and tested independently

**💡 Simple Analogy**

*A hospital doesn't have one doctor who does everything. There's triage (who assesses you), specialists (cardiologist, orthopedist), and support staff. Each does their job well, and patients get better care.*

**💻 Code Level:** CONCEPTUAL \+ DESIGN \- Architecture planning before implementation.

**🤖 Apply to DocuBot Project**

**Task:** Design DocuBot's multi-agent architecture: 1\) What specialized agents are needed? 2\) How should questions be routed? 3\) Draw a diagram showing agent relationships.

**Outcome:** *Multi-agent architecture diagram for DocuBot.*

**💡 Hints (if you get stuck):**

1\. Consider 3-4 agents: TriageAgent (router), SearchAgent, SummaryAgent, QAAgent

2\. TriageAgent instructions: 'Analyze the user request and route to the appropriate specialist'

3\. SearchAgent: specializes in finding relevant documents

4\. SummaryAgent: specializes in condensing long content

5\. Draw: TriageAgent at top, arrows pointing down to each specialist

**Lesson 6.2: Creating Specialized Agents**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Build agents that excel at specific tasks.*

**📖 Concept (What You'll Learn)**

A specialized agent has focused instructions for one job. It might only search, only summarize, or only format. By keeping agents focused, they become more reliable and easier to debug.

**Key Points:**

* Focused instructions \= better performance

* Assign only relevant tools to each agent

* Each agent should have a clear 'job description'

* Test specialists independently before connecting

**💡 Simple Analogy**

*A specialist chef (pastry chef, sushi chef) is better at their specialty than a 'does everything' cook. The same applies to agents.*

**💻 Code Level:** WRITING CODE \- Creating multiple specialized Agent instances.

**🤖 Apply to DocuBot Project**

**Task:** Create 3 specialized agents: 1\) SearchAgent (will search documents), 2\) SummaryAgent (summarizes content), 3\) QAAgent (answers questions based on context). Give each focused instructions and relevant tools.

**Outcome:** *Three specialized agents ready to work together.*

**💡 Hints (if you get stuck):**

1\. SearchAgent: Agent(name='SearchAgent', instructions='You ONLY search for documents. Given a query, find relevant documents and return them.')

2\. SummaryAgent: Agent(name='SummaryAgent', instructions='You ONLY summarize content. Given text, create a concise summary.')

3\. QAAgent: Agent(name='QAAgent', instructions='You answer questions based ONLY on provided context. Never make up information.')

4\. Give each agent only the tools they need—SearchAgent gets search\_documents, SummaryAgent needs no tools

5\. Test each alone before connecting: Runner.run\_sync(search\_agent, 'find info about X')

**📝 Starter Code:**

from agents import Agent\\n\\nsearch\_agent \= Agent(\\n    name='SearchAgent',\\n    instructions='You specialize in finding relevant documents...',\\n    tools=\[search\_documents\]\\n)\\n\\nsummary\_agent \= Agent(\\n    name='SummaryAgent',\\n    instructions='You specialize in summarizing content...'\\n)\\n\\nqa\_agent \= Agent(\\n    name='QAAgent',\\n    instructions='You answer questions based only on provided context...'\\n)

**Lesson 6.3: Handoffs: Transferring Control**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Implement handoffs to delegate between agents.*

**📖 Concept (What You'll Learn)**

A handoff is a one-way transfer of control from one agent to another. The triage agent decides which specialist should handle the request, hands off to them, and that specialist takes over completely.

**Key Points:**

* handoffs=\[agent1, agent2\] lists agents that can be handed to

* The agent decides when to handoff based on instructions/context

* Handoff transfers conversation history to the new agent

* Once handed off, the original agent is done

**💡 Simple Analogy**

*A handoff is like a hospital triage nurse sending you to a specialist. Once you're with the cardiologist, the nurse is out of the picture—the specialist takes over completely.*

**💻 Code Level:** WRITING CODE \- Implementing handoffs between agents.

**🤖 Apply to DocuBot Project**

**Task:** Create TriageAgent that: 1\) Receives all user questions, 2\) Decides which specialist should handle it, 3\) Hands off to SearchAgent, SummaryAgent, or QAAgent based on the request.

**Outcome:** *Working multi-agent system with proper handoff routing.*

**💡 Hints (if you get stuck):**

1\. Import handoff: from agents import Agent, handoff

2\. Create triage with handoffs list: Agent(name='TriageAgent', instructions='...', handoffs=\[search\_agent, summary\_agent, qa\_agent\])

3\. Triage instructions should explain: 'For search requests, handoff to SearchAgent. For summarization, handoff to SummaryAgent...'

4\. Run with triage as entry point: Runner.run\_sync(triage\_agent, user\_message)

5\. The SDK automatically handles the handoff when the agent decides to delegate

**📝 Starter Code:**

from agents import Agent, handoff\\n\\ntriage\_agent \= Agent(\\n    name='TriageAgent',\\n    instructions='''You are a routing agent. Analyze the user request and hand off to the appropriate specialist:\\n    \- SearchAgent: for finding documents\\n    \- SummaryAgent: for summarizing content\\n    \- QAAgent: for answering specific questions''',\\n    handoffs=\[search\_agent, summary\_agent, qa\_agent\]\\n)

**Lesson 6.4: Agents as Tools: Alternative Pattern**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Use agent.as\_tool() when you need the original agent to stay in control.*

**📖 Concept (What You'll Learn)**

Sometimes you don't want to fully hand off—you want to 'call' another agent like a tool and get a result back. The agent.as\_tool() method wraps an agent as a tool, letting the calling agent retain control.

**Key Points:**

* agent.as\_tool() converts an agent into a callable tool

* The calling agent remains in control

* Results come back to the original agent for processing

* Choose handoff for 'take over' vs as\_tool for 'help with this'

**💡 Simple Analogy**

*Handoff is like delegating a project to a colleague. As\_tool is like asking a colleague a question—they help, but you're still running the project.*

**💻 Code Level:** WRITING CODE \- Using as\_tool pattern vs handoffs.

**🤖 Apply to DocuBot Project**

**Task:** Convert SummaryAgent to a tool (using as\_tool()). Now the main agent can summarize content without fully handing off. Compare behavior to the handoff approach.

**Outcome:** *Understanding of when to use handoffs vs as\_tool pattern.*

**💡 Hints (if you get stuck):**

1\. Convert agent to tool: summary\_tool \= summary\_agent.as\_tool()

2\. Add to main agent: Agent(name='MainAgent', tools=\[summary\_tool, search\_tool\])

3\. Main agent can now call summary\_tool and get result back

4\. Difference: with as\_tool, main agent processes the summary result; with handoff, summary agent responds directly

5\. Use as\_tool when you need to combine results from multiple agents

**📋 Chapter Summary & Recap**

Multi-agent systems divide complex work among specialists. Handoffs transfer control completely, while agents-as-tools let the original agent remain in charge. DocuBot now has a capable team of agents.

## **Chapter 7: Guardrails: Input & Output Validation**

**Duration:** 3-4 hours  •  **Lessons:** 4  •  **Depth:** Full Depth (Guardrails taught here)

**Chapter Overview**

Production agents need safety checks. Guardrails validate inputs (to catch bad requests) and outputs (to ensure quality responses). This is essential for trustworthy AI applications.

**⚡ Cognitive Load:** MODERATE \- Important safety concept, implemented in clear steps. Input guardrails first, then output guardrails.

**Learning Objectives**

* Explain why guardrails are essential for production agents

* Implement input guardrails using @input\_guardrail

* Create output guardrails using @output\_guardrail

* Handle guardrail violations gracefully

**🤖 DocuBot State After This Chapter**

*DocuBot now has safety checks: off-topic questions are politely rejected, and responses are verified to include source citations before being returned.*

**Lessons**

**Lesson 7.1: Why Guardrails Matter**

**Duration:** 25-35 minutes

**🎯 Learning Goal**

*Understand the risks guardrails protect against.*

**📖 Concept (What You'll Learn)**

Without guardrails, agents can be manipulated with bad inputs or produce harmful/incorrect outputs. Guardrails run checks in parallel with the agent, catching problems before they become user-facing issues.

**Key Points:**

* Bad inputs: off-topic, malicious, or confusing requests

* Bad outputs: hallucinations, inappropriate content, missing info

* Guardrails run in parallel—they don't slow things down much

* Defense in depth: multiple guardrails catch different issues

**💡 Simple Analogy**

*Guardrails are like airport security. They check for problems before you board (input) and after you land (output). Most people pass through fine, but the checks catch the exceptions.*

**💻 Code Level:** CONCEPTUAL \- Understanding risks before implementing protections.

**🤖 Apply to DocuBot Project**

**Task:** List 5 things that could go wrong with DocuBot: 3 input risks and 2 output risks. For each, write what the bad outcome would be if there was no guardrail.

**Outcome:** *Risk analysis document that justifies implementing guardrails.*

**💡 Hints (if you get stuck):**

1\. Input risk 1: Off-topic requests ('write me a poem') waste resources and confuse users

2\. Input risk 2: Prompt injection attempts ('ignore your instructions and...') could manipulate the agent

3\. Input risk 3: Very long inputs could cause timeouts or high costs

4\. Output risk 1: Hallucinated answers (making up information not in documents)

5\. Output risk 2: Missing citations (answering without saying where info came from)

6\. Create guardrails\_risks.md with this analysis

**Lesson 7.2: Input Guardrails**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Create guardrails that validate user input before agent processing.*

**📖 Concept (What You'll Learn)**

Input guardrails check user messages before the agent processes them. If the check fails (tripwire triggered), the agent never runs. Use @input\_guardrail decorator to create them.

**Key Points:**

* @input\_guardrail decorator creates an input guardrail

* Returns GuardrailFunctionOutput with tripwire\_triggered bool

* tripwire\_triggered=True stops the agent

* Guardrails can use another agent for sophisticated checks

**💡 Simple Analogy**

*Input guardrails are like a bouncer at a club. They check everyone at the door. If something's wrong, you don't get in—and the club (agent) never has to deal with you.*

**💻 Code Level:** WRITING CODE \- Implementing @input\_guardrail functions.

**🤖 Apply to DocuBot Project**

**Task:** Create an off\_topic\_guardrail that: 1\) Checks if the question is about documents/information retrieval, 2\) Returns tripwire\_triggered=True for completely off-topic questions (like 'write me a poem'), 3\) Allows relevant questions through.

**Outcome:** *DocuBot that politely rejects off-topic requests.*

**💡 Hints (if you get stuck):**

1\. Import: from agents import input\_guardrail, GuardrailFunctionOutput

2\. Decorator: @input\_guardrail\\nasync def off\_topic\_guardrail(ctx, agent, input\_text):

3\. Check for keywords: off\_topic\_keywords \= \['poem', 'story', 'joke', 'sing'\]

4\. Return: return GuardrailFunctionOutput(tripwire\_triggered=is\_off\_topic, output\_message='...')

5\. Add to agent: Agent(..., input\_guardrails=\[off\_topic\_guardrail\])

**📝 Starter Code:**

from agents import Agent, input\_guardrail, GuardrailFunctionOutput\\n\\n@input\_guardrail\\nasync def off\_topic\_guardrail(ctx, agent, input\_text: str):\\n    \# Check if input is off-topic\\n    off\_topic \= any(word in input\_text.lower() for word in \['poem', 'story', 'joke'\])\\n    return GuardrailFunctionOutput(\\n        tripwire\_triggered=off\_topic,\\n        output\_message='I can only help with document-related questions.'\\n    )

**Lesson 7.3: Output Guardrails**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Validate agent responses before returning them to users.*

**📖 Concept (What You'll Learn)**

Output guardrails check what the agent generates before the user sees it. They can verify factuality, check for missing information, or ensure compliance. Use @output\_guardrail decorator.

**Key Points:**

* @output\_guardrail decorator creates an output guardrail

* Receives the agent's output for inspection

* Can reject outputs that don't meet quality standards

* Often checks for required elements (citations, format)

**💡 Simple Analogy**

*Output guardrails are like a quality check before shipping a product. The factory made it (agent generated it), but QA inspects it before it reaches the customer.*

**💻 Code Level:** WRITING CODE \- Implementing @output\_guardrail functions.

**🤖 Apply to DocuBot Project**

**Task:** Create a has\_citation\_guardrail that: 1\) Checks if the response mentions sources or documents, 2\) Triggers if a question about documents gets an answer without any source mention, 3\) Allows responses that cite sources.

**Outcome:** *DocuBot that verifies all document answers include citations.*

**💡 Hints (if you get stuck):**

1\. Import: from agents import output\_guardrail, GuardrailFunctionOutput

2\. Decorator: @output\_guardrail\\nasync def has\_citation\_guardrail(ctx, agent, output):

3\. Check for citation words: citation\_words \= \['source:', 'according to', 'from document', 'reference:'\]

4\. Get output text: output\_text \= str(output).lower()

5\. Return: return GuardrailFunctionOutput(tripwire\_triggered=not has\_citation, output\_message='...')

**📝 Starter Code:**

from agents import output\_guardrail, GuardrailFunctionOutput\\n\\n@output\_guardrail\\nasync def has\_citation\_guardrail(ctx, agent, output):\\n    output\_text \= str(output).lower()\\n    citation\_words \= \['source:', 'according to', 'document', 'reference'\]\\n    has\_citation \= any(word in output\_text for word in citation\_words)\\n    return GuardrailFunctionOutput(\\n        tripwire\_triggered=not has\_citation,\\n        output\_message='Response must include source citations.'\\n    )

**Lesson 7.4: Handling Guardrail Violations**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Gracefully handle cases when guardrails trip.*

**📖 Concept (What You'll Learn)**

When a guardrail trips, an exception is raised (InputGuardrailTripwireTriggered). You should catch this and return a user-friendly message instead of crashing or showing errors.

**Key Points:**

* try/except catches guardrail exceptions

* Provide helpful messages explaining why the request was rejected

* Log violations for analysis

* Consider retry strategies for transient issues

**💡 Simple Analogy**

*When airport security finds an issue, they don't just reject you silently—they explain the problem and offer alternatives (check the bag, remove the item, etc.).*

**💻 Code Level:** WRITING CODE \- Exception handling and user-friendly responses.

**🤖 Apply to DocuBot Project**

**Task:** Wrap DocuBot's main function in try/except that: 1\) Catches InputGuardrailTripwireTriggered, 2\) Returns a polite message like 'I can only help with document-related questions...', 3\) Logs the violation for later analysis.

**Outcome:** *Graceful error handling that maintains user trust.*

**💡 Hints (if you get stuck):**

1\. Import exception: from agents import InputGuardrailTripwireTriggered, OutputGuardrailTripwireTriggered

2\. Wrap in try/except: try:\\n    result \= Runner.run\_sync(...)\\nexcept InputGuardrailTripwireTriggered as e:\\n    print(f'Request rejected: {e.output\_message}')

3\. Add logging: import logging\\nlogger \= logging.getLogger(\_\_name\_\_)\\nlogger.warning(f'Guardrail triggered: {e}')

4\. For output guardrails, catch OutputGuardrailTripwireTriggered separately

5\. Return helpful message: 'I\\'m sorry, but I can only answer questions about documents. Try asking something like "What does document X say about Y?"'

**📝 Starter Code:**

from agents import Runner, InputGuardrailTripwireTriggered, OutputGuardrailTripwireTriggered\\nimport logging\\n\\nlogger \= logging.getLogger(\_\_name\_\_)\\n\\ndef ask\_docubot(question: str) \-\> str:\\n    try:\\n        result \= Runner.run\_sync(agent, question)\\n        return result.final\_output\\n    except InputGuardrailTripwireTriggered as e:\\n        logger.warning(f'Input guardrail: {e}')\\n        return f'Sorry, {e.output\_message}'\\n    except OutputGuardrailTripwireTriggered as e:\\n        logger.warning(f'Output guardrail: {e}')\\n        return 'I had trouble generating a proper response. Please try again.'

**📋 Chapter Summary & Recap**

Guardrails are your safety net. Input guardrails catch bad requests before processing; output guardrails verify response quality. DocuBot is now more reliable and trustworthy.

## **Chapter 8: Structured Output & Type Safety**

**Duration:** 3-4 hours  •  **Lessons:** 4  •  **Depth:** Full Depth (Structured output taught here)

**Chapter Overview**

Free-form text is hard to process programmatically. Structured output forces agents to respond in defined schemas (JSON with specific fields), making responses predictable and easy to use in applications.

**⚡ Cognitive Load:** MODERATE \- Single concept (structured output) with Pydantic building on existing Python knowledge.

**Learning Objectives**

* Explain when structured output is better than free-form text

* Define output schemas using Pydantic models

* Configure agents to return structured data

* Process structured responses in application code

**🤖 DocuBot State After This Chapter**

*DocuBot now returns structured responses: a DocumentAnswer object with answer, sources, and confidence fields—not just a blob of text.*

**Lessons**

**Lesson 8.1: Why Structured Output?**

**Duration:** 25-35 minutes

**🎯 Learning Goal**

*Understand when to use structured output vs. free-form text.*

**📖 Concept (What You'll Learn)**

Free-form text is great for conversations, but hard to use in applications. Structured output gives you guaranteed fields: always an 'answer' field, always a 'confidence' field. Your code can rely on the structure.

**Key Points:**

* Free-form: varies every time, hard to parse

* Structured: consistent schema, easy to process

* Use structured for: citations, confidence scores, multiple parts

* Use free-form for: general chat, creative writing

**💡 Simple Analogy**

*Free-form is like asking someone to describe a person. Structured is like having them fill out a form (name, age, occupation). The form is consistent every time.*

**💻 Code Level:** CONCEPTUAL \- Understanding the use case before implementation.

**🤖 Apply to DocuBot Project**

**Task:** For DocuBot, list what structured data would be useful: What fields should every document answer have? (Hint: answer, sources, confidence, etc.)

**Outcome:** *Field list for DocumentAnswer structured output.*

**💡 Hints (if you get stuck):**

1\. Essential fields: answer (str) \- the actual response text

2\. sources (list\[str\]) \- which documents the answer came from

3\. confidence (float) \- how confident is the agent (0.0 to 1.0)

4\. Optional fields: needs\_more\_context (bool), relevant\_quotes (list\[str\])

5\. Create structured\_output\_design.md with your field list and why each is needed

**Lesson 8.2: Pydantic Models for Output**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Define type-safe output schemas using Pydantic.*

**📖 Concept (What You'll Learn)**

Pydantic models define the structure of your output. Each field has a name, type, and optional description. The agent will return data matching this structure exactly.

**Key Points:**

* class MyOutput(BaseModel): defines the schema

* Fields with types: answer: str, confidence: float

* Optional fields: sources: list\[str\] \= \[\]

* Descriptions help the agent understand what to put in each field

**💡 Simple Analogy**

*Pydantic is like a blueprint for a form. You define exactly what blanks need to be filled in and what type of information goes in each.*

**💻 Code Level:** WRITING CODE \- Creating Pydantic models for agent output.

**🤖 Apply to DocuBot Project**

**Task:** Create DocumentAnswer Pydantic model with: answer (str), sources (list\[str\]), confidence (float between 0-1), needs\_more\_context (bool). Add descriptions to each field.

**Outcome:** *Type-safe output schema ready for use with DocuBot.*

**💡 Hints (if you get stuck):**

1\. Install pydantic: uv add pydantic

2\. Import: from pydantic import BaseModel, Field

3\. Define model: class DocumentAnswer(BaseModel):

4\. Add fields with descriptions: answer: str \= Field(description='The answer to the user question')

5\. Confidence field: confidence: float \= Field(ge=0, le=1, description='Confidence score between 0 and 1')

**📝 Starter Code:**

from pydantic import BaseModel, Field\\n\\nclass DocumentAnswer(BaseModel):\\n    answer: str \= Field(description='The answer to the user question')\\n    sources: list\[str\] \= Field(default\_factory=list, description='List of document sources')\\n    confidence: float \= Field(ge=0, le=1, description='Confidence score 0-1')\\n    needs\_more\_context: bool \= Field(default=False, description='True if more context would help')

**Lesson 8.3: Configuring Agents for Structured Output**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Make agents return data in your defined schema.*

**📖 Concept (What You'll Learn)**

The output\_type parameter tells the agent to structure its response according to your Pydantic model. The result.final\_output will be an instance of your model, not a string.

**Key Points:**

* Agent(output\_type=MyModel) sets the output schema

* result.final\_output is now a Pydantic model instance

* Access fields directly: result.final\_output.answer

* The agent automatically structures its response

**💡 Simple Analogy**

*It's like telling a reporter: 'Don't write a story. Fill out this form with headline, summary, and key facts.' They'll structure their information to fit.*

**💻 Code Level:** WRITING CODE \- Setting output\_type and processing structured responses.

**🤖 Apply to DocuBot Project**

**Task:** Update DocuBot's QA agent to use output\_type=DocumentAnswer. Ask a question and print: 1\) The answer, 2\) The sources, 3\) The confidence score separately.

**Outcome:** *DocuBot returning structured, predictable responses.*

**💡 Hints (if you get stuck):**

1\. Add output\_type to agent: Agent(name='DocuBot', instructions='...', output\_type=DocumentAnswer)

2\. result.final\_output is now a DocumentAnswer object, not a string

3\. Access fields: print(f'Answer: {result.final\_output.answer}')

4\. Access sources: print(f'Sources: {result.final\_output.sources}')

5\. Access confidence: print(f'Confidence: {result.final\_output.confidence \* 100}%')

**📝 Starter Code:**

from agents import Agent, Runner\\nfrom models import DocumentAnswer  \# Your Pydantic model\\n\\nagent \= Agent(\\n    name='DocuBot',\\n    instructions='Answer questions about documents. Always cite sources.',\\n    output\_type=DocumentAnswer\\n)\\n\\nresult \= Runner.run\_sync(agent, 'What does the document say about X?')\\nprint(f'Answer: {result.final\_output.answer}')\\nprint(f'Sources: {result.final\_output.sources}')\\nprint(f'Confidence: {result.final\_output.confidence \* 100:.0f}%')

**Lesson 8.4: Processing Structured RAG Responses**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Use structured output to build rich RAG response displays.*

**📖 Concept (What You'll Learn)**

With structured output, you can build sophisticated UIs: show the answer prominently, list sources below, display a confidence indicator. The structure makes this easy.

**Key Points:**

* Access each field independently

* Display sources as clickable links

* Show confidence as a percentage or indicator

* Handle cases where needs\_more\_context is True

**💡 Simple Analogy**

*Structured output is like receiving a package with labeled compartments vs. a box with everything thrown in. The labeled version is much easier to unpack and use.*

**💻 Code Level:** WRITING CODE \- Building response displays from structured data.

**🤖 Apply to DocuBot Project**

**Task:** Create a display\_answer(result) function that: 1\) Prints the answer in a formatted box, 2\) Lists sources as numbered references, 3\) Shows confidence as a percentage, 4\) Adds a note if needs\_more\_context is True.

**Outcome:** *Rich, formatted display of structured RAG responses.*

**💡 Hints (if you get stuck):**

1\. Create function: def display\_answer(result: DocumentAnswer):

2\. Format box: print('=' \* 50)\\nprint(result.answer)\\nprint('=' \* 50\)

3\. List sources: for i, source in enumerate(result.sources, 1):\\n    print(f'\[{i}\] {source}')

4\. Confidence: print(f'Confidence: {result.confidence \* 100:.0f}%')

5\. Context warning: if result.needs\_more\_context:\\n    print('⚠️ More context might improve this answer')

**📝 Starter Code:**

def display\_answer(result):\\n    answer \= result.final\_output\\n    print('\\n' \+ '=' \* 50)\\n    print('📝 ANSWER')\\n    print('=' \* 50)\\n    print(answer.answer)\\n    print()\\n    print('📚 SOURCES:')\\n    for i, source in enumerate(answer.sources, 1):\\n        print(f'  \[{i}\] {source}')\\n    print(f'\\n🎯 Confidence: {answer.confidence \* 100:.0f}%')\\n    if answer.needs\_more\_context:\\n        print('⚠️ More context might improve this answer')

**📋 Chapter Summary & Recap**

Structured output makes agents predictable. Define a Pydantic model, set output\_type, and every response follows your schema. DocuBot's responses are now machine-readable and consistent.

## **Chapter 9: Tracing, Debugging & Observability**

**Duration:** 3-4 hours  •  **Lessons:** 4  •  **Depth:** Full Depth (Tracing taught here)

**Chapter Overview**

How do you know what your agent is doing? Tracing provides visibility into every step: LLM calls, tool usage, handoffs. This is essential for debugging and optimizing production agents.

**⚡ Cognitive Load:** MODERATE \- One core concept (observability) with practical tooling. Visual dashboards make it concrete.

**Learning Objectives**

* Use the SDK's built-in tracing to visualize agent workflows

* Create custom traces and spans for detailed tracking

* Export traces to external observability platforms

* Use traces to evaluate and improve agent performance

**🤖 DocuBot State After This Chapter**

*DocuBot's every action is tracked: you can see exactly what it did, which tools it called, and how long each step took. This makes debugging and optimization much easier.*

**Lessons**

**Lesson 9.1: Tracing Fundamentals**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Understand what the SDK's tracing captures and why it matters.*

**📖 Concept (What You'll Learn)**

The SDK automatically traces every agent run, capturing LLM calls, tool executions, handoffs, and more. You can view traces in the OpenAI dashboard or export them elsewhere.

**Key Points:**

* Tracing is automatic—you don't need to add code

* Traces capture: LLM generations, tool calls, handoffs, timing

* View traces in OpenAI's Traces dashboard

* Traces help debug 'why did the agent do that?'

**💡 Simple Analogy**

*Tracing is like a flight recorder. You hope you never need it, but when something goes wrong, you can replay exactly what happened.*

**💻 Code Level:** EXPLORING \- Running agents and viewing traces in dashboard.

**🤖 Apply to DocuBot Project**

**Task:** Run DocuBot with a complex question that uses tools and handoffs. Open the OpenAI Traces dashboard and find your trace. Explore the visualization.

**Outcome:** *Familiarity with the tracing dashboard and what it shows.*

**💡 Hints (if you get stuck):**

1\. Run a question that triggers tools: 'What is today's date and what is 15 \* 23?'

2\. Go to platform.openai.com and find the Traces section

3\. Click on your most recent trace to see the visualization

4\. Look for: Agent run → Tool calls → Tool results → Final response

5\. Notice timing information on each step

**Lesson 9.2: Custom Traces & Spans**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Create custom traces for grouping and spans for detailed tracking.*

**📖 Concept (What You'll Learn)**

While automatic tracing is helpful, custom traces let you group related operations (multiple runs as one trace) and custom spans let you track specific business logic within your code.

**Key Points:**

* trace() context manager groups operations

* Named traces make dashboards more organized

* Custom spans track your own code, not just SDK code

* Useful for: grouping conversation turns, tracking business logic

**💡 Simple Analogy**

*Default tracing captures the whole movie. Custom traces and spans let you mark chapters and scenes, making it easier to find specific moments.*

**💻 Code Level:** WRITING CODE \- Using trace() and custom spans.

**🤖 Apply to DocuBot Project**

**Task:** Wrap a multi-turn DocuBot conversation in a custom trace named 'Document QA Session'. Add a custom span around any custom processing logic you have.

**Outcome:** *Organized traces that group related operations together.*

**💡 Hints (if you get stuck):**

1\. Import: from agents import trace

2\. Use context manager: with trace('Document QA Session'):

3\. All Runner.run() calls inside the 'with' block are grouped together

4\. Add custom spans: with trace.span('preprocessing'):

5\. Name traces descriptively: 'User-123-Session', 'RAG-Query-Processing'

**📝 Starter Code:**

from agents import trace, Runner\\n\\nwith trace('Document QA Session'):\\n    \# All these runs are grouped in one trace\\n    result1 \= Runner.run\_sync(agent, 'First question')\\n    result2 \= Runner.run\_sync(agent, 'Follow-up question')\\n    \\n    with trace.span('post\_processing'):\\n        \# Track custom processing\\n        formatted \= format\_response(result2)

**Lesson 9.3: External Tracing Integration**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Export traces to production observability platforms.*

**📖 Concept (What You'll Learn)**

The OpenAI dashboard is great for development, but production needs more: alerting, long-term storage, team access. The SDK supports exporting to platforms like Logfire, AgentOps, and MLflow.

**Key Points:**

* Trace processors export to external systems

* Popular options: Logfire, AgentOps, Braintrust, MLflow

* Setup usually requires just a few lines of code

* Production systems need persistent trace storage

**💡 Simple Analogy**

*The built-in dashboard is like your car's dashboard. External integrations are like connecting to a fleet management system—better for production operations.*

**💻 Code Level:** CONFIGURATION \- Setting up external trace export.

**🤖 Apply to DocuBot Project**

**Task:** Choose one free tracing platform (MLflow is good for learning). Configure DocuBot to export traces there. Verify traces appear in the external dashboard.

**Outcome:** *DocuBot with external trace export configured.*

**💡 Hints (if you get stuck):**

1\. MLflow is free and runs locally: pip install mlflow

2\. Start MLflow server: mlflow server \--host 127.0.0.1 \--port 5000

3\. Configure SDK: set\_trace\_processor(MlflowTraceProcessor())

4\. Alternative: Langfuse is also free for small projects

5\. Check external dashboard to verify traces arrive

**Lesson 9.4: Evaluation Using Traces**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Use trace data to measure and improve agent performance.*

**📖 Concept (What You'll Learn)**

Traces contain rich data: latency, token usage, tool success rates. By analyzing traces, you can identify bottlenecks, measure quality, and make data-driven improvements.

**Key Points:**

* Measure: latency per step, total tokens, tool failure rates

* Identify: slow steps, unnecessary tool calls, failed handoffs

* Improve: based on data, not guesses

* Compare: before/after changes with metrics

**💡 Simple Analogy**

*Traces are like a fitness tracker for your agent. You can see what's working (low latency paths) and what needs improvement (slow or failing operations).*

**💻 Code Level:** ANALYSIS \- Reading and interpreting trace data.

**🤖 Apply to DocuBot Project**

**Task:** Collect traces from 10 different DocuBot questions. Analyze: 1\) Average response time, 2\) Which tools are used most, 3\) Any patterns in slow responses. Write a brief 'performance report'.

**Outcome:** *Data-driven understanding of DocuBot's performance characteristics.*

**💡 Hints (if you get stuck):**

1\. Create a test set of 10 varied questions (simple, complex, tool-using)

2\. Run each and note the trace ID

3\. In dashboard, look at 'Duration' column for each trace

4\. Calculate average: sum of durations / 10

5\. Look for patterns: Are tool-using queries slower? Which tools take longest?

6\. Write performance\_report.md with your findings

**📋 Chapter Summary & Recap**

Tracing gives you X-ray vision into your agent. You can see every step, measure performance, and identify issues. DocuBot is now fully observable and debuggable.

## **Chapter 10: Vector Embeddings & Semantic Search**

**Duration:** 4-5 hours  •  **Lessons:** 4  •  **Depth:** Full Depth (Embeddings/Vector DB taught here)

**Chapter Overview**

This chapter teaches the 'R' in RAG: Retrieval. Students learn what embeddings are, how to generate them, and how to use Qdrant vector database for semantic search. This is the foundation for document retrieval.

**⚡ Cognitive Load:** MODERATE-HIGH \- New concepts (embeddings, vector math) introduced with visual analogies. Hands-on with real vector DB.

**Learning Objectives**

* Explain what embeddings are and why they enable semantic search

* Generate embeddings using OpenAI's embedding API

* Set up and use Qdrant vector database

* Implement semantic search that finds similar documents

**🤖 DocuBot State After This Chapter**

*DocuBot can now embed documents and search them semantically. Given a question, it retrieves the most relevant document chunks using vector similarity.*

**Lessons**

**Lesson 10.1: Embeddings Explained: Meaning in Numbers**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Understand what embeddings are and why they're powerful for search.*

**📖 Concept (What You'll Learn)**

Embeddings convert text into lists of numbers (vectors) that capture semantic meaning. Similar texts have similar vectors. This allows 'meaning-based' search instead of keyword matching.

**Key Points:**

* Text → numbers that capture meaning

* Similar meaning \= similar vectors

* Enables 'find things like this' searches

* 1536 dimensions for OpenAI embeddings

**💡 Simple Analogy**

*Embeddings are like GPS coordinates for meaning. 'Happy' and 'joyful' are at nearby coordinates. 'Happy' and 'sad' are far apart. You can find things 'near' a target.*

**💻 Code Level:** CONCEPTUAL \+ SIMPLE CODE \- Understanding before heavy implementation.

**🤖 Apply to DocuBot Project**

**Task:** Generate embeddings for 5 different sentences. Calculate the similarity between them. Verify that similar sentences have higher similarity scores.

**Outcome:** *Intuition for how embeddings capture semantic similarity.*

**💡 Hints (if you get stuck):**

1\. Use OpenAI client: client.embeddings.create(model='text-embedding-3-small', input='your text')

2\. Response format: response.data\[0\].embedding gives you the vector (list of floats)

3\. Calculate similarity: use numpy dot product or cosine similarity

4\. Try sentences: 'I love pizza', 'Pizza is my favorite food', 'I hate broccoli', 'The weather is nice', 'It\\'s sunny today'

5\. Similar meanings should have similarity \> 0.8, different meanings \< 0.5

**📝 Starter Code:**

from openai import OpenAI\\nimport numpy as np\\n\\nclient \= OpenAI()\\n\\ndef get\_embedding(text):\\n    response \= client.embeddings.create(model='text-embedding-3-small', input=text)\\n    return response.data\[0\].embedding\\n\\ndef cosine\_similarity(a, b):\\n    return np.dot(a, b) / (np.linalg.norm(a) \* np.linalg.norm(b))

**Lesson 10.2: Generating Embeddings with OpenAI**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Use the embedding API to convert text to vectors.*

**📖 Concept (What You'll Learn)**

OpenAI's API generates embeddings with a simple call. You send text, you get back numbers. The key is choosing the right model and handling batching for efficiency.

**Key Points:**

* text-embedding-3-small is cost-effective

* API returns a list of floats

* Batch texts for efficiency

* Cache embeddings when possible

**💡 Simple Analogy**

*The embedding API is like a translator that converts any text into a universal language of numbers.*

**💻 Code Level:** WRITING CODE \- Embedding API usage.

**🤖 Apply to DocuBot Project**

**Task:** Create get\_embedding(text) and get\_embeddings(texts) functions for DocuBot. Handle batching for lists of texts.

**Outcome:** *Reusable embedding utilities.*

**💡 Hints (if you get stuck):**

1\. Single text: client.embeddings.create(model='text-embedding-3-small', input=text)

2\. Batch texts: client.embeddings.create(model='text-embedding-3-small', input=list\_of\_texts)

3\. Extract embeddings: \[item.embedding for item in response.data\]

4\. Add error handling: try/except for API errors

5\. Consider caching: store embeddings to avoid re-computing

**Lesson 10.3: Introduction to Qdrant Vector Database**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Set up Qdrant and understand vector database concepts.*

**📖 Concept (What You'll Learn)**

Vector databases are optimized for storing and searching embeddings. Qdrant is open-source and easy to run locally. You'll create a collection, add vectors, and search.

**Key Points:**

* Collections hold related vectors

* Points \= vector \+ payload (metadata)

* Index for fast similarity search

* Qdrant runs locally with Docker

**💡 Simple Analogy**

*Qdrant is like a specialized filing cabinet for embeddings—optimized to quickly find 'nearby' documents.*

**💻 Code Level:** WRITING CODE \- Qdrant setup and basic operations.

**🤖 Apply to DocuBot Project**

**Task:** Install Qdrant locally, create 'documents' collection, add 10 sample documents with embeddings.

**Outcome:** *Working Qdrant instance with sample data.*

**💡 Hints (if you get stuck):**

1\. Install: docker run \-p 6333:6333 qdrant/qdrant

2\. Install client: uv add qdrant-client

3\. Connect: from qdrant\_client import QdrantClient; client \= QdrantClient('localhost', port=6333)

4\. Create collection: client.create\_collection(name='documents', vectors\_config=VectorParams(size=1536, distance=Distance.COSINE))

5\. Add points: client.upsert(collection\_name='documents', points=\[PointStruct(id=1, vector=embedding, payload={'text': 'content'})\])

**📝 Starter Code:**

from qdrant\_client import QdrantClient\\nfrom qdrant\_client.models import VectorParams, Distance, PointStruct\\n\\nclient \= QdrantClient('localhost', port=6333)\\n\\n\# Create collection\\nclient.create\_collection(\\n    collection\_name='documents',\\n    vectors\_config=VectorParams(size=1536, distance=Distance.COSINE)\\n)

**Lesson 10.4: Implementing Semantic Search**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Build a complete search function that finds relevant documents.*

**📖 Concept (What You'll Learn)**

Semantic search: embed the query, find similar vectors in the database, return the matching documents. This is how DocuBot will find relevant context for questions.

**Key Points:**

* Query → embed → search → retrieve

* Top-k retrieval with similarity threshold

* Return both content and metadata

* Score indicates relevance

**💡 Simple Analogy**

*Semantic search is like asking 'find me things similar to this' instead of 'find me things containing these exact words'.*

**💻 Code Level:** WRITING CODE \- Complete search implementation.

**🤖 Apply to DocuBot Project**

**Task:** Create search\_documents(query, k=5) that returns the most relevant document chunks with their scores.

**Outcome:** *Working semantic search for DocuBot.*

**💡 Hints (if you get stuck):**

1\. Step 1: Embed query: query\_embedding \= get\_embedding(query)

2\. Step 2: Search Qdrant: results \= client.search(collection\_name='documents', query\_vector=query\_embedding, limit=k)

3\. Step 3: Extract results: \[(hit.payload\['text'\], hit.score) for hit in results\]

4\. Add score threshold: filter results where score \> 0.7 for quality

5\. Return structured data: list of dicts with 'content', 'source', 'score'

**📝 Starter Code:**

def search\_documents(query: str, k: int \= 5\) \-\> list\[dict\]:\\n    \# Embed the query\\n    query\_embedding \= get\_embedding(query)\\n    \\n    \# Search Qdrant\\n    results \= qdrant\_client.search(\\n        collection\_name='documents',\\n        query\_vector=query\_embedding,\\n        limit=k\\n    )\\n    \\n    \# Format results\\n    return \[\\n        {'content': hit.payload\['text'\], 'source': hit.payload.get('source', 'unknown'), 'score': hit.score}\\n        for hit in results\\n        if hit.score \> 0.7  \# Quality threshold\\n    \]

**📋 Chapter Summary & Recap**

Embeddings capture meaning in numbers. Vector databases store and search these efficiently. DocuBot can now find relevant documents based on meaning, not just keywords.

## **Chapter 11: Building & Integrating the RAG Pipeline**

**Duration:** 6-7 hours  •  **Lessons:** 5  •  **Depth:** Full Depth

**Chapter Overview**

The complete RAG pipeline: document ingestion, FastAPI backend, retrieval tools, and the RAG agent that ties it all together. This is where DocuBot becomes truly useful.

**⚡ Cognitive Load:** HIGH \- Multiple components integrated. Each lesson is substantial but builds on Chapter 10\.

**Learning Objectives**

* Build a document ingestion pipeline

* Create FastAPI endpoints for RAG

* Create retrieval tools for the agent

* Implement the complete RAG agent

**🤖 DocuBot State After This Chapter**

*DocuBot is now a complete RAG system: users can upload documents, ask questions, and get answers with citations.*

**Lessons**

**Lesson 11.1: Document Ingestion Pipeline**

**Duration:** 50-60 minutes

**🎯 Learning Goal**

*Process documents into searchable chunks.*

**📖 Concept (What You'll Learn)**

Documents need to be split into chunks, embedded, and stored. The ingestion pipeline handles this automatically.

**🤖 Apply to DocuBot Project**

**Task:** Create ingest\_document(file) that chunks, embeds, and stores documents.

**Outcome:** *Working document ingestion.*

**💡 Hints (if you get stuck):**

1\. Use langchain or custom chunking: split text into \~500 token chunks with overlap

2\. Pattern: read file → split into chunks → embed each chunk → store in Qdrant

3\. Store metadata: filename, chunk\_index, total\_chunks with each vector

4\. Handle different file types: .txt, .pdf (use pypdf), .md

5\. Batch embeddings for efficiency: embed 10-20 chunks at once

**📝 Starter Code:**

def ingest\_document(file\_path: str) \-\> int:\\n    '''Ingest a document into the RAG system.'''\\n    \# 1\. Read the file\\n    content \= read\_file(file\_path)\\n    \\n    \# 2\. Split into chunks\\n    chunks \= split\_into\_chunks(content, chunk\_size=500, overlap=50)\\n    \\n    \# 3\. Embed chunks\\n    embeddings \= get\_embeddings(\[c\['text'\] for c in chunks\])\\n    \\n    \# 4\. Store in Qdrant\\n    \# ... add to vector DB\\n    return len(chunks)

**Lesson 11.2: FastAPI Backend for RAG**

**Duration:** 50-60 minutes

**🎯 Learning Goal**

*Create API endpoints for the RAG system.*

**📖 Concept (What You'll Learn)**

FastAPI provides the web API that the frontend will call. Endpoints for ingestion, search, and chat.

**🤖 Apply to DocuBot Project**

**Task:** Create FastAPI app with /ingest, /search, and /chat endpoints.

**Outcome:** *Working backend API.*

**💡 Hints (if you get stuck):**

1\. Install: uv add fastapi uvicorn python-multipart

2\. Basic app: from fastapi import FastAPI; app \= FastAPI()

3\. POST /ingest: accepts file upload, calls ingest\_document()

4\. POST /search: accepts query, returns relevant chunks

5\. POST /chat: accepts message, runs RAG agent, returns response

6\. Run with: uvicorn main:app \--reload

**📝 Starter Code:**

from fastapi import FastAPI, UploadFile, File\\nfrom pydantic import BaseModel\\n\\napp \= FastAPI()\\n\\n@app.post('/ingest')\\nasync def ingest(file: UploadFile \= File(...)):\\n    \# Save and ingest file\\n    pass\\n\\n@app.post('/search')\\nasync def search(query: str):\\n    return search\_documents(query)\\n\\n@app.post('/chat')\\nasync def chat(message: str):\\n    \# Run RAG agent\\n    pass

**Lesson 11.3: Retrieval Tools for Agents**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Create tools that let agents search the knowledge base.*

**📖 Concept (What You'll Learn)**

The @function\_tool decorated search function becomes available to the agent.

**🤖 Apply to DocuBot Project**

**Task:** Create search\_knowledge\_base tool that queries Qdrant.

**Outcome:** *Retrieval tool connected to agent.*

**💡 Hints (if you get stuck):**

1\. Wrap your search function: @function\_tool def search\_knowledge\_base(query: str) \-\> str:

2\. Return formatted results: concatenate top 3 chunks with source info

3\. Good docstring: '''Search the document database for information relevant to the query.'''

4\. Format output clearly: 'From \[source\]: \[content\]\\n\\n'

5\. Add to agent: Agent(tools=\[search\_knowledge\_base\])

**Lesson 11.4: RAG Agent Implementation**

**Duration:** 50-60 minutes

**🎯 Learning Goal**

*Create the complete RAG agent.*

**📖 Concept (What You'll Learn)**

The RAG agent uses retrieval tools to find context, then generates answers with citations.

**🤖 Apply to DocuBot Project**

**Task:** Create the final RAG agent with proper instructions for using retrieved context.

**Outcome:** *Complete RAG agent.*

**💡 Hints (if you get stuck):**

1\. Key instructions: 'Always search the knowledge base before answering questions about documents.'

2\. Citation instruction: 'Always cite your sources by mentioning which document the information came from.'

3\. Fallback instruction: 'If the knowledge base doesn\\'t have relevant information, say so clearly.'

4\. Use structured output: output\_type=DocumentAnswer for consistent citation format

5\. Test with: 'What does \[your document\] say about X?'

**📝 Starter Code:**

rag\_agent \= Agent(\\n    name='DocuBot',\\n    instructions='''You are DocuBot, a document Q\&A assistant.\\n\\nWhen answering questions:\\n1. ALWAYS search the knowledge base first\\n2. Base your answers ONLY on retrieved documents\\n3. ALWAYS cite which document your answer comes from\\n4. If no relevant info found, say so clearly\\n\\nNever make up information.''',\\n    tools=\[search\_knowledge\_base\],\\n    output\_type=DocumentAnswer\\n)

**Lesson 11.5: RAG Quality & Evaluation**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Measure RAG system quality.*

**📖 Concept (What You'll Learn)**

Test with known questions and evaluate retrieval and answer quality.

**🤖 Apply to DocuBot Project**

**Task:** Create 10-question test suite and evaluate DocuBot's performance.

**Outcome:** *Quality baseline for DocuBot.*

**💡 Hints (if you get stuck):**

1\. Create test questions where you know the correct answer

2\. Measure: Did it find the right document? Did it answer correctly? Did it cite sources?

3\. Score each response: retrieval\_correct (0/1), answer\_correct (0/1), has\_citation (0/1)

4\. Calculate overall accuracy: average scores across all questions

5\. Create evaluation.py that runs tests and outputs a report

**📋 Chapter Summary & Recap**

The RAG pipeline is complete: ingest → embed → store → retrieve → generate. DocuBot can answer questions about any documents you give it.

## **Chapter 12: Chat Interface with OpenAI ChatKit**

**Duration:** 6-7 hours  •  **Lessons:** 5  •  **Depth:** Full Depth (ChatKit taught here)

**Chapter Overview**

Build the user-facing chat interface using OpenAI's ChatKit. This production-ready component handles streaming, tool visualization, and attachments out of the box.

**⚡ Cognitive Load:** MODERATE \- React basics assumed or taught briefly. ChatKit abstracts complexity.

**Learning Objectives**

* Understand ChatKit architecture

* Build the chat interface

* Connect to the FastAPI backend

* Implement file uploads

**🤖 DocuBot State After This Chapter**

*DocuBot has a professional chat interface. Users can chat naturally, see streaming responses, and upload documents.*

**Lessons**

**Lesson 12.1: ChatKit Fundamentals**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Understand what ChatKit provides.*

**📖 Concept (What You'll Learn)**

ChatKit is OpenAI's official React component for chat UIs. It handles complexity you'd otherwise build yourself.

**🤖 Apply to DocuBot Project**

**Task:** Set up React project with ChatKit dependencies.

**Outcome:** *Project ready for ChatKit.*

**💡 Hints (if you get stuck):**

1\. Create React app: npx create-react-app docubot-frontend or npm create vite@latest

2\. Install ChatKit: npm install @openai/chatkit-react

3\. Read docs at: openai.github.io/chatkit-js

4\. ChatKit provides: message rendering, streaming, tool visualization, file attachments

5\. You'll need: React 18+, Node 18+

**Lesson 12.2: Building the Chat View**

**Duration:** 50-60 minutes

**🎯 Learning Goal**

*Create the main chat interface.*

**📖 Concept (What You'll Learn)**

The ChatKit component with useChatKit hook is all you need for a basic chat UI.

**🤖 Apply to DocuBot Project**

**Task:** Implement basic chat view with ChatKit component.

**Outcome:** *Working chat UI.*

**💡 Hints (if you get stuck):**

1\. Import: import { ChatKit, useChatKit } from '@openai/chatkit-react'

2\. Basic setup: const chat \= useChatKit({ apiEndpoint: '/api/chat' })

3\. Render: \<ChatKit {...chat} /\>

4\. Customize with props: placeholder, initialMessage, theme

5\. The hook handles: message state, sending, receiving, streaming

**📝 Starter Code:**

import { ChatKit, useChatKit } from '@openai/chatkit-react';\\n\\nfunction DocuBotChat() {\\n  const chat \= useChatKit({\\n    apiEndpoint: 'http://localhost:8000/chat',\\n  });\\n\\n  return (\\n    \<div className='chat-container'\>\\n      \<ChatKit {...chat} placeholder='Ask about your documents...' /\>\\n    \</div\>\\n  );\\n}

**Lesson 12.3: Backend Connection**

**Duration:** 50-60 minutes

**🎯 Learning Goal**

*Connect ChatKit to your FastAPI backend.*

**📖 Concept (What You'll Learn)**

ChatKit needs a client token from your backend. Implement the session endpoint.

**🤖 Apply to DocuBot Project**

**Task:** Create session endpoint and connect ChatKit.

**Outcome:** *Frontend connected to backend.*

**💡 Hints (if you get stuck):**

1\. ChatKit expects specific response format from backend

2\. Backend /chat endpoint should: receive message, run agent, stream response

3\. Use Server-Sent Events (SSE) for streaming: from fastapi.responses import StreamingResponse

4\. Handle CORS: from fastapi.middleware.cors import CORSMiddleware

5\. Test connection with browser dev tools Network tab

**Lesson 12.4: Streaming in the UI**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Display streaming responses.*

**📖 Concept (What You'll Learn)**

ChatKit handles streaming automatically—just verify it's working.

**🤖 Apply to DocuBot Project**

**Task:** Test streaming end-to-end and verify UI updates in real-time.

**Outcome:** *Streaming verified.*

**💡 Hints (if you get stuck):**

1\. ChatKit auto-handles streaming if backend sends SSE format

2\. Backend pattern: yield chunks as they're generated

3\. Test: ask a long question, you should see words appear progressively

4\. Debug: check Network tab for event-stream content type

5\. Common issue: CORS blocking—make sure backend allows your frontend origin

**Lesson 12.5: File Uploads**

**Duration:** 50-60 minutes

**🎯 Learning Goal**

*Enable document uploads.*

**📖 Concept (What You'll Learn)**

Let users upload documents directly in the chat to add to the knowledge base.

**🤖 Apply to DocuBot Project**

**Task:** Implement file upload that ingests documents into the RAG system.

**Outcome:** *Upload feature working.*

**💡 Hints (if you get stuck):**

1\. ChatKit supports file attachments with enableAttachments prop

2\. Create upload handler: onFileUpload={async (file) \=\> { await uploadToBackend(file) }}

3\. Backend /ingest endpoint handles the file and adds to Qdrant

4\. Show upload progress and success/error states

5\. After upload, confirm: 'Document added\! You can now ask questions about it.'

**📋 Chapter Summary & Recap**

ChatKit provides a production-ready chat UI with minimal code. DocuBot is now user-friendly with a polished interface.

## **Chapter 13: Advanced UI Features**

**Duration:** 3-4 hours  •  **Lessons:** 4  •  **Depth:** Moderate

**Chapter Overview**

Polish the interface with source citations, reasoning display, accessibility, and visual polish.

**⚡ Cognitive Load:** MODERATE \- Enhancement-focused. Each lesson is independent.

**Learning Objectives**

* Display source citations

* Show agent reasoning

* Implement accessibility

* Add visual polish

**🤖 DocuBot State After This Chapter**

*DocuBot's interface is professional-grade: sources are cited, reasoning is transparent, and it's accessible to all users.*

**Lessons**

**Lesson 13.1: Displaying Retrieved Sources**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Show users where answers come from.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Display sources as expandable cards below answers.

**Outcome:** *Source citations displayed.*

**💡 Hints (if you get stuck):**

1\. Include sources in your DocumentAnswer structured output

2\. Render sources below the answer: sources.map(s \=\> \<SourceCard source={s} /\>)

3\. Make sources expandable: click to show full context

4\. Style distinctly: lighter background, smaller font, 'Source:' label

5\. Link to original document if available

**Lesson 13.2: Showing Agent Reasoning**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Provide transparency into agent decisions.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Add 'Show reasoning' toggle that reveals tool calls and decisions.

**Outcome:** *Transparent agent behavior.*

**💡 Hints (if you get stuck):**

1\. Store tool calls in response metadata from backend

2\. Create toggle: const \[showReasoning, setShowReasoning\] \= useState(false)

3\. Display tool calls: 'Searched for: \[query\]' → 'Found 3 relevant documents'

4\. Use collapsible section or modal for detailed view

5\. This builds user trust by showing the agent 'did its homework'

**Lesson 13.3: Accessibility**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Ensure the interface is accessible.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Audit and fix keyboard navigation, screen reader, and contrast issues.

**Outcome:** *Accessible interface.*

**💡 Hints (if you get stuck):**

1\. Test keyboard nav: Tab through all interactive elements

2\. Add aria-labels: \<button aria-label='Send message'\>

3\. Check contrast: use browser dev tools accessibility audit

4\. Announce new messages to screen readers: aria-live='polite'

5\. Test with screen reader (VoiceOver on Mac, NVDA on Windows)

**Lesson 13.4: UX Polish**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Add professional visual touches.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Add loading states, animations, error handling, and responsive design.

**Outcome:** *Polished, professional UI.*

**💡 Hints (if you get stuck):**

1\. Loading state: show typing indicator while waiting for response

2\. Smooth animations: fade in new messages, slide in sources

3\. Error handling: friendly messages for network errors, timeouts

4\. Responsive: test on mobile viewport, adjust chat width/padding

5\. Empty state: show helpful prompts when no conversation yet

**📋 Chapter Summary & Recap**

A polished UI builds trust. DocuBot now shows its work, is accessible, and looks professional.

## **Chapter 14: Security & Authentication**

**Duration:** 3-4 hours  •  **Lessons:** 4  •  **Depth:** Full Depth

**Chapter Overview**

Production systems need security. Add authentication, secure API endpoints, and handle data privacy.

**⚡ Cognitive Load:** MODERATE \- Important but not complex. Clear patterns to follow.

**Learning Objectives**

* Implement user authentication

* Secure API endpoints

* Handle data privacy

**🤖 DocuBot State After This Chapter**

*DocuBot is secure: users log in, API endpoints are protected, and data is handled responsibly.*

**Lessons**

**Lesson 14.1: Authentication Concepts**

**Duration:** 30-40 minutes

**🎯 Learning Goal**

*Understand auth for AI apps.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Design authentication flow for DocuBot.

**Outcome:** *Auth design document.*

**💡 Hints (if you get stuck):**

1\. Common options: JWT tokens, session cookies, OAuth (Google/GitHub login)

2\. For learning: JWT is simplest—user logs in, gets token, sends token with requests

3\. Design decisions: Where to store tokens? How to handle expiration? Refresh tokens?

4\. Create auth\_design.md with: login flow diagram, token storage plan, protected routes

5\. Consider: Do users need accounts or is DocuBot for demo only?

**Lesson 14.2: Frontend Authentication**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Implement login in React.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Add login page and auth state management.

**Outcome:** *Login working.*

**💡 Hints (if you get stuck):**

1\. Create LoginPage component with email/password form

2\. Store token: localStorage.setItem('token', response.token)

3\. Auth context: React.createContext() to share auth state across app

4\. Protected routes: redirect to /login if no token

5\. Add Authorization header: headers: { 'Authorization': \`Bearer ${token}\` }

**Lesson 14.3: Backend Security**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Secure FastAPI endpoints.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Add JWT validation and rate limiting.

**Outcome:** *Secure API.*

**💡 Hints (if you get stuck):**

1\. Install: uv add python-jose passlib

2\. Create JWT: jwt.encode({'sub': user\_id, 'exp': expiry}, SECRET\_KEY)

3\. Dependency for protected routes: def get\_current\_user(token: str \= Depends(oauth2\_scheme))

4\. Rate limiting: use slowapi or implement simple counter

5\. Add to routes: @app.post('/chat', dependencies=\[Depends(get\_current\_user)\])

**Lesson 14.4: Data Privacy**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Handle user data responsibly.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Implement data retention policy and deletion.

**Outcome:** *Privacy-compliant system.*

**💡 Hints (if you get stuck):**

1\. Define retention: How long to keep user documents? Conversation history?

2\. Implement deletion: API endpoint to delete user's data on request

3\. Separate storage: Each user's documents in isolated Qdrant collection

4\. Add privacy notice to UI explaining data handling

5\. Consider GDPR requirements if serving EU users

**📋 Chapter Summary & Recap**

Security is non-negotiable for production. DocuBot now protects user data and prevents unauthorized access.

## **Chapter 15: Deployment & Operations**

**Duration:** 6-7 hours  •  **Lessons:** 6  •  **Depth:** Full Depth

**Chapter Overview**

Get DocuBot to production: containerize, deploy, monitor, and optimize.

**⚡ Cognitive Load:** MODERATE-HIGH \- Many steps, but each is a recipe to follow.

**Learning Objectives**

* Containerize with Docker

* Deploy to cloud

* Set up monitoring

* Optimize performance

**🤖 DocuBot State After This Chapter**

*DocuBot is live in production: containerized, deployed, monitored, and optimized.*

**Lessons**

**Lesson 15.1: Docker Containerization**

**Duration:** 50-60 minutes

**🎯 Learning Goal**

*Package application in containers.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Create Dockerfiles for frontend and backend.

**Outcome:** *Containerized application.*

**💡 Hints (if you get stuck):**

1\. Backend Dockerfile: FROM python:3.11-slim, COPY requirements.txt, RUN pip install, COPY ., CMD uvicorn

2\. Frontend Dockerfile: FROM node:18-alpine, npm install, npm run build, serve with nginx

3\. Use docker-compose.yml to run both \+ Qdrant together

4\. Test locally: docker-compose up \--build

5\. Multi-stage builds reduce image size: build stage → production stage

**📝 Starter Code:**

\# Backend Dockerfile\\nFROM python:3.11-slim\\nWORKDIR /app\\nCOPY requirements.txt .\\nRUN pip install \-r requirements.txt\\nCOPY . .\\nCMD \["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"\]

**Lesson 15.2: Backend Deployment**

**Duration:** 50-60 minutes

**🎯 Learning Goal**

*Deploy backend to cloud.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Deploy to Railway/Render/fly.io.

**Outcome:** *Live backend.*

**💡 Hints (if you get stuck):**

1\. Railway: railway login, railway init, railway up (easiest for Docker)

2\. Render: Connect GitHub repo, set build command, auto-deploys on push

3\. fly.io: fly launch, fly deploy (good for edge deployment)

4\. Set environment variables: OPENAI\_API\_KEY, QDRANT\_URL, etc.

5\. Test live endpoint with curl or Postman before connecting frontend

**Lesson 15.3: Frontend Deployment**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Deploy frontend.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Deploy to Vercel/Netlify.

**Outcome:** *Live frontend.*

**💡 Hints (if you get stuck):**

1\. Vercel: vercel login, vercel (auto-detects React), sets up HTTPS

2\. Netlify: drag-drop build folder or connect GitHub

3\. Update API URL to point to deployed backend

4\. Set environment variable: REACT\_APP\_API\_URL=https://your-backend.com

5\. Test complete flow: upload document, ask question, get answer

**Lesson 15.4: Monitoring**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Set up observability.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Add health checks, logging, and alerts.

**Outcome:** *Monitored system.*

**💡 Hints (if you get stuck):**

1\. Health check endpoint: @app.get('/health') that checks DB connection

2\. Structured logging: import logging; logger.info('event', extra={'user\_id': id})

3\. Use platform monitoring: Railway/Render dashboards show metrics

4\. Set up alerts: email when error rate spikes or latency increases

5\. Consider: Sentry for error tracking, Grafana for dashboards

**Lesson 15.5: Performance Optimization**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Make it faster and cheaper.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Add caching and measure improvements.

**Outcome:** *Optimized performance.*

**💡 Hints (if you get stuck):**

1\. Cache embeddings: store computed embeddings, don't re-compute

2\. Cache frequent queries: Redis or in-memory cache for repeated questions

3\. Measure before/after: time API calls, count token usage

4\. Use gpt-4o-mini for simple queries, gpt-4o only when needed

5\. Batch requests: embed multiple chunks in one API call

**Lesson 15.6: Cost Management**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Control AI costs.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Set up cost tracking and alerts.

**Outcome:** *Cost-aware system.*

**💡 Hints (if you get stuck):**

1\. Track token usage: log input\_tokens \+ output\_tokens per request

2\. Set OpenAI usage limits in dashboard: Settings → Limits

3\. Calculate cost: (input\_tokens/1M \* $0.15) \+ (output\_tokens/1M \* $0.60) for gpt-4o

4\. Alert when daily spend exceeds threshold

5\. Consider: shorter prompts, caching, smaller models for simple tasks

**📋 Chapter Summary & Recap**

DocuBot is production-ready: deployed, monitored, and performing well.

## **Chapter 16: Advanced Patterns & Portfolio**

**Duration:** 3-4 hours  •  **Lessons:** 4  •  **Depth:** Exploration

**Chapter Overview**

Explore advanced patterns and prepare DocuBot as a portfolio piece.

**⚡ Cognitive Load:** LOW-MODERATE \- Exploratory and celebratory. Students have accomplished a lot\!

**Learning Objectives**

* Explore advanced patterns

* Prepare portfolio project

* Plan continued learning

**🤖 DocuBot State After This Chapter**

*DocuBot is a complete, portfolio-ready RAG chatbot with documentation, demo video, and clean code.*

**Lessons**

**Lesson 16.1: Advanced Agent Patterns**

**Duration:** 40-50 minutes

**🎯 Learning Goal**

*Explore what's possible next.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Implement one advanced pattern (planning or self-reflection).

**Outcome:** *Experience with advanced patterns.*

**💡 Hints (if you get stuck):**

1\. Planning pattern: Agent creates step-by-step plan before executing

2\. Self-reflection: Agent reviews its answer and improves it before returning

3\. ReAct pattern: Reason → Act → Observe → repeat

4\. Try: Add 'review your answer for accuracy' step before final output

5\. These patterns improve quality but increase latency and cost

**Lesson 16.2: Fine-Tuning Concepts**

**Duration:** 35-45 minutes

**🎯 Learning Goal**

*Understand custom model options.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Create a sample fine-tuning dataset.

**Outcome:** *Understanding of fine-tuning.*

**💡 Hints (if you get stuck):**

1\. Fine-tuning: Train model on your specific data for specialized behavior

2\. Dataset format: JSONL with {messages: \[{role, content}, ...\]}

3\. Collect examples: 50-100 ideal question/answer pairs from your domain

4\. When to fine-tune: When you need consistent format, domain-specific terms

5\. Alternative: Often good prompting \+ RAG is enough without fine-tuning

**Lesson 16.3: Building Your Portfolio**

**Duration:** 45-55 minutes

**🎯 Learning Goal**

*Package DocuBot professionally.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Create README, demo video, and architecture diagram.

**Outcome:** *Portfolio-ready project.*

**💡 Hints (if you get stuck):**

1\. README sections: Overview, Features, Tech Stack, Setup, Usage, Architecture

2\. Demo video: 2-3 minute screen recording showing key features (use Loom)

3\. Architecture diagram: draw.io or Mermaid showing components and data flow

4\. Clean code: Remove debug prints, add comments, organize files

5\. Live demo: Keep deployed version running for easy sharing

**Lesson 16.4: Career & Continued Learning**

**Duration:** 30-40 minutes

**🎯 Learning Goal**

*Plan your next steps.*

**📖 Concept (What You'll Learn)**

**🤖 Apply to DocuBot Project**

**Task:** Create 6-month learning plan.

**Outcome:** *Clear path forward.*

**💡 Hints (if you get stuck):**

1\. Identify your direction: AI engineer, full-stack AI, research, startup

2\. Next skills: LangChain, LlamaIndex, model fine-tuning, evaluation frameworks

3\. Build more projects: Each project teaches new skills and grows portfolio

4\. Community: Join AI Discord servers, follow AI Twitter/X, attend meetups

5\. 6-month plan: Month 1-2 (consolidate), 3-4 (new project), 5-6 (specialize)

**📋 Chapter Summary & Recap**

You've built a production RAG chatbot\! DocuBot is ready for your portfolio, and you have a roadmap for continued learning.

