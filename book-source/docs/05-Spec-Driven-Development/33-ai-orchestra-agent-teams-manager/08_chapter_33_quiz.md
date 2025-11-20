---
sidebar_position: 8
title: "Chapter 33: AI Orchestra Quiz"
---

# Chapter 33: AI Orchestra - Agent Teams Manager Quiz

Test your understanding of multi-agent orchestration, decomposition thinking, and contract-based coordination through specifications.

<Quiz
  title="Chapter 33: AI Orchestra Assessment"
  questions={[    {
      question: "You have three agents working on authentication, database, and API features. Each agent keeps asking you questions about how their work connects. What does this communication pattern reveal about your decomposition?",
      options: [
        "The agents need better prompts to work independently",
        "You need to add more agents to reduce workload",
        "The agents should coordinate directly without your involvement",
        "Your task boundaries have unclear or overlapping interfaces"
      ],
      correctOption: 3,
      explanation: "Frequent cross-agent questions indicate unclear task boundaries or poorly defined interfaces. When agents constantly need clarification about how their work connects, it reveals that the decomposition hasn't properly separated concerns or specified clear contracts between components. Better prompts (option A) won't fix structural decomposition problems. Direct agent coordination (option C) without clear contracts creates N-squared communication complexity. Adding more agents (option D) would amplify the communication problem rather than solve it. Proper decomposition requires clear interfaces and well-defined contracts that allow agents to work autonomously within their boundaries.",
      source: "Lesson 1: Git Worktrees for Parallel Specifications"
    },
    {
      question: "After running 'git worktree add ../feat-auth feature/auth', you try to check out the 'feature/auth' branch in your main directory. Git prevents this. Why does this limitation actually support better parallel development?",
      options: [
        "It forces you to commit changes before switching branches",
        "It requires you to create separate branches for isolation",
        "It prevents accidental conflicts from editing same branch simultaneously",
        "It reduces disk space usage by sharing repository data"
      ],
      correctOption: 2,
      explanation: "Git's one-branch-per-worktree rule prevents simultaneous editing of the same branch in multiple locations, which would create hidden conflicts and confusion. This limitation enforces true isolation—each worktree must work on a different branch, ensuring changes don't interfere. Forcing commits (option A) isn't the purpose; you can have uncommitted changes in each worktree. Creating separate branches (option C) is what you're already doing, not a prevention mechanism. Disk space (option D) is a side benefit, not the reason for the limitation. The constraint ensures that parallel work stays truly parallel without hidden interference.",
      source: "Lesson 1: Git Worktrees for Parallel Specifications"
    },
    {
      question: "You're orchestrating five agents to build separate microservices. After two days, you notice all five agents are blocked waiting for your decisions. What orchestration principle have you violated?",
      options: [
        "You parallelized work that has sequential dependencies",
        "You didn't establish clear acceptance criteria upfront",
        "You created too many agents for effective management",
        "You failed to specify integration contracts between services"
      ],
      correctOption: 1,
      explanation: "When all agents are blocked waiting for decisions, it indicates missing upfront acceptance criteria. Without clear success criteria defined in specifications, agents can't proceed autonomously and must constantly ask for validation. Sequential dependencies (option B) would block agents in sequence, not all simultaneously. Five agents (option C) is a reasonable number if properly coordinated. Missing integration contracts (option D) would cause integration problems, not universal blocking. The core issue is that without predefined acceptance criteria, agents lack the guidance to make autonomous progress and must wait for manager approval at every step.",
      source: "Lesson 2: Parallel Planning and Tasks"
    },
    {
      question: "You decompose a project into six parallel tasks. Three finish in 2 hours, two finish in 4 hours, and one takes 8 hours. You spent 1 hour planning decomposition. What's your actual time savings compared to sequential work totaling 20 hours?",
      options: [
        "11 hours saved (9 hours total vs 20 sequential)",
        "12 hours saved (8 hours total vs 20 sequential)",
        "9 hours saved (11 hours total vs 20 sequential)",
        "10 hours saved (10 hours total vs 20 sequential)"
      ],
      correctOption: 0,
      explanation: "Parallel execution time equals the longest task (8 hours) plus planning overhead (1 hour) = 9 hours total. Sequential would take 20 hours, so savings = 20 - 9 = 11 hours. Option B incorrectly ignores planning overhead. Option C incorrectly adds all parallel task times (2+4+8=14) instead of using the longest task. Option D uses an arbitrary 10-hour total. The critical insight is that parallel speedup is limited by the longest task (the critical path) plus any coordination overhead, not by summing parallel durations.",
      source: "Lesson 7: Capstone Project Measurement"
    },
    {
      question: "Your three-agent team experiences a merge conflict when integrating the authentication and API modules. What does this conflict most likely indicate about your decomposition strategy?",
      options: [
        "The agents worked too quickly without coordination",
        "The agents should have worked in sequence",
        "The specifications were too detailed and restrictive",
        "The task boundaries overlapped in shared responsibilities"
      ],
      correctOption: 3,
      explanation: "Merge conflicts during integration typically reveal overlapping responsibilities where two tasks modified the same code or data structures. This indicates the decomposition didn't cleanly separate concerns along proper boundaries. Working too quickly (option A) doesn't cause conflicts; unclear boundaries do. Detailed specifications (option C) actually prevent conflicts by clarifying interfaces. Sequential work (option D) avoids conflicts but sacrifices parallelization benefits. The conflict is feedback that your decomposition should be revised to create clearer boundaries where authentication and API interact through well-defined interfaces rather than shared implementation details.",
      source: "Lesson 3: Parallel Implementation and Integration"
    },
    {
      question: "You're deciding whether to parallelize a feature across three agents or assign it to one agent. The feature requires frequent decisions about trade-offs between security, performance, and usability. What factor should guide your decision?",
      options: [
        "Whether the feature can be divided into independent modules",
        "How quickly you need the feature completed overall",
        "How much coordination overhead the trade-off decisions create",
        "Whether you have enough agents available for assignment"
      ],
      correctOption: 2,
      explanation: "Frequent trade-off decisions across security, performance, and usability create high coordination overhead that can eliminate parallelization benefits. When decisions are tightly coupled, the cost of coordinating agents (N-squared communication) often exceeds the speedup from parallel work. Independent modules (option A) are necessary but insufficient if decisions are interdependent. Speed requirements (option B) don't matter if coordination overhead slows everything down. Agent availability (option C) is a resource constraint, not a decision criterion. The key insight is recognizing when coordination costs outweigh parallelization benefits—tightly coupled decisions favor single-agent execution.",
      source: "Lesson 2: Parallel Planning and Tasks"
    },
    {
      question: "Your agent team's integration contracts specify that the Database module 'Provides: User schema and query interface' and the API module 'Depends: User query methods'. During integration, the API agent requests direct access to the database connection. How should you respond?",
      options: [
        "Update the contract to include database connection access",
        "Reject the request as a contract violation",
        "Allow temporary access until integration completes successfully",
        "Create a new contract between modules"
      ],
      correctOption: 1,
      explanation: "The request violates the integration contract, which specifies that Database provides a query interface, not direct connection access. Allowing this violates encapsulation and creates hidden dependencies that undermine the contract's purpose. Updating the contract (option A) weakens the interface abstraction. Temporary access (option C) creates technical debt and defeats contract discipline. Creating a new contract (option D) is unnecessary; the existing contract already defines the relationship. Contract violations during integration are feedback that either the agent misunderstands the architecture or the specification needs clarification. Enforcing contract boundaries maintains the architectural integrity that enables parallel work.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "You're analyzing your four-agent project's timeline. Planning took 1 hour, parallel execution took 6 hours (longest task), but integration took 4 hours due to interface mismatches. What does this ratio reveal?",
      options: [
        "Integration time indicates poor contract specification upfront",
        "The longest task created an unavoidable bottleneck",
        "Planning time was insufficient for proper decomposition",
        "Four agents exceeded optimal team size limits"
      ],
      correctOption: 0,
      explanation: "Integration taking 4 hours (67% of execution time) indicates contracts weren't clearly specified upfront. Well-defined integration contracts should make integration relatively smooth (15-20% of execution time). Insufficient planning (option A) is related but less specific; the issue isn't planning duration but contract clarity. The longest task (option B) being 6 hours is expected; bottleneck duration doesn't explain integration problems. Four agents (option D) is reasonable; team size isn't the issue. High integration time specifically points to unclear interfaces and acceptance criteria, meaning agents built to different assumptions. This is a decomposition quality problem, not a coordination scale problem.",
      source: "Lesson 7: Capstone Project Measurement"
    },
    {
      question: "When decomposing a complex feature, you identify that Component A must finish before Component B can start, but Components C and D can work independently. What does this dependency structure suggest about your orchestration approach?",
      options: [
        "Use four parallel agents and manage dependencies carefully",
        "Assign two agents to complete A and B first",
        "Use three parallel agents with sequential integration phases",
        "Complete the sequential path first, then parallelize independents"
      ],
      correctOption: 2,
      explanation: "The optimal approach assigns one agent to A, another to B (starting after A completes), and a third agent working parallel on C and D combined or split if they're large enough. This creates three parallel workstreams: A→B sequential path, C independent, D independent. Option B inefficiently assigns two agents to sequential work. Option C creates unnecessary coordination for the A→B dependency. Option D sacrifices parallelization benefits by waiting unnecessarily. The key insight is identifying the critical path (A→B) and running independent work (C, D) parallel to it, minimizing total completion time.",
      source: "Lesson 2: Parallel Planning and Tasks"
    },
    {
      question: "Your SpecKit Plus workflow shows one agent finishing in 2 hours while others are still working at 5 hours. Instead of starting integration, you assign the finished agent a new independent task. What orchestration principle does this demonstrate?",
      options: [
        "Optimizing resource allocation for continuous delivery",
        "Prioritizing throughput over minimizing coordination overhead",
        "Balancing workload distribution across available agents",
        "Maximizing agent utilization to reduce idle time"
      ],
      correctOption: 3,
      explanation: "Assigning new work to finished agents maximizes utilization and reduces idle time while other agents complete their tasks. This is a core principle of parallel orchestration—keeping all agents productive rather than letting them sit idle waiting for integration. Throughput prioritization (option B) is vague; the specific principle is utilization. Workload balancing (option C) happens during initial decomposition, not after tasks complete. Continuous delivery (option D) is an outcome, not the principle being demonstrated. The orchestration insight is recognizing that idle agents represent wasted capacity that could be working on additional independent tasks.",
      source: "Lesson 6: SpecKit Orchestrated Execution"
    },
    {
      question: "You notice your three-agent project spent 30% of total time on coordination meetings and status updates. What does this proportion indicate about your orchestration design?",
      options: [
        "Contracts lacked sufficient detail for autonomous work",
        "Three agents exceeded the optimal team size",
        "Agents needed better prompting for independent execution",
        "The decomposition created too many inter-task dependencies"
      ],
      correctOption: 0,
      explanation: "30% coordination overhead indicates contracts lacked sufficient clarity for agents to work autonomously. With clear contracts specifying Provides/Depends/Acceptance Criteria, coordination should be 10-15% of total time. Three agents (option A) is well within optimal range (3-7). Better prompting (option C) won't fix structural contract issues. While dependencies (option D) increase coordination, the high percentage specifically points to unclear contracts forcing frequent check-ins. Well-defined contracts enable autonomous work; vague contracts require constant manager mediation. The 30% overhead signals a specification quality problem, not a scale or dependency problem.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "Your integration contract states: 'API module Provides: REST endpoints for user operations. Database module Depends: User data models.' During integration, neither module can proceed. What's the most likely contract design flaw?",
      options: [
        "The Provides and Depends are too vague",
        "The contract creates a circular dependency loop",
        "The contract lacks explicit acceptance criteria",
        "The modules need additional dependency specifications"
      ],
      correctOption: 1,
      explanation: "The contract creates a circular dependency: API provides endpoints (which need database) while Database depends on data models (which need API definition). This deadlock reveals a fundamental flaw in dependency direction. Vagueness (option B) would cause confusion, not blocking. Missing acceptance criteria (option C) would cause integration mismatches, not deadlock. Additional dependencies (option D) would worsen the circular problem. The correct contract should specify that Database provides data models, and API depends on those models to build endpoints. This establishes clear dependency direction: Database → API, eliminating the circular dependency.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "After completing a four-agent project, you measure a 2.5x speedup compared to sequential execution. Your manager questions why it's not closer to 4x. Which explanation demonstrates understanding of parallel overhead?",
      options: [
        "Agent capability differences created uneven task completion",
        "Some tasks had dependencies preventing full parallelization",
        "The critical path task limited overall speedup",
        "Planning, coordination, and integration overhead reduce theoretical maximum"
      ],
      correctOption: 3,
      explanation: "All factors contribute, but option A provides the comprehensive explanation: Amdahl's Law shows that planning, coordination, and integration overhead prevent linear speedup. Even with perfect parallelization, these sequential portions limit gains. Dependencies (option B) are one factor but don't explain the general principle. Critical path (option C) affects timeline but doesn't explain overhead. Agent differences (option D) would be addressed in decomposition. The fundamental insight is that parallel work always includes sequential overhead (planning, integration, coordination) that prevents theoretical N-agent = Nx speedup. Understanding this sets realistic expectations for orchestration outcomes.",
      source: "Lesson 7: Capstone Project Measurement"
    },
    {
      question: "You're reviewing two possible decomposition strategies: Strategy A creates three large independent modules; Strategy B creates six small modules with multiple integration points. Both complete the same feature. What's the key trade-off?",
      options: [
        "Strategy A optimizes for simplicity; Strategy B optimizes speed",
        "Strategy A reduces integration risk; Strategy B increases throughput",
        "Strategy A minimizes coordination; Strategy B maximizes parallelization potential",
        "Strategy A reduces agent count; Strategy B improves utilization"
      ],
      correctOption: 2,
      explanation: "The core trade-off is coordination overhead versus parallelization depth. Strategy A minimizes coordination (3 independent modules = minimal integration) at the cost of less parallelization (larger chunks). Strategy B maximizes parallelization potential (6 modules = more parallel work) but increases coordination complexity (more integration points). Throughput (option B) isn't clearly higher with either strategy. Speed optimization (option C) oversimplifies; both can be fast. Agent count (option D) is a resource decision, not an architectural trade-off. Understanding this trade-off is essential for choosing decomposition granularity based on team size, time constraints, and integration complexity tolerance.",
      source: "Lesson 4: Scaling Decomposition Thinking"
    },
    {
      question: "Your specification states: 'Authentication module must validate user credentials and return session tokens.' Agent implementation returns user objects instead of tokens. The code works but violates the contract. Why should you require changes?",
      options: [
        "Other modules depend on the specified token interface",
        "Specifications must be followed precisely for consistency",
        "Returning user objects violates security encapsulation principles",
        "Contract violations create technical debt requiring future refactoring"
      ],
      correctOption: 0,
      explanation: "The critical issue is that other modules were built expecting token interfaces based on the contract. Changing the return type breaks integration contracts that other agents relied on for their implementations. Technical debt (option A) is a concern but not the primary integration issue. Precise following (option B) is important but doesn't explain why. Security encapsulation (option C) might be true but isn't the contract enforcement reason. The fundamental principle is that contracts enable autonomous parallel work precisely because agents can trust interface specifications. Allowing violations undermines the entire contract-based coordination approach.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "You're scaling your decomposition thinking from AI agents to a human development team. The most valuable transferable insight is understanding what fundamental principle?",
      options: [
        "How to minimize communication overhead through contracts",
        "How to identify natural boundaries and interfaces",
        "How to measure productivity across parallel workers",
        "How to balance independence with integration requirements"
      ],
      correctOption: 1,
      explanation: "Identifying natural boundaries and interfaces is the most fundamental transferable skill because it applies universally whether coordinating AI agents, human developers, or system components. This skill determines decomposition quality regardless of execution context. Minimizing communication (option B) is important but secondary to proper boundary identification. Productivity measurement (option C) is a management technique, not a decomposition principle. Balancing independence and integration (option D) is a consequence of good boundary identification. The core transferable insight is recognizing where systems naturally divide along stable interfaces—this thinking transfers across AI agents, human teams, microservices, and organizational design.",
      source: "Lesson 4: Scaling Decomposition Thinking"
    },
    {
      question: "During integration, you discover two agents independently implemented user authentication using different hashing algorithms. Both implementations work. What does this situation reveal about your specification quality?",
      options: [
        "The integration contract was missing security requirements",
        "The agents should have coordinated on technical decisions",
        "The specifications lacked sufficient implementation detail requirements",
        "The acceptance criteria didn't validate implementation consistency"
      ],
      correctOption: 2,
      explanation: "The specification should have specified the hashing algorithm as a concrete requirement to ensure consistency across modules that interact with authentication. Vague specifications allow agents to make independent technical choices that cause integration problems. Agent coordination (option B) shouldn't be necessary if specifications are clear. The contract issue (option C) is related but less specific than missing implementation requirements. Acceptance criteria (option D) test outcomes, but the problem is earlier—the specification didn't constrain the implementation approach. This illustrates the principle that specifications must include enough technical constraints to ensure compatible implementations while avoiding over-specification.",
      source: "Lesson 1: Git Worktrees for Parallel Specifications"
    },
    {
      question: "You assign agents to work on Frontend, Backend, and Database modules with completion hooks configured to notify when each finishes. The Backend hook triggers, but integration can't proceed because Database hasn't completed. What coordination mechanism is missing?",
      options: [
        "Acceptance criteria validation before triggering completion hooks",
        "Integration contracts defining the Backend-Database interface clearly",
        "Sequential ordering of tasks based on dependencies",
        "Dependency specification in the completion hook configuration"
      ],
      correctOption: 3,
      explanation: "Completion hooks should be configured with dependency awareness so Backend's completion doesn't trigger integration steps that require Database. The hook should wait for both Backend AND Database before proceeding. Integration contracts (option B) define interfaces but don't control hook triggering logic. Sequential ordering (option C) would eliminate parallelization benefits. Acceptance criteria (option D) validate quality but don't control workflow dependencies. The missing mechanism is dependency-aware hook configuration that understands the integration dependency graph and only triggers integration when all prerequisites are met.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "You're measuring a three-agent project. Agent A took 4 hours, Agent B took 6 hours, Agent C took 3 hours. Planning was 1 hour, integration was 2 hours. A sequential approach would take 15 hours. What's the actual speedup?",
      options: [
        "1.5x speedup (10 hours parallel vs 15 sequential)",
        "1.67x speedup (9 hours parallel vs 15 sequential)",
        "2.5x speedup (6 hours parallel vs 15 sequential)",
        "1.88x speedup (8 hours parallel vs 15 sequential)"
      ],
      correctOption: 1,
      explanation: "Total parallel time = longest task (6 hours) + planning (1 hour) + integration (2 hours) = 9 hours. Speedup = 15 sequential / 9 parallel = 1.67x. Option B incorrectly calculates 10 hours total. Option C only counts execution time (6 hours), ignoring overhead. Option D uses 8 hours, missing integration time. The critical insight is that parallel speedup must account for ALL overhead (planning + integration) plus the critical path (longest task), not the sum of all parallel work durations.",
      source: "Lesson 7: Capstone Project Measurement"
    },
    {
      question: "Your SpecKit Plus manager role involves reviewing agent outputs and approving next steps. You find yourself making detailed implementation decisions for each agent. What anti-pattern have you fallen into?",
      options: [
        "Micromanaging execution instead of focusing on strategic coordination",
        "Failing to delegate sufficient authority to agents",
        "Over-specifying implementation details in contracts upfront",
        "Not trusting agent capabilities for autonomous work"
      ],
      correctOption: 0,
      explanation: "The manager role should focus on strategic coordination—decomposition, contract design, integration oversight—not implementation details. Making detailed decisions for each agent is micromanagement that eliminates parallelization benefits. Delegation (option B) is related but less specific. Over-specification (option C) would happen during planning, not during execution review. Trust issues (option D) might be a cause but don't describe the anti-pattern itself. The key insight is that orchestration requires shifting from execution management to strategic oversight, letting agents handle implementation autonomously within contract boundaries.",
      source: "Lesson 6: SpecKit Orchestrated Execution"
    },
    {
      question: "You're decomposing a feature that requires consistent error handling across all modules. Where should you specify the error handling approach to enable autonomous parallel development?",
      options: [
        "In each module's individual specification document",
        "In the acceptance criteria for each task",
        "In the integration contract between modules",
        "In a shared architectural specification referenced by contracts"
      ],
      correctOption: 3,
      explanation: "Cross-cutting concerns like error handling should be specified in shared architectural documentation that all module specifications reference. This ensures consistency without duplicating specifications. Individual specs (option A) create inconsistency risk as each agent might interpret differently. Integration contracts (option C) define module interactions, not shared implementation patterns. Acceptance criteria (option D) test outcomes but don't specify approaches. The principle is: module-specific concerns go in module specs, cross-module interactions go in integration contracts, and cross-cutting patterns go in shared architectural specifications that enable consistent autonomous implementation.",
      source: "Lesson 1: Git Worktrees for Parallel Specifications"
    },
    {
      question: "Your project has seven agents working in parallel. You spend most of your time answering questions and coordinating between agents. What does this situation indicate about scaling limits?",
      options: [
        "Seven agents exceeds the effective coordination limit",
        "The decomposition created too many inter-agent dependencies",
        "Specifications lack clarity for autonomous agent work",
        "The manager role needs additional coordination tools"
      ],
      correctOption: 2,
      explanation: "Constant questions and coordination indicate specifications aren't clear enough for autonomous work, not that seven agents is too many. With well-defined contracts, 3-7 agents is optimal. Exceeding limits (option A) would show as coordination breaking down completely. Too many dependencies (option B) is possible but the symptom (questions) specifically points to unclear specifications. Additional tools (option D) won't fix specification quality problems. The insight is that agent count within recommended range suggests the problem is specification clarity, not scale. Clear contracts enable autonomous work; vague specifications require constant manager intervention regardless of team size.",
      source: "Lesson 4: Scaling Decomposition Thinking"
    },
    {
      question: "Your integration contract specifies: 'API Provides: User endpoints. Database Depends: API schema.' During integration, the Database agent says they can't proceed without the API's data model. What's the contract flaw?",
      options: [
        "Dependencies are specified in the wrong direction",
        "The Provides statement is too vague about deliverables",
        "The contract should include the data model explicitly",
        "Database and API need bidirectional dependency specification"
      ],
      correctOption: 0,
      explanation: "The dependencies are backwards. Database should provide the data model; API should depend on that model to build endpoints. The current contract has Database depending on API schema, creating the wrong dependency direction. Vagueness (option B) isn't the issue; the direction is wrong. Explicit data models (option C) might help but doesn't fix directional error. Bidirectional dependencies (option D) create circular dependency problems. The principle is that data models typically flow from storage layer to application layer, so Database should Provide models and API should Depend on them.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "After three parallel agent projects, you notice integration always takes longer than planned. What measurement should you track to improve future estimates?",
      options: [
        "Number of merge conflicts per integration",
        "Ratio of integration time to execution time",
        "Time spent on coordination vs autonomous work",
        "Percentage of tasks completed within estimates"
      ],
      correctOption: 1,
      explanation: "Tracking the ratio of integration time to execution time (e.g., integration is 25% of longest task) provides a consistent metric for improving future planning. This ratio helps you understand integration overhead patterns. Merge conflicts (option B) indicate decomposition problems but don't help estimation. Coordination time (option C) is useful but broader than integration-specific. Task completion percentage (option D) measures estimation accuracy generally, not integration specifically. The insight is that measuring integration as a proportion of execution time creates a reusable heuristic for better decomposition and timeline planning.",
      source: "Lesson 7: Capstone Project Measurement"
    },
    {
      question: "You're deciding whether to use three agents for 6 hours each or six agents for 3 hours each. Both approaches complete the same work. What factor should most influence your choice?",
      options: [
        "Agent availability and resource constraints matter",
        "Integration complexity grows with finer decomposition granularity",
        "Coordination overhead increases with more agents",
        "The critical path duration determines speedup"
      ],
      correctOption: 2,
      explanation: "Coordination overhead scales with agent count (N-squared communication complexity). Six agents create significantly more coordination overhead than three agents for the same total work. Integration complexity (option B) is related but coordination overhead is more direct. Resource constraints (option C) matter but don't explain the trade-off. Critical path (option D) affects timeline but doesn't explain agent count decisions. The principle is that adding agents beyond necessary parallelization increases coordination costs without proportional speedup benefits. Three agents for longer durations often outperforms six agents for shorter durations due to reduced coordination overhead.",
      source: "Lesson 2: Parallel Planning and Tasks"
    },
    {
      question: "Your completion hook is configured to trigger integration tests when any agent finishes their module. The tests fail because dependent modules aren't complete. What hook design principle should you apply?",
      options: [
        "Hooks should trigger based on dependency completion",
        "Hooks need validation logic before running tests",
        "Completion criteria should include dependency verification",
        "Integration tests should wait until all modules finish"
      ],
      correctOption: 3,
      explanation: "Integration tests require all integrated components to be complete, so hooks should trigger integration testing only when all required modules finish, not when any individual module finishes. Dependency-based triggering (option A) is vague; the specific principle is waiting for all modules. Validation logic (option B) would check individual module quality, not multi-module integration readiness. Dependency verification in completion criteria (option D) checks if a module meets its contracts, but doesn't address when to run integration tests. The principle is that integration-level tests require integration-level completion, while module-level tests can run on individual completion.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "You're teaching decomposition thinking to a new team lead. They ask when to stop breaking down tasks. What guidance demonstrates mature understanding of decomposition limits?",
      options: [
        "Stop when each task can be completed independently",
        "Stop when coordination overhead exceeds parallelization benefits",
        "Stop when tasks align with team member capabilities",
        "Stop when each task represents a deployable component"
      ],
      correctOption: 1,
      explanation: "The optimal decomposition granularity is where further breakdown would create more coordination overhead than it saves through parallelization. This is the fundamental economic trade-off. Independent completion (option A) is necessary but doesn't indicate when to stop breaking down. Team capabilities (option C) affect assignment but not decomposition limits. Deployable components (option D) is an architectural concern, not a decomposition limit. The insight is that decomposition follows diminishing returns: early breakdown creates huge parallelization gains with minimal coordination costs, but eventually coordination overhead dominates, making finer decomposition counterproductive.",
      source: "Lesson 4: Scaling Decomposition Thinking"
    },
    {
      question: "Your git worktree for the 'feature/auth' branch shows uncommitted changes. You want to switch to reviewing another agent's work in a different worktree. What's the best practice?",
      options: [
        "Simply switch worktrees since they're isolated",
        "Stash the changes to preserve work in progress",
        "Commit the changes before switching worktrees",
        "Create a temporary branch for the uncommitted work"
      ],
      correctOption: 0,
      explanation: "Worktrees are fully isolated, so uncommitted changes in one worktree don't affect other worktrees. You can freely switch between worktree directories without committing, stashing, or any other git operations. Committing (option A) is unnecessary and creates noise in history. Stashing (option B) is unnecessary due to isolation. Temporary branches (option D) add complexity for no benefit. The key insight is that worktree isolation means each directory maintains its own working state completely independently—uncommitted changes, staged files, and branch state are all separate.",
      source: "Lesson 1: Git Worktrees for Parallel Specifications"
    },
    {
      question: "You're analyzing bottlenecks in your four-agent project. Agent A took 8 hours while Agents B, C, D finished in 3-4 hours. Where should you focus optimization efforts for the next iteration?",
      options: [
        "Accept the bottleneck and optimize integration instead",
        "Redistribute work more evenly across all agents",
        "Improve Agent A's specification for faster execution",
        "Break down Agent A's task into smaller pieces"
      ],
      correctOption: 3,
      explanation: "The longest task defines the critical path, so breaking it into smaller parallel pieces directly reduces total time. Agent A's 8 hours is the bottleneck limiting overall speedup. Redistributing work (option B) might help but less directly than attacking the bottleneck. Better specifications (option C) might help execution but won't address the fundamental size imbalance. Accepting the bottleneck (option D) ignores the optimization opportunity. The principle from critical path analysis is that optimizing the longest task has maximum impact on total timeline—reducing a 3-hour task to 2 hours doesn't help if an 8-hour task still exists.",
      source: "Lesson 7: Capstone Project Measurement"
    },
    {
      question: "Your specification states: 'Authentication module accepts username/password and returns success/failure.' During testing, an edge case arises: what should happen when the database is unavailable? Where should this decision be documented?",
      options: [
        "In the Authentication module's specification as error handling",
        "In the integration contract between Authentication and Database",
        "In the shared architectural error handling guidelines",
        "In the acceptance criteria for authentication testing"
      ],
      correctOption: 2,
      explanation: "Database unavailability is a cross-cutting infrastructure concern that should be addressed in shared architectural guidelines, not module-specific specs. This ensures consistent error handling across all modules that interact with the database. Module specification (option A) would duplicate this across many modules. Integration contract (option B) defines the Auth-Database interface, not infrastructure failure handling. Acceptance criteria (option D) tests the behavior but doesn't define the policy. The principle is that system-wide concerns (error handling, logging, retry policies) belong in shared architectural documentation that all modules reference.",
      source: "Lesson 1: Git Worktrees for Parallel Specifications"
    },
    {
      question: "You're comparing two completed projects: Project A used 3 agents and achieved 2.4x speedup; Project B used 6 agents and achieved 2.8x speedup. What does this comparison reveal about coordination overhead?",
      options: [
        "Project B had better decomposition despite more agents",
        "Doubling agents provided diminishing speedup returns due to overhead",
        "Project A had higher coordination overhead than B",
        "Six agents approached the effective coordination limit"
      ],
      correctOption: 1,
      explanation: "Doubling agents (3 to 6) only increased speedup from 2.4x to 2.8x (17% improvement), demonstrating diminishing returns due to coordination overhead. If overhead were negligible, 6 agents would approach 6x speedup, not 2.8x. Better decomposition (option A) might be true but doesn't explain the diminishing returns pattern. Higher overhead in A (option C) is backwards; B had more overhead. Coordination limit (option D) is premature at 6 agents; the issue is diminishing returns. The insight is that each additional agent adds less marginal speedup due to increasing coordination costs (N-squared communication complexity).",
      source: "Lesson 4: Scaling Decomposition Thinking"
    },
    {
      question: "During integration, you discover that two modules use inconsistent naming conventions (camelCase vs snake_case). Both work correctly but the inconsistency bothers you. Should you require changes?",
      options: [
        "Yes, if naming conventions were specified upfront",
        "No, functional correctness matters more than style",
        "Yes, consistency is essential for maintainability",
        "No, unless the inconsistency affects integration"
      ],
      correctOption: 0,
      explanation: "The decision depends on whether naming conventions were specified in the original contracts. If they were specified, then enforcement maintains contract discipline and specification-driven development. If not specified, requiring changes now is retroactive requirement addition. Maintainability (option A) is important but doesn't justify changing the rules mid-project. Functional correctness (option B) isn't the only consideration in professional development. Integration impact (option D) is too narrow; the real issue is contract adherence. The principle is that specifications define the contract; if something wasn't specified, you can't enforce it retroactively without undermining the specification-driven process.",
      source: "Lesson 6: SpecKit Orchestrated Execution"
    },
    {
      question: "You're orchestrating five agents and notice one agent frequently finishes early and waits idle. What optimization strategy should you employ for future iterations?",
      options: [
        "Assign that agent additional tasks when available",
        "Redistribute work to balance completion times better",
        "Keep backup tasks ready for early finishers",
        "Use that agent for integration and testing work"
      ],
      correctOption: 2,
      explanation: "Keeping backup independent tasks ready for early finishers maximizes utilization without disrupting the current project's decomposition. This approach maintains the current project's plan while eliminating idle time. Assigning additional tasks (option A) is the action, but 'backup tasks ready' (option D) is the strategy. Redistributing work (option B) is for the next iteration's planning phase, not a mid-project optimization. Integration work (option C) might not be ready when the agent finishes early. The principle is proactive capacity planning—anticipating idle time and having independent work queued ensures continuous utilization.",
      source: "Lesson 6: SpecKit Orchestrated Execution"
    },
    {
      question: "Your integration contract states: 'Frontend Depends: User data API endpoints. Backend Provides: REST API with user operations.' Integration fails because Frontend expected GraphQL. What contract element was missing?",
      options: [
        "Technology stack specification in architectural guidelines",
        "Explicit interface definition with endpoint details",
        "Acceptance criteria for API compatibility testing",
        "The API protocol specification (REST vs GraphQL)"
      ],
      correctOption: 3,
      explanation: "The contract stated 'REST API' but Frontend agent either missed it or had conflicting expectations. The missing element is explicit, unambiguous protocol specification that both parties acknowledge. Endpoint details (option B) matter but the fundamental mismatch is protocol-level. Acceptance criteria (option C) would catch the problem but doesn't prevent it. Technology stack guidelines (option D) are broader than the specific contract need. The lesson is that contracts must explicitly specify technical details that affect compatibility—'API' is too vague, 'REST API with JSON responses' is specific enough for agents to build compatible implementations.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "You measure that planning took 10% of project time, execution 60%, and integration 30%. For the next project, you want to reduce total time. Where should you focus improvement efforts?",
      options: [
        "Reduce integration time by improving contract clarity upfront",
        "Reduce execution time by breaking critical path tasks",
        "Reduce planning time by using templates and patterns",
        "Balance improvement across all three phases equally"
      ],
      correctOption: 0,
      explanation: "Integration taking 30% of time indicates poor contract quality. Industry best practice is 10-15% integration time. Improving contract clarity upfront (during the 10% planning phase) can reduce the 30% integration time significantly. Execution optimization (option B) has merit but integration overhead is the bigger problem. Reducing planning time (option C) is counterproductive; better planning reduces integration time. Balanced improvement (option D) ignores that integration is the outlier. The insight is that high integration time is a leading indicator of specification quality problems—investing more in planning/contracts pays dividends in reduced integration overhead.",
      source: "Lesson 7: Capstone Project Measurement"
    },
    {
      question: "You're decomposing a system for parallel development. One module naturally interfaces with five other modules. What does this architecture pattern suggest about your decomposition?",
      options: [
        "The decomposition has too many dependencies for effective parallelization",
        "The module is a central component requiring careful contract design",
        "The module should be split into smaller pieces",
        "The five-way interface creates a coordination bottleneck"
      ],
      correctOption: 1,
      explanation: "A module interfacing with five others isn't necessarily a problem—it might be a legitimate architectural hub (like a database layer or API gateway). The key is careful contract design to specify each interface clearly. Too many dependencies (option B) is a premature conclusion; five interfaces might be optimal. Splitting it (option C) might just distribute the coupling rather than solving it. Coordination bottleneck (option D) assumes problems that might not exist with good contracts. The principle is recognizing when centrality is architectural (requiring careful specification) versus problematic (requiring redesign).",
      source: "Lesson 4: Scaling Decomposition Thinking"
    },
    {
      question: "Your specification for a payment module includes 15 pages of detailed requirements. Agents keep asking clarification questions despite the extensive documentation. What does this pattern indicate?",
      options: [
        "The specification is too detailed and overwhelming for agents",
        "The specification lacks clear prioritization of requirements",
        "The specification includes unnecessary implementation details",
        "The specification is comprehensive but poorly organized"
      ],
      correctOption: 3,
      explanation: "Extensive documentation with frequent questions indicates poor organization, not insufficient detail. Well-organized specifications enable autonomous work; disorganized specs require constant clarification regardless of length. Too detailed (option A) isn't the issue; lack of structure is. Prioritization (option B) helps but doesn't address navigation problems. Implementation details (option C) might be excessive but don't cause clarification questions. The principle is that specification quality depends on organization and clarity, not just completeness. A well-structured 5-page spec outperforms a disorganized 15-page spec.",
      source: "Lesson 1: Git Worktrees for Parallel Specifications"
    },
    {
      question: "You're orchestrating agents to build a feature with unclear requirements. The client will provide feedback during development. How should this uncertainty affect your decomposition strategy?",
      options: [
        "Plan for iterative refinement with shorter feedback cycles",
        "Create flexible contracts with loose coupling between modules",
        "Use larger modules to minimize integration rework costs",
        "Delay decomposition until requirements are fully defined"
      ],
      correctOption: 2,
      explanation: "Uncertain requirements favor coarser decomposition (fewer, larger modules) because requirement changes often cross module boundaries, creating expensive integration rework. Larger modules internalize changes rather than requiring contract renegotiation. Flexible contracts (option B) sound appealing but vague contracts prevent autonomous work. Iterative refinement (option C) is good practice but doesn't address decomposition granularity. Delaying decomposition (option D) eliminates parallelization benefits. The principle is that decomposition granularity should account for requirement stability—unstable requirements favor coarser decomposition to minimize integration rework.",
      source: "Lesson 2: Parallel Planning and Tasks"
    },
    {
      question: "After integration, your test suite reveals that two modules implement conflicting assumptions about data validation (client-side vs server-side). What does this conflict indicate about specification quality?",
      options: [
        "The agents should have coordinated on architecture",
        "The specification didn't allocate validation responsibility clearly",
        "The integration contract was missing validation requirements",
        "The acceptance criteria didn't test validation behavior"
      ],
      correctOption: 1,
      explanation: "Conflicting validation assumptions indicate the specification failed to clearly allocate responsibility for validation logic. This is a classic architecture decision that must be made during decomposition. Agent coordination (option B) shouldn't be necessary if specifications are clear. Integration contracts (option C) might mention validation but the root issue is responsibility allocation. Acceptance criteria (option D) test outcomes but don't prevent the conflict. The principle is that specifications must explicitly assign responsibilities for cross-cutting concerns, not assume agents will infer or coordinate.",
      source: "Lesson 3: Parallel Implementation and Integration"
    },
    {
      question: "You're reviewing completion hooks for a five-module project. Three modules have completion hooks configured; two don't. What risk does this inconsistency create?",
      options: [
        "Integration workflow won't track all module completion",
        "Manual tracking required for modules without hooks",
        "Coordination overhead increases due to inconsistent monitoring",
        "Completion timing becomes difficult to measure accurately"
      ],
      correctOption: 0,
      explanation: "Missing completion hooks for two modules means the integration workflow can't automatically detect when those modules finish, potentially starting integration prematurely or missing completion entirely. Manual tracking (option B) is a workaround but describes the symptom, not the risk. Coordination overhead (option C) is vague. Measurement difficulty (option D) is a secondary concern. The principle is that automation requires consistent instrumentation—partial hook coverage creates gaps in automated workflow orchestration, requiring manual intervention that defeats the automation purpose.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "Your team consistently achieves 2.2-2.5x speedup with four agents across multiple projects. Your manager asks why you don't use eight agents for 4-5x speedup. What's the most accurate response?",
      options: [
        "Coordination overhead increases non-linearly with agent count",
        "Four agents is optimal for our project complexity",
        "Doubling agents doesn't double speedup due to Amdahl's Law",
        "Eight agents would exceed effective management span of control"
      ],
      correctOption: 2,
      explanation: "Amdahl's Law explains that speedup is limited by the sequential portion of work (planning, integration, coordination). Doubling agents doesn't double speedup because these sequential portions remain constant. Non-linear overhead (option A) is a factor but Amdahl's Law is more comprehensive. Optimal team size (option B) is vague without explaining why. Span of control (option C) is a management concept but doesn't explain speedup limits. The key insight is that even with perfect parallelization, sequential overhead creates an asymptotic speedup limit that more agents can't overcome.",
      source: "Lesson 7: Capstone Project Measurement"
    },
    {
      question: "During decomposition, you identify a shared utility library that three modules will use. How should you handle this dependency to enable parallel development?",
      options: [
        "Include utility development in each module's scope",
        "Specify the utility interface and let each module implement independently",
        "Assign one agent to build it while others work",
        "Build the utility library first before parallel work starts"
      ],
      correctOption: 3,
      explanation: "Shared dependencies should be built first to unblock parallel work on dependent modules. This creates a sequential phase before parallelization but prevents blocking later. Independent implementation (option B) defeats the purpose of a shared library and creates duplication. Parallel utility development (option C) blocks the three dependent modules until it completes. Including it in each scope (option D) is the same as option B. The principle is that dependency graphs have a natural order—build foundations first, then parallelize on top of them.",
      source: "Lesson 2: Parallel Planning and Tasks"
    },
    {
      question: "Your SpecKit Plus project has specifications in git, agents working in worktrees, and completion hooks configured. You realize one agent misunderstood their module's purpose. What's the most efficient recovery approach?",
      options: [
        "Restart the agent in their worktree with clarified specification",
        "Reassign the module to a different agent",
        "Revise the specification and have the agent continue",
        "Continue with misunderstood implementation and fix during integration"
      ],
      correctOption: 0,
      explanation: "Restarting with a clarified specification in the same worktree is most efficient—the isolated environment is already set up, and the agent can start fresh with better understanding. Reassigning (option B) adds unnecessary context transfer overhead. Revising mid-stream (option C) compounds confusion. Fixing during integration (option D) creates technical debt and integration overhead. The principle is that worktree isolation makes restarts cheap—the other agents continue unaffected while this agent gets a clean start with corrected understanding.",
      source: "Lesson 6: SpecKit Orchestrated Execution"
    },
    {
      question: "You're teaching decomposition to a junior developer. They ask how to identify good task boundaries. What's the most valuable guidance?",
      options: [
        "Divide work so each piece is independently testable",
        "Look for places where components communicate through interfaces",
        "Create boundaries where different skill sets are required",
        "Separate concerns based on likely change frequency"
      ],
      correctOption: 1,
      explanation: "Interface-based boundaries are the most fundamental decomposition principle—they create natural separation points with clear contracts. This applies universally across system design, parallel development, and team organization. Independent testing (option B) is valuable but a consequence of good boundaries, not the identification method. Skill sets (option C) affect assignment but not architectural decomposition. Change frequency (option D) is a valid principle but secondary to interface identification. The core insight is that systems naturally divide where components interact through well-defined interfaces—identifying these interface points reveals natural decomposition boundaries.",
      source: "Lesson 4: Scaling Decomposition Thinking"
    },
    {
      question: "Your integration reveals that Module A calls Module B with parameters in a different order than Module B expects. Both modules followed their individual specifications. What specification element was missing?",
      options: [
        "Shared API documentation for parameter ordering",
        "Explicit interface definitions in each module's specification",
        "Acceptance criteria for cross-module integration testing",
        "The integration contract between Module A and B"
      ],
      correctOption: 3,
      explanation: "Missing or unclear integration contract allowed Module A and B to have incompatible assumptions about their interface. Integration contracts explicitly define how modules interact, including parameter order. Individual specifications (option B) define module internals but not cross-module interfaces. Acceptance criteria (option C) would catch the problem but don't prevent it. Shared API documentation (option D) is essentially what the integration contract should provide. The principle is that integration contracts bridge the gap between module specifications, ensuring compatible interfaces.",
      source: "Lesson 3: Parallel Implementation and Integration"
    },
    {
      question: "You're analyzing why integration took longer than expected. You discover agents interpreted 'user authentication' differently—one built OAuth, another built JWT. Where did the specification fail?",
      options: [
        "The agents should have asked for clarification earlier",
        "The specification should have mandated a specific technology",
        "The specification used ambiguous terminology without technical constraints",
        "The acceptance criteria didn't validate authentication method"
      ],
      correctOption: 2,
      explanation: "The term 'user authentication' is ambiguous without technical constraints. Specifications must be precise enough to prevent incompatible interpretations while avoiding over-specification. Mandating technology (option B) might be over-specification depending on context. Agent questions (option C) indicate specification problems but don't identify the root cause. Acceptance criteria (option D) validate outcomes but don't prevent interpretation divergence. The principle is balancing specificity with flexibility—provide enough technical constraints to ensure compatible implementations without dictating every implementation detail. 'JWT-based authentication' would have prevented the divergence while allowing implementation flexibility.",
      source: "Lesson 1: Git Worktrees for Parallel Specifications"
    },
    {
      question: "After completing several orchestration projects, you notice a pattern: projects with more detailed specifications have shorter integration times. What decomposition principle does this correlation support?",
      options: [
        "Clear contracts enable more effective autonomous parallel work",
        "Detailed specifications reduce agent autonomy but improve outcomes",
        "Upfront planning investment pays dividends during integration",
        "Specification quality directly impacts integration efficiency"
      ],
      correctOption: 0,
      explanation: "The correlation demonstrates that clear contracts (detailed specifications) enable autonomous work that integrates smoothly because agents built to compatible assumptions. Reduced autonomy (option B) is incorrect—clear specifications increase effective autonomy by reducing need for coordination. Upfront planning (option C) is true but less specific than the contract principle. Specification quality (option A) is related but 'clear contracts enabling autonomous work' captures the mechanism. The insight is that detailed specifications aren't restrictive micromanagement—they're enabling infrastructure that allows agents to work independently while ensuring compatible outcomes.",
      source: "Lesson 5: Contract-Based Autonomous Coordination"
    },
    {
      question: "You're measuring a project where four agents worked for 5 hours each (20 agent-hours total) but completed in 7 hours wall-clock time (including planning and integration). What does this reveal about work distribution?",
      options: [
        "The agents had significant idle time or blocking",
        "Not all agents worked the full duration in parallel",
        "Coordination overhead consumed 2 hours of the timeline",
        "Planning and integration added 2 hours sequentially"
      ],
      correctOption: 1,
      explanation: "If four agents worked truly parallel for their full 5-hour durations, wall-clock time would be 5 hours (plus overhead). Seven hours indicates agents weren't all working the full duration—some finished early, some started late, or work had sequential dependencies. Idle time (option A) is a consequence but doesn't explain the math. Coordination overhead (option B) doesn't explain why agent-hours sum to 20 but wall-clock is 7. Sequential overhead (option D) would add to the longest task, but 7 hours total suggests staggered work. The insight is that agent-hours vs wall-clock time reveals work distribution patterns and parallelization effectiveness.",
      source: "Lesson 7: Capstone Project Measurement"
    },
    {
      question: "Your worktree setup has three directories: ../main (main branch), ../feat-api (feature/api), ../feat-db (feature/database). You want to merge feature/api into main. What's the correct git workflow?",
      options: [
        "cd ../feat-api && git merge main first",
        "cd ../feat-api && git checkout main && merge",
        "cd ../main && git checkout feature/api && merge",
        "cd ../main && git merge feature/api"
      ],
      correctOption: 3,
      explanation: "To merge feature/api into main, switch to the target branch's worktree (main) and merge the source branch (feature/api). Option A correctly navigates to main directory and merges feature/api. Option B attempts to checkout main in the feature worktree, which violates the one-branch-per-worktree rule. Option C tries to checkout feature/api in main's worktree (same violation). Option D merges in the wrong direction (main into feature). The principle is: always switch to the destination branch's worktree, then merge the source branch by name.",
      source: "Lesson 1: Git Worktrees for Parallel Specifications"
    },
    {
      question: "You're reviewing a completed project that achieved 1.8x speedup with three agents. The team wants to know if this is good performance. What context do you need to provide an accurate assessment?",
      options: [
        "The ratio of execution time to coordination overhead",
        "The nature of dependencies and parallelization potential",
        "All factors including overhead, dependencies, and critical path",
        "The sequential portion of work based on task dependencies"
      ],
      correctOption: 2,
      explanation: "Comprehensive assessment requires understanding all factors: coordination overhead, dependency constraints, critical path duration, and planning/integration time. A project with high sequential portions achieving 1.8x is excellent; a perfectly parallelizable project achieving 1.8x is poor. Overhead ratio (option A) is one factor but insufficient. Dependencies (option B) matter but don't explain the full picture. Sequential portions (option C) are critical but incomplete without overhead context. The principle is that speedup is relative to theoretical maximum given constraints—1.8x might represent 90% efficiency or 50% efficiency depending on context.",
      source: "Lesson 7: Capstone Project Measurement"
    }
  ]}
  questionsPerBatch={18}
/>
