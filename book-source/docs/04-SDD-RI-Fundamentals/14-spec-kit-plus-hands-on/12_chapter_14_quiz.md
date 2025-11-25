---
sidebar_position: 12
title: "Chapter 14: Spec-Kit Plus Hands-On Quiz"
---

# Chapter 14: Spec-Kit Plus Hands-On Quiz

Test your understanding of SDD-RI through the video generation project. This assessment covers the complete workflow from specification to skill creation.

<Quiz
  title="Chapter 14: SDD-RI Video Generation Assessment"
  questions={[
    {
      question: "You're writing a specification for video generation. Your first draft says 'Generate a good product demo video.' During clarify phase, this is flagged as inadequate. What makes this specification insufficient?",
      options: [
        "It doesn't specify which AI tool to use for generation",
        "It lacks measurable success criteria and constraints",
        "It focuses on video instead of the underlying business problem",
        "It doesn't include implementation details like Playwright MCP"
      ],
      correctOption: 1,
      explanation: "The specification lacks measurable success criteria ('good' is subjective) and constraints (duration, format, resolution). A proper specification would say: 'Generate a 45-60 second MP4 video at 1080p showing the sign-up flow with clear CTA.' The first option is incorrect because tool choice belongs in the planning phase, not specification. The third option is partially valid but not the primary issue—video IS the deliverable. The fourth option confuses specification with implementation; Playwright MCP details belong in the plan, not the spec.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "During the planning phase for video generation, AI researches Gemini.google.com capabilities. Why is this research step essential before creating the implementation plan?",
      options: [
        "To document AI usage for compliance and audit purposes",
        "To generate marketing materials about Gemini features",
        "To discover actual constraints that shape architectural decisions",
        "To compare Gemini with competitor AI video tools"
      ],
      correctOption: 2,
      explanation: "Research discovers actual constraints (free tier limits, timeout behavior, output formats) that shape architectural decisions. Without research, you might plan for capabilities Gemini doesn't have or miss constraints that cause implementation failures. The plan must reflect real tool behavior, not assumed capabilities. Compliance documentation and competitor comparison aren't purposes of the planning phase. Marketing materials are completely unrelated to SDD-RI workflow.",
      source: "Lesson 6: Plan Phase"
    },
    {
      question: "Your tasks.md includes a video generation prompt as a deliverable. A teammate questions why a 'prompt' is a task output. What principle does this demonstrate?",
      options: [
        "Tasks should document AI conversations for future reference",
        "Tasks deliver concrete artifacts, including research-backed prompts",
        "Prompts must be version-controlled separately from code",
        "AI prompts require approval before execution"
      ],
      correctOption: 1,
      explanation: "Tasks deliver concrete artifacts—the video prompt IS an artifact derived from planning research. It's not just documentation; it's an executable specification that Gemini will process. The prompt encodes decisions about scene structure, timing, style keywords, and constraints discovered during planning. This transforms 'generate video' from vague to actionable. Version control and approval are process concerns, not the core principle being demonstrated.",
      source: "Lesson 7: Tasks Phase"
    },
    {
      question: "After executing /sp.implement, you have a video file but it's only 30 seconds instead of the specified 45-60 seconds. The checkpoint pattern says you should:",
      options: [
        "Accept the video since it partially meets requirements",
        "Delete the video and start the entire workflow from specification",
        "Iterate with refined prompt, maintaining checkpoint before retry",
        "Document the failure and move to the next project phase"
      ],
      correctOption: 2,
      explanation: "The checkpoint pattern supports iteration: you have a stable state (video exists, workflow works), so you iterate with a refined prompt while keeping the checkpoint. If the retry fails, you can rollback. Accepting partial success violates spec requirements. Deleting and starting over wastes the working infrastructure you've built. Documenting failure without attempting fixes abandons the goal prematurely. The iteration loop is designed for exactly this scenario—quality refinement through cycles.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "You've completed the video generation workflow (L04-L08) and now need to create a 'generate-video' skill. What should this skill primarily capture?",
      options: [
        "The specific video prompt you used for your product demo",
        "Step-by-step Playwright MCP commands with exact selectors",
        "Decision frameworks for video generation that apply across projects",
        "Documentation of Gemini.google.com's current feature set"
      ],
      correctOption: 2,
      explanation: "Skills capture decision frameworks that apply across projects—how to structure prompts, what quality gates to apply, when to iterate. Project-specific prompts and exact selectors are too narrow; they work for THIS video but not future videos. Gemini documentation changes over time and isn't reusable intelligence. The P+Q+P framework (Persona + Questions + Principles) encodes transferable expertise: 'When generating video, ask about target audience, duration constraints, and quality standards' applies universally.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "In Lesson 11 (YouTube Capstone), your YouTube upload specification is shorter than your original video generation specification. Why is this a sign of successful intelligence accumulation?",
      options: [
        "YouTube upload is inherently simpler than video generation",
        "You've learned to write more concise specifications through practice",
        "Skills handle complexity that would otherwise be in the specification",
        "The upload task requires less stakeholder coordination"
      ],
      correctOption: 2,
      explanation: "Skills handle complexity that would otherwise be in the specification. Your 'upload-youtube' skill encodes workflow decisions (browser automation, metadata fields, validation steps), so the specification only needs to define intent and success criteria. The skill IS reusable intelligence—it removes decisions from future specs. YouTube upload isn't inherently simpler; it involves browser automation, authentication, metadata management. Concise writing is good but doesn't explain WHY you can write less. Stakeholder coordination is unchanged.",
      source: "Lesson 11: YouTube Capstone"
    },
    {
      question: "Your video generation took 3.5 hours (Lessons 4-8). Your YouTube upload took 70 minutes using skills you created. What does this 65% time reduction demonstrate?",
      options: [
        "YouTube is a faster platform than Gemini for content creation",
        "Intelligence accumulation accelerates subsequent projects",
        "Skills eliminate the need for specification in future projects",
        "Second attempts at any task are naturally faster"
      ],
      correctOption: 1,
      explanation: "Intelligence accumulation accelerates subsequent projects—the skills you created encode decisions and workflows that you don't need to rediscover. This is the core SDD-RI value proposition: build intelligence once, reuse across projects. Platform speed differences don't explain the reduction (both use Playwright MCP). Skills don't eliminate specification; they make it simpler. While practice helps, 65% reduction specifically comes from reusing encoded intelligence, not just familiarity.",
      source: "Lesson 11: YouTube Capstone"
    },
    {
      question: "The Constitution phase for your video project defines 'All videos must be MP4 format with H.264 codec.' During implementation, Gemini generates WebM format. What should happen?",
      options: [
        "Accept WebM since Gemini determined it's the best format",
        "Update the Constitution to allow WebM format",
        "Fail the quality gate and iterate until MP4 is achieved",
        "Create exception documentation for this specific video"
      ],
      correctOption: 2,
      explanation: "The Constitution defines non-negotiable standards. If Gemini outputs WebM, you fail the quality gate and iterate—either refine the prompt to request MP4 specifically, or add a conversion step. Accepting non-compliant output violates the cascade principle. Updating Constitution for one failure undermines its purpose as stable foundation. Exception documentation creates drift. The Constitution exists precisely to prevent accepting 'close enough' outputs that cause downstream problems (YouTube might reject WebM, or playback fails on certain devices).",
      source: "Lesson 3: Constitution Phase"
    },
    {
      question: "Playwright MCP session persistence saves your Google login. Why is this an architectural decision documented in an ADR rather than just implementation detail?",
      options: [
        "Session persistence affects multiple features and has long-term implications",
        "ADRs are required for all browser automation configurations",
        "Session persistence is a security decision requiring formal approval",
        "Implementation details must be documented somewhere for compliance"
      ],
      correctOption: 0,
      explanation: "Session persistence affects multiple features (video generation, YouTube upload, any future Gemini automation) and has long-term implications (security considerations, session expiration handling, credential management). ADR-worthy decisions have broad impact and are costly to reverse. Not all browser configs need ADRs—only significant architectural choices. While security is relevant, the ADR documents the reasoning, not approval. Compliance isn't the driver; understanding future maintenance implications is.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "Your video generation prompt includes 'Scene 1 (0-10s): Modern SaaS dashboard with clean UI.' This scene-by-scene structure was discovered during which phase?",
      options: [
        "Specification phase when defining success criteria",
        "Clarification phase when resolving ambiguities",
        "Planning phase when researching Gemini capabilities",
        "Implementation phase when iterating on output quality"
      ],
      correctOption: 2,
      explanation: "Scene-by-scene structure is discovered during planning when AI researches what prompt structures work best with Gemini. This is research-driven planning: you learn that Gemini responds better to explicit scene breakdowns than vague descriptions. Specification defines WHAT (45-60 second video showing sign-up flow), not HOW to prompt. Clarification refines existing spec content, not prompt engineering. Implementation might refine the structure, but the pattern is discovered in planning.",
      source: "Lesson 6: Plan Phase"
    },
    {
      question: "The quality gate 'Duration is 45-60 seconds' uses ffprobe for validation. What makes this gate effective compared to 'Video is the right length'?",
      options: [
        "ffprobe is a standard tool that all developers know",
        "Technical tools are always more reliable than human judgment",
        "The gate is measurable, automated, and produces pass/fail result",
        "Duration checking requires specialized video processing software"
      ],
      correctOption: 2,
      explanation: "Effective quality gates are measurable (45-60 seconds is precise), automated (ffprobe runs without manual inspection), and produce unambiguous pass/fail results. 'Right length' is subjective—one person's 'right' might be 30 seconds, another's 90 seconds. Tool familiarity and reliability aren't the key factors; measurability is. Not all duration checks require specialized software, but the principle—objective, automated verification—applies to any effective quality gate.",
      source: "Lesson 8: Implement Phase"
    },
    {
      question: "During brownfield adoption, you want to add Spec-Kit Plus to an existing project with its own folder structure. The safest approach is:",
      options: [
        "Reorganize existing folders to match Spec-Kit Plus conventions",
        "Initialize Spec-Kit Plus alongside existing structure with clear boundaries",
        "Create a new repository and migrate code after Spec-Kit Plus setup",
        "Defer adoption until the project has fewer active changes"
      ],
      correctOption: 1,
      explanation: "Initialize alongside existing structure with clear boundaries—this is the strangler fig pattern. Spec-Kit Plus can coexist with existing patterns; new features use SDD-RI while legacy code remains unchanged until touched. Reorganizing existing folders risks breaking working code. Migration to new repository is disruptive and unnecessary. Waiting for 'fewer changes' often means waiting forever. Brownfield adoption works precisely because it doesn't require wholesale changes.",
      source: "Lesson 10: Brownfield Adoption"
    },
    {
      question: "Your 'upload-youtube' skill includes a Persona: 'Content publisher focused on SEO optimization and audience engagement.' What purpose does this Persona serve?",
      options: [
        "Documents the intended user of the skill for future reference",
        "Activates reasoning mode with domain-specific decision frameworks",
        "Provides marketing description for skill catalog listings",
        "Identifies which team member should use the skill"
      ],
      correctOption: 1,
      explanation: "The Persona activates reasoning mode—when AI adopts this persona, it thinks about SEO, audience engagement, and publishing best practices rather than generic code generation. This is the P+Q+P framework: Persona provides cognitive stance, Questions guide analysis, Principles constrain decisions. The persona isn't documentation, marketing, or team assignment; it's an activation mechanism that transforms how AI reasons about the task.",
      source: "Lesson 9: Designing Reusable Intelligence"
    },
    {
      question: "The specification Non-Goals section states 'No post-generation video editing.' A stakeholder later asks for trimming capability. How should this be handled?",
      options: [
        "Add trimming to current specification since stakeholder requested it",
        "Explain that Non-Goals explicitly excluded editing; create separate feature spec if needed",
        "Implement trimming anyway since it's a minor enhancement",
        "Remove the Non-Goals section to allow more flexibility"
      ],
      correctOption: 1,
      explanation: "Non-Goals explicitly excluded editing. If trimming is now needed, it requires a separate feature specification with its own planning and implementation. Non-Goals prevent scope creep by making exclusions explicit—when stakeholders request excluded features, the answer is 'that's a new feature, not part of this scope.' Adding it silently violates the specification contract. Implementing 'minor' enhancements without specification is how scope creep happens. Removing Non-Goals defeats their purpose.",
      source: "Lesson 4: Specify Phase"
    },
    {
      question: "After completing Chapter 14, a student can explain 'why YouTube upload was faster than video generation.' This explanation demonstrates understanding of:",
      options: [
        "Technical differences between Gemini and YouTube APIs",
        "How skills encode decisions that don't need to be remade",
        "Practice effects where second attempts are naturally faster",
        "Specification complexity differences between upload and generation"
      ],
      correctOption: 1,
      explanation: "The explanation demonstrates understanding of intelligence accumulation—skills encode decisions (workflow steps, quality gates, prompt patterns) that don't need to be remade. This is the core learning outcome: students experience acceleration and can articulate WHY it happened. Technical API differences don't explain 65% time reduction. Practice helps but doesn't account for the dramatic difference. Specification complexity differences are a symptom (skills make specs simpler), not the root cause.",
      source: "Lesson 11: YouTube Capstone"
    },
    {
      question: "Throughout Chapter 14, you collaborated with AI to build a video generation system. What approach best supports effective AI collaboration in learning?",
      options: [
        "Detailed explanations of how AI processes information internally",
        "Natural interaction where AI suggests approaches and adapts to your feedback",
        "Studying theoretical frameworks before hands-on practice",
        "Reading documentation about AI collaboration patterns"
      ],
      correctOption: 1,
      explanation: "Effective AI collaboration happens through natural interaction—AI suggests approaches you might not know, adapts when you provide feedback, and together you converge on solutions. Students learn by DOING, not by studying theory about how collaboration works. Detailed AI explanations distract from the task. Theory-first delays hands-on learning. Reading documentation doesn't build practical skill. The best learning happens through action.",
      source: "Chapter 14: Hands-On AI Collaboration"
    },
    {
      question: "Horizontal Intelligence (ADRs, PHRs) and Vertical Intelligence (Skills, Subagents) serve different purposes. Which statement correctly distinguishes them?",
      options: [
        "Horizontal captures decisions over time; Vertical delegates specialized tasks",
        "Horizontal is for teams; Vertical is for individuals",
        "Horizontal documents failures; Vertical documents successes",
        "Horizontal is required; Vertical is optional"
      ],
      correctOption: 0,
      explanation: "Horizontal Intelligence (ADRs, PHRs) captures decisions and reasoning over time—why we chose Playwright MCP, what prompts worked. Vertical Intelligence (Skills, Subagents) delegates specialized tasks through P+Q+P frameworks. Both serve different accumulation patterns: horizontal builds institutional memory; vertical builds executable expertise. The other options mischaracterize these concepts—both are for teams, both capture successes and failures, and both are integral to SDD-RI.",
      source: "Lesson 1: Spec-Kit Plus Foundation"
    },
    {
      question: "Your video generation task has checkpoint after Phase 2 (session setup) and Phase 4 (validation). Phase 3 (generation) fails midway. What does the checkpoint pattern enable?",
      options: [
        "Automatic rollback to the beginning of the project",
        "Resumption from Phase 2 checkpoint without repeating session setup",
        "Skip to Phase 4 validation despite incomplete generation",
        "Manual review of all phases before any retry attempt"
      ],
      correctOption: 1,
      explanation: "Checkpoints enable resumption from stable states—you don't repeat Phase 1-2 (setup and session already work), you resume from Phase 2 checkpoint and retry Phase 3. This is the value of granular checkpoints: minimize work lost on failure. Automatic rollback to beginning wastes completed work. Skipping to validation makes no sense—there's nothing to validate. Manual review of all phases is unnecessary overhead when you know exactly where failure occurred.",
      source: "Lesson 8: Implement Phase"
    }
  ]}
  questionsPerBatch={18}
/>
