# TypeScript SDK Patterns

## Query Object Methods

```typescript
interface Query extends AsyncGenerator<SDKMessage, void> {
  interrupt(): Promise<void>;
  rewindFiles(userMessageUuid: string): Promise<void>;
  setPermissionMode(mode: PermissionMode): Promise<void>;
  setModel(model?: string): Promise<void>;
  supportedCommands(): Promise<SlashCommand[]>;
  supportedModels(): Promise<ModelInfo[]>;
  mcpServerStatus(): Promise<McpServerStatus[]>;
  accountInfo(): Promise<AccountInfo>;
}
```

## Complete Options Type

```typescript
interface Options {
  abortController?: AbortController;
  additionalDirectories?: string[];
  agents?: Record<string, AgentDefinition>;
  allowDangerouslySkipPermissions?: boolean;
  allowedTools?: string[];
  betas?: SdkBeta[];  // e.g., ['context-1m-2025-08-07']
  canUseTool?: CanUseTool;
  continue?: boolean;
  cwd?: string;
  disallowedTools?: string[];
  enableFileCheckpointing?: boolean;
  env?: Dict<string>;
  forkSession?: boolean;
  hooks?: Partial<Record<HookEvent, HookCallbackMatcher[]>>;
  includePartialMessages?: boolean;
  maxBudgetUsd?: number;
  maxThinkingTokens?: number;
  maxTurns?: number;
  mcpServers?: Record<string, McpServerConfig>;
  model?: string;
  outputFormat?: { type: 'json_schema', schema: JSONSchema };
  permissionMode?: PermissionMode;
  plugins?: SdkPluginConfig[];
  resume?: string;
  sandbox?: SandboxSettings;
  settingSources?: SettingSource[];
  systemPrompt?: string | { type: 'preset'; preset: 'claude_code'; append?: string };
}
```

## Message Types

```typescript
type SDKMessage =
  | SDKAssistantMessage
  | SDKUserMessage
  | SDKResultMessage
  | SDKSystemMessage
  | SDKPartialAssistantMessage
  | SDKCompactBoundaryMessage;

type SDKResultMessage = {
  type: 'result';
  subtype: 'success' | 'error_max_turns' | 'error_during_execution';
  session_id: string;
  duration_ms: number;
  is_error: boolean;
  num_turns: number;
  total_cost_usd: number;
  result?: string;
  structured_output?: unknown;
};
```

## Custom Permission Handler

```typescript
const canUseTool: CanUseTool = async (toolName, input, { signal, suggestions }) => {
  // Block dangerous commands
  if (toolName === 'Bash' && String(input.command).includes('rm -rf')) {
    return {
      behavior: 'deny',
      message: 'Dangerous command blocked',
      interrupt: true
    };
  }

  // Redirect sensitive paths
  if (toolName === 'Write' && String(input.file_path).includes('config')) {
    return {
      behavior: 'allow',
      updatedInput: { ...input, file_path: `./sandbox/${input.file_path}` }
    };
  }

  return { behavior: 'allow', updatedInput: input };
};
```

## Streaming Input Pattern

```typescript
import { query } from "@anthropic-ai/claude-agent-sdk";
import { readFileSync } from "fs";

async function* generateMessages() {
  yield {
    type: "user" as const,
    message: {
      role: "user" as const,
      content: "First question"
    }
  };

  await new Promise(r => setTimeout(r, 1000));

  // With image
  yield {
    type: "user" as const,
    message: {
      role: "user" as const,
      content: [
        { type: "text", text: "Analyze this diagram" },
        {
          type: "image",
          source: {
            type: "base64",
            media_type: "image/png",
            data: readFileSync("diagram.png", "base64")
          }
        }
      ]
    }
  };
}

for await (const message of query({
  prompt: generateMessages(),
  options: { maxTurns: 5 }
})) {
  console.log(message);
}
```

## Complete Hook Example

```typescript
import { query, HookCallback, PreToolUseHookInput } from "@anthropic-ai/claude-agent-sdk";
import { appendFileSync } from "fs";

const auditLogger: HookCallback = async (input, toolUseID, { signal }) => {
  appendFileSync("audit.log", `${new Date().toISOString()}: ${input.tool_name}\n`);
  return {};
};

const securityCheck: HookCallback = async (input, toolUseID, { signal }) => {
  const preInput = input as PreToolUseHookInput;

  if (preInput.tool_name === 'Bash') {
    const command = String(preInput.tool_input?.command ?? '');
    const dangerous = ['rm -rf', 'sudo', 'chmod 777'];

    if (dangerous.some(d => command.includes(d))) {
      return {
        hookSpecificOutput: {
          hookEventName: 'PreToolUse',
          permissionDecision: 'deny',
          permissionDecisionReason: `Blocked: ${command.slice(0, 50)}`
        }
      };
    }
  }
  return {};
};

const sandboxWrites: HookCallback = async (input, toolUseID, { signal }) => {
  const preInput = input as PreToolUseHookInput;

  if (['Write', 'Edit'].includes(preInput.tool_name)) {
    const filePath = preInput.tool_input?.file_path as string;
    return {
      hookSpecificOutput: {
        hookEventName: 'PreToolUse',
        permissionDecision: 'allow',
        updatedInput: {
          ...preInput.tool_input,
          file_path: `/sandbox${filePath}`
        }
      }
    };
  }
  return {};
};

for await (const message of query({
  prompt: "Task description",
  options: {
    hooks: {
      PreToolUse: [
        { hooks: [auditLogger] },
        { matcher: 'Bash', hooks: [securityCheck] },
        { matcher: 'Write|Edit', hooks: [sandboxWrites] }
      ],
      PostToolUse: [{ hooks: [auditLogger] }]
    }
  }
})) {
  console.log(message);
}
```

## Custom MCP Tools with Zod

```typescript
import { z } from "zod";
import { query, tool, createSdkMcpServer } from "@anthropic-ai/claude-agent-sdk";

const weatherServer = createSdkMcpServer({
  name: "weather",
  version: "1.0.0",
  tools: [
    tool(
      "get_weather",
      "Get current temperature",
      {
        latitude: z.number().describe("Latitude"),
        longitude: z.number().describe("Longitude"),
        units: z.enum(["celsius", "fahrenheit"]).default("celsius")
      },
      async (args) => {
        const temp = await fetchWeather(args.latitude, args.longitude);
        return {
          content: [{
            type: "text",
            text: `Temperature: ${temp}°${args.units === "celsius" ? "C" : "F"}`
          }]
        };
      }
    ),

    tool(
      "get_forecast",
      "Get 5-day forecast",
      {
        location: z.object({
          city: z.string(),
          country: z.string().optional()
        }),
        days: z.number().min(1).max(7).default(5)
      },
      async (args) => {
        const forecast = await fetchForecast(args.location, args.days);
        return {
          content: [{
            type: "text",
            text: JSON.stringify(forecast, null, 2)
          }]
        };
      }
    )
  ]
});

// Use with streaming input (required for MCP)
async function* prompt() {
  yield {
    type: "user" as const,
    message: { role: "user" as const, content: "What's the weather in Paris?" }
  };
}

for await (const message of query({
  prompt: prompt(),
  options: {
    mcpServers: { weather: weatherServer },
    allowedTools: ["mcp__weather__get_weather", "mcp__weather__get_forecast"]
  }
})) {
  if (message.type === "result") console.log(message.result);
}
```

## Subagent Definition

```typescript
const options = {
  allowedTools: ['Read', 'Grep', 'Glob', 'Task'],
  agents: {
    'security-reviewer': {
      description: 'Security and vulnerability analysis',
      prompt: `You are a security expert. Analyze code for:
        - SQL injection
        - XSS vulnerabilities
        - Authentication issues
        - Secrets in code`,
      tools: ['Read', 'Grep', 'Glob'],
      model: 'opus'  // Use more capable model
    },

    'test-writer': {
      description: 'Test generation and coverage analysis',
      prompt: 'Generate comprehensive tests for code.',
      tools: ['Read', 'Write', 'Bash']
      // model inherits from parent
    }
  }
};
```

## Session Management

```typescript
let sessionId: string | undefined;

// First query
for await (const message of query({
  prompt: "Start analysis",
  options: { model: "claude-sonnet-4-5" }
})) {
  if (message.type === 'system' && message.subtype === 'init') {
    sessionId = message.session_id;
  }
}

// Resume session
for await (const message of query({
  prompt: "Continue the analysis",
  options: { resume: sessionId }
})) {
  console.log(message);
}

// Fork session (creates new branch)
for await (const message of query({
  prompt: "Try alternative approach",
  options: { resume: sessionId, forkSession: true }
})) {
  // New session ID, original preserved
}
```

## Hook Event Types (TypeScript-only)

```typescript
type HookEvent =
  | 'PreToolUse'       // Before tool executes
  | 'PostToolUse'      // After tool completes
  | 'PostToolUseFailure' // Tool failed
  | 'Notification'     // Agent status messages
  | 'UserPromptSubmit' // Prompt submitted
  | 'SessionStart'     // Session initialized
  | 'SessionEnd'       // Session terminated
  | 'Stop'             // Agent stops
  | 'SubagentStart'    // Subagent spawned
  | 'SubagentStop'     // Subagent finished
  | 'PreCompact'       // Before compaction
  | 'PermissionRequest'; // Permission dialog
```

## Sandbox Configuration

```typescript
const options = {
  sandbox: {
    enabled: true,
    autoAllowBashIfSandboxed: true,
    excludedCommands: ['docker'],
    allowUnsandboxedCommands: false,
    network: {
      allowLocalBinding: true,
      allowUnixSockets: ['/var/run/docker.sock'],
      httpProxyPort: 8080
    },
    ignoreViolations: {
      file: ['/tmp/*'],
      network: ['localhost']
    }
  }
};
```

## System Prompt Preset

```typescript
// Use Claude Code's system prompt with additions
const options = {
  systemPrompt: {
    type: 'preset',
    preset: 'claude_code',
    append: 'Always follow PEP 8 style guidelines.'
  },
  settingSources: ['project']  // Required to load CLAUDE.md
};
```
