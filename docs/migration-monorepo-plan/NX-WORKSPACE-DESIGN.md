# Nx Workspace Structure Design

**Date**: 2025-12-15
**Status**: Final Specification
**Complexity**: MEDIUM
**Approach**: Polyglot Monorepo (Node.js + Python via Nx custom executors)

---

## Executive Summary

This document specifies the complete Nx workspace structure for the Panaversity platform. The design accommodates:
- **2 production projects**: `book-source` (Docusaurus) + `panaversity-fs` (Python MCP)
- **6 npm packages**: 4 plugins + 2 supporting libs
- **Mixed build systems**: npm (root orchestrator) + uv (Python isolated)
- **Production CI/CD**: Zero-downtime deployment with `nx affected` caching

**Key Design Principles**:
1. **Polyglot isolation**: Python runs independently; Nx orchestrates via custom executors
2. **Plugin-first**: Plugins are first-class Nx projects, not hidden in `node_modules`
3. **Production-safe**: CLI/CI patterns use `nx affected` to minimize deployment risk
4. **Cache-enabled**: All targets support `inputs`/`outputs` for Nx caching + Nx Cloud

---

## 1. Target Directory Structure

```
storage/
├── nx.json                              # Workspace configuration
├── pnpm-workspace.yaml                  # pnpm workspace definition
├── .nxignore                            # Paths excluded from Nx
│
├── apps/                                # Deployable projects
│   ├── website/                         # Docusaurus site (renamed from book-source)
│   │   ├── project.json
│   │   ├── package.json
│   │   ├── docs/
│   │   ├── plugins/                     # Plugins live here for clarity
│   │   ├── src/
│   │   ├── static/
│   │   └── ...
│   │
│   └── panaversity-fs-py/               # Python MCP server (Nx custom executor)
│       ├── project.json
│       ├── pyproject.toml
│       ├── src/
│       ├── tests/
│       └── ...
│
├── libs/                                # Shared libraries
│   ├── remark-interactive-python/       # Plugin: Python interactivity
│   │   ├── project.json
│   │   ├── package.json
│   │   └── index.js
│   │
│   ├── remark-content-enhancements/     # Plugin: Remark enhancements
│   │   ├── project.json
│   │   ├── package.json
│   │   └── index.js
│   │
│   ├── docusaurus-plugin-og-image/      # Plugin: OG image generation
│   │   ├── project.json
│   │   ├── package.json
│   │   └── index.js
│   │
│   ├── docusaurus-plugin-structured-data/  # Plugin: Structured data
│   │   ├── project.json
│   │   ├── package.json
│   │   └── index.js
│   │
│   ├── docusaurus-summaries-plugin/     # Plugin: Summaries
│   │   ├── project.json
│   │   ├── package.json
│   │   └── index.js
│   │
│   ├── docusaurus-panaversityfs-plugin/ # Plugin: FS integration
│   │   ├── project.json
│   │   ├── package.json
│   │   └── index.js
│   │
│   └── docusaurus-core-plugins/         # Shared Docusaurus utilities
│       ├── project.json
│       ├── package.json
│       └── src/
│
├── tools/                               # Build tools & scripts
│   ├── executors/                       # Nx custom executors
│   │   ├── python-executor/             # Custom executor for Python targets
│   │   │   ├── impl.ts
│   │   │   ├── schema.json
│   │   │   └── package.json
│   │   └── project.json
│   │
│   └── scripts/                         # Shared build scripts
│       ├── hydrate-book.py              # Move from panaversity-fs
│       ├── ingest-book.py               # Move from panaversity-fs
│       └── project.json
│
├── .claude/                             # Claude Code configuration
├── .github/                             # GitHub Actions (update references)
├── .specify/                            # Platform specifications
├── context/                             # Context files
├── docs/                                # Architecture documentation
├── history/                             # PHRs and learning history
├── papers/                              # Research papers
├── specs/                               # Feature specifications
│
└── [Removed during migration]
    ├── apps/learn-app/                     # MOVE → apps/website/
    ├── panaversity-fs/                  # MOVE → apps/panaversity-fs-py/
```

---

## 2. Nx Configuration Files

### 2.1 nx.json

**Location**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/nx.json`

```json
{
  "$schema": "./node_modules/nx/schemas/nx-schema.json",
  "version": 2,
  "projectNameAndRootFormat": "as-provided",

  "workspaceLayout": {
    "appsDir": "apps",
    "libsDir": "libs"
  },

  "defaultProject": "website",

  "namedInputs": {
    "default": ["{projectRoot}/**/*"],
    "production": [
      "default",
      "!{projectRoot}/**/*.spec.ts",
      "!{projectRoot}/**/*.test.ts",
      "!{projectRoot}/**/*.spec.js",
      "!{projectRoot}/**/*.test.js"
    ],
    "npmInputs": [
      "{projectRoot}/package.json",
      "{projectRoot}/**/*.ts",
      "{projectRoot}/**/*.js",
      "{projectRoot}/**/*.jsx",
      "{projectRoot}/**/*.tsx",
      "{projectRoot}/**/*.json"
    ],
    "pythonInputs": [
      "{projectRoot}/pyproject.toml",
      "{projectRoot}/uv.lock",
      "{projectRoot}/**/*.py"
    ],
    "docusaurusInputs": [
      "{projectRoot}/package.json",
      "{projectRoot}/**/*.ts",
      "{projectRoot}/**/*.js",
      "{projectRoot}/**/*.jsx",
      "{projectRoot}/**/*.tsx",
      "{projectRoot}/**/*.json",
      "{projectRoot}/**/*.md",
      "{projectRoot}/**/*.mdx",
      "{projectRoot}/docs/**/*",
      "{projectRoot}/static/**/*"
    ]
  },

  "targetDefaults": {
    "test": {
      "inputs": ["default", "^production"],
      "cache": true
    },
    "lint": {
      "inputs": ["default"],
      "cache": true
    },
    "build": {
      "cache": true,
      "dependsOn": ["^build"]
    }
  },

  "pluginsConfig": {
    "@nx/js": {
      "analyzeSourceFiles": true
    }
  },

  "tasksRunnerOptions": {
    "default": {
      "runner": "nx/tasks-runners/default",
      "options": {
        "cacheableOperations": [
          "build",
          "test",
          "lint",
          "typecheck"
        ],
        "useDaemonProcess": true
      }
    }
  },

  "cli": {
    "defaultCollection": "@nx/node",
    "analytics": false
  }
}
```

### 2.2 pnpm-workspace.yaml

**Location**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/pnpm-workspace.yaml`

```yaml
packages:
  # Main applications
  - "apps/**"

  # Shared libraries / plugins
  - "libs/**"

  # Build tools
  - "tools/**"

include-filtered-lockfile: true
```

### 2.3 .nxignore

**Location**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/.nxignore`

```
# Documentation
docs/
papers/

# Configuration and metadata
.claude/
.github/
.specify/
history/
specs/
context/

# Root-level assets
cover.png
README.md
CLAUDE.md
MIGRATION-SUMMARY.md
MONOREPO-MIGRATION-ANALYSIS.md
NX-WORKSPACE-DESIGN.md

# Hidden folders
.hypothesis/
.pytest_cache/
.panaversity/

# Python
.venv/
*.egg-info/

# Lock files (managed by pnpm)
pnpm-lock.yaml
uv.lock

# Build outputs
.docusaurus/
dist/
build/
```

---

## 3. Project Configuration Files

### 3.1 apps/website/project.json

**Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/website/project.json`

```json
{
  "name": "website",
  "description": "Docusaurus site for Panaversity AI-Native Software Development book",
  "projectType": "application",
  "sourceRoot": "apps/website",
  "prefix": "panaversity",

  "targets": {
    "build": {
      "executor": "@nx/node:build",
      "outputs": ["{options.outputPath}"],
      "options": {
        "outputPath": "dist/apps/website",
        "main": "apps/website/docusaurus.config.ts",
        "tsConfig": "apps/website/tsconfig.json",
        "buildLibsFromSource": false
      },
      "configurations": {
        "production": {
          "optimization": true,
          "sourceMap": false,
          "namedChunks": false,
          "extractLicenses": true,
          "vendorChunk": false
        }
      },
      "dependsOn": [
        "^build",
        {
          "projects": "self",
          "target": "hydrate"
        }
      ]
    },

    "serve": {
      "executor": "@docusaurus/plugin-runner:serve",
      "options": {
        "outputPath": "dist/apps/website",
        "port": 3000
      },
      "dependsOn": ["hydrate"]
    },

    "deploy": {
      "executor": "nx:run-commands",
      "options": {
        "command": "cd apps/website && npm run deploy"
      },
      "dependsOn": ["build"]
    },

    "hydrate": {
      "executor": "nx:run-commands",
      "options": {
        "command": "python tools/scripts/hydrate-book.py",
        "env": {
          "BOOK_ID": "ai-native-python",
          "SOURCE_DIR": "apps/website/docs",
          "OUTPUT_DIR": ".docusaurus/content"
        }
      },
      "inputs": [
        "npmInputs",
        {
          "externalDependencies": ["panaversity-fs"]
        }
      ],
      "outputs": [".docusaurus/content"]
    },

    "ingest": {
      "executor": "nx:run-commands",
      "options": {
        "command": "python tools/scripts/ingest-book.py",
        "env": {
          "BOOK_ID": "ai-native-python",
          "SOURCE_DIR": "apps/website/docs"
        }
      },
      "inputs": ["docusaurusInputs"]
    },

    "lint": {
      "executor": "@nx/eslint:lint",
      "options": {
        "lintFilePatterns": [
          "apps/website/**/*.ts",
          "apps/website/**/*.tsx",
          "apps/website/**/*.js",
          "apps/website/**/*.jsx"
        ]
      },
      "outputs": ["{options.outputPath}"],
      "inputs": ["npmInputs"]
    },

    "typecheck": {
      "executor": "nx:run-commands",
      "options": {
        "command": "tsc --noEmit -p apps/website/tsconfig.json"
      },
      "inputs": ["npmInputs"]
    },

    "test": {
      "executor": "@nx/jest:jest",
      "options": {
        "jestConfig": "apps/website/jest.config.ts",
        "passWithNoTests": true
      },
      "outputs": ["{workspaceRoot}/coverage/{projectRoot}"],
      "inputs": ["npmInputs"]
    }
  },

  "tags": [
    "type:application",
    "tech:docusaurus",
    "tech:react",
    "scope:website",
    "ci:deploy"
  ]
}
```

### 3.2 apps/panaversity-fs-py/project.json

**Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/panaversity-fs-py/project.json`

```json
{
  "name": "panaversity-fs-py",
  "description": "Agent-Native Multi-Book Storage System - Python MCP Server",
  "projectType": "application",
  "sourceRoot": "apps/panaversity-fs-py",

  "targets": {
    "build": {
      "executor": "tools/executors/python-executor:build",
      "outputs": ["dist/apps/panaversity-fs-py"],
      "options": {
        "command": "uv build",
        "cwd": "apps/panaversity-fs-py"
      },
      "inputs": ["pythonInputs"]
    },

    "test": {
      "executor": "tools/executors/python-executor:test",
      "options": {
        "command": "uv run pytest --cov=src/panaversity_fs --cov-report=term-summary",
        "cwd": "apps/panaversity-fs-py"
      },
      "outputs": ["{workspaceRoot}/coverage/apps/panaversity-fs-py"],
      "inputs": ["pythonInputs"]
    },

    "test:watch": {
      "executor": "tools/executors/python-executor:test",
      "options": {
        "command": "uv run pytest-watch",
        "cwd": "apps/panaversity-fs-py"
      }
    },

    "lint": {
      "executor": "tools/executors/python-executor:lint",
      "options": {
        "command": "uv run ruff check src/ tests/",
        "cwd": "apps/panaversity-fs-py"
      },
      "inputs": ["pythonInputs"]
    },

    "format": {
      "executor": "tools/executors/python-executor:format",
      "options": {
        "command": "uv run black src/ tests/",
        "cwd": "apps/panaversity-fs-py"
      }
    },

    "typecheck": {
      "executor": "tools/executors/python-executor:typecheck",
      "options": {
        "command": "uv run mypy src/",
        "cwd": "apps/panaversity-fs-py"
      },
      "inputs": ["pythonInputs"]
    },

    "db:migrate": {
      "executor": "tools/executors/python-executor:db",
      "options": {
        "command": "uv run alembic upgrade head",
        "cwd": "apps/panaversity-fs-py"
      }
    },

    "db:downgrade": {
      "executor": "tools/executors/python-executor:db",
      "options": {
        "command": "uv run alembic downgrade -1",
        "cwd": "apps/panaversity-fs-py"
      }
    },

    "serve": {
      "executor": "tools/executors/python-executor:serve",
      "options": {
        "command": "uv run fastapi run src/panaversity_fs/main.py --reload",
        "cwd": "apps/panaversity-fs-py"
      }
    },

    "docker:build": {
      "executor": "nx:run-commands",
      "options": {
        "command": "docker build -f apps/panaversity-fs-py/Dockerfile.prod -t panaversity-fs:latest ."
      }
    }
  },

  "tags": [
    "type:application",
    "tech:python",
    "tech:fastapi",
    "tech:mcp",
    "scope:backend",
    "ci:deploy"
  ]
}
```

### 3.3 Plugin Project Examples

**libs/remark-interactive-python/project.json**

```json
{
  "name": "remark-interactive-python",
  "description": "Remark plugin to transform Python code blocks into interactive components",
  "projectType": "library",
  "sourceRoot": "libs/remark-interactive-python",

  "targets": {
    "build": {
      "executor": "@nx/node:build",
      "outputs": ["{options.outputPath}"],
      "options": {
        "outputPath": "dist/libs/remark-interactive-python",
        "main": "libs/remark-interactive-python/index.js",
        "generatePackageJson": true
      },
      "configurations": {
        "production": {
          "optimization": true,
          "sourceMap": false
        }
      }
    },

    "test": {
      "executor": "@nx/jest:jest",
      "options": {
        "jestConfig": "libs/remark-interactive-python/jest.config.ts",
        "passWithNoTests": true
      },
      "outputs": ["{workspaceRoot}/coverage/{projectRoot}"],
      "inputs": ["npmInputs"]
    },

    "lint": {
      "executor": "@nx/eslint:lint",
      "options": {
        "lintFilePatterns": ["libs/remark-interactive-python/**/*.{ts,tsx,js,jsx}"]
      },
      "outputs": ["{options.outputPath}"],
      "inputs": ["npmInputs"]
    }
  },

  "tags": [
    "type:library",
    "tech:remark",
    "tech:npm",
    "scope:plugins",
    "ci:build"
  ]
}
```

**libs/docusaurus-panaversityfs-plugin/project.json**

```json
{
  "name": "docusaurus-panaversityfs-plugin",
  "description": "Docusaurus plugin to fetch content from PanaversityFS MCP server",
  "projectType": "library",
  "sourceRoot": "libs/docusaurus-panaversityfs-plugin",

  "targets": {
    "build": {
      "executor": "@nx/node:build",
      "outputs": ["{options.outputPath}"],
      "options": {
        "outputPath": "dist/libs/docusaurus-panaversityfs-plugin",
        "main": "libs/docusaurus-panaversityfs-plugin/index.js",
        "generatePackageJson": true
      },
      "configurations": {
        "production": {
          "optimization": true,
          "sourceMap": false
        }
      }
    },

    "test": {
      "executor": "@nx/jest:jest",
      "options": {
        "jestConfig": "libs/docusaurus-panaversityfs-plugin/jest.config.ts",
        "passWithNoTests": true
      },
      "outputs": ["{workspaceRoot}/coverage/{projectRoot}"],
      "inputs": ["npmInputs"]
    },

    "lint": {
      "executor": "@nx/eslint:lint",
      "options": {
        "lintFilePatterns": ["libs/docusaurus-panaversityfs-plugin/**/*.{ts,tsx,js,jsx}"]
      },
      "outputs": ["{options.outputPath}"],
      "inputs": ["npmInputs"]
    }
  },

  "tags": [
    "type:library",
    "tech:docusaurus",
    "tech:npm",
    "scope:plugins",
    "ci:build",
    "external:panaversity-fs"
  ]
}
```

**Pattern for all remaining plugins** (apply to 4 other plugins):
- `libs/remark-content-enhancements/project.json`
- `libs/docusaurus-plugin-og-image/project.json`
- `libs/docusaurus-plugin-structured-data/project.json`
- `libs/docusaurus-summaries-plugin/project.json`

(See Section 6 for template)

### 3.4 tools/executors/project.json

```json
{
  "name": "tools-executors",
  "projectType": "library",
  "sourceRoot": "tools/executors",

  "targets": {
    "build": {
      "executor": "@nx/node:build",
      "outputs": ["{options.outputPath}"],
      "options": {
        "outputPath": "dist/tools/executors",
        "main": "tools/executors/index.ts",
        "tsConfig": "tools/executors/tsconfig.json"
      }
    }
  }
}
```

### 3.5 tools/scripts/project.json

```json
{
  "name": "tools-scripts",
  "projectType": "library",
  "sourceRoot": "tools/scripts",

  "targets": {
    "lint": {
      "executor": "@nx/eslint:lint",
      "options": {
        "lintFilePatterns": ["tools/scripts/**/*.py"]
      }
    }
  },

  "tags": [
    "type:library",
    "tech:python",
    "scope:tools"
  ]
}
```

---

## 4. Root package.json

**Location**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/package.json`

```json
{
  "name": "panaversity-root",
  "version": "1.0.0",
  "description": "Panaversity Monorepo - AI-Native Software Development",
  "private": true,

  "packageManager": "pnpm@9.12.0",
  "engines": {
    "node": ">=20.0.0",
    "pnpm": ">=9.0.0"
  },

  "scripts": {
    "nx": "nx",
    "nx:help": "nx help",
    "nx:show": "nx show projects",
    "nx:graph": "nx graph",
    "nx:affected": "nx affected -t build test",
    "nx:test": "nx run-many -t test",
    "nx:build": "nx run-many -t build",
    "nx:lint": "nx run-many -t lint",
    "nx:format": "nx format:write",
    "nx:typecheck": "nx run-many -t typecheck",
    "start": "nx serve website",
    "build": "nx build website",
    "build:affected": "nx affected -t build",
    "test": "nx run-many -t test",
    "test:affected": "nx affected -t test",
    "lint": "nx run-many -t lint",
    "format": "nx format:write",
    "clean": "nx reset && rm -rf dist .docusaurus",
    "dev:website": "nx serve website",
    "dev:fs": "nx serve panaversity-fs-py"
  },

  "devDependencies": {
    "@docusaurus/core": "3.9.2",
    "@docusaurus/preset-classic": "3.9.2",
    "@nx/eslint": "^20.0.0",
    "@nx/js": "^20.0.0",
    "@nx/node": "^20.0.0",
    "@nx/workspace": "^20.0.0",
    "@typescript-eslint/eslint-plugin": "^7.0.0",
    "@typescript-eslint/parser": "^7.0.0",
    "eslint": "^9.0.0",
    "nx": "^20.0.0",
    "typescript": "~5.6.2"
  },

  "dependencies": {
    "better-auth": "^1.4.3"
  }
}
```

---

## 5. Python Custom Executor (tools/executors/python-executor)

**Location**: `tools/executors/python-executor/`

### 5.1 schema.json

```json
{
  "$schema": "http://json-schema.org/draft-07/schema",
  "type": "object",
  "description": "Executor for running Python commands in Nx projects",
  "properties": {
    "command": {
      "type": "string",
      "description": "The Python command to execute (e.g., 'uv run pytest')"
    },
    "cwd": {
      "type": "string",
      "description": "Working directory for the command",
      "default": "."
    },
    "env": {
      "type": "object",
      "description": "Environment variables to pass to the command",
      "additionalProperties": {
        "type": "string"
      }
    }
  },
  "required": ["command"]
}
```

### 5.2 impl.ts

```typescript
import { ExecutorContext, runCommand } from '@nx/devkit';

export interface PythonExecutorOptions {
  command: string;
  cwd?: string;
  env?: Record<string, string>;
}

export default async function runPythonExecutor(
  options: PythonExecutorOptions,
  context: ExecutorContext
) {
  const { command, cwd = context.projectGraph?.nodes[context.projectName]?.data?.sourceRoot, env = {} } = options;

  const result = await runCommand(command, {
    cwd,
    env: {
      ...process.env,
      ...env,
    },
  });

  return { success: result === 0 };
}
```

### 5.3 package.json

```json
{
  "name": "@panaversity/python-executor",
  "version": "1.0.0",
  "description": "Nx executor for Python commands",
  "main": "dist/impl.js"
}
```

---

## 6. Plugin project.json Template

Apply this template to all remaining plugins. Update `{PLUGIN_NAME}` and `{DESCRIPTION}` for each:

```json
{
  "name": "{PLUGIN_NAME}",
  "description": "{DESCRIPTION}",
  "projectType": "library",
  "sourceRoot": "libs/{PLUGIN_NAME}",

  "targets": {
    "build": {
      "executor": "@nx/node:build",
      "outputs": ["{options.outputPath}"],
      "options": {
        "outputPath": "dist/libs/{PLUGIN_NAME}",
        "main": "libs/{PLUGIN_NAME}/index.js",
        "generatePackageJson": true
      },
      "configurations": {
        "production": {
          "optimization": true,
          "sourceMap": false
        }
      },
      "inputs": ["npmInputs"]
    },

    "test": {
      "executor": "@nx/jest:jest",
      "options": {
        "jestConfig": "libs/{PLUGIN_NAME}/jest.config.ts",
        "passWithNoTests": true
      },
      "outputs": ["{workspaceRoot}/coverage/{projectRoot}"],
      "inputs": ["npmInputs"]
    },

    "lint": {
      "executor": "@nx/eslint:lint",
      "options": {
        "lintFilePatterns": ["libs/{PLUGIN_NAME}/**/*.{ts,tsx,js,jsx}"]
      },
      "outputs": ["{options.outputPath}"],
      "inputs": ["npmInputs"]
    }
  },

  "tags": [
    "type:library",
    "tech:npm",
    "scope:plugins",
    "ci:build"
  ]
}
```

---

## 7. Workspace Dependencies & Constraints

### 7.1 Dependency Graph

```
┌─────────────────────────────────────────────┐
│           apps/website                      │
│       (Docusaurus Main App)                 │
└────────────────┬────────────────────────────┘
                 │
                 │ depends on ↓
       ┌─────────┴──────────────────────────┐
       │                                    │
    ┌──┴──────────────────────┐    ┌───────┴────────────────┐
    │   libs/plugins (6x)     │    │ apps/panaversity-fs-py │
    │                         │    │  (Python MCP Server)   │
    └─────────────────────────┘    └────────────────────────┘
       • remark-interactive-python
       • remark-content-enhancements
       • docusaurus-plugin-og-image
       • docusaurus-plugin-structured-data
       • docusaurus-summaries-plugin
       • docusaurus-panaversityfs-plugin
```

### 7.2 Project Dependencies (project.json)

**website** depends on:
- All 6 plugins (via npm package.json)
- panaversity-fs-py (hydrate target reads from MCP server)

**All plugins** depend on:
- Docusaurus peer dependencies (peerDependencies)
- NO cross-plugin dependencies

**panaversity-fs-py** is independent:
- No Node.js dependencies
- Isolated Python environment
- Communicates with website via HTTP (MCP protocol)

### 7.3 Nx Lint Rules (future)

Add to `.eslintrc.json`:

```json
{
  "overrides": [
    {
      "files": ["apps/website/**/*"],
      "rules": {
        "@nx/enforce-module-boundaries": [
          "error",
          {
            "enforceBuildableLibsInline": true,
            "allow": [],
            "depConstraints": [
              {
                "sourceTag": "scope:website",
                "onlyDependOnLibsWithTags": ["scope:plugins", "scope:docusaurus"]
              },
              {
                "sourceTag": "scope:plugins",
                "onlyDependOnLibsWithTags": []
              }
            ]
          }
        ]
      }
    }
  ]
}
```

---

## 8. Migration Checklist

### Phase 1: Foundation (Safe - No CI Changes)

- [ ] **Step 1.1**: Create directory structure
  - [ ] Create `apps/`, `libs/`, `tools/` directories
  - [ ] Move `apps/learn-app/` → `apps/website/` (keep internal structure)
  - [ ] Move `panaversity-fs/` → `apps/panaversity-fs-py/` (keep internal structure)
  - [ ] Move plugins from `apps/website/plugins/` → `libs/*/` (6 projects)
  - [ ] Create `tools/executors/python-executor/`
  - [ ] Create `tools/scripts/` and move Python scripts from panaversity-fs

- [ ] **Step 1.2**: Create configuration files
  - [ ] Write `nx.json` (as specified in Section 2.1)
  - [ ] Write `pnpm-workspace.yaml` (as specified in Section 2.2)
  - [ ] Write `.nxignore` (as specified in Section 2.3)
  - [ ] Update root `package.json` (as specified in Section 4)

- [ ] **Step 1.3**: Update build references
  - [ ] Update `apps/learn-app/package.json` paths (if any relative imports)
  - [ ] Update `panaversity-fs/pyproject.toml` package name
  - [ ] Update CI script references from `panaversity-fs/scripts/` to `tools/scripts/`

- [ ] **Step 1.4**: Validation
  - [ ] Run `nx show projects` → shows 8 projects
  - [ ] Run `nx graph --file=graph.json` → valid JSON with all projects
  - [ ] Run `nx reset && npm install`
  - [ ] Verify no npm errors

### Phase 2: Configuration (Safe - No CI Changes)

- [ ] **Step 2.1**: Create all project.json files
  - [ ] `apps/website/project.json` (Section 3.1)
  - [ ] `apps/panaversity-fs-py/project.json` (Section 3.2)
  - [ ] 6× `libs/*/project.json` (use template from Section 6)
  - [ ] `tools/executors/project.json` (Section 3.4)
  - [ ] `tools/scripts/project.json` (Section 3.5)

- [ ] **Step 2.2**: Create Python executor
  - [ ] Create `tools/executors/python-executor/schema.json` (Section 5.1)
  - [ ] Create `tools/executors/python-executor/impl.ts` (Section 5.2)
  - [ ] Create `tools/executors/python-executor/package.json` (Section 5.3)
  - [ ] Build executor: `nx build tools-executors`

- [ ] **Step 2.3**: Test individual targets
  - [ ] `nx build website` → produces dist/apps/website
  - [ ] `nx test website` → runs tests (if any)
  - [ ] `nx lint website` → runs linting
  - [ ] `nx build panaversity-fs-py` → runs uv build
  - [ ] `nx test panaversity-fs-py` → runs pytest

- [ ] **Step 2.4**: Test affected detection
  - [ ] Create feature branch
  - [ ] Modify `apps/website/docs/index.md`
  - [ ] Run `nx affected -t build` → should detect only website
  - [ ] Run `nx affected -t test` → should test only affected

### Phase 3: CI/CD (High Risk - Production Impact)

- [ ] **Step 3.1**: Update GitHub Actions
  - [ ] Update `deploy.yml` to use `nx affected -t build` + `nx build website`
  - [ ] Update script references: `panaversity-fs/scripts/hydrate-book.py` → `tools/scripts/hydrate-book.py`
  - [ ] Add Nx caching to workflow (optional, requires Nx Cloud)

- [ ] **Step 3.2**: Test in feature branch
  - [ ] Create branch `feat/nx-migration-phase-3`
  - [ ] Run workflow manually (workflow_dispatch)
  - [ ] Verify GitHub Pages deployment succeeds
  - [ ] Check all env vars are preserved

- [ ] **Step 3.3**: Production merge
  - [ ] Create PR with migration changes
  - [ ] Get approval from team lead
  - [ ] Merge to main on low-traffic time
  - [ ] Monitor deployment for 1 hour

- [ ] **Step 3.4**: Validation
  - [ ] Website loads: https://panaversity.ai
  - [ ] Book content is present
  - [ ] Python backend is running
  - [ ] No errors in browser console

### Phase 4: Optimization (Post-Migration)

- [ ] Connect to Nx Cloud: `npx nx connect`
- [ ] Enable remote caching in GitHub Actions
- [ ] Update documentation: update references from old paths to new Nx patterns
- [ ] Create ADR: ADR-0021 "Adopt Nx as Monorepo Orchestrator"

---

## 9. Key Nx Commands Reference

### Discovery & Visualization

```bash
# Show all projects
nx show projects

# Show project details
nx show project website --web

# Visualize dependency graph
nx graph

# Visualize focus on one project
nx graph --focus=website

# Show affected projects (since main)
nx show projects --affected
```

### Build & Test

```bash
# Build all projects
nx run-many -t build

# Build affected only
nx affected -t build

# Test all
nx run-many -t test

# Test affected
nx affected -t test

# Build specific project
nx build website
nx build panaversity-fs-py

# Run specific target on specific project
nx hydrate website
```

### Code Quality

```bash
# Lint all
nx run-many -t lint

# Format code
nx format:write

# Typecheck all
nx run-many -t typecheck
```

### Workspace Management

```bash
# Reset Nx cache
nx reset

# Print configuration
nx show project website

# Verify project structure
nx lint --maxWarnings 0
```

### CI/CD (GitHub Actions)

```bash
# In GitHub Actions workflow:
# - Use nx affected -t build to build only changed projects
# - Use nx affected -t test to test only changed projects
# - Cache management: automatic with Nx Cloud or manual with actions/cache

# Example workflow command:
nx affected -t build --base=origin/main --head=HEAD
```

---

## 10. Caching Strategy

### 10.1 Inputs & Outputs Definition

**website (Docusaurus)**:
- **Inputs**: TypeScript files, MDX, styles, plugins (→ `docusaurusInputs` namedInput)
- **Outputs**: `dist/apps/website/` (build), `.docusaurus/content` (hydrate)
- **Cache invalidation**: When any `docs/`, `src/`, or plugin changes

**panaversity-fs-py (Python)**:
- **Inputs**: `pyproject.toml`, `uv.lock`, all `.py` files (→ `pythonInputs` namedInput)
- **Outputs**: `dist/apps/panaversity-fs-py/` (build)
- **Cache invalidation**: When `pyproject.toml`, `uv.lock`, or `.py` changes

**Plugins**:
- **Inputs**: Plugin source files + `package.json`
- **Outputs**: `dist/libs/{plugin}/`
- **Cache invalidation**: Per-plugin, no cross-plugin cache sharing

### 10.2 Nx Cloud Integration (Phase 4)

When enabled:
```bash
npx nx connect
```

Benefits:
- **Remote caching**: CI builds cache results to Nx Cloud
- **Distributed caching**: Developers fetch cached artifacts instead of rebuilding
- **Computation sharing**: Pull requests reuse builds from main branch
- **Faster feedback**: Tests run only on affected code

---

## 11. Common Errors & Solutions

### Error: "Cannot find module '@panaversity/...'

**Cause**: Plugin resolution failure (incorrect tsconfig paths)

**Fix**:
```json
{
  "compilerOptions": {
    "baseUrl": ".",
    "paths": {
      "@panaversity/remark-*": ["libs/remark-*/index.js"],
      "@panaversity/docusaurus-*": ["libs/docusaurus-*/index.js"]
    }
  }
}
```

### Error: "Python command not found in nx test"

**Cause**: Custom executor not built

**Fix**:
```bash
nx build tools-executors
nx test panaversity-fs-py
```

### Error: "project.json not found" when running nx build

**Cause**: Directory not created or named incorrectly

**Fix**:
1. Verify directory exists: `ls apps/website/project.json`
2. Verify Nx finds it: `nx show projects | grep website`
3. If not found: `nx sync` to regenerate

### Error: CI workflow fails after migration

**Cause**: Script paths changed (e.g., `panaversity-fs/scripts/` → `tools/scripts/`)

**Fix**: Update `.github/workflows/*.yml` to reference new paths:
```yaml
- run: python tools/scripts/hydrate-book.py
```

---

## 12. Architecture Decisions

### ADR-0021: Adopt Nx as Monorepo Orchestrator

**Decision**: Use Nx 20+ with pnpm for polyglot monorepo

**Rationale**:
1. **Native MCP support**: Nx has official MCP server for AI-assisted operations
2. **Polyglot support**: Node.js native, Python via custom executors
3. **Production-safe**: `nx affected` enables CI optimizations without risk
4. **Cache-enabled**: Native support for inputs/outputs + Nx Cloud
5. **Zero-downtime migration**: Can migrate incrementally (3 phases)

**Alternatives considered**:
- **Bazel**: 3-6 month learning curve, overkill for current complexity
- **Turborepo**: Node.js only, doesn't support Python well
- **Lerna**: Legacy, doesn't support polyglot well
- **Manual CI**: Would require rewriting workflows, error-prone

**Risks mitigated**:
- **CI breaks production**: Test on feature branch first (Phase 3)
- **Python isolation**: Custom executor maintains separation
- **Plugin resolution**: tsconfig paths + Nx project references
- **Package manager confusion**: Standardize on pnpm

---

## 13. File Checklist: Create These Files

Before migration starts, create exactly these files:

1. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/nx.json` (Section 2.1)
2. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/pnpm-workspace.yaml` (Section 2.2)
3. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/.nxignore` (Section 2.3)
4. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/package.json` (Section 4 - update root)
5. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/website/project.json` (Section 3.1)
6. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/panaversity-fs-py/project.json` (Section 3.2)
7. `libs/*/project.json` (6 plugins, use template Section 6)
8. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/tools/executors/project.json` (Section 3.4)
9. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/tools/scripts/project.json` (Section 3.5)
10. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/tools/executors/python-executor/schema.json` (Section 5.1)
11. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/tools/executors/python-executor/impl.ts` (Section 5.2)
12. `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/tools/executors/python-executor/package.json` (Section 5.3)

**Total**: 5 configuration files + 7 project.json files + 3 Python executor files = **15 files**

---

## Conclusion

This Nx workspace design provides:

✅ **Complete project inventory**: 8 projects (2 apps + 6 libs)
✅ **Production-safe**: Incremental 3-phase migration strategy
✅ **Polyglot support**: Node.js + Python with clean isolation
✅ **CI/CD optimized**: `nx affected` enables sub-second build decisions
✅ **Cache-enabled**: All targets support inputs/outputs for Nx Cloud
✅ **Zero-downtime**: Can test thoroughly before production merge

**Next step**: Start Phase 1 (foundation) on feature branch `feat/nx-migration`
