# Codebase Curation Guide for Chapter 10

**Purpose**: Select appropriate open-source codebases for lessons 1-8 and capstone project.

**Target Audience**: Instructors, content creators, students self-selecting practice repositories.

---

## Selection Criteria

### Universal Requirements (All Codebases)

**1. Open Source License**
- Must be publicly accessible (GitHub, GitLab, etc.)
- License allows educational analysis (MIT, Apache 2.0, BSD, GPL)
- No proprietary or NDA-restricted code

**2. Active or Recently Maintained**
- Last commit within 2 years (not abandoned)
- Demonstrates modern practices (not legacy patterns)
- Dependency versions reasonably current

**3. Clean Documentation**
- README.md exists and explains project purpose
- Code structure is inferrable (module organization visible)
- No obfuscated or minified code

**4. Educational Value**
- Representative of real-world professional code
- Demonstrates architectural patterns (not toy examples)
- Contains variety for analysis (auth, database, APIs, etc.)

**5. Analysis-Friendly**
- Moderate complexity (not trivial, not overwhelming)
- Multiple modules/components (not single-file scripts)
- Accessible to non-experts in that language/framework

---

## Complexity Tiers

### Small Codebases (5-15 files, &lt;2K LOC)
**Use for**: Lessons 1-3 (foundation, early practice)

**Characteristics**:
- Single module or microservice
- Clear entry point (one main file)
- 2-3 external dependencies max
- Can be understood in 30-45 minutes

**Example Scenarios**:
- Simple REST API (3-4 endpoints)
- CLI tool with subcommands
- Data processing script with pipeline stages
- Authentication middleware module

**Example Repositories** (as of 2025-01-18):
- **Flask minimal API**: Simple REST API with auth (Python, Flask, SQLite)
  - URL: github.com/pallets/flask/tree/main/examples/tutorial
  - Files: ~10, LOC: ~1.5K
  - Good for: Understanding API structure, database basics

- **Todo CLI (Go)**: Command-line task manager
  - URL: github.com/spf13/cobra (examples directory)
  - Files: ~8, LOC: ~1K
  - Good for: CLI architecture, file I/O patterns

### Medium Codebases (20-40 files, 2K-10K LOC)
**Use for**: Lessons 4-7 (platform tools, skills creation)

**Characteristics**:
- Multiple modules with clear separation
- 5-10 external dependencies
- Architecture patterns visible (MVC, layered, etc.)
- Can be understood in 1-2 hours

**Example Scenarios**:
- Full-stack web app (backend API + basic frontend)
- Multi-endpoint REST API with database
- E-commerce cart module
- Authentication/authorization system

**Example Repositories** (as of 2025-01-18):
- **RealWorld API (FastAPI)**: Conduit blogging platform backend
  - URL: github.com/nsidnev/fastapi-realworld-example-app
  - Files: ~35, LOC: ~5K
  - Good for: Modern Python patterns, API design, database ORM

- **E-commerce Cart (Node.js)**: Shopping cart microservice
  - URL: github.com/developit/cart (or similar express-based cart)
  - Files: ~25, LOC: ~3K
  - Good for: State management, session handling, integration patterns

### Large Codebases (50+ files, 10K+ LOC)
**Use for**: Lesson 8 (capstone project)

**Characteristics**:
- Production-scale application
- 10+ external dependencies
- Multiple modules with inter-dependencies
- Requires 2-3 hours for comprehensive analysis

**Example Scenarios**:
- Full e-commerce platform
- SaaS application backend
- Open-source CMS or blog platform
- API gateway with multiple services

**Example Repositories** (as of 2025-01-18):
- **Django Oscar**: E-commerce framework
  - URL: github.com/django-oscar/django-oscar
  - Files: ~200+, LOC: ~50K
  - Good for: Production architecture, security patterns, scalability

- **Hasura GraphQL Engine**: GraphQL API layer
  - URL: github.com/hasura/graphql-engine (server directory)
  - Files: ~100+, LOC: ~30K
  - Good for: API design, authentication, database patterns

**IMPORTANT**: For capstone, students should analyze SUBSET (one module, not entire codebase). Example: "Analyze Django Oscar's checkout module" not "Analyze all of Django Oscar."

---

## Language/Framework Distribution

### Recommended Mix (Per Chapter Deployment)

**Primary Focus** (60% of examples):
- **Python**: Django, FastAPI, Flask (most accessible to B1 tier students)
- **JavaScript/TypeScript**: Express, NestJS (second most common)

**Secondary Coverage** (30% of examples):
- **Go**: Gin, Echo (growing in popularity, good architectural patterns)
- **Ruby**: Rails (classic web framework, good for architecture analysis)

**Tertiary Exposure** (10% of examples):
- **Rust**: Actix, Rocket (modern, security-focused)
- **Java/Kotlin**: Spring Boot (enterprise patterns)

**Rationale**: Students don't need to know these languages to analyze them (that's the point of AI-assisted analysis). Variety demonstrates platform-agnostic skills.

---

## Curation Workflow

### Step 1: Repository Discovery
**Sources**:
- GitHub Awesome Lists (awesome-python, awesome-nodejs, etc.)
- "RealWorld" implementations (realworld.io — same app in multiple frameworks)
- Production open-source projects (Stripe SDKs, Shopify themes, etc.)

**Search Queries**:
- `"fastapi example" stars:>50 pushed:>2023-01-01`
- `"express api tutorial" language:javascript stars:>100`
- `topic:authentication language:python`

### Step 2: Initial Screening (5 minutes per repo)
**Check**:
- [ ] README explains purpose clearly
- [ ] License is open (MIT, Apache, BSD, GPL)
- [ ] Last commit within 2 years
- [ ] File count appropriate for tier (small/medium/large)
- [ ] No red flags (malware, abandoned, overly complex)

### Step 3: Deep Evaluation (15 minutes per repo)
**Analyze Structure**:
- [ ] Entry point identifiable (`main.py`, `index.js`, etc.)
- [ ] Modules logically organized (not dumped in root)
- [ ] Dependencies reasonable (not 100+ packages for small app)
- [ ] Code quality decent (not code golf, not obfuscated)

**Test Analysis Viability**:
- [ ] Can you answer "What does this codebase do?" in 2 sentences?
- [ ] Can you identify 2-3 key modules/components?
- [ ] Can you spot 1-2 obvious analysis questions (security, architecture, etc.)?

### Step 4: Document for Students
**Create Codebase Brief** (template below):
```markdown
## [Codebase Name]

**URL**: [GitHub link]
**Language**: [Python / JavaScript / etc.]
**Framework**: [Django / Express / etc.]
**Size**: [Small / Medium / Large]
**Files**: [Approximate count]
**LOC**: [Approximate lines of code]

**Purpose**: [1-2 sentence description of what this code does]

**Why This Codebase**:
[1-2 sentences explaining educational value. Example: "Demonstrates modern FastAPI patterns with database ORM, useful for API architecture analysis"]

**Recommended Use**: [Lesson X exercise / Capstone project]

**Key Modules to Analyze**:
1. [Module/Directory 1]: [Purpose]
2. [Module/Directory 2]: [Purpose]
3. [Module/Directory 3]: [Purpose]

**Suggested Analysis Questions**:
- [Example: "Is the authentication implementation secure?"]
- [Example: "How is the database schema organized?"]
- [Example: "What external APIs does this integrate with?"]
```

---

## Pre-Curated Repository List (Starting Point)

### Small Codebases (Lessons 1-3)

#### 1. Flask Tutorial Example
- **URL**: github.com/pallets/flask/tree/main/examples/tutorial
- **Size**: Small (10 files, ~1.5K LOC)
- **Purpose**: Simple blog application with auth and database
- **Analysis Focus**: Entry points, database schema, authentication patterns

#### 2. Todo API (FastAPI)
- **URL**: github.com/tiangolo/full-stack-fastapi-template (api subdirectory)
- **Size**: Small (12 files, ~2K LOC)
- **Purpose**: CRUD REST API for task management
- **Analysis Focus**: API design, CRUD operations, validation

### Medium Codebases (Lessons 4-7)

#### 3. RealWorld FastAPI
- **URL**: github.com/nsidnev/fastapi-realworld-example-app
- **Size**: Medium (35 files, ~5K LOC)
- **Purpose**: Conduit blogging platform backend (Medium clone)
- **Analysis Focus**: Layered architecture, JWT auth, database relationships

#### 4. Stripe Python SDK (subset)
- **URL**: github.com/stripe/stripe-python (stripe/api_resources/)
- **Size**: Medium (40 files, ~8K LOC)
- **Purpose**: Python client for Stripe payment API
- **Analysis Focus**: SDK architecture, API abstraction, error handling

### Large Codebases (Lesson 8 Capstone)

#### 5. Saleor (E-commerce Platform)
- **URL**: github.com/saleor/saleor (saleor/checkout/ module only)
- **Size**: Large subset (60 files, ~15K LOC for checkout module)
- **Purpose**: Headless e-commerce GraphQL API
- **Analysis Focus**: Payment processing, cart logic, security patterns
- **Note**: Students analyze CHECKOUT MODULE only, not entire platform

#### 6. PostHog (Product Analytics)
- **URL**: github.com/PostHog/posthog (posthog/api/ directory)
- **Size**: Large subset (80 files, ~20K LOC for API layer)
- **Purpose**: Self-hosted product analytics platform
- **Analysis Focus**: API architecture, data ingestion, authentication
- **Note**: Students analyze API LAYER only, not frontend or infrastructure

---

## Anti-Patterns to Avoid

### ❌ DO NOT SELECT:

**Toy Examples**:
- Todo apps with &lt;100 LOC
- "Hello World" tutorials
- Code golf submissions
- Academic exercises with no real-world applicability

**Overly Complex**:
- Entire operating systems (Linux kernel)
- Full-featured IDEs (VSCode)
- Game engines (Unity, Unreal)
- Enterprise ERPs (Odoo full codebase)

**Poorly Maintained**:
- Last commit >3 years ago
- Dependency versions from 2015
- Broken README links
- Code that doesn't run without significant setup

**Obfuscated or Unusual**:
- Minified JavaScript
- Code generators output
- Binary blobs checked into repo
- Languages too niche (Brainfuck, Whitespace, etc.)

**Legal/Ethical Issues**:
- Proprietary code leaked online
- Code violating terms of service
- Malware or exploit code
- Scraped/stolen repositories

---

## Codebase Diversity Checklist

**Per chapter deployment, ensure variety across**:

- [ ] **Languages**: Min 2 (Python + JavaScript), ideally 3-4
- [ ] **Frameworks**: Min 3 different frameworks
- [ ] **Domains**: Mix of (e-commerce, auth, APIs, data processing, tooling)
- [ ] **Architecture Styles**: Monoliths, microservices, layered, MVC
- [ ] **Database Types**: SQL (PostgreSQL), NoSQL (MongoDB), in-memory (Redis)
- [ ] **Complexity Tiers**: 2-3 small, 2-3 medium, 1-2 large

**Rationale**: Students learn AI-assisted analysis is language/framework-agnostic. Variety prevents overfitting to one tech stack.

---

## Updating Repository List

**Maintenance Schedule**: Review every 6 months (repositories age, dependencies deprecate)

**Update Triggers**:
- Repository archived or deleted
- Major security vulnerability discovered
- Framework deprecated (e.g., Flask 0.x EOL)
- Better alternative found

**Documentation**:
- Keep changelog of codebase replacements
- Document why old repo was removed
- Provide migration path for students mid-chapter

---

## Sample Usage in Lessons

### Lesson 1 Exercise (Small Codebase)
**Codebase**: Flask Tutorial Example
**Task**: "Identify the entry point (where does execution start?). List 3 key modules. What database is used?"
**Time**: 15 minutes
**Tools**: Manual exploration (no AI yet, Stage 1)

### Lesson 3 Exercise (Small Codebase)
**Codebase**: Todo API (FastAPI)
**Task**: "Using Claude Code or Gemini CLI, analyze the API design. How many endpoints? What authentication method? Provide 4-layer context."
**Time**: 30 minutes
**Tools**: Claude Code Read/Glob OR Gemini CLI @filename

### Lesson 5 Exercise (Medium Codebase)
**Codebase**: RealWorld FastAPI
**Task**: "Create custom Gemini CLI command that searches for all database models and lists relationships. Analyze data schema."
**Time**: 45 minutes
**Tools**: Gemini CLI custom TOML commands

### Lesson 8 Capstone (Large Codebase Subset)
**Codebase**: Saleor Checkout Module
**Task**: "Produce 2-page technical assessment report. Architecture diagram, security findings, technical debt score, recommendation."
**Time**: 2-3 hours
**Tools**: Claude Code OR Gemini CLI (student choice)

---

## Instructor Notes

### Providing Codebases to Students

**Option 1: Direct Links** (Simplest)
- Provide GitHub URLs
- Students clone repositories locally
- Risk: Repositories can be deleted/archived

**Option 2: Frozen Snapshots** (Most Reliable)
- Fork repositories to course organization account
- Tag specific commit (e.g., `v1.0-chapter10`)
- Students clone from frozen snapshot
- Advantage: Consistency across cohorts

**Option 3: Bundled Examples** (Fully Controlled)
- Include sample codebases in course materials repo
- Students download zip files
- Advantage: Guaranteed availability
- Disadvantage: Artificial (not real open-source exploration)

**Recommendation**: **Option 2** (Frozen snapshots) for consistent student experience.

---

## Self-Curation Guide for Students

**If selecting your own codebase for capstone**:

1. **Choose domain you're interested in**: E-commerce, fintech, social media, productivity, etc.
2. **Search GitHub**: `topic:[your-domain] language:[python/javascript] stars:>100 pushed:>2023-01-01`
3. **Apply Selection Criteria** (above): Open license, active, documented, appropriate size
4. **Test with 15-minute exploration**: Can you identify purpose, entry point, key modules?
5. **Get instructor approval** (if required): Share brief (URL, purpose, why chosen)

**Quality Check**: "Can I produce a 2-page technical assessment on this in 2-3 hours?" If yes → Good choice.

---

**Guide Version**: 1.0.0
**Last Updated**: 2025-01-18
**Source**: Chapter 10, Infrastructure Planning
**Constitution**: v6.0.0 Compliance

**Usage**: Instructors use this to select/update repository list. Students use "Self-Curation Guide" if choosing own capstone codebase.
