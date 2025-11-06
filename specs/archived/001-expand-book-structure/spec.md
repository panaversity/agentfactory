# Feature Specification: Expand Book Structure (7 Parts → 13 Parts, 32 Chapters → 46 Chapters)

**Feature Branch**: `001-expand-book-structure`  
**Created**: 2025-10-29  
**Status**: Draft  
**Input**: User description: "Expand book structure from 7 parts (32 chapters) to 13 parts (46 chapters). Book: 'Colearning Programming & Agentic AI with Python and TypeScript: The AI-Driven Way'"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Update Constitution and Core Documentation (Priority: P1)

The project constitution, chapter index, and core documentation must reflect the new 13-part, 46-chapter structure to serve as the authoritative source of truth for all content creation work.

**Why this priority**: Without updated constitution and chapter index, no content work can proceed correctly. This is the foundation for all subsequent work.

**Independent Test**: Constitution, chapter-index, and directory structure documentation accurately reflect 13 parts and 46 chapters. AI agents can reference these documents to understand the complete book structure.

**Acceptance Scenarios**:

1. **Given** the current constitution references 7 parts and 32 chapters, **When** the constitution is updated, **Then** it accurately describes 13 parts and 46 chapters with correct part assignments
2. **Given** the chapter-index.md exists, **When** it is updated, **Then** it contains all 46 chapters organized into 13 parts with correct numbering
3. **Given** AI agents need structure information, **When** they reference the constitution or chapter-index, **Then** they receive accurate information about all 13 parts and 46 chapters

---

### User Story 2 - Restructure Existing Content to Match New Part Organization (Priority: P2)

Existing content from Parts 1-5 must be reorganized to match the new structure (Part 1: 5→3 chapters, Part 3: 4→2 chapters, Part 4: 8→12 chapters, Part 5: 5→3 chapters).

**Why this priority**: Preserves existing work while adapting it to the new structure. Must happen before new parts can be added.

**Independent Test**: All existing chapters are correctly mapped to their new part assignments and chapter numbers. No content is lost or duplicated.

**Acceptance Scenarios**:

1. **Given** Part 1 currently has 5 chapters, **When** restructured to 3 chapters, **Then** content is consolidated appropriately without losing essential material
2. **Given** Part 3 currently has 4 chapters, **When** restructured to 2 chapters, **Then** content is consolidated logically
3. **Given** Part 4 currently has 8 chapters, **When** expanded to 12 chapters, **Then** new chapters are identified and scaffolding is created
4. **Given** Part 5 currently has 5 chapters, **When** restructured to 3 chapters, **Then** content is reorganized appropriately

---

### User Story 3 - Create Scaffolding for New Parts 6-13 (Priority: P3)

Eight new parts (Parts 6-13) covering TypeScript, Agentic AI, MCP, Realtime Agents, Docker/Kubernetes, Databases, Kafka/Dapr, and Stateful Agents must be scaffolded with part specs and chapter specs.

**Why this priority**: Establishes the structure for advanced content. Can be done after core restructuring is complete.

**Independent Test**: Each of the 8 new parts has a complete part spec, part intro file, and chapter spec files for all chapters within that part.

**Acceptance Scenarios**:

1. **Given** the book needs Part 6 (Agentic AI Fundamentals), **When** scaffolding is created, **Then** part-6-spec.md, intro.md, and 3 chapter specs exist
2. **Given** the book needs Part 7 (MCP Fundamentals), **When** scaffolding is created, **Then** part-7-spec.md, intro.md, and 3 chapter specs exist
3. **Given** the book needs Parts 8-13, **When** scaffolding is created, **Then** each part has complete spec structure matching the chapter count (3, 3, 3, 3, 2, 2 respectively)
4. **Given** all 8 new parts are scaffolded, **When** AI agents query the structure, **Then** they can generate content for any chapter in Parts 6-13

---

### User Story 4 - Update Docusaurus Directory Structure (Priority: P2)

The physical directory structure under `book-source/docs/` must be reorganized to match the new 13-part, 46-chapter structure while maintaining Docusaurus compatibility.

**Why this priority**: Content creation requires correct physical file organization. Must be aligned with spec restructuring.

**Independent Test**: Docusaurus successfully builds the site with all 13 parts visible in the sidebar, all 46 chapters navigable, and no broken links.

**Acceptance Scenarios**:

1. **Given** the current directory structure has 7 part folders, **When** updated to 13 parts, **Then** all part folders follow naming convention (e.g., `08-TypeScript-The-Language-of-Realtime/`)
2. **Given** each part needs chapter folders, **When** restructured, **Then** all 46 chapters have correctly numbered folders
3. **Given** Docusaurus needs to build the site, **When** `npm run build` is executed, **Then** the build succeeds with no errors
4. **Given** users navigate the site, **When** they access the sidebar, **Then** all 13 parts and 46 chapters are correctly ordered and navigable

---

### User Story 5 - Update README and CLAUDE.md with New Structure (Priority: P3)

Project documentation (README.md, CLAUDE.md) must reference the new 13-part structure so contributors and AI agents understand the expanded scope.

**Why this priority**: Provides clear communication about the book's expanded scope to all stakeholders.

**Independent Test**: README and CLAUDE.md accurately describe the 13-part structure and can be referenced by contributors to understand the book's organization.

**Acceptance Scenarios**:

1. **Given** README.md describes the book, **When** updated, **Then** it accurately lists all 13 parts with chapter counts
2. **Given** CLAUDE.md provides AI agent context, **When** updated, **Then** it references the constitution and chapter-index for structure details
3. **Given** a new contributor reads the documentation, **When** they want to understand the book structure, **Then** they can clearly see the 13-part organization and progression

---

### Edge Cases

- What happens when existing chapter content doesn't fit cleanly into the new structure (e.g., Part 1: 5→3 chapters)?
  - **Answer**: Content must be consolidated intelligently, preserving essential material. A mapping document should track where old content moved.
  
- How does the system handle chapter renumbering cascades (e.g., what was Chapter 6 might become Chapter 4)?
  - **Answer**: The chapter-index.md serves as the authoritative mapping. Old chapter files should be moved/renamed to match new numbering with clear commit messages tracking the changes.
  
- What happens if Docusaurus sidebar generation breaks due to structure changes?
  - **Answer**: The `_category_.json` files in each folder must be updated to reflect correct sidebar positions and labels.
  
- How do we handle cross-references between chapters when chapter numbers change?
  - **Answer**: All internal links must be updated as part of the restructuring. A validation script should check for broken internal references.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Constitution MUST be updated to accurately reflect 13 parts and 46 chapters with correct scaffolding ranges and part assignments
- **FR-002**: Chapter-index.md MUST list all 46 chapters organized into 13 parts with sequential global chapter numbering (1-46)
- **FR-003**: Directory-structure.md MUST be updated to provide examples and validation rules for the expanded structure
- **FR-004**: Part specs MUST be created for all 8 new parts (Parts 6-13) following the existing part-spec template
- **FR-005**: Each new part MUST have chapter specs for all chapters within that part (ranging from 2-12 chapters per part)
- **FR-006**: Existing part specs (Parts 1-5) MUST be updated to reflect new chapter counts and content organization
- **FR-007**: Physical directory structure under `book-source/docs/` MUST be reorganized to match the new 13-part structure
- **FR-008**: All part folders MUST follow naming convention: `NN-Title-Case-With-Hyphens/` (e.g., `06-Agentic-AI-Fundamentals/`)
- **FR-009**: Each part folder MUST contain an `intro.md` file describing the part's purpose and learning outcomes
- **FR-010**: Chapter folders MUST follow naming convention: `NN-lowercase-with-hyphens/` within their part folder
- **FR-011**: README.md MUST be updated to reference the new 13-part structure
- **FR-012**: CLAUDE.md MUST reference the constitution and chapter-index for structure information (not hardcode it)
- **FR-013**: Docusaurus build process MUST successfully generate a site with all 13 parts and 46 chapters navigable
- **FR-014**: All internal cross-references MUST be validated and updated to reflect new chapter numbering
- **FR-015**: A mapping document MUST track how existing chapters were reorganized (old number → new number)

### Key Entities

- **Part**: A major section of the book covering a broad topic area (e.g., "Python: The Language of AI Agents"). Contains 2-12 chapters. Has a part spec (`part-N-spec.md`) and intro file (`intro.md`).

- **Chapter**: A focused unit within a part covering specific learning objectives. Has a chapter spec (`chapter-NN-spec.md`), chapter plan (`chapter-NN-plan.md`), and physical directory with README and lesson files.

- **Chapter Index**: The authoritative mapping of all 46 chapters to their parts, maintaining global sequential numbering (1-46) and showing logical organization.

- **Part Spec**: A detailed specification document for a part that defines the part's purpose, learning outcomes, chapter breakdown, and pedagogical approach.

- **Directory Structure**: The physical organization of markdown files under `book-source/docs/` following Docusaurus conventions and the naming rules defined in `specs/book/directory-structure.md`.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All documentation (constitution, chapter-index, directory-structure, README, CLAUDE.md) accurately reflects 13 parts and 46 chapters with no inconsistencies
- **SC-002**: Docusaurus build completes successfully (`npm run build`) with zero errors and all 13 parts visible in the generated sidebar
- **SC-003**: All 8 new parts (Parts 6-13) have complete scaffolding (part spec, intro file, chapter specs) following existing templates
- **SC-004**: Content creators can reference the chapter-index and immediately understand which chapters belong to which parts and what the global chapter number is
- **SC-005**: All existing chapters from Parts 1-5 are successfully mapped to their new positions with no content loss
- **SC-006**: 100% of internal cross-references are validated and updated to reflect the new chapter numbering
- **SC-007**: A contributor can navigate from Part 1 Chapter 1 to Part 13 Chapter 46 via the Docusaurus sidebar without encountering broken links or missing pages
