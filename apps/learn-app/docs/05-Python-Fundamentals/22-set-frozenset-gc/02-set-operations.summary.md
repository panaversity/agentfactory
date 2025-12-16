### Core Concept
Set operations (union, intersection, difference, symmetric difference) solve data comparison problems mathematically. These operations answer distinct questions: "What items appear anywhere?" (union), "What's common?" (intersection), "What's unique to one?" (difference).

### Key Mental Models
- **Union (|)**: Combines sets, keeping all unique items; answers "items in any set"
- **Intersection (&)**: Finds common elements across sets; answers "items in both sets"
- **Difference (-)**: Subtracts one set from another; answers "items in first but not second" (non-commutative)
- **Symmetric Difference (^)**: Finds items in either set but not both; answers "what distinguishes these sets"

### Critical Patterns
- **Commutativity**: Union, intersection, symmetric difference are commutative; difference is not
- **Chaining Operations**: Combine operators to answer complex questions (`(a | b) - c`)
- **Set Comprehensions**: Filter/transform sets inline using `{x for x in source if condition}`
- **Real-World Problems**: Friend matching (intersection), inventory differences (symmetric difference)

### AI Collaboration Keys
- AI visualizes Venn diagrams showing how operations overlap
- AI connects mathematical set theory to business problems (customer segmentation)

### Common Mistakes
- Forgetting difference is non-commutative; `a - b` differs from `b - a`
- Using wrong operation for the question (intersection when union needed)
- Not seeing set comprehensions reduce both filtering and duplicates simultaneously

### Connections
- **Builds on**: Set creation and properties (Lesson 1)
- **Leads to**: Hash table internals explaining why operations are fast (Lesson 3)
