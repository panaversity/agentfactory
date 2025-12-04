### Core Concept
Hash functions map objects to integers, enabling O(1) lookup instead of O(n) scanning. Sets store elements in hash table slots, making membership testing instant even with millions of items—a dramatic performance advantage at scale.

### Key Mental Models
- **Hash Function**: Converts any object to an integer consistently; same object always produces same hash
- **Hash Table Slots**: Stores elements indexed by hash value; lookup jumps directly to slot without scanning
- **O(1) Average Case**: Set operations take constant time; lists require linear scanning
- **Immutability Requirement**: Hash values must never change; only immutable objects can be hashed

### Critical Patterns
- **Performance Scaling**: Set lookup stays fast as size grows; list lookup slows down proportionally
- **Collision Handling**: When hashes collide (map to same slot), Python chains or probes for alternatives
- **Rehashing**: Python automatically resizes hash table to keep collision rate low
- **Design Decision**: For frequent lookups, choose sets; for order-dependent operations, choose lists

### AI Collaboration Keys
- AI creates timing benchmarks showing 100x+ performance difference at scale
- AI walks through why immutability is required for hash stability

### Common Mistakes
- Assuming all hash values are unique (collisions happen and are handled)
- Not recognizing O(1) vs O(n) difference becomes critical with large datasets
- Trying to use mutable objects in sets when immutability is the constraint

### Connections
- **Builds on**: Set creation and uniqueness property (Lesson 1)
- **Leads to**: Frozensets (need hashability for advanced uses like dict keys in Lesson 4)
