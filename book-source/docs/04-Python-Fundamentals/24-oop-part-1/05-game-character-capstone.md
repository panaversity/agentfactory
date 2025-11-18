---
title: "Game Character System (Capstone)"
chapter: 24
lesson: 5
duration_minutes: 70

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Multi-Class System Design"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design interactions between 4+ classes with clear responsibilities and data flow"

  - name: "OOP Pattern Integration"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can combine encapsulation, properties, methods, and ABC into cohesive system"

  - name: "Class Relationship Design"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can model object relationships (composition, association) in class design"

  - name: "Game Mechanics Modeling"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can translate real-world requirements into class structure and method logic"

  - name: "Code Organization for Scale"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can organize multi-class project with clear module structure"

  - name: "Project Planning with AI"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Create"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can describe project architecture to AI and refine design collaboratively"

  - name: "Synthesis and Integration"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student integrates all Chapter 24 concepts (classes, constructors, encapsulation, methods, ABC) into working system"

learning_objectives:
  - objective: "Design a multi-class object system that models a real-world scenario (turn-based game) using proper OOP principles"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Game Character System implementation with 4+ interacting classes"

  - objective: "Integrate encapsulation, properties, constructors, and all method types into a cohesive, working system"
    proficiency_level: "B2"
    bloom_level: "Synthesize"
    assessment_method: "Implementation demonstrates all Chapter 24 concepts in context"

  - objective: "Apply abstract base classes to enforce contracts across multiple character types"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Character base class with abstract methods implemented in Player and Enemy subclasses"

  - objective: "Collaborate with AI to design, implement, and extend a real-world system"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Use AI for architecture planning and implementation refinement"

cognitive_load:
  new_concepts: 0
  assessment: "Integration lesson - no new concepts, only synthesis of Chapter 24 (classes, constructors, attributes, properties, methods, ABC). All concepts taught in Lessons 1-4 ✓"

differentiation:
  extension_for_advanced: "Add Boss class with special abilities, Shop system for buying/selling, save/load game state with JSON, networking for multiplayer turn-based combat. Refactor using composition vs inheritance patterns from Chapter 25."
  remedial_for_struggling: "Build Character class completely before Player/Enemy. Test each class independently before integration. Focus on one feature at a time (health system → inventory → combat) rather than implementing everything at once."

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-24.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Game Character System (Capstone)

This capstone lesson integrates everything from Lessons 1-4 into a real system. You'll move from learning individual OOP concepts to designing and building a complete multi-class game system using the Four-Layer approach: (1) Specification discovery through AI, (2) Hands-on implementation with feedback, (3) Peer review with AI, (4) Extension and optimization.

---

## Part 1: Student Discovers Requirements Through AI Collaboration

**Your Role**: System architect working with AI to clarify requirements

### Collaborative Specification Exercise

**Stage 1: Ask AI to Describe the System**

Ask your AI companion:

> "I need to build a Game Character System for a turn-based RPG. I want:
> - A base Character class (Player and Enemy inherit from this)
> - Characters have: name, health, max_health, attack_power, defense
> - Characters can: attack(), take_damage(), heal()
> - Player can have an inventory (list of items)
> - Enemy has an AI strategy (random or aggressive)
>
> Before I code, help me think through the design:
> 1. What attributes should be protected/private vs public?
> 2. Should attack() be different for Player vs Enemy?
> 3. How would you structure the class hierarchy?
> 4. What data validation is critical?"

**Stage 2: Clarify Through Questions**

Based on AI's response, ask:

> "For my game design, I need to clarify:
> 1. Should healing restore HP instantly or trigger a cooldown?
> 2. Can attack_power change during battle?
> 3. What happens if health goes below 0? (instant death or states like 'knocked down'?)
> 4. Should Enemy class be able to do anything Player can't?"

**Your task 1**: Document AI's responses in `system_design_specification.md`:
- Base class design (what goes in Character vs subclasses?)
- Attribute access levels (public/protected/private)
- Method type selection (instance/class/static)
- Game logic rules

### Your Requirements Document

Create `game_system_requirements.md` with:

```markdown
# Game Character System Requirements

## Classes Needed

### Character (Base Class)
**Attributes**:
- name: str
- _health: float (protected)
- _max_health: float (protected)
- _attack_power: float (protected)
- _defense: float (protected)

**Methods**:
- attack(target: Character) -> float: Deal damage based on attack_power and target defense
- take_damage(amount: float): Reduce health, prevent going below 0
- heal(amount: float): Restore health, cap at max_health
- is_alive() -> bool: Check if health > 0

### Player(Character)
**Additional Attributes**:
- level: int
- experience: int
- inventory: list

**Additional Methods**:
- gain_experience(amount: int)
- use_item(item: str)
- level_up()

### Enemy(Character)
**Additional Attributes**:
- enemy_type: str
- ai_strategy: str ('random' or 'aggressive')

**Additional Methods**:
- choose_action() -> str: Decide attack or defend based on AI
```

**Deliverable**: Specification document that you'll use as the blueprint for implementation.

---

## Part 2: Student Implements the System Hands-On

**Your Role**: Developer building from specification

### Implementation Exercise

Using your specification, build the complete system:

```python
from abc import ABC, abstractmethod

class Character(ABC):
    """Base class for all game characters"""

    def __init__(self, name: str, max_health: float, attack_power: float, defense: float):
        self.name = name
        self._max_health = max_health
        self._health = max_health
        self._attack_power = attack_power
        self._defense = defense

    def take_damage(self, amount: float) -> float:
        """Reduce health by damage minus defense"""
        actual_damage = max(0, amount - self._defense)
        self._health = max(0, self._health - actual_damage)
        return actual_damage

    def attack(self, target: 'Character') -> float:
        """Attack target and return damage dealt"""
        if not self.is_alive():
            return 0
        damage = max(0, self._attack_power + random.randint(-2, 2) - target._defense)
        target.take_damage(damage)
        return damage

    def heal(self, amount: float) -> float:
        """Restore health, return amount actually healed"""
        old_health = self._health
        self._health = min(self._max_health, self._health + amount)
        return self._health - old_health

    def is_alive(self) -> bool:
        """Check if character is alive"""
        return self._health > 0

    def get_health(self) -> float:
        """Safe accessor for health"""
        return self._health


class Player(Character):
    """Player-controlled character"""

    def __init__(self, name: str, max_health: float = 100, attack_power: float = 15, defense: float = 5):
        super().__init__(name, max_health, attack_power, defense)
        self.level = 1
        self.experience = 0
        self.inventory = []

    def gain_experience(self, amount: int):
        """Gain experience and level up if threshold reached"""
        self.experience += amount
        if self.experience >= 100:
            self.level_up()

    def level_up(self):
        """Increase stats and reset experience"""
        self.level += 1
        self._max_health += 20
        self._attack_power += 5
        self._defense += 2
        self._health = self._max_health
        self.experience = 0


class Enemy(Character):
    """AI-controlled enemy character"""

    def __init__(self, name: str, enemy_type: str, max_health: float = 50,
                 attack_power: float = 10, defense: float = 2, ai_strategy: str = "aggressive"):
        super().__init__(name, max_health, attack_power, defense)
        self.enemy_type = enemy_type
        self.ai_strategy = ai_strategy

    def choose_action(self, player: Player) -> str:
        """Decide action based on AI strategy"""
        if not self.is_alive():
            return "dead"

        if self.ai_strategy == "aggressive":
            return "attack"
        elif self.ai_strategy == "cautious":
            if self._health < self._max_health * 0.3:
                return "defend"
            return "attack"
        else:  # random
            return "attack" if random.random() > 0.5 else "defend"
```

**Your task 2**: Write and test the implementation:
- Create Player and Enemy objects
- Simulate a battle (3-5 turns)
- Test edge cases (healing overcap, damage to dead characters)
- Document what works and what needs refinement

### Your Implementation

Create `game_character_system.py` with:
- Complete Character, Player, Enemy classes
- Test code demonstrating a battle
- Comments explaining your design decisions

**Deliverable**: Fully functional game system matching your specification.

---

## Part 3: AI Reviews Your Implementation

**Your Role**: Developer receiving code review from AI

### Code Review Exercise

Paste your code and ask:

> "Review my Game Character System for design quality. Check:
> 1. Are my access levels (public/protected/private) appropriate?
> 2. Did I use inheritance effectively or is there duplication?
> 3. Are my method types (instance/class/static) correct?
> 4. What edge cases did I miss?
> 5. What would a professional production system add that I'm missing?"

### Convergence Activity

Based on AI's feedback, ask:

> "For the three things I should fix immediately, show me the refactored code and explain why each change improves the design."

**Deliverable**: List of improvements and refactored code sections addressing AI's feedback.

---

## Part 4: Student Extends the System

**Your Role**: Designer adding features based on deeper understanding

### Extension Exercise

Choose one extension and implement it:

**Option A: Combat System**
```python
# Add turn-based combat simulation
class Battle:
    def __init__(self, player: Player, enemy: Enemy):
        self.player = player
        self.enemy = enemy
        self.turn = 0

    def simulate_turn(self):
        """Execute one combat round"""
        # Player acts
        # Enemy decides action
        # Apply effects
        # Check end conditions
        pass

    def run_battle(self) -> str:
        """Simulate until one character dies, return winner"""
        pass
```

**Option B: Item System**
```python
class Item:
    def __init__(self, name: str, item_type: str, value: float):
        self.name = name
        self.item_type = item_type
        self.value = value

    def use(self, character: Character):
        """Apply item effects"""
        pass
```

**Option C: Experience and Progression**
- Implement experience scaling
- Add equipment (weapons, armor)
- Create skill system

**Your task**: Implement one extension that requires new classes or significant refactoring.

**Deliverable**: Extended system with 1-2 new features, tested and documented.

---

## Part 5: AI Validation and Professional Polish

**Your Role**: Engineer ensuring production quality

### Final Validation Exercise

Ask AI:

> "Final review: Does my Game Character System demonstrate:
> 1. Proper use of OOP principles (encapsulation, inheritance, abstraction)?
> 2. Professional Python style (type hints, docstrings, naming)?
> 3. Robust error handling and edge cases?
> 4. Clear code organization?
> 5. Appropriate access control and method design?
>
> Give me a final checklist of what would make this production-ready."

### Your Final Deliverables

1. **game_system_requirements.md** - Full specification
2. **game_character_system.py** - Complete implementation
3. **ai_review_feedback.md** - AI's code review and your responses
4. **extended_system.py** - One implemented extension
5. **professional_checklist.md** - Production-readiness criteria and how you meet them

---

## Summary: Capstone Integration Pattern

**Part 1 (Discover)**: You clarified requirements through AI conversation
**Part 2 (Implement)**: You built a complete system from specification
**Part 3 (Review)**: AI reviewed your code for design quality
**Part 4 (Extend)**: You added features demonstrating deeper understanding
**Part 5 (Polish)**: You validated against professional standards

### What You've Demonstrated

1. **Specification Design**: Translating requirements into class hierarchies
2. **Implementation**: Building working OOP systems
3. **Refactoring**: Improving design based on feedback
4. **Extension**: Adding features without breaking existing code
5. **Professional Quality**: Code that meets production standards

### Connections to Chapter 24

This capstone uses:
- **Lesson 1 (OOP Why)**: You built a real system that justifies OOP
- **Lesson 2 (Classes)**: You designed and created multiple classes
- **Lesson 3 (Attributes)**: You managed public, protected, and instance attributes
- **Lesson 4 (Methods)**: You used inheritance, instance methods, and access control
- **Lesson 5 (Capstone)**: You integrated everything into a coherent system

### What's Next

Chapter 25 introduces inheritance in depth, abstract base classes (ABC), and composition patterns. Your Game Character System is ready for those advanced topics.
