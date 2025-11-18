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

This capstone integrates everything from Lessons 1-4, but instead of manually typing 200+ lines of code, you'll work at a higher level: **planning with AI, having AI generate code, then validating and refining the design**. This is how professionals build complex systems with AI collaboration.

**Key shift**: You focus on game design decisions and validation. AI handles typing and syntax details.

---

## Part 1: Student Plans Game Architecture with AI

**Your Role**: Game designer collaborating with AI to plan (not write formal specs - just plain language planning)

### Planning Exercise: Design Your Game System

You'll have a **design conversation** with AI about what you want to build. No formal specs - just game design thinking.

#### 💬 AI CoLearning Prompt - Initial Game Design

> "I want to build a turn-based RPG character system. Help me think through the design (don't write code yet, just help me plan):
>
> **What I want:**
> - Player and Enemy characters that can fight each other
> - Characters have health, attack power, defense
> - Player can level up and has inventory
> - Enemy has different AI behaviors (random or aggressive)
>
> **Help me decide:**
> 1. Should I have a base Character class that both Player and Enemy inherit from? What goes in the base vs subclasses?
> 2. How do I prevent health from going negative or above max?
> 3. Should attack() be in the base class or different for Player vs Enemy?
> 4. What's the simplest way to handle inventory for Player?
> 5. How should I structure this so I can easily add new character types later?
>
> Give me a simple game design plan with class structure and main decisions. Use plain language, not technical jargon."

**Expected AI Output**: A plain-language plan explaining:
- Character (base) → Player/Enemy (subclasses)
- Base class has shared behavior (attack, take_damage, heal)
- Use properties to keep health between 0 and max
- Player adds inventory list, Enemy adds AI behavior
- Design rationale in simple terms

**Your task**: Read AI's plan. Does the game design make sense? Can you visualize how the classes work together?

---

### Challenge AI's Design with "What If" Scenarios

Now test if AI thought through edge cases:

#### 💬 AI CoLearning Prompt - Edge Case Planning

> "I like your plan, but I need to think through some game scenarios:
>
> **Scenario 1**: Player attacks Enemy, Enemy health would go to -10. What should happen?
> **Scenario 2**: Dead character (health = 0) tries to attack. What should happen?
> **Scenario 3**: Player heals for 50 but only needs 20 to reach max_health. What should happen?
>
> For each, tell me what REAL games do and why. How should my design handle these cases?"

**Expected Learning**: AI will explain game design patterns (health clamps at 0/max, dead characters can't act, healing caps at max, etc.). You're learning **design thinking**, not just coding syntax.

---

### Your Simple Game Design Plan

After your conversation with AI, write down a simple plan (no formal template needed):

**My Game Design Notes:**
- Character (base class) - shared stuff all characters need (health, attack, defend)
- Player extends Character - adds inventory and experience
- Enemy extends Character - adds AI behavior
- Health stays between 0 and max (use properties)
- Dead characters can't attack (check is_alive() first)
- Keep it simple - can extend later

**Key Learning**: You planned the ARCHITECTURE through conversation, not by writing formal specs. That skill comes later in Part 5.

**Deliverable**: Simple game design notes from your AI conversation. You've planned the architecture - now AI will help generate the implementation.

---

## Part 2: AI Generates Code, Student Validates Design

**Your Role**: Design validator working with AI to generate and refine the system

### AI-Assisted Implementation

Instead of typing 200+ lines manually, you'll have AI generate the system, then YOU validate it against your design plan.

#### 💬 AI CoLearning Prompt - Generate Base System

> "Based on my game design plan, generate the Python code for my character system:
>
> **Requirements:**
> - Character base class with:
>   - name, _health, _max_health, _attack_power, _defense attributes (use protected _underscore)
>   - attack(target), take_damage(amount), heal(amount), is_alive() methods
>   - Use @property for health to keep it between 0 and max
> - Player class inheriting from Character with:
>   - inventory (list), level (int), experience (int)
>   - gain_experience(amount), use_item(item), level_up() methods
> - Enemy class inheriting from Character with:
>   - enemy_type (str), ai_strategy ('random' or 'aggressive')
>   - choose_action() method
>
> **Important:**
> - Add type hints to everything
> - Health must stay between 0 and max (validate in property setter)
> - Dead characters (health = 0) can't attack (check in attack method)
> - Include docstrings
>
> Generate the complete code. I'll validate it against my design."

**Expected AI Output**: ~100-150 lines of complete code implementing your design.

---

### Validation Exercise: Check AI's Implementation

**Your task**: Read the AI-generated code and validate these design decisions:

**✅ Validation Checklist:**

1. **Inheritance Structure**:
   - Does Player inherit from Character? ✓
   - Does Enemy inherit from Character? ✓
   - Is shared behavior in the base class? ✓

2. **Encapsulation (Lesson 4 concept)**:
   - Are health, attack_power, defense protected (_underscore)? ✓
   - Is there a @property for health with validation? ✓
   - Does the setter prevent health < 0 or > max? ✓

3. **Edge Cases (from Part 1 planning)**:
   - Dead character attack: Does attack() check is_alive()? ✓
   - Negative health: Does take_damage() clamp at 0? ✓
   - Overheal: Does heal() cap at max_health? ✓

4. **Type Hints (Chapter 24 standard)**:
   - Does every method have parameter and return type hints? ✓
   - Are attributes typed in `__init__`? ✓

---

### Challenge AI's Code with Test Cases

#### 💬 AI CoLearning Prompt - Test Edge Cases

> "I want to validate your code handles edge cases correctly. Show me test code for:
>
> **Test 1: Health Clamping**
> ```python
> player = Player("Hero", max_health=100, attack_power=20, defense=5)
> enemy = Enemy("Goblin", max_health=50, attack_power=15, defense=2)
>
> # Attack multiple times - health should stop at 0, not go negative
> enemy.take_damage(30)  # 50 - 30 = 20
> enemy.take_damage(30)  # Should clamp at 0, not -10
> print(enemy.health)  # Should print 0
> ```
>
> **Test 2: Dead Character Can't Attack**
> ```python
> print(enemy.attack(player))  # Should return 0 (can't attack when dead)
> ```
>
> **Test 3: Overheal Caps at Max**
> ```python
> player.take_damage(20)  # Now at 80 health
> player.heal(50)  # Should heal only to 100, not 130
> print(player.health)  # Should print 100
> ```
>
> Run these tests and show me the output. Do they all pass as expected?"

**Expected Learning**: You validate AI's implementation through testing, not by reading every line. This is professional workflow - specify requirements, generate, validate.

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

## Part 3: Student Challenges AI with "What If" Extensions

**Your Role**: Game designer testing design flexibility

### Extension Challenge Exercise

Now that you have a working system, challenge AI to extend it. Can the design handle new features?

#### 💬 AI CoLearning Prompt - Add Boss Character

> "I want to add a Boss character to my game. Bosses should:
> - Have multiple health bars (when one empties, another appears)
> - Have special attacks that do area damage
> - Drop better loot when defeated
>
> How should I design this? Should Boss inherit from Enemy, or be its own class? Show me the code changes needed. Explain the design trade-offs."

**Expected Learning**: AI will show you how to extend your class hierarchy. You'll see if your original design was flexible or needs refactoring. This is real-world design validation.

---

#### 💬 AI CoLearning Prompt - Add Equipment System

> "I want Players to equip weapons and armor that boost their stats:
> - Weapon increases attack_power
> - Armor increases defense
> - Players can equip/unequip items
>
> Should equipment be:
> a) Attributes on Player class?
> b) Separate Equipment class with composition?
> c) Something else?
>
> Show me the design and explain why it's better than alternatives. Generate the Equipment class code."

**Expected Learning**: AI will introduce **composition** (objects containing other objects) as an alternative to inheritance. You're learning advanced OOP design patterns.

---

#### 💬 AI CoLearning Prompt - Add Save/Load System

> "I want to save the game state to a JSON file and load it later:
> - Save all character attributes (health, inventory, level, etc.)
> - Load and recreate characters from saved data
>
> Show me:
> 1. How to convert a Player object to JSON (serialization)
> 2. How to recreate a Player object from JSON (deserialization)
> 3. What methods should I add to my classes to support this?
>
> Generate the code for save_game() and load_game() functions."

**Expected Learning**: You're seeing how your OOP design enables features you didn't originally plan. Good design is flexible.

**Deliverable**: Extended game system with 1-2 new features (Boss, Equipment, or Save/Load) generated and validated through AI collaboration.

---

## Part 4: Build Your OOP Design Framework

**Your Role**: Experienced designer extracting lessons learned

### Reflection Exercise

After building and extending your system with AI, extract the **design patterns** you learned.

#### 💬 AI CoLearning Prompt - Synthesize Lessons

> "Based on my Game Character System project, help me create a simple design guide for future OOP projects:
>
> **What I learned:**
> 1. When to use inheritance (base class + subclasses)
> 2. When to use composition (objects containing other objects)
> 3. How to use properties for validation
> 4. How to design for extensibility
>
> Give me a simple framework with:
> - When to use each pattern
> - Common mistakes to avoid
> - Quick decision guide ('If X, use Y')
>
> Use my game system as the example throughout."

**Expected AI Output**: A personalized OOP design guide based on your actual project experience.

---

### Your OOP Design Framework

Save AI's framework as your reference. Example structure:

**My OOP Design Guide (From Game System Project):**

**When to Use Inheritance:**
- Base class = shared behavior (Character: attack, take_damage, heal)
- Subclasses = specialization (Player adds inventory, Enemy adds AI)
- Ask: "Is this an IS-A relationship?" (Player IS-A Character ✓)

**When to Use Composition:**
- Objects contain other objects (Player HAS-A Equipment)
- More flexible than inheritance (can swap equipment)
- Ask: "Is this a HAS-A relationship?" (Player HAS-A Inventory ✓)

**Design for Extensibility:**
- Use protected attributes (_health) so subclasses can access
- Use properties for validation (keeps data valid)
- Keep methods focused (single responsibility)

**Common Mistakes I Avoided:**
- Making everything inherit (sometimes composition is better)
- Forgetting validation (properties saved me)
- Not planning for extensions (had to refactor Boss class)

**Deliverable**: Personal OOP design framework extracted from your project. You've learned by building, not just reading.

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

## Try With AI

Ready to build a complete multi-class system that demonstrates OOP mastery?

**🔍 Explore System Design:**
> "I want to build a game character system with Warriors, Mages, and Archers. Each has health, name, and class-specific abilities (warrior: melee_attack, mage: cast_spell, archer: shoot_arrow). Help me design: should they share a base Character class? What attributes are common vs specific? How should abilities differ?"

**🎯 Practice Class Hierarchy:**
> "Guide me through building the character system step-by-step. Start with base Character class (__init__ with name, health, level). Then create Warrior subclass adding strength attribute and attack() method. Show me how Warrior inherits from Character and extends it. What goes in base vs subclass?"

**🧪 Test Encapsulation and Validation:**
> "Add validation to the character system: health can't go below 0 or above max_health (100), level starts at 1, name can't be empty. Use properties or validation in __init__. Then create a take_damage(amount) method. Show me how validation prevents invalid states (negative health, empty name)."

**🚀 Apply Complete System:**
> "Build a complete battle system: create 3 characters (Warrior, Mage, Archer), implement attacks that reduce target health, add is_alive() method, create fight_round(attacker, defender) function. Demonstrate full combat until one character's health reaches 0. Show all OOP principles: encapsulation, methods, independent object state."

---
