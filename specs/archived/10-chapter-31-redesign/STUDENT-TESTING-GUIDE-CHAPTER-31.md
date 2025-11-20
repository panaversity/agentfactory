# Student Testing Guide: Chapter 31 - Spec-Kit Plus Hands-On

**Version**: 1.0
**Date**: 2025-11-05
**Purpose**: Guide for testing Chapter 31 with 1-2 students before full publication

---

## Overview

This guide helps you run a pilot test of Chapter 31 with real students to validate:
- Instructions clarity
- Time estimates accuracy
- Exercise difficulty appropriateness
- Technical accuracy of commands
- Student comprehension and engagement

---

## Test Setup

### **Student Profile** (Ideal Test Participants)

**Prerequisites They Should Have**:
- ✅ Completed Chapter 30 (Spec-Driven Development Fundamentals)
- ✅ Python 3.13+ installed
- ✅ Basic terminal/command-line familiarity
- ✅ Access to Claude Code (Anthropic) OR Gemini CLI (Google)
- ✅ 13-15 hours available over 2-3 weeks

**Experience Level**: Intermediate (B1 proficiency)
- Can write Python code independently
- Understands basic programming concepts (functions, variables, data types)
- Has worked with AI tools before (ChatGPT, GitHub Copilot, Claude, etc.)

---

## Testing Protocol

### **Phase 1: Pre-Test Briefing** (30 minutes)

**Explain to Student**:
1. **Purpose**: "You're helping us validate a new chapter on specification-driven development with AI tools"
2. **Your Role**: "Complete the chapter as a normal student would. Take notes on anything confusing, unclear, or incorrect"
3. **Feedback Method**: "Use the feedback form after each lesson (provided below)"
4. **Time Tracking**: "Track how long each lesson actually takes you"
5. **Think-Aloud**: "If possible, narrate your thoughts as you work through exercises"

**Provide to Student**:
- Link to Chapter 31: `book-source/docs/05-Spec-Driven-Development/31-spec-kit-plus-hands-on/`
- Feedback form (see template below)
- Your contact info for questions

---

### **Phase 2: Lesson-by-Lesson Testing** (13-15 hours over 2-3 weeks)

**Student completes lessons in sequence**:

| Lesson | Estimated Time | Actual Time | Feedback Due |
|--------|----------------|-------------|--------------|
| 1: Installation & Setup | 1.5 hrs | _____ hrs | After completion |
| 2: Constitution Phase | 1.5 hrs | _____ hrs | After completion |
| 3: Specify Phase (Evals) | 2 hrs | _____ hrs | After completion |
| 4: Clarify Phase | 1.5 hrs | _____ hrs | After completion |
| 5: Plan Phase + ADRs | 2 hrs | _____ hrs | After completion |
| 6: Tasks Phase | 1.5 hrs | _____ hrs | After completion |
| 7: Implement + Validate | 2.5 hrs | _____ hrs | After completion |
| 8: Capstone Integration | 3-4 hrs | _____ hrs | After completion |

**Observer Notes** (if doing live observation):
- Where does student pause or re-read?
- Which commands cause confusion?
- Where does student need to look up external resources?
- When does student express frustration or satisfaction?

---

### **Phase 3: Post-Test Debrief** (45 minutes)

**Questions to Ask Student**:

1. **Overall Experience**:
   - On a scale of 1-10, how would you rate this chapter?
   - What was the most valuable thing you learned?
   - What was the most frustrating part?

2. **Clarity**:
   - Which instructions were unclear or ambiguous?
   - Which concepts were confusing?
   - Where did you need to re-read multiple times?

3. **Pacing**:
   - Which lessons felt too fast? Too slow? Just right?
   - Did the difficulty progression feel natural?
   - Were time estimates accurate?

4. **Exercises**:
   - Which "Try With AI" activities were most helpful?
   - Which exercises felt too easy? Too hard?
   - Did you complete all exercises or skip any?

5. **Technical Accuracy**:
   - Did all commands work as described?
   - Did you encounter any errors not mentioned in the chapter?
   - Were any examples outdated or incorrect?

6. **Outcomes**:
   - Can you now build a project using Spec-Kit Plus independently?
   - Do you feel confident using the 7 core commands?
   - Would you recommend this chapter to others?

---

## Feedback Form Template

### **Student Feedback Form: Chapter 31, Lesson [X]**

**Your Name**: ___________________________
**Date**: ___________________________
**Lesson**: ___________________________ (e.g., "Lesson 3: Specify Phase")

#### **1. Time & Pacing**
- Estimated time: _____ hours
- Actual time: _____ hours
- Pacing felt: ☐ Too fast  ☐ Just right  ☐ Too slow

#### **2. Clarity (Rate 1-5, 5=very clear)**
- Instructions: ☐ 1 ☐ 2 ☐ 3 ☐ 4 ☐ 5
- Concepts: ☐ 1 ☐ 2 ☐ 3 ☐ 4 ☐ 5
- Examples: ☐ 1 ☐ 2 ☐ 3 ☐ 4 ☐ 5
- Exercises: ☐ 1 ☐ 2 ☐ 3 ☐ 4 ☐ 5

#### **3. Difficulty (Rate 1-5, 5=very difficult)**
- Overall difficulty: ☐ 1 ☐ 2 ☐ 3 ☐ 4 ☐ 5
- Appropriate for stated proficiency level? ☐ Yes ☐ No

#### **4. Technical Issues**
Did you encounter any errors or problems?
- ☐ No issues
- ☐ Yes (describe): _________________________________________________

Commands that didn't work:
_________________________________________________________________

#### **5. Content Quality**
- Most helpful part: _________________________________________________
- Most confusing part: _________________________________________________
- Suggestions for improvement: _________________________________________________

#### **6. Engagement**
- Did you complete the "Try With AI" activity? ☐ Yes ☐ No ☐ Partially
- Was it helpful? ☐ Very ☐ Somewhat ☐ Not really
- Would you continue to next lesson? ☐ Yes ☐ Maybe ☐ No

#### **7. Open Comments**
Any other feedback, suggestions, or observations:

_________________________________________________________________
_________________________________________________________________
_________________________________________________________________

---

## Critical Success Criteria

### **Must Be True for Publication**:

1. **Technical Accuracy**: ☐ All commands work as described (100% success rate)
2. **Time Estimates**: ☐ Actual time within ±30% of estimated (e.g., 1.5hr lesson = 1-2 hrs actual)
3. **Clarity**: ☐ Average clarity rating ≥4.0/5.0 across all lessons
4. **Completion Rate**: ☐ Student completes all 8 lessons without giving up
5. **Comprehension**: ☐ Student can complete capstone (Lesson 8) independently
6. **Recommendation**: ☐ Student would recommend chapter to peers

### **Nice to Have**:

- ☐ Student expresses excitement about spec-first development
- ☐ Student reports "aha moments" about evals-first or human control
- ☐ Student plans to use Spec-Kit Plus in future projects
- ☐ Time estimates are accurate within ±20%

---

## Common Issues to Watch For

### **Installation Problems** (Lesson 1):
- API key configuration errors
- Python version mismatches
- Directory permission issues
- Command not found errors

**Mitigation**: Have troubleshooting section ready with common fixes

### **Conceptual Confusion** (Lessons 2-4):
- "What's the difference between Constitution and Spec?"
- "When do I use /sp.clarify vs just editing the spec?"
- "What are evals exactly?"

**Mitigation**: Ensure definitions are clear and examples concrete

### **Command Errors** (Lessons 5-7):
- Commands not recognized by AI tool
- Unexpected command outputs
- PHRs not auto-created as described

**Mitigation**: Verify all commands against actual Spec-Kit Plus implementation

### **Time Pressure** (Lesson 8):
- Capstone takes longer than 3-4 hours
- Student feels overwhelmed by independent work
- Student skips optional complexity

**Mitigation**: Clarify that 3-4 hours is minimum; 5-6 hours is normal for thorough work

---

## Data Collection Checklist

**For Each Test Session, Collect**:
- ☐ Completed feedback forms (8 total, one per lesson)
- ☐ Actual time tracking data
- ☐ Screenshots of any errors encountered
- ☐ Student's final capstone project artifacts
- ☐ Post-test debrief notes
- ☐ Any communication/questions during the test

---

## Analysis & Iteration

**After Testing, Review**:

1. **Aggregate Time Data**:
   - Calculate average actual time per lesson
   - Identify lessons that consistently run over
   - Update time estimates if >30% variance

2. **Identify Clarity Issues**:
   - List all sections rated <4.0 for clarity
   - Prioritize fixes for critical path content
   - Rewrite ambiguous instructions

3. **Fix Technical Errors**:
   - Document any command failures
   - Verify against Spec-Kit Plus source
   - Update lesson content or command references

4. **Assess Difficulty Progression**:
   - Check if A2→B1→B2 progression feels natural
   - Verify cognitive load (5/7/10 concepts) is appropriate
   - Adjust complexity if students struggle or get bored

5. **Incorporate Feedback**:
   - Create GitHub issues for major improvements
   - Make minor fixes immediately
   - Document any deferred improvements

---

## Test Completion Report Template

### **Chapter 31 Student Test Report**

**Test Date**: ___________________________
**Student ID**: ___________________________ (use pseudonym)
**Observer**: ___________________________

#### **Summary Statistics**

| Metric | Target | Actual | Status |
|--------|--------|--------|--------|
| Completion rate | 100% (8/8 lessons) | ___ / 8 | ☐ Pass ☐ Fail |
| Average clarity rating | ≥4.0/5.0 | _____ | ☐ Pass ☐ Fail |
| Time estimate accuracy | Within ±30% | _____ % variance | ☐ Pass ☐ Fail |
| Technical errors | 0 critical | _____ critical | ☐ Pass ☐ Fail |
| Would recommend | Yes | ☐ Yes ☐ No | ☐ Pass ☐ Fail |

#### **Key Findings**

**What Worked Well**:
1. _________________________________________________________________
2. _________________________________________________________________
3. _________________________________________________________________

**What Needs Improvement**:
1. _________________________________________________________________
2. _________________________________________________________________
3. _________________________________________________________________

**Critical Issues** (block publication):
- _________________________________________________________________

**Minor Issues** (fix post-publication):
- _________________________________________________________________

#### **Recommendations**

☐ **Ready for Publication** (all critical criteria met)
☐ **Needs Revision** (critical issues identified)
☐ **Needs Re-Test** (after fixes, test again)

**Next Steps**:
1. _________________________________________________________________
2. _________________________________________________________________
3. _________________________________________________________________

---

## Contact & Support

**For Questions During Test**:
- Instructor/Observer: [Your contact info]
- Technical Issues: [Support channel]
- Urgent Problems: [Emergency contact]

**For Feedback Submission**:
- Email: [Your email]
- Form: [Google Form / Typeform link]
- Slack: [Slack channel]

---

**Good luck with the test! This feedback will directly improve the learning experience for thousands of students.**
