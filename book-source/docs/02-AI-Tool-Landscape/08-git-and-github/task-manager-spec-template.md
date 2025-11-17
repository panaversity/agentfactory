# Project: Personal Task Manager CLI

## Intent
Create a command-line task manager demonstrating Git workflow with AI-generated code. Students will manage real project using Git safety patterns learned in Lessons 1-6.

## Features
1. **Add Task**: User types task description, program adds to list
2. **View All Tasks**: Display numbered list of all tasks with completion status
3. **Mark Complete**: User provides task number, program marks as done
4. **Delete Task**: User provides task number, program removes it
5. **Persist to JSON**: All tasks saved to `tasks.json` file (survive program restart)

## Success Criteria
- ✅ All features work without errors
- ✅ Tasks persist across program runs (stored in tasks.json)
- ✅ Program shows help text explaining commands
- ✅ User-friendly: clear prompts, handles invalid input gracefully

## Technical Constraints
- **Language**: Python 3.8+
- **Data Storage**: JSON file (no database)
- **Interface**: Command-line only (no GUI)
- **Error Handling**: Validate user input, display helpful messages

## Non-Goals (What This Project Is NOT)
- ❌ Web interface (CLI only)
- ❌ Database integration (JSON sufficient)
- ❌ User authentication (single-user tool)
- ❌ Cloud sync (local storage only)
- ❌ Advanced features (priorities, due dates, categories deferred to v2)

## Validation Tests
After implementation, verify:
1. Add 3 tasks → Close program → Reopen → All 3 tasks still present
2. Mark task complete → Status changes
3. Delete task → Removed from list
4. Run with no tasks.json → Program creates file automatically
5. Type invalid command → Program shows helpful error (not crash)
