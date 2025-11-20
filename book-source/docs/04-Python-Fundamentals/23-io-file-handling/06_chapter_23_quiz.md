---
sidebar_position: 6
title: "Chapter 23: IO and File Handling Quiz"
---

# Chapter 23: IO and File Handling Quiz

Test your understanding of console I/O, file operations, cross-platform path handling, and structured data formats.

<Quiz
  title="Chapter 23: IO and File Handling Assessment"
  questions={[    {
      question: "You're building a CLI tool that processes user input for age. The user types '25'. What type does input() return, and what must you do before using it in calculations?",
      options: [
        "Returns string; must convert with int() before calculations",
        "Returns integer; can use directly in math operations",
        "Returns number type; works with any arithmetic automatically",
        "Returns string; Python auto-converts when needed in math"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 2. The input() function ALWAYS returns a string, regardless of what the user types. Even if they type '25', you receive the string '25', not the integer 25. You must explicitly convert using int() or float() before performing calculations. Option 1 is wrong because input() never returns integers directly. Option 3 is wrong because Python doesn't have a generic 'number' type from input(). Option 4 is wrong because Python does NOT auto-convert strings to numbers in arithmetic operations—it will raise a TypeError if you try to add a string to a number. This is covered in Lesson 1: Understanding input() return types.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "When validating numeric input, you ask for a positive integer but the user types 'hello'. Which exception does int('hello') raise, and how should you handle it?",
      options: [
        "Raises TypeError; catch and show user-friendly message",
        "Raises ValueError; catch and prompt user to retry",
        "Raises InputError; catch and request valid number",
        "Raises ConversionError; catch and display error details"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. When int() receives a string that cannot be converted to an integer (like 'hello'), it raises a ValueError, not TypeError. The professional pattern is to wrap conversion in try/except ValueError, show a helpful error message ('Please enter a whole number'), and use a loop to allow the user to retry. Option 1 is wrong because TypeError occurs when you pass the wrong type entirely (like int([1,2,3])), not for invalid string content. Options 3 and 4 are wrong because InputError and ConversionError are not Python built-in exceptions. This validation pattern is essential for building robust CLI tools and is covered in Lesson 1: Input Validation with Type Conversion.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "You're formatting a price of 19.999 for display. Which f-string format specifier displays it as '$20.00' (2 decimal places)?",
      options: [
        "Use f'${price:.2d}' to round to two places",
        "Use f'${price:2}' to specify two decimal digits",
        "Use f'${price:.2f}' to round and format decimals",
        "Use f'${price:round(2)}' to format as currency value"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. The format specifier :.2f means 'format as a float with 2 decimal places', which automatically rounds 19.999 to 20.00. The colon introduces the format spec, .2 specifies precision (2 digits after decimal), and f means floating-point format. Option 2 is wrong because :2 without the .f would be right-alignment in a field of width 2, not decimal precision. Option 3 is wrong because d is for integers (no decimal point), not floats. Option 4 is wrong because round() is a function call, not a format specifier—you can't use function calls inside format specs like :round(2). F-string formatting is covered in Lesson 1: Formatting Output with F-Strings.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "Your CLI asks for a number between 1-10. The user enters '15'. Which validation pattern ensures they retry until providing valid input?",
      options: [
        "Use do-while loop checking value after each attempt",
        "Use for loop iterating until range check passes",
        "Use if/else with recursive function call on failure",
        "Use while True loop with break on valid input"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. The pattern is: while True: get input, validate with try/except and range checks, if valid then break (or return), otherwise show error and loop continues. This allows unlimited retries until valid input. Option 2 is wrong because for loops require knowing iteration count in advance—you don't know how many tries the user needs. Option 3 technically works but recursion for input validation is non-idiomatic Python and risks stack overflow with stubborn users. Option 4 is wrong because Python doesn't have a do-while loop construct (though you can simulate it, the standard pattern is while True with break). This retry pattern is fundamental for CLI robustness and is covered in Lesson 1: Error Recovery Loops.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "You're building a calculator that displays results. For the number 1234567.89, which f-string shows '1,234,567.89' (thousands separator with 2 decimals)?",
      options: [
        "Use f'{value:,.2f}' for thousands separator and precision",
        "Use f'{value:.2f,}' to add comma after decimal formatting",
        "Use f'{value:sep,.2f}' to specify separator type explicitly",
        "Use f'{value.format(\",\",.2)}' for comma-separated decimal formatting"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. The format specifier :,.2f combines two features: the comma (,) adds thousands separators, and .2f formats as float with 2 decimal places. They work together in that order: :,.2f. Option 2 is wrong because the comma must come BEFORE the precision spec (.2f), not after—the order matters in format specifiers. Option 3 is wrong because 'sep' is not a valid format specifier keyword. Option 4 is wrong because .format() is the old string formatting method (not f-strings), and the syntax shown doesn't work—you can't call .format() on a number. Combining format specifiers is covered in Lesson 1: Professional Formatted Output.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "When describing input validation requirements to AI, which prompt produces the most robust validation function?",
      options: [
        "Request specific syntax: write while loop with int conversion",
        "Describe intent: get positive number with retry on errors",
        "Ask for complete code: validation function for user input",
        "Specify exception names: catch ValueError and TypeError together"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. In AI-native development, you describe INTENT (what you want: positive number, retry on errors) rather than prescribing syntax. This lets the AI generate the appropriate pattern (while True, try/except ValueError, range check, helpful errors). Option 2 prescribes implementation details—the AI might follow it literally without adding helpful error messages or edge case handling. Option 3 is too vague—doesn't specify constraints (positive? integer or float? retry or fail once?). Option 4 focuses on wrong detail—you typically don't need TypeError for input validation, and specifying exceptions without context doesn't convey the full requirement. This workflow principle is covered in Lesson 1: AI Workflow Integration for input validation.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "Your program asks for email input. The user enters 'test' (missing @). Which validation approach provides the best user experience?",
      options: [
        "Accept any input and validate later during processing",
        "Use regex pattern validation with cryptic error message",
        "Check for '@' and '.' then retry with message",
        "Crash with ValueError to force correct format entry"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. User experience means: immediate validation (don't wait until processing), clear error messages ('Email must contain @ and domain'), and retry opportunity. Basic checks like '@' and '.' presence are sufficient for CLI input—they catch most typos while keeping validation simple. Option 2 is technically more thorough but 'cryptic error message' ruins UX—regex validation is fine if you explain what's wrong clearly. Option 3 is wrong because delayed validation means user continues the flow only to fail later—catch errors early. Option 4 is terrible UX—never crash intentionally; always catch exceptions and let users retry with guidance. User-friendly validation is covered in Lesson 1: Error Recovery patterns.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "You ask AI to generate formatted output code. The AI uses :.2f for currency but forgets thousands separators on large amounts. How do you refine the prompt?",
      options: [
        "Request: add thousands separator for amounts over 1000",
        "Specify exact syntax: change :.2f to :,.2f everywhere",
        "Ask AI to explain: why no commas in output",
        "Provide example: show $1,234.56 as desired format"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. Providing a concrete example ('show $1,234.56 as desired format') is the most effective refinement—AI sees exactly what output you want and can infer you need :,.2f. Examples are clearer than abstract descriptions. Option 1 works but is verbose and prescriptive (AI might add conditional logic instead of using format spec). Option 2 works but teaches you nothing—you're just telling AI the answer instead of collaborating. Option 3 is purely exploratory—asking 'why' helps you learn but doesn't refine the code. For refining output, concrete examples of desired vs actual output work best. This AI collaboration pattern is covered in Lesson 1: Effective AI prompting for formatting.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "You're validating user age input (must be 0-120). Which validation order prevents unnecessary error messages?",
      options: [
        "Check range first then attempt int conversion afterward",
        "Convert to int first then check if value within range",
        "Check string length first then convert and validate range",
        "Validate all conditions simultaneously in single expression"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Always convert first (int()), THEN check range. Why? If conversion fails (user typed 'abc'), you want the 'not a number' error, not a range error. If you check range on the string first, you get confusing behavior (strings don't compare numerically). Order matters: try: age = int(input()) / if age < 0 or age > 120: (error). Option 1 reverses the order—you can't check numeric range on a string. Option 3 wastes effort—string length doesn't correlate with valid age ('5' is length 1, '500' is invalid age). Option 4 is wrong because you can't validate simultaneously—conversion can raise exception, stopping execution before range check. This validation sequencing is covered in Lesson 1: Input Validation patterns.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "You're building a CLI menu. The user presses Enter without typing anything. What does input() return, and how should you handle it?",
      options: [
        "Returns empty string; check with if not input().strip()",
        "Returns None value; check with if input() is None",
        "Returns zero; check with if input() == 0 first",
        "Returns whitespace; check with if input().isspace() for blank"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. When the user presses Enter without typing, input() returns an empty string '', not None or 0. Best practice: use .strip() to remove whitespace, then check if the result is empty with if not user_input.strip(). This handles both completely empty input and input with only spaces/tabs. Option 2 is wrong because input() never returns None—it always returns a string. Option 3 is wrong because input() never returns the integer 0—it would return the string '0' if user typed zero. Option 4 is partially right that whitespace is possible, but checking .isspace() misses completely empty strings ('' is not considered whitespace, it's just empty). Empty input handling is covered in Lesson 1: Input validation edge cases.",
      source: "Lesson 1: Console I/O and User Input Validation"
    },
    {
      question: "Why does Python require explicit context managers (with statement) for file operations instead of automatic cleanup like some languages?",
      options: [
        "Explicit syntax is required for cross-platform file compatibility",
        "Python lacks automatic garbage collection for file objects",
        "Context managers improve performance by reducing file overhead",
        "Explicit control prevents resource leaks and data corruption"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Python uses 'explicit is better than implicit' philosophy. Context managers (with statement) guarantee cleanup even if exceptions occur, preventing resource leaks (OS limits on open files) and data corruption (unflushed write buffers). Python COULD auto-close files on garbage collection, but that's unpredictable timing—with statement ensures immediate, deterministic cleanup. Option 2 is wrong; Python HAS garbage collection, but relying on it for file cleanup is risky because you don't control when it runs. Option 3 is wrong; context managers don't improve performance, they improve safety/correctness. Option 4 is wrong; context managers aren't about cross-platform compatibility specifically. This design principle is covered in Lesson 2: Context Managers and Why They're Essential.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "You open a file in 'w' mode and write two lines. Then you open the same file in 'w' mode again. What happens to the original content?",
      options: [
        "Python raises FileExistsError to prevent data loss",
        "New content appends after original content automatically",
        "Original content is completely deleted before new write",
        "Original content remains; new content writes to separate file"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. Opening a file in 'w' (write) mode IMMEDIATELY truncates (deletes) all existing content, even before you write anything. This is the most dangerous aspect of 'w' mode and a common source of data loss for beginners. If you want to add to existing content, use 'a' (append) mode instead. Option 2 describes append mode ('a'), not write mode. Option 3 is wrong; Python does NOT protect you from overwriting—'w' mode silently deletes content without warnings or errors. Option 4 is wrong; there's no separate file creation, the original file is overwritten in place. Understanding the destructive nature of 'w' mode is critical and covered in Lesson 2: File Modes Explained.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "You're reading a 1GB log file to find lines containing 'ERROR'. Which approach prevents running out of memory?",
      options: [
        "Iterate file object directly line by line",
        "Use readlines() to get all lines as list",
        "Use read() to load entire file into memory",
        "Use readline() in loop loading full file"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 3. Iterating the file object directly (for line in file:) is memory-efficient because it reads one line at a time, processes it, then discards it before reading the next. This works for files of ANY size. Options 1 and 2 are wrong because read() and readlines() load the ENTIRE file into memory—with a 1GB file, you'll use 1GB+ of RAM, which may crash your program. Option 4 (readline() in loop) works but is less Pythonic than direct iteration—you have to manually check for end-of-file. The idiomatic pattern is: with open(file) as f: for line in f: (process). This efficient pattern is covered in Lesson 2: Reading Methods for different needs.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "You write three lines to a file using file.write(). The file contains 'Line1Line2Line3' on a single line. What did you forget?",
      options: [
        "Forgot to use writelines() instead of write() calls",
        "Forgot to add '\\n' newline explicitly to each write",
        "Forgot to flush() the buffer after each line",
        "Forgot to open file in line mode with 'l' flag"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. Unlike print() which auto-adds newlines, write() writes EXACTLY what you give it. If you write('Line1') then write('Line2'), they concatenate without newlines. You must explicitly include '\\n': write('Line1\\n'). This is a common beginner mistake. Option 2 is wrong because writelines() ALSO doesn't add newlines—it just writes a list of strings without modification. Option 3 is wrong; flushing affects when data goes to disk, not newline formatting. Option 4 is wrong; there's no 'l' (line) flag in file modes—modes are 'r', 'w', 'a', 'r+', etc. Always remember: write() is literal, you control formatting. This is covered in Lesson 2: Writing Files with Proper Formatting.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "You try to read 'config.txt' but the file doesn't exist. Which exception does Python raise, and what's the professional handling pattern?",
      options: [
        "Raises RuntimeError; catch and exit program with error message",
        "Raises IOError; catch and notify user of missing file",
        "Raises OSError; catch and request user provide file path",
        "Raises FileNotFoundError; catch and create default config file"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Python 3 raises FileNotFoundError when opening a non-existent file in read mode. The professional pattern is: catch FileNotFoundError, then CREATE a default config file (with open(path, 'w') as f: write defaults), so the program continues working. This is better than just notifying the user—be helpful by fixing the problem. Option 2 is partially right (IOError is the old Python 2 name, now FileNotFoundError in Python 3), but just 'notify user' isn't as helpful as creating defaults. Option 3 is wrong; OSError is the parent class but you should catch the specific FileNotFoundError. Option 4 is wrong; RuntimeError is for different errors, and exiting immediately is poor UX. Creating defaults on missing files is covered in Lesson 2: File Exceptions.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "Your program opens a file with 'with open()' then crashes inside the block. What happens to the file?",
      options: [
        "File closes only if exception is caught explicitly",
        "File remains open causing resource leak until program ends",
        "File automatically closes despite exception guaranteeing cleanup",
        "File closes automatically only on successful completion"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. The ENTIRE POINT of context managers is exception safety—the file closes automatically even if an exception occurs inside the with block. This is guaranteed by the context manager protocol (__exit__ is called regardless of how the block ends). Option 2 describes what happens WITHOUT context managers (manual open/close). Option 3 is wrong; you don't need to catch the exception for cleanup—the with statement handles it. Option 4 is wrong; cleanup happens on BOTH success and failure. This guarantee makes context managers essential for safety: with open() as f: (risky code) guarantees cleanup. This critical safety feature is covered in Lesson 2: Context Manager exception safety.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "You're implementing log file append functionality. Which file mode ensures new entries add to the end without deleting existing logs?",
      options: [
        "Use 'a' append mode to add without deletion",
        "Use 'w' write mode seeking to end before writing",
        "Use 'r+' read-write mode with manual position control",
        "Use 'w+' write-read mode appending after initial truncate"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Mode 'a' (append) is designed exactly for this use case—it positions the write pointer at the end of the file and preserves existing content. Every write() adds to the end. This is perfect for logs. Option 2 is wrong; 'w' mode TRUNCATES the file immediately when opened, deleting all content—seeking doesn't help because content is already gone. Option 3 technically works but is error-prone—you'd open with 'r+', manually seek to end, then write. Why do manual work when 'a' does it automatically? Option 4 is wrong; 'w+' truncates just like 'w'. For append operations, always use 'a' mode. This mode selection is covered in Lesson 2: File Modes and when to use each.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "When asking AI to generate file reading code, which prompt style produces safer, more maintainable code?",
      options: [
        "Request manual approach: open, read, close explicitly",
        "Describe requirement: safely read with automatic cleanup",
        "Ask for minimal code: shortest way to read file",
        "Specify error handling: catch all possible file exceptions"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. Describing the requirement ('safely read with automatic cleanup') prompts the AI to generate code with context managers (with open()), which is the modern, safe pattern. The AI understands 'safe' means exception handling and 'automatic cleanup' means context managers. Option 2 requests the OLD, manual pattern (open/read/close) which is error-prone—you might forget to close or fail to close on exceptions. Option 3 ('shortest way') might produce one-liner hacks that sacrifice safety for brevity. Option 4 focuses on catching 'all possible' exceptions, which often leads to overly broad except clauses that hide bugs. For file I/O, emphasizing safety and cleanup in prompts produces best practices. This AI workflow is covered in Lesson 2: AI collaboration for file operations.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "You're processing a 10MB file. Is it better to use read() to load it all at once or iterate line by line?",
      options: [
        "Load all at once for faster processing speed",
        "Use read() for better performance with moderate files",
        "Iterate line by line to minimize memory usage",
        "Depends on file type; text needs read(), binary needs iteration"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. For files in the MB range and larger, line-by-line iteration is safer because it uses constant memory (one line at a time) regardless of file size. 10MB fits in memory, but what if the file grows to 100MB or 1GB later? Iterating scales indefinitely. Option 2 (read() for moderate files) works but doesn't scale—what's 'moderate'? Building the habit of iterating is better. Option 3 prioritizes performance over safety; the performance difference is negligible and not worth the memory risk. Option 4 is wrong; text vs binary doesn't determine read strategy—size and processing needs do. For any file where you process content line-by-line (logs, CSV, etc.), iterate. This best practice is covered in Lesson 2: Efficient reading methods.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "Your application reads files from user-provided paths. Which security check prevents users from accessing '/etc/passwd' outside your app directory?",
      options: [
        "Check file extension is in allowed list before reading",
        "Block paths containing forward slashes or dots",
        "Validate path string matches expected filename pattern",
        "Resolve path and check it stays within base directory"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Use Path.resolve() to canonicalize the path (resolving '..' and '.'), then use .is_relative_to(base_dir) to verify the resolved path stays within your allowed directory. This prevents path traversal attacks like '../../../etc/passwd'. Option 2 (blocking slashes/dots) is ineffective—legitimate filenames have these characters, and attackers can bypass with encoding. Option 3 (regex matching) doesn't prevent traversal—'../../outside/valid_name.txt' might match your pattern but escape your directory. Option 4 (file extension check) is irrelevant to path traversal—attackers can use '../etc/passwd.txt' to match your extension check while still escaping. Path security validation is critical and covered in Lesson 2: Resource cleanup and safety.",
      source: "Lesson 2: File I/O Fundamentals with Context Managers"
    },
    {
      question: "Why does pathlib's Path() object use the / operator for joining paths instead of a .join() method?",
      options: [
        "Python syntax requires operators for path concatenation",
        "The / operator provides intuitive cross-platform path building",
        "The / character is universally valid in all filesystems",
        "Operator overloading improves performance for path operations"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. The / operator was chosen for intuitiveness and readability: Path('data') / 'notes' / 'file.txt' reads naturally like building a path. It works identically on Windows (producing backslashes internally) and Unix (forward slashes), making code cross-platform without thinking about separators. Option 2 is wrong; Python doesn't 'require' operators—this was a design choice for ergonomics. Option 3 is misleading; / is the Unix separator, but Windows uses backslashes (\\)—the OPERATOR works everywhere, not the character itself. Option 4 is wrong; performance isn't the reason, convenience is—operator overloading doesn't inherently improve performance. The / operator makes pathlib code cleaner than os.path.join(). This design is explained in Lesson 3: Creating and Manipulating Paths.",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "You create Path('config') / 'app.json'. On Windows, what does str(path) return?",
      options: [
        "Returns 'config\\\\app.json' with Windows backslash automatically",
        "Returns 'config/app.json' with forward slash everywhere",
        "Returns platform-independent string without separators",
        "Returns error because forward slash invalid on Windows"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Path objects automatically use the correct path separator for the operating system. On Windows, str(path) returns 'config\\\\app.json' (double backslash in repr, single in actual string). On Unix/Mac, it returns 'config/app.json'. This is the whole point of pathlib—write Path('a') / 'b' once, and it works correctly everywhere. Option 2 is wrong; pathlib doesn't enforce forward slashes everywhere, it uses OS-native separators. Option 3 is wrong; the string representation always includes separators. Option 4 is wrong; the / OPERATOR in Python code is always valid—it's pathlib's way of joining, not a literal path character. Cross-platform compatibility is covered in Lesson 3: Understanding pathlib vs os.path.",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "You need to check if a path exists and is specifically a file (not a directory). Which combination of checks is correct?",
      options: [
        "Use path.exists() and path.is_file() to verify both",
        "Use path.exists() then check path.suffix for extension",
        "Use only path.is_file() which implies existence",
        "Use path.stat() and check file mode bits manually"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 2. The method .is_file() returns True only if the path exists AND is a file (not a directory). It's a single check that verifies both conditions. If the path doesn't exist, .is_file() returns False. So you don't need to call .exists() separately—it's redundant. Option 1 works but wastes a system call checking .exists() when .is_file() already does that. Option 3 is wrong; checking suffix doesn't verify it's a file (directories can have extensions like 'project.dir'). Option 4 technically works but is over-complicated—why manually check mode bits when .is_file() does it for you? Always use the specific check (.is_file() or .is_dir()) rather than generic .exists(). This is covered in Lesson 3: Checking Existence and File Types.",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "You try Path('data/notes/2025/november').mkdir() but 'data' doesn't exist. What happens, and how do you fix it?",
      options: [
        "Raises PermissionError; fix with mkdir(mode=0o777) for access",
        "Automatically creates all missing parent directories without error",
        "Creates only final directory; manually create parents first",
        "Raises FileNotFoundError; fix with mkdir(parents=True) to create all"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. By default, mkdir() only creates the final directory and requires all parent directories to already exist. If 'data' doesn't exist, you get FileNotFoundError. The fix: mkdir(parents=True) tells Python to create ALL missing parent directories (like mkdir -p in Unix). Also add exist_ok=True so it doesn't error if the directory already exists: mkdir(parents=True, exist_ok=True). Option 2 is wrong; auto-creation is NOT the default, you must request it with parents=True. Option 3 is inefficient; why manually create parents when parents=True does it automatically? Option 4 is wrong; PermissionError is different (insufficient OS permissions), not missing parents. Always use mkdir(parents=True, exist_ok=True) for robust directory creation. This is covered in Lesson 3: Creating Directories.",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "You want to find all .txt files in a directory. Which pathlib method searches for files matching a pattern?",
      options: [
        "Use .glob('*.txt') to match pattern against filenames",
        "Use .filter('*.txt') to select files by extension",
        "Use .find('*.txt') to search directory for files",
        "Use .list('*.txt') to enumerate matching files only"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. The .glob(pattern) method searches for files matching a glob pattern. For example, path.glob('*.txt') returns an iterator of all .txt files in that directory. The pattern syntax supports wildcards: * (anything), ? (one char), **/*.txt (recursive in subdirectories). Options 2, 3, and 4 are wrong because .filter(), .find(), and .list() are not Path methods in pathlib—these names don't exist. The correct method is .glob(). Common patterns: '*.txt' (all .txt files), 'test_*' (files starting with test_), '**/*.py' (all .py files recursively in subdirectories). .glob() returns an iterator, so wrap in list() to get all results: list(path.glob('*.txt')). This is covered in Lesson 3: Finding Files with Glob Patterns.",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "A user provides '../../../etc/passwd' as input. How do you prevent reading files outside your 'notes/' directory?",
      options: [
        "String replace all '..' with empty string before processing",
        "Resolve path, verify with is_relative_to(base_dir) before accessing",
        "Reject paths containing dots, slashes, or special characters",
        "Trust user input after checking file extension is valid"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. The secure pattern: (base_dir / user_input).resolve() converts relative path to absolute (resolving '..' and '.'), then .is_relative_to(base_dir.resolve()) checks if the final path is still within base_dir. If not, reject it. This prevents path traversal attacks. Option 2 (string replace '..') is EASILY bypassed: '....//....' reduces to '../..' after replacing once, or attackers use URL encoding. Option 3 (reject dots/slashes) breaks legitimate use—users need dots and slashes for subdirectories like 'work/project.txt'. Option 4 (trust based on extension) is completely insecure—'../../../etc/passwd.txt' has valid extension but escapes the directory. ALWAYS validate with .resolve() + .is_relative_to(). Path security is covered in Lesson 3: Path Traversal Security and Validation.",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "What's the difference between Path.parent and Path.parents? How do you access a grandparent directory?",
      options: [
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. The .parent property gives the immediate parent directory (one level up). The .parents property (note the 's') is a sequence of ALL ancestor directories: parents[0] is parent, parents[1] is grandparent, parents[2] is great-grandparent, etc. For grandparent, use path.parents[1] OR path.parent.parent (both work). Option 2 is wrong; both are Path objects, not strings. Option 3 incorrectly describes the difference and invents a non-existent method. Option 4 is wrong; parent vs parents isn't about relative vs absolute, and .ancestor() doesn't exist. Example: Path('/a/b/c/file.txt').parent is '/a/b/c', .parents[0] is '/a/b/c', .parents[1] is '/a/b', .parents[2] is '/a'. This is covered in Lesson 3: Path properties.",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "You're organizing files by year. How do you create the structure 'backups/2025/january/data/' ensuring no errors if 'backups' already exists?",
      options: [
        "Use mkdir() on full path; catch FileExistsError; continue",
        "Loop through each level creating directories individually with exists check",
        "Use Path('backups/2025/january/data').mkdir(parents=True, exist_ok=True)",
        "Check if each parent exists first; create only missing ones"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. The combination mkdir(parents=True, exist_ok=True) creates all missing parent directories AND doesn't error if any part already exists. This is the clean, one-line solution. parents=True creates 'backups' then '2025' then 'january' then 'data'. exist_ok=True allows the call to succeed even if 'backups' already exists. Option 2 (manual loop) works but is tedious and error-prone—why write 10 lines when one line suffices? Option 3 (catch FileExistsError) works but is less clean than exist_ok=True. Option 4 (check existence first) wastes system calls—mkdir with exist_ok=True is atomic and cleaner. Always use mkdir(parents=True, exist_ok=True) for creating nested directories. This best practice is covered in Lesson 3: Creating Nested Directories safely.",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "When asking AI to generate path handling code, which prompt produces the most portable, maintainable result?",
      options: [
        "Specify exact paths: use 'data/notes/work.txt' format",
        "Describe structure: organize files by category using pathlib",
        "Request string concatenation: join paths with forward slashes",
        "Ask for os.path: use traditional path module instead"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. Describing the STRUCTURE ('organize by category') and specifying 'using pathlib' gives AI the context to generate cross-platform code with Path objects and the / operator. The AI will create Path('data') / category / filename patterns that work everywhere. Option 2 (exact paths with '/') hardcodes Unix separators—code breaks on Windows. Option 3 (string concatenation with '/') is the OLD, platform-specific approach—exactly what pathlib solves. Option 4 (os.path) is outdated; pathlib is the modern standard. For path operations, always prompt for pathlib and describe the directory structure you want. The AI will generate portable code using Path('/') syntax. This AI workflow is covered in Lesson 3: pathlib for cross-platform development.",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "You use Path('file.txt').resolve() on a relative path. What does resolve() return, and why is it useful for security?",
      options: [
        "Returns absolute path resolving symlinks and dots; enables boundary validation",
        "Returns canonical form removing extra slashes for cleaner paths",
        "Returns normalized path converting backslashes to forward slashes always",
        "Returns secure path removing special characters before file operations"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. The .resolve() method converts relative paths to absolute, resolves symbolic links to their targets, and normalizes '.' (current dir) and '..' (parent dir). For security, this is critical: attacker provides '../../../etc/passwd', resolve() converts it to '/etc/passwd' (absolute), then you check if that's within your allowed base directory with .is_relative_to(). Without resolve(), '..' tricks bypass simple string checks. Option 2 is incomplete; resolve() does MORE than just remove slashes. Option 3 is wrong; resolve() uses OS-native separators (backslashes on Windows), not always forward slashes. Option 4 is wrong; resolve() doesn't 'remove special characters', it canonicalizes the path. Always use resolve() before security checks. This is covered in Lesson 3: Path security with resolve().",
      source: "Lesson 3: Cross-Platform Path Handling with pathlib"
    },
    {
      question: "You're storing employee data: name, email, department, start_year. Should you use CSV or JSON, and why?",
      options: [
        "Use CSV because it's human-readable and Excel-compatible",
        "Use JSON because it supports string and number types",
        "Use CSV because data is tabular with flat structure",
        "Use JSON because it's the standard for data storage"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. Employee data with name/email/department/start_year is TABULAR (rows of employees, columns of attributes) with FLAT structure (no nesting—each employee is a single row). CSV is perfect for this: compact, Excel-compatible, simple. Option 2 (JSON supports types) is true but irrelevant—CSV handles strings and numbers fine (you convert after reading). Option 3 is partially right (Excel compatibility is a pro) but doesn't explain WHY CSV fits—the key is 'tabular structure'. Option 4 is wrong; JSON is standard for APIs and nested data, but not 'the' standard for all storage—CSV dominates tabular data. If you later need nested data (employees with multiple addresses), switch to JSON. For flat tabular data, CSV wins. This decision framework is covered in Lesson 4: Choosing Between CSV and JSON.",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "You're reading a CSV file. What's the key difference between csv.reader and csv.DictReader?",
      options: [
        "reader uses indexes; DictReader uses line numbers for access",
        "reader is faster; DictReader includes header parsing overhead",
        "reader handles any CSV; DictReader requires specific field count",
        "reader returns tuples; DictReader returns dictionaries with column names"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. csv.reader yields each row as a list/tuple where you access values by index: row[0], row[1]. csv.DictReader automatically reads the first row as headers and yields each subsequent row as a dictionary: row['name'], row['email']. DictReader is self-documenting and robust to column reordering. Option 2 (speed) is misleading—DictReader has trivial overhead; the clarity is worth it. Option 3 is wrong; both handle variable column counts. Option 4 is nonsense; DictReader uses column names from headers, not line numbers. Professional recommendation: use csv.DictReader for production code—accessing row['name'] is clearer than row[2]. This comparison is covered in Lesson 4: The csv module (reader vs DictReader).",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "When writing CSV files, you forget newline='' in open(). What problem occurs on Windows?",
      options: [
        "Extra blank lines appear between rows in output",
        "File writes without newlines creating single-line output",
        "PermissionError occurs because mode is incorrect",
        "Data corrupts due to encoding mismatch"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. On Windows, omitting newline='' causes Python's text mode to translate \\n to \\r\\n (Windows line endings), but csv.writer also adds \\n, resulting in \\r\\n\\n (double newlines) which displays as blank lines between rows in editors. The fix: with open(file, 'w', newline='', encoding='utf-8'). This tells Python 'don't translate newlines, csv.writer handles it'. Option 2 is wrong; the problem is TOO MANY newlines, not missing ones. Option 3 is wrong; PermissionError is unrelated to newline parameter. Option 4 is wrong; encoding issues are separate. This platform-specific quirk (blank lines on Windows) is why newline='' is standard for CSV writing. This detail is covered in Lesson 4: Writing CSV Files.",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "You're saving JSON with emoji and accented characters. Which parameters ensure they display correctly in the file?",
      options: [
        "Use encoding='unicode' in open() and ascii=True in dump()",
        "Use encoding='utf-8' in open() and ensure_ascii=False in dump()",
        "Use encoding='utf-8' in open() with default JSON settings",
        "Use ensure_ascii=True to preserve international characters automatically"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. You need BOTH: encoding='utf-8' in open() tells Python to write UTF-8 bytes, and ensure_ascii=False in json.dump() tells JSON to NOT escape non-ASCII characters (default escapes emoji as \\uXXXX). Without encoding='utf-8', Python might default to ASCII or locale-specific encoding (crashes on emoji). Without ensure_ascii=False, emoji becomes \\ud83d\\udc4b (escaped, ugly, unreadable). Option 2 is wrong; 'unicode' isn't a valid encoding name. Option 3 is wrong; default JSON settings have ensure_ascii=True (escapes international chars). Option 4 is backwards; ensure_ascii=True ESCAPES characters, False preserves them. Always use both parameters for international text. This is covered in Lesson 4: Character Encoding (UTF-8).",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "You load a JSON file but it contains syntax errors (missing bracket). Which exception does json.load() raise?",
      options: [
        "Raises SyntaxError showing parsing failure location",
        "Raises ValueError indicating malformed JSON structure",
        "Raises JSONDecodeError with line number and error position",
        "Raises FormatError with message describing invalid JSON"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. Python 3 raises json.JSONDecodeError when JSON parsing fails. This exception includes helpful attributes: .msg (error message like 'Expecting property name'), .lineno (line number), .colno (column number), .pos (character position). This helps you locate the error. Option 2 is wrong; JSONDecodeError is the specific exception (it inherits from ValueError, but you should catch JSONDecodeError specifically). Option 3 is wrong; SyntaxError is for Python code syntax, not JSON. Option 4 is wrong; FormatError doesn't exist. When handling JSON loading, catch JSONDecodeError specifically: except json.JSONDecodeError as e: print(f'JSON error at line {e.lineno}'). This error handling is covered in Lesson 4: Handling Format-Specific Exceptions.",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "You're exporting API data that includes nested objects (user with multiple addresses). Which format handles this naturally, and why?",
      options: [
        "JSON for nested data; CSV flattens to multiple rows",
        "CSV handles nesting by creating separate columns per level",
        "Both formats support nesting; choose by preference",
        "JSON handles nesting naturally with objects and arrays"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. JSON was designed for hierarchical data: objects can contain other objects and arrays. Example: {'user': 'Alice', 'addresses': [{'street': '123 Main'}, {'street': '456 Oak'}]} naturally represents nesting. Option 2 is wrong; CSV CAN'T represent nesting without hacks (like separate tables or JSON strings in cells). Option 3 is wrong; CSV fundamentally doesn't support nesting—it's flat rows and columns. Option 4 is partially right (JSON for nested, CSV flattens) but incomplete—CSV doesn't 'flatten to multiple rows' automatically; you'd need manual denormalization (repeating user info for each address row). For any nested/hierarchical data (API responses, config with sections), JSON is the clear choice. This is covered in Lesson 4: Understanding JSON structure.",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "You use csv.DictWriter to export data. What happens if you forget to call writeheader()?",
      options: [
        "DictWriter automatically infers headers from first row",
        "CSV file has data but no header row",
        "Python raises MissingHeaderError before writing data",
        "Data writes with column names prepended automatically"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. Calling writeheader() is OPTIONAL but recommended—it writes the header row (column names) as the first line. If you forget it, writerows() still writes data rows, but WITHOUT the header. The file works but isn't self-documenting—readers don't know what columns represent. Option 2 is wrong; DictWriter doesn't auto-write headers, you must call writeheader() explicitly. Option 3 is wrong; Python doesn't enforce headers, you choose whether to include them. Option 4 is wrong; data rows don't include column names automatically. Best practice: ALWAYS call writeheader() after creating DictWriter—it makes CSVs readable and Excel-compatible. This is covered in Lesson 4: Writing CSV Files with csv.DictWriter.",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "When prompting AI to convert between CSV and JSON, which approach ensures international characters aren't corrupted?",
      options: [
        "Specify: use UTF-8 encoding and ensure_ascii=False for output",
        "Request: handle special characters with escape sequences automatically",
        "Ask for: default encoding for maximum compatibility",
        "Describe: preserve all data without specifying encoding details"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Being EXPLICIT about encoding prevents issues: 'use UTF-8 encoding for file I/O and ensure_ascii=False for JSON output'. This tells AI exactly how to handle international characters (café, José, emoji). The AI will generate: open(file, 'r', encoding='utf-8') for input and json.dump(data, f, ensure_ascii=False) for output. Option 2 (escape sequences) is what you DON'T want—escaping makes data unreadable. Option 3 ('default encoding') is dangerous—defaults vary by platform (ASCII on some, UTF-8 on others). Option 4 (vague 'preserve all') doesn't give AI enough guidance—it might use defaults that fail. For international data, be explicit in prompts about UTF-8 + ensure_ascii=False. This workflow is covered in Lesson 4: Encoding best practices.",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "You're storing application configuration with sections (database, cache, logging). Which format is better, and why?",
      options: [
        "CSV because it's simpler for non-programmers",
        "CSV because configuration is structured data",
        "JSON because it's human-readable and editable",
        "JSON because it supports nested objects for sections"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Configuration naturally has HIERARCHY: sections (database, cache) contain settings (host, port, timeout). JSON represents this perfectly: {'database': {'host': 'localhost', 'port': 5432}, 'cache': {'timeout': 300}}. Option 2 is wrong; while config is 'structured', it's NESTED structure, which CSV can't represent. Option 3 is partially right (JSON is readable/editable) but doesn't explain WHY—the key is hierarchy. Option 4 is backwards; JSON is actually EASIER for non-programmers to edit than CSV for hierarchical data (you can see the structure). CSV is only simpler for flat tabular data. For config files with sections, always use JSON (or YAML/TOML which also support hierarchy). This decision pattern is covered in Lesson 4: Format selection.",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "You ask AI to load CSV but it generates code that crashes on files with commas in values. What did the AI miss?",
      options: [
        "CSV format requires semicolon delimiter for comma-containing values",
        "AI forgot to escape commas before parsing lines",
        "CSV module handles quoted values automatically; AI used string splitting",
        "AI should detect commas and switch to pipe delimiter"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. The csv module AUTOMATICALLY handles commas within quoted values: 'Johnson, Jr.',25,Engineering parses correctly because 'Johnson, Jr.' is quoted. If the AI used naive string splitting (line.split(',')), it would break on commas in values. The fix: always use csv.reader or csv.DictReader instead of manual splitting. Option 2 is wrong; you don't 'escape' commas in CSV, you quote the field. Option 3 is wrong; semicolon is a different dialect, not required for comma-containing values. Option 4 is wrong; CSV doesn't auto-switch delimiters—if you need pipes, specify dialect. When asking AI for CSV parsing, say 'use csv.reader to handle quoted fields correctly'. This is covered in Lesson 4: Understanding CSV format and edge cases.",
      source: "Lesson 4: Structured Data Formats (CSV and JSON)"
    },
    {
      question: "Your note-taking app stores each note as a separate JSON file. What's the key advantage of this approach versus one large JSON file?",
      options: [
        "Separate files consume less disk space overall",
        "Separate files allow modifying one note without rewriting all",
        "Separate files load faster when accessing single note",
        "Separate files prevent data corruption if one fails"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. With one-file-per-note, updating a note means reading and writing just that file (e.g., 2KB), not the entire database (e.g., 500KB for 100 notes). This is more efficient for small updates. Option 2 is actually WRONG; separate files usually consume MORE disk space (filesystem overhead per file). Option 3 is true but secondary—the main win is write efficiency. Option 4 is partially true (isolated corruption) but not the primary advantage. For 10-50 notes, file-per-note scales well. For 10,000+ notes, you'd want a database. But for CLI apps with moderate data, separate files provide efficient CRUD operations without database complexity. This design pattern is covered in Lesson 5: Application Architecture for note-taking app.",
      source: "Lesson 5: Capstone - Note-Taking App"
    },
    {
      question: "In a menu-driven CLI, why is 'while True' with 'break' preferred over 'for i in range(10)' for the main loop?",
      options: [
        "while True allows unlimited iterations until explicit exit",
        "for loops consume more memory with large range values",
        "while True provides better exception handling capabilities",
        "for loops require knowing iteration count in advance"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. A menu loop should run indefinitely until the user chooses 'Exit'—you don't know how many operations they'll perform. while True runs until explicit break (when user selects exit option). Option 4 is also correct reasoning but option 1 is more direct. Option 2 is wrong; range(10) doesn't consume much memory, and you'd need range(999999) for a menu anyway (absurd). Option 3 is wrong; loop type doesn't affect exception handling—you can catch exceptions in either loop. The key insight: menu loops are UNBOUNDED (run until exit), not BOUNDED (10 iterations). for loops are for known iteration counts (process 100 files); while loops are for unknown counts (menu until exit). This pattern is covered in Lesson 5: Menu Loop Implementation.",
      source: "Lesson 5: Capstone - Note-Taking App"
    },
    {
      question: "Your app's search function finds notes by keyword. Should you load all notes into memory once at startup or load on each search?",
      options: [
        "Load on search but cache results for future searches",
        "Load at startup once to optimize search performance",
        "Load on each search for 10-50 notes; negligible overhead",
        "Load at startup into database for efficient querying"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. For small-scale apps (10-50 notes, ~100KB total), loading all notes on each search is SIMPLE and has negligible overhead (<10ms). Premature optimization complicates code without meaningful benefit. Option 2 (load at startup) adds complexity: you need to track which notes change, reload on updates, etc. Option 3 (caching) is over-engineering for this scale—cache invalidation is hard. Option 4 (database) is massive overkill for 50 notes. The lesson: choose SIMPLE solutions for small-scale problems, optimize only when you have evidence of performance issues (1000+ notes). Simplicity beats premature optimization. This design philosophy is covered in Lesson 5: Performance considerations for CLI apps.",
      source: "Lesson 5: Capstone - Note-Taking App"
    },
    {
      question: "When implementing CRUD operations, which error handling layer is most important: input validation, file operations, or data parsing?",
      options: [
        "Data parsing handles corrupted storage",
        "Input validation prevents most issues early",
        "File operations catch system-level errors",
        "All three layers are equally critical for robustness"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. Robust applications handle errors at ALL layers: (1) Input validation catches user mistakes early ('title is empty'), (2) File operations catch system errors (disk full, permissions), (3) Data parsing catches corruption (malformed JSON). Each layer catches different failure modes. Option 2 (input validation most important) is wrong—validation can't catch disk failures. Option 3 (file operations) is wrong—doesn't catch corrupt JSON. Option 4 (data parsing) is wrong—doesn't catch missing files. Defense in depth: validate early (input), handle gracefully (file I/O), recover when possible (parsing). Each layer provides value. This multi-layer approach is covered in Lesson 5: Application-Level Error Handling.",
      source: "Lesson 5: Capstone - Note-Taking App"
    },
    {
      question: "Your delete function removes a note. Should you ask for confirmation before deletion, and why or why not?",
      options: [
        "No, trust user intent and implement undo instead",
        "Yes, require confirmation because deletion is irreversible",
        "Yes, but only for notes older than threshold",
        "No, store deleted notes in trash for recovery"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. For destructive operations (delete, overwrite), ALWAYS confirm with the user: 'Delete \"Important Note\"? (yes/no)'. This prevents accidental data loss from typos or misclicks. Confirmation is simple UX that saves users from disasters. Option 2 (undo instead of confirm) is good UX but complex to implement—you need deleted item history, restore logic, etc. For a CLI capstone, confirmation is simpler and sufficient. Option 3 (confirm only old notes) is arbitrary—users care about ALL their data, not just old data. Option 4 (trash folder) is great UX but adds complexity. Start with simple confirmation, add trash/undo later if needed. Confirmation is covered in Lesson 5: CRUD implementation best practices.",
      source: "Lesson 5: Capstone - Note-Taking App"
    },
    {
      question: "You organize notes in subdirectories by category (work/, personal/). What happens if a user creates a note with category 'projects' that doesn't exist yet?",
      options: [
        "Create the directory automatically with mkdir(parents=True, exist_ok=True)",
        "Raise error asking user to create category first",
        "Default to 'uncategorized' category for unknown categories",
        "Store note in root directory without category folder"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Best UX: automatically create missing category directories when saving notes. Your save_note() function should: category_dir = BASE_DIR / category; category_dir.mkdir(parents=True, exist_ok=True). This is helpful—why force users to manually create directories? Option 2 (raise error) is poor UX—why make users do extra steps? Option 3 (default to uncategorized) loses user intent—they SAID 'projects', honor that. Option 4 (store in root) loses organization. The principle: be helpful, not pedantic. If user specifies a category, create the directory automatically. This pattern (automatic directory creation) is covered in Lesson 5: Data organization with pathlib.",
      source: "Lesson 5: Capstone - Note-Taking App"
    },
    {
      question: "Your list_notes() function groups notes by category. Which data structure is best for this grouping operation?",
      options: [
        "Use set of categories with separate list of notes",
        "Use list of tuples pairing categories with note lists",
        "Use nested list with category at first index position",
        "Use dictionary mapping category name to list of notes"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 1. A dictionary mapping category (string) → list of notes is perfect: {'work': [note1, note2], 'personal': [note3]}. You can iterate: for category, notes in grouped.items(), and access is O(1) by category name. Option 2 (list of tuples) works but is less convenient—no name-based access, must iterate to find category. Option 3 (nested list) is confusing and doesn't support name-based access. Option 4 (set of categories + separate list) loses the association between category and notes. When grouping data by key, use dict[key_type, list[value_type]]. This is Pythonic and efficient. This data structure choice is covered in Lesson 5: Organizing data structures.",
      source: "Lesson 5: Capstone - Note-Taking App"
    },
    {
      question: "When searching notes by keyword, should you match case-sensitively ('Python' != 'python') or case-insensitively?",
      options: [
        "Case-insensitive for titles; case-sensitive for body text",
        "Case-sensitive matching is faster and more accurate",
        "Case-insensitive matching provides better user experience",
        "Let user specify case sensitivity with search flag"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 1. Users expect search to 'just work'—if they search for 'python', they want results with 'Python', 'PYTHON', etc. Implement with keyword.lower() in note.get('title', '').lower(). This is standard search UX. Option 2 (case-sensitive is faster) is wrong—the performance difference (.lower() call) is trivial (<1ms), and accuracy is WORSE (misses relevant results). Option 3 (different rules for title/body) is inconsistent UX—confusing. Option 4 (user flag) adds complexity—for a simple CLI, default to case-insensitive without options. Advanced apps can add flags later, but start with helpful defaults. Case-insensitive search is covered in Lesson 5: Search and filter implementation.",
      source: "Lesson 5: Capstone - Note-Taking App"
    },
    {
      question: "Your app loads notes with glob('**/*.json'). What does '**/' mean in this glob pattern?",
      options: [
        "Matches directories recursively at any depth below base",
        "Matches exactly two directory levels from base directory",
        "Matches hidden directories starting with dot characters",
        "Matches all files regardless of depth or extension"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. The pattern **/ (double star slash) means 'match directories recursively at any depth'. So notes/**/*.json finds all .json files in notes/, notes/work/, notes/work/2025/, etc.—any depth. This is useful for hierarchical storage. Option 2 is wrong; ** doesn't mean 'exactly two levels', it means 'any depth including zero levels'. Option 3 is wrong; ** doesn't specifically match hidden directories (though it includes them). Option 4 is wrong; the full pattern **/*.json matches files at any depth but ONLY .json files (extension matters). The recursive glob ** is very powerful for finding files in nested structures. This is covered in Lesson 5: Using glob patterns for file discovery.",
      source: "Lesson 5: Capstone - Note-Taking App"
    },
    {
      question: "When prompting AI to implement CRUD operations, which approach produces the most maintainable code?",
      options: [
        "Provide complete CRUD example; AI copies pattern exactly",
        "Describe each operation's purpose; AI generates consistent functions",
        "Request minimal code; AI optimizes for brevity",
        "Specify implementation details; AI follows instructions literally"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 1. Describing PURPOSE ('create new note with validation', 'update existing note preserving ID', 'delete with confirmation') lets AI generate well-structured, consistent functions. The AI understands the domain and produces idiomatic code. Option 2 (provide example) works but you're doing the design work—better to let AI generate from requirements. Option 3 (minimal code) often sacrifices clarity and error handling for brevity—bad for maintenance. Option 4 (specify details) is micromanaging—you lose AI's ability to suggest best practices. For CRUD operations, describe WHAT each operation does, let AI determine HOW. This leverages AI's knowledge of patterns. This AI workflow is covered in Lesson 5: Collaborative development with AI.",
      source: "Lesson 5: Capstone - Note-Taking App"
    }
  ]}
/>
