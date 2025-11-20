---
sidebar_position: 6
title: "Chapter 22: Exception Handling Quiz"
---

# Chapter 22: Exception Handling Quiz

Test your understanding of exception handling fundamentals, control flow patterns, custom exceptions, error handling strategies, and building robust programs that handle errors gracefully.

<Quiz
  title="Chapter 22: Exception Handling Assessment"
  questions={[    {
      question: "You're building a user registration function. The user enters 'twenty-five' for age. Which exception type will `int('twenty-five')` raise?",
      options: [
        "ValueError because the string isn't a valid integer",
        "TypeError because int() expects numbers not text",
        "AttributeError because strings lack numeric conversion method",
        "ZeroDivisionError because the conversion fails mathematically"
      ],
      correctOption: 0,
      explanation: "The correct answer is ValueError. The int() function accepts strings as input (so no TypeError), but raises ValueError when the string doesn't represent a valid integer. TypeError occurs when you use wrong types for operations (like 5 + 'text'), not for failed conversions. AttributeError happens when accessing missing attributes. ZeroDivisionError is specific to division by zero, unrelated to type conversion.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "When reading a Python traceback after a crash, which strategy helps you identify where the error actually occurred?",
      options: [
        "Read top-to-bottom to follow the execution path chronologically",
        "Read bottom-to-top starting with exception type and location",
        "Search for the first line containing your filename",
        "Look for the longest line since detailed errors matter"
      ],
      correctOption: 1,
      explanation: "The correct answer is reading bottom-to-top. Python tracebacks show the exception type and message on the last line, which tells you what went wrong. The second-to-last line shows where the error occurred. Working upward shows the call chain. Reading top-to-bottom is less efficient because you don't know what you're looking for. While finding your filename helps, the exception message (at bottom) is the starting point. Line length doesn't indicate error importance.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "You're processing API responses that might have missing required fields. Which exception type best represents this validation failure?",
      options: [
        "AttributeError because response objects lack required attributes",
        "IndexError because required fields aren't at expected positions",
        "KeyError because dictionary keys are missing from response",
        "RuntimeError because the error happens during program execution"
      ],
      correctOption: 2,
      explanation: "The correct answer is KeyError. When accessing dictionary keys that don't exist (response['missing_field']), Python raises KeyError. This is the standard exception for missing dictionary keys. IndexError applies to list/tuple indices, not dictionary keys. AttributeError is for missing object attributes (obj.missing_attr), not dictionary keys. RuntimeError is a generic exception type that doesn't communicate the specific problem (missing key).",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "Your function catches ValueError from int() conversion. What information should you include in the error message to help debugging?",
      options: [
        "Only the exception type since developers know what ValueError means",
        "A generic message saying conversion failed without specifics",
        "Python version number and system information for context",
        "The actual input value that caused conversion to fail"
      ],
      correctOption: 3,
      explanation: "The correct answer is including the actual input value. Error messages should answer 'what failed and why' by showing context. Knowing the input was 'twenty-five' instead of '25' immediately reveals the problem. Just stating 'ValueError' doesn't help locate the issue. Python version is irrelevant for logic errors like this. Generic messages ('conversion failed') don't provide actionable debugging information. Good error messages include: what operation failed, what input caused it, and ideally what was expected.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "You're writing a calculator function. What's the key difference between catching ZeroDivisionError versus preventing division by zero with an if statement?",
      options: [
        "Conditionals prevent errors; exceptions handle errors after occurring",
        "Exception handling executes faster than conditional checks always",
        "Exceptions work for all errors; conditionals work selectively",
        "Both approaches are identical in functionality and performance"
      ],
      correctOption: 0,
      explanation: "The correct answer is that conditionals prevent errors while exceptions handle them. Using `if denominator != 0:` checks before the operation, avoiding the exception entirely (LBYL: Look Before You Leap). Using try/except performs the operation and handles failure if it occurs (EAFP: Easier to Ask Forgiveness than Permission). Python prefers EAFP because exceptions aren't slow for the success case. Exceptions aren't faster than conditionals. Exceptions don't handle all math errors (OverflowError, FloatingPointError exist separately). The approaches differ in philosophy: prevention vs. handling.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "When does the else block execute in a try/except/else/finally structure?",
      options: [
        "Always after except block finishes handling the caught exception",
        "Only when try block completes without raising any exceptions",
        "Only when finally block completes its cleanup operations successfully",
        "When try block raises exception but except doesn't catch"
      ],
      correctOption: 1,
      explanation: "The correct answer is when try completes without exceptions. The else block represents the 'success path'—code that should only run when no exceptions occurred in try. If an exception is raised and caught, except runs but else is skipped. The finally block always runs regardless of exceptions. If try raises an uncaught exception, else doesn't run (exception propagates after finally). This separation clarifies intent: try contains risky code, except handles errors, else handles success, finally guarantees cleanup.",
      source: "Lesson 2: Except, Else, and Finally"
    },
    {
      question: "You're processing a file. Where should file.close() go to guarantee it runs even if processing raises an exception?",
      options: [
        "In the else block after successful file processing",
        "In the except block to close on errors",
        "In the finally block so cleanup always occurs",
        "In the try block right before processing ends"
      ],
      correctOption: 2,
      explanation: "The correct answer is the finally block. Finally executes regardless of whether try succeeds, except catches an error, or an exception propagates uncaught. This guarantees the file closes. Putting close() in else means it won't run if an exception occurs (leaving file open). Putting it in except means it won't run on success. Putting it at the end of try means it won't execute if an exception occurs before that line. The finally block is specifically designed for cleanup operations that must happen no matter what.",
      source: "Lesson 2: Except, Else, and Finally"
    },
    {
      question: "What happens if both a try block and finally block contain return statements?",
      options: [
        "Python raises SyntaxError because multiple returns are invalid",
        "Python returns a tuple containing both return values",
        "The try block return executes; finally return is ignored",
        "The finally block return value overrides try block return"
      ],
      correctOption: 3,
      explanation: "The correct answer is that finally's return overrides try's return. When finally contains a return, it replaces any return value from try or except blocks. This is dangerous because finally is meant for cleanup, not altering control flow. It hides the intended return value and makes debugging difficult. This is valid Python (no SyntaxError), but considered bad practice. Professional code avoids return statements in finally blocks. The correct pattern: return from try/except, use finally only for cleanup without return.",
      source: "Lesson 2: Except, Else, and Finally"
    },
    {
      question: "You're reading a config file with JSON.load(). The file exists but contains invalid JSON. Which two exceptions should you catch in separate except blocks?",
      options: [
        "IOError for file access; ValueError for data issues",
        "FileNotFoundError for missing file; JSONDecodeError for invalid content",
        "PermissionError for access rights; TypeError for wrong formats",
        "OSError for system errors; AttributeError for missing methods"
      ],
      correctOption: 1,
      explanation: "The correct answer is FileNotFoundError and JSONDecodeError. FileNotFoundError handles missing files specifically. JSONDecodeError (a subclass of ValueError) handles malformed JSON content. Catching these separately allows different recovery strategies: missing file might use defaults, corrupted JSON might log error and fail. IOError is outdated (replaced by OSError). PermissionError and TypeError don't match the failure modes. OSError is too broad (includes many unrelated errors). AttributeError isn't raised by json.load(). Specific exceptions enable precise error handling.",
      source: "Lesson 2: Except, Else, and Finally"
    },
    {
      question: "When reviewing code with try/except/else/finally, how do you verify the finally block executes in all scenarios?",
      options: [
        "Test success case, caught exception case, uncaught exception case",
        "Only test the success path since finally always runs",
        "Check that finally contains a return statement to guarantee",
        "Verify finally appears last in the code block structure"
      ],
      correctOption: 0,
      explanation: "The correct answer is testing all three scenarios: success (no exception), caught exception, and uncaught exception. While finally is guaranteed to run, testing proves it handles all cases correctly (especially cleanup with different variable states). Testing only success doesn't verify finally runs after exceptions. Return statements in finally are bad practice, not a verification method. Finally's position in code doesn't prove it executes correctly. Comprehensive testing ensures finally's cleanup logic works regardless of whether try succeeds, except catches, or exceptions propagate.",
      source: "Lesson 2: Except, Else, and Finally"
    },
    {
      question: "Why would you create a custom InvalidEmailError exception instead of using built-in ValueError for email validation?",
      options: [
        "Custom exceptions execute faster than built-in exception types",
        "ValueError can't be used for string validation purposes",
        "Built-in exceptions require administrative privileges to raise in code",
        "Custom exceptions communicate domain-specific intent and enable targeted catching"
      ],
      correctOption: 3,
      explanation: "The correct answer is that custom exceptions communicate domain intent and enable targeted catching. InvalidEmailError tells readers exactly what failed (email validation), while ValueError is generic. Callers can catch InvalidEmailError specifically to handle email issues differently from other ValueError causes. Custom exceptions aren't faster (slightly slower due to extra class). ValueError works fine for validation, but doesn't specify what kind. No exceptions require special privileges. Custom exceptions add semantic meaning, enabling precise error handling strategies based on business logic.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "You're designing authentication exceptions. What's the benefit of creating an AuthenticationError base class with subclasses for specific failures?",
      options: [
        "Base classes reduce memory usage by sharing exception code",
        "Python requires base classes for all custom exception hierarchies",
        "Callers can catch all auth errors broadly or specific types",
        "Base classes prevent the same exception from being raised twice"
      ],
      correctOption: 2,
      explanation: "The correct answer is enabling flexible catching. With a base AuthenticationError class and subclasses (UserNotFoundError, InvalidPasswordError), callers choose granularity. `except AuthenticationError` catches all authentication failures generically. `except UserNotFoundError` handles missing users specifically (maybe prompt registration). This flexibility adapts to different contexts. Base classes don't reduce memory (each instance allocates separately). Python doesn't require base classes (you can inherit directly from Exception). Base classes don't prevent re-raising. Exception hierarchies enable both broad and specific error handling strategies.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "When should a custom exception's __init__ method store context as instance attributes (like username, attempt_count)?",
      options: [
        "When callers need programmatic access to error context for recovery",
        "Never; exception messages should contain all needed information",
        "Only when the exception inherits from multiple base classes",
        "Always; Python requires attributes for all custom exception types"
      ],
      correctOption: 0,
      explanation: "The correct answer is when callers need programmatic access. Storing self.username and self.attempt_count lets callers examine context: `except InvalidCredentialsError as e: if e.attempt_count >= 3: lock_account()`. This enables logic-driven recovery beyond just logging messages. Messages alone are for humans, not programmatic decisions. Multiple inheritance doesn't dictate attribute requirements. Python doesn't require attributes (simple custom exceptions just inherit from Exception with no __init__). Attributes enable callers to make decisions based on error context, not just display messages.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "What's the difference between `raise ValueError('Bad age')` and `raise ValueError('Bad age') from original_error`?",
      options: [
        "The from clause prevents original_error from appearing in logs",
        "Using from preserves the original exception as __cause__ for debugging",
        "The from syntax is required when raising exceptions inside except",
        "Both are identical; from is deprecated in Python versions"
      ],
      correctOption: 1,
      explanation: "The correct answer is that from preserves the original exception as __cause__. Exception chaining (`raise ... from e`) links the new exception to the original, showing both in tracebacks. This helps debugging by revealing the root cause alongside business-context errors. Without from, the original exception is lost. From doesn't hide the original (it explicitly preserves it). From isn't required in except blocks (you can raise without it). From is modern Python (3+), not deprecated. Exception chaining communicates 'this business error happened because this technical error occurred first.'",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "You're validating order data. When should validation functions raise exceptions versus returning True/False?",
      options: [
        "Always return booleans since exceptions are slower than conditionals",
        "Return booleans always; exceptions are only for system errors",
        "Use exceptions for user input; booleans for programmatic validation",
        "Raise exceptions when validation failure is exceptional and needs handling"
      ],
      correctOption: 3,
      explanation: "The correct answer is raising exceptions when failures need handling. Pythonic design: functions return validated data or raise exceptions (no ambiguity). Exceptions communicate what went wrong (via error message), while booleans don't explain the failure. Exceptions aren't just for system errors—ValueError is perfect for validation. The user-input distinction doesn't determine the pattern. Exceptions aren't slow for typical use cases. The pattern: `validate_age(age)` returns int or raises ValueError. This is clearer than `is_valid, error_msg = validate_age(age)` because callers must handle failures explicitly.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "Your API client encounters a network timeout. Why is retry logic appropriate for this error but not for a 404 Not Found response?",
      options: [
        "Timeout errors indicate server problems; not found indicates client bugs",
        "HTTP libraries only support retrying timeout errors automatically in code",
        "Network timeouts are transient; retrying may succeed; not found permanent",
        "Retry logic only applies to connection errors not response errors"
      ],
      correctOption: 2,
      explanation: "The correct answer is that timeouts are transient while 404 is permanent. Network timeouts often resolve (congestion clears, connection stabilizes), so retry can succeed. A 404 Not Found means the resource doesn't exist—retry won't change that. This is the transient vs. permanent distinction. HTTP libraries don't automatically retry (you implement it). Timeouts don't always mean server problems (could be network issues). Retry applies to any transient error, whether connection or response (429 rate limit is retriable). Strategy choice depends on error nature: transient → retry, permanent → fail or fallback.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "When implementing retry logic with exponential backoff, why increase wait time between attempts instead of retrying immediately?",
      options: [
        "Exponential delays give overwhelmed services time to recover between attempts",
        "Python requires increasing delays for try/except to work correctly",
        "Immediate retries are faster and more likely to succeed",
        "Exponential backoff prevents exceptions from being raised in code"
      ],
      correctOption: 0,
      explanation: "The correct answer is giving services time to recover. If a service is overloaded, immediate retries worsen the problem (adding to the load). Exponential backoff (1s, 2s, 4s, 8s) spaces requests, allowing recovery. This is respectful of shared resources. Python doesn't require delays for exceptions to work. Immediate retries are faster initially but less likely to succeed if the service needs recovery time. Backoff doesn't prevent exceptions (it spaces retry attempts). The pattern balances persistence (keep trying) with courtesy (don't overwhelm struggling services).",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "You're loading user preferences. The file is missing. What's the difference between retry strategy and fallback strategy for this scenario?",
      options: [
        "Retry attempts reading multiple times; fallback uses defaults immediately",
        "Fallback uses default preferences immediately; retry tries reading repeatedly",
        "Retry logs errors while fallback silently uses defaults without logging",
        "Both strategies are identical when handling missing file errors"
      ],
      correctOption: 1,
      explanation: "The correct answer is that fallback uses defaults immediately while retry attempts repeatedly. For a missing file (permanent error), retry is inappropriate—file won't appear. Fallback recognizes the file is missing and uses defaults immediately, letting the program continue. Retry makes sense for transient errors (file temporarily locked). Logging is orthogonal to strategy choice (both can log). The strategies differ fundamentally: retry assumes the error is temporary, fallback assumes it's permanent and provides an alternative.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "When processing a CSV with 1000 rows, 5 have malformed data. Which strategy lets valid rows succeed while skipping bad ones?",
      options: [
        "Retry logic attempts processing each malformed row multiple times",
        "Fallback strategy replaces malformed rows with default row values",
        "Graceful degradation skips invalid rows and continues processing valid",
        "Fail-fast strategy stops processing immediately on first malformed row"
      ],
      correctOption: 2,
      explanation: "The correct answer is graceful degradation. This strategy continues operation when non-critical failures occur (individual row errors don't stop batch processing). Try/except inside the row loop catches per-row exceptions, logs them, and continues. Retry doesn't help malformed data (won't become valid). Fallback could work but may introduce incorrect data. Fail-fast would stop at row 1, losing 995 valid rows. Graceful degradation balances robustness (don't crash) with data integrity (skip invalid, keep valid) and transparency (log what was skipped).",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "What essential information should error logs include to enable effective debugging of production issues?",
      options: [
        "User passwords and API keys to reproduce exact conditions",
        "Only the exception message since other context is visible",
        "Complete system memory dumps and all variable state always",
        "Exception type, timestamp, context data, and operation attempted successfully"
      ],
      correctOption: 3,
      explanation: "The correct answer is exception type, timestamp, context, and operation. Good logs answer: What failed (exception type/message)? When (timestamp)? What was being attempted (operation)? What data was involved (relevant variables, not all state)? This balance provides debugging information without bloat. Exception messages alone lack context (when did this happen? what data?). Full memory dumps are excessive and slow. Never log passwords or secrets (security violation). Effective logs are selective: enough context to diagnose, not so much they're unusable or reveal sensitive data.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "You catch an exception but need the calling function to know the operation failed. What's the appropriate pattern?",
      options: [
        "Return None and expect caller to check for failure",
        "Log the exception details then re-raise it for caller",
        "Catch the exception and print an error message only",
        "Store exception in global variable caller can check later"
      ],
      correctOption: 1,
      explanation: "The correct answer is log then re-raise. This pattern records the error locally (for debugging), then propagates it to callers (so they can decide how to handle). Re-raising preserves the exception type and traceback. Returning None is ambiguous (None could be valid in some contexts). Printing without re-raising hides failure from callers (they think success occurred). Global variables create coupling and concurrency issues. The pattern: `except SomeError as e: log(e); raise` lets both levels contribute: local logs context, caller decides recovery strategy.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "Your CSV parser encounters FileNotFoundError. What should the error handling strategy be for this scenario versus ValueError on individual rows?",
      options: [
        "FileNotFoundError should stop program; ValueError should skip row and continue",
        "Both should skip and continue since processing can proceed",
        "Both should retry with exponential backoff until successful completion",
        "FileNotFoundError should log only; ValueError should raise immediately upward"
      ],
      correctOption: 0,
      explanation: "The correct answer is FileNotFoundError stops, ValueError skips row. FileNotFoundError is fatal—can't parse a non-existent file. The program should exit with a helpful message. ValueError on a row is non-fatal—bad data in one row doesn't invalidate the whole file. Skip the row, log the error, continue. Continuing without a file makes no sense. Retrying won't make a missing file appear. Logging without stopping wastes CPU parsing nothing. This demonstrates error classification: fatal errors (file access) stop early; non-fatal errors (data validation) degrade gracefully.",
      source: "Lesson 5: Capstone: Robust CSV Parser"
    },
    {
      question: "In validation functions like validate_age(), when should you raise exceptions versus returning validated data?",
      options: [
        "Return None for invalid; return validated data for valid",
        "Always return tuples with success flag and validation errors",
        "Raise exceptions for invalid input; return validated data for valid",
        "Raise exceptions only when Python built-ins fail during validation"
      ],
      correctOption: 2,
      explanation: "The correct answer is raise for invalid, return for valid. This is Pythonic: functions either succeed (return result) or fail (raise exception). No ambiguity. Callers use try/except to handle validation failures. Returning tuples (success, data_or_error) is verbose and error-prone (callers might forget to check success). Returning None is ambiguous (None might be valid in some contexts). Raising only for built-in failures misses validation logic errors. The pattern: `age = validate_age(user_input)` either assigns valid age or raises ValueError. Explicit error handling via exceptions.",
      source: "Lesson 5: Capstone: Robust CSV Parser"
    },
    {
      question: "Your CSV parser processes 100 rows successfully, then hits a PermissionError reading the next chunk. Where should this exception be caught?",
      options: [
        "Not caught; let it crash showing default Python traceback",
        "Inside the row processing loop to skip problematic rows",
        "In the validation functions to handle data access issues",
        "Outside the file operation to halt processing and report"
      ],
      correctOption: 3,
      explanation: "The correct answer is outside file operations. PermissionError is fatal—can't continue reading without permissions. Catch it at the file-opening level, report helpful message, exit gracefully. Catching inside the row loop doesn't make sense (permission applies to the whole file). Validation functions validate data, not file access. Letting it crash shows users raw tracebacks (confusing for non-programmers). The architecture: outer try/except for file operations (fatal), inner try/except for row processing (non-fatal). Each level handles errors at its appropriate scope.",
      source: "Lesson 5: Capstone: Robust CSV Parser"
    },
    {
      question: "When testing exception handling in your CSV parser, which test case is most valuable for verifying robustness?",
      options: [
        "Test with mix of valid and invalid rows throughout",
        "Test with completely empty file to check edge handling",
        "Test with all valid data to confirm normal operation",
        "Test with maximum file size to verify performance limits"
      ],
      correctOption: 0,
      explanation: "The correct answer is mixed valid/invalid data. This tests the core promise: parser handles errors gracefully, processes valid data, skips invalid, and reports accurately. All-valid tests happy path only (doesn't test error handling). Empty file is one edge case (useful, but narrow). Max file size tests performance, not error handling. Mixed data proves graceful degradation: some rows pass validation, others fail, parser continues, summary reflects reality. This simulates production: real data is messy. Robust parsers handle messiness without crashing.",
      source: "Lesson 5: Capstone: Robust CSV Parser"
    },
    {
      question: "What makes an error message helpful for users when your CSV parser encounters validation failures?",
      options: [
        "Show the full Python traceback with stack information",
        "Include row number, field name, actual value, and what expected",
        "Display only the exception type without additional context details",
        "Print generic message that validation failed without specific details"
      ],
      correctOption: 1,
      explanation: "The correct answer is row number, field, value, expectation. Helpful errors answer: Where (row 5)? What field (age)? What was wrong (got 'twenty')? What's valid (0-150)? This lets users fix issues. Tracebacks are for developers, not end users. Exception type alone doesn't help users ('ValueError' doesn't say what to fix). Generic messages frustrate users ('validation failed' doesn't identify the problem). Good messages are actionable, specific, and contextual.",
      source: "Lesson 5: Capstone: Robust CSV Parser"
    },
    {
      question: "You're debugging exception handling. AI generated try/except code that catches Exception instead of specific types. What's the risk?",
      options: [
        "Catching Exception prevents logging from working correctly in code",
        "Exception is too slow; specific types execute faster always",
        "Python raises SyntaxError for overly broad exception catching patterns",
        "Catches too broadly; might hide unexpected bugs that should crash"
      ],
      correctOption: 3,
      explanation: "The correct answer is catching too broadly hides bugs. `except Exception` catches almost everything (except SystemExit, KeyboardInterrupt), including bugs you didn't anticipate. This hides errors that should crash during development (revealing bugs). Specific exceptions communicate intent: 'I expect ValueError here'. Broad catches don't slow execution significantly. Python doesn't raise SyntaxError for this (it's valid). Catching Exception doesn't break logging. Best practice: catch specific exceptions you expect and can handle; let unexpected ones crash during development.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "When reviewing AI-generated exception handling code, you see exceptions caught but not logged. What problem does this create?",
      options: [
        "Python requires logging for all caught exceptions by language",
        "Not logging exceptions makes the program run faster overall",
        "Silent failures hide errors; issues occur without visible evidence",
        "Users prefer silent failures over seeing error messages displayed"
      ],
      correctOption: 2,
      explanation: "The correct answer is silent failures hide errors. Catching exceptions without logging (or re-raising or user notification) means errors occur invisibly. Users think everything worked, but operations failed silently. This makes debugging nearly impossible (no record of what went wrong). Skipping logging doesn't meaningfully improve performance. Python doesn't require logging (but best practice does). Users don't prefer silent failures—they prefer clear communication when issues occur. The rule: never catch exceptions silently. Always log, notify user, or re-raise.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "You ask AI to add exception handling to a file operation. It wraps everything in try/except Exception and continues. How would you improve this?",
      options: [
        "Ask for broader exception catching to handle more error types",
        "Request specific exception types with different handling for each error",
        "Remove exception handling since catching everything is correct approach",
        "Add more nested try/except blocks to catch multiple times"
      ],
      correctOption: 1,
      explanation: "The correct answer is requesting specific exception types. Tell AI: 'Catch FileNotFoundError, PermissionError, and IOError separately. For FileNotFoundError, use default config. For PermissionError, notify user. For IOError, retry twice.' Specific exceptions enable precise recovery strategies. Broader catching makes the problem worse. Removing exception handling leaves crashes. Nested try/except without purpose adds complexity. Good prompt: 'Handle FileNotFoundError (use defaults), PermissionError (exit with message), and JSONDecodeError (log and use defaults) separately.'",
      source: "Lesson 2: Except, Else, and Finally"
    },
    {
      question: "AI generates validation code using assertions instead of raising ValueError. What's wrong with using assert for data validation?",
      options: [
        "Assertions can be disabled with python optimization flags bypassing",
        "Assert statements are slower than raising ValueError in production",
        "Python doesn't support assertions for string or numeric validation",
        "Assertions work fine; this is the recommended validation pattern"
      ],
      correctOption: 0,
      explanation: "The correct answer is assertions can be disabled. Running `python -O` disables assertions (optimization mode), so `assert age > 0` is skipped entirely, bypassing validation. Assertions are for debugging/development checks, not production validation. Use `if not age > 0: raise ValueError()` for validation that can't be disabled. Assertions aren't slower (but inappropriate purpose). Python supports assertions, but they're not for user input validation. Assertions check invariants (things that should never happen if code is correct), not user data validity.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "You describe error handling requirements to AI: 'Handle file errors and data errors.' AI catches Exception broadly. What's a more specific prompt?",
      options: [
        "Handle all errors the same way with single broad catch",
        "Use except Exception for file errors and except Error for",
        "Catch FileNotFoundError and PermissionError for files; ValueError for data",
        "Don't use exception handling; check return codes for errors"
      ],
      correctOption: 2,
      explanation: "The correct answer is specifying exception types explicitly. 'File errors' is ambiguous (FileNotFoundError? IOError? PermissionError?). 'Data errors' is vague (ValueError? TypeError?). Specific prompt: 'Catch FileNotFoundError (file missing, use defaults), PermissionError (notify user, exit), ValueError (invalid data, skip row, log).' This gets precise handling. 'Exception' and 'Error' aren't distinct types. Single broad catch is the problem you're fixing. Return codes aren't Pythonic. Prompt clearly: name exception types, describe recovery strategies.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "What's the difference between catching a specific exception versus its parent class in the exception hierarchy?",
      options: [
        "There's no hierarchy; all exceptions are independent types",
        "Parent classes execute faster than catching specific exception types",
        "Specific exceptions can't be caught; only parent classes work",
        "Specific exceptions enable targeted handling; parent catches all subclasses"
      ],
      correctOption: 3,
      explanation: "The correct answer is specific exceptions enable targeted handling. Catching ValueError handles only ValueError. Catching Exception (parent) catches ValueError, TypeError, all subclasses. Hierarchy enables flexibility: `except ValueError` is precise, `except Exception` is broad. Parent classes aren't faster. All exceptions can be caught specifically. Python has exception hierarchy: BaseException → Exception → ValueError/TypeError/etc. Use specific exceptions when you have targeted recovery; use parent classes when handling broadly. Example: `except ArithmeticError` catches ZeroDivisionError, OverflowError, FloatingPointError.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "When asking AI to implement retry logic, what details ensure correct implementation?",
      options: [
        "Specify max attempts, delay strategy, and which exceptions retry",
        "Just say 'add retry logic' and AI infers all",
        "Provide the complete code yourself instead of using AI",
        "Request retry without specifying exception types to handle everything"
      ],
      correctOption: 0,
      explanation: "The correct answer is specifying max attempts, delays, and exceptions. Good prompt: 'Retry on ConnectionError and Timeout, max 3 attempts, exponential backoff starting at 1 second. Other exceptions propagate immediately.' This gets precise behavior. 'Add retry logic' is ambiguous (retry everything? how many times? which errors?). You should use AI for boilerplate (the retry loop structure), but specify requirements. Retrying everything is incorrect (permanent errors shouldn't retry). Details ensure AI generates code matching your requirements, not generic boilerplate.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "AI suggests using bare except: clause to catch all errors. Why is this problematic?",
      options: [
        "Bare except executes slower than specific exception catching patterns",
        "Catches KeyboardInterrupt and SystemExit preventing clean program shutdown",
        "Python deprecated bare except in recent language versions completely",
        "Bare except only catches Exception class not other types"
      ],
      correctOption: 1,
      explanation: "The correct answer is catching KeyboardInterrupt and SystemExit. Bare `except:` catches everything, including KeyboardInterrupt (Ctrl+C) and SystemExit (exit commands), preventing clean shutdown. Users can't stop the program normally. Use `except Exception:` at minimum (doesn't catch system exits). Bare except isn't faster. It's not deprecated (valid syntax, but discouraged). Bare except catches more than Exception (that's the problem). Best practice: catch specific exceptions. If you must catch broadly, use `except Exception:`, not bare except, to allow system exit signals.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "You're teaching AI about your error handling style. What pattern helps AI generate code matching your project standards?",
      options: [
        "Use AI's default patterns since they're usually correct enough",
        "Tell AI to search GitHub for popular exception patterns",
        "Provide example code showing preferred exception handling patterns clearly",
        "Avoid examples; let AI infer style from vague descriptions"
      ],
      correctOption: 2,
      explanation: "The correct answer is providing example code. Show AI: 'Here's how we handle file operations: [code example with try/except/else/finally, specific exceptions, logging patterns]. Generate similar code for database operations.' Examples communicate style clearly: logging format, exception types, error messages, recovery strategies. AI can't search GitHub during conversation. Defaults might not match your standards (verbosity, logging, recovery strategies). Vague descriptions lead to inconsistent code. Teaching through examples is effective: 'Do it like this example, but for database instead of files.'",
      source: "Lesson 5: Capstone: Robust CSV Parser"
    },
    {
      question: "When should you wrap a whole function in try/except versus using multiple try/except blocks for different operations?",
      options: [
        "Wrapping whole function is always incorrect; never use pattern",
        "Single wrapper always; multiple blocks are redundant and slower",
        "Multiple blocks only when function exceeds 50 lines of",
        "Multiple blocks when operations have different error handling strategies needed"
      ],
      correctOption: 3,
      explanation: "The correct answer is multiple blocks for different strategies. Example: file opening needs try/except for FileNotFoundError (fatal, exit). Row processing needs try/except for ValueError (skip row, continue). Different errors, different handling, different blocks. Single wrapper forces same handling for all errors (inappropriate). Multiple blocks aren't slower meaningfully. Function length doesn't determine structure. Whole-function wrapping is valid when all operations share recovery strategy. The decision: if different operations need different error responses, use separate try/except blocks. If all errors handled identically, one block suffices.",
      source: "Lesson 2: Except, Else, and Finally"
    },
    {
      question: "You're writing a data processing pipeline. Where should you catch exceptions: at each processing stage or in the main orchestrator?",
      options: [
        "Only in main orchestrator to centralize all error handling",
        "Both: stages catch recoverable errors locally; orchestrator catches fatal ones",
        "Only in stages since that's where errors occur directly",
        "Neither; let Python handle all exceptions with default behavior"
      ],
      correctOption: 1,
      explanation: "The correct answer is both levels with different roles. Stages catch expected errors they can handle (invalid data in row: skip and continue). Orchestrator catches unexpected/fatal errors (file missing: can't run pipeline, exit gracefully). This layered approach matches error scope: local errors handled locally, global errors handled globally. Centralizing all handling in orchestrator makes stages fragile (can't handle own errors). Handling only in stages misses pipeline-level failures. Default Python behavior crashes with tracebacks (not user-friendly). Layered exception handling: each level handles errors at appropriate scope.",
      source: "Lesson 5: Capstone: Robust CSV Parser"
    },
    {
      question: "What's the advantage of using context managers (with statement) for file operations compared to manual try/except/finally?",
      options: [
        "Context managers guarantee cleanup even if you forget to",
        "Manual try/except/finally is faster than context managers in Python",
        "Context managers only work for files not other resource types",
        "With statement disables exception handling making code execute faster"
      ],
      correctOption: 0,
      explanation: "The correct answer is guaranteed cleanup. `with open(file) as f:` ensures f.close() runs even if exceptions occur, without explicit finally block. Context managers encapsulate try/finally pattern, reducing boilerplate and preventing cleanup mistakes. Manual try/finally isn't faster (context managers use the same mechanism). Context managers work for many resources (files, locks, database connections, network sockets). With doesn't disable exception handling (exceptions propagate normally). Context managers are syntactic sugar for try/finally, ensuring cleanup happens reliably. Best practice: use with for resources requiring cleanup.",
      source: "Lesson 2: Except, Else, and Finally"
    },
    {
      question: "You're implementing a fallback pattern for missing configuration. Should fallback values be hardcoded in the exception handler or defined elsewhere?",
      options: [
        "Fallback shouldn't use defaults; raise exception to caller instead",
        "Hardcode in exception handler to keep all error logic",
        "Use random values generated at runtime in exception block",
        "Define defaults separately; reference them in exception handler cleanly"
      ],
      correctOption: 3,
      explanation: "The correct answer is defining defaults separately. Put defaults at module level or in a config: `DEFAULT_CONFIG = {'port': 8000, ...}`. Reference in handler: `except FileNotFoundError: return DEFAULT_CONFIG.copy()`. This separates concerns: defaults are configuration, exception handling is control flow. Hardcoding in handlers duplicates defaults if used elsewhere. Random values are nonsensical (fallbacks should be predictable). Raising to caller might be appropriate for libraries, but applications should handle missing config gracefully. Separating defaults improves maintainability (update in one place) and testability (can test defaults independently).",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "When logging exceptions, what's the benefit of including the full traceback versus just the exception message?",
      options: [
        "Tracebacks are required for Python to handle exceptions correctly",
        "Exception messages contain all information needed for debugging always",
        "Tracebacks show call chain helping identify where error originated",
        "Tracebacks are only useful during development not in production"
      ],
      correctOption: 2,
      explanation: "The correct answer is tracebacks show the call chain. Exception message says what went wrong ('division by zero'). Traceback shows where (which function, which line, via what call path). This is critical for debugging production issues where you can't reproduce locally. Messages alone often insufficient (same error can occur in multiple places). Tracebacks aren't required for exception handling (they're for debugging). Tracebacks are especially valuable in production (can't attach debugger). Use logging.exception() or log the traceback: provides context for diagnosing issues in deployed code.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "You're designing a validation function. Should it return None for invalid input or raise ValueError?",
      options: [
        "Return None so callers can check for failure optionally",
        "Raise ValueError to force callers to handle validation failures",
        "Return empty string for invalid input as safe default",
        "Both are equivalent; choice doesn't matter for validation logic"
      ],
      correctOption: 1,
      explanation: "The correct answer is raising ValueError. This forces explicit error handling: callers must use try/except or let exceptions propagate (making errors visible). Returning None is ambiguous (None might be valid). Callers might forget to check, silently propagating invalid data. Empty string is incorrect for non-string validation. The choices aren't equivalent: exceptions make errors explicit and unignorable; None/special returns make errors easy to ignore. Pythonic pattern: functions succeed (return result) or fail loudly (raise exception). No silent failures.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "What's the relationship between exception handling and program performance?",
      options: [
        "Exceptions are fast when not raised; slow only when raised",
        "Exception handling always slows programs down regardless of use",
        "Try/except blocks should be avoided in production for speed",
        "Raising exceptions is faster than returning error codes always"
      ],
      correctOption: 0,
      explanation: "The correct answer is exceptions are fast when not raised. Try/except has minimal overhead when no exception occurs. Raising and catching exceptions is slower (building traceback, unwinding stack), but still acceptable for exceptional cases. Exception handling doesn't always slow programs (only when exceptions raised). Try/except isn't avoided for performance (Pythonic style prefers it). Exceptions aren't faster than return codes when raised, but are more maintainable and explicit. Performance guideline: use exceptions for exceptional cases (not control flow), avoid raising in tight loops. Normal exception handling has negligible performance impact.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "You catch an exception and want to add context before re-raising. What's the correct pattern?",
      options: [
        "Catch exception, print context, let it propagate without re-raising",
        "Catch exception, modify its message attribute, re-raise same instance",
        "Catch exception, raise new exception with 'from e' preserving",
        "Catch exception, store in variable, return variable to caller"
      ],
      correctOption: 2,
      explanation: "The correct answer is raising new exception with 'from e'. Pattern: `except ValueError as e: raise ConfigError('Config invalid') from e`. This preserves the original exception (as __cause__) while adding business context. Exception messages are immutable (can't modify). Printing doesn't add context to the exception object (just logs). Returning exceptions to caller is non-Pythonic (exceptions should be raised). Exception chaining communicates: 'this business error happened because this technical error occurred underneath.' Both exceptions appear in traceback for comprehensive debugging.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "When should error handling include user-facing messages versus technical details?",
      options: [
        "User-facing messages should include full stack traces for clarity",
        "Always show technical details so users can report accurately",
        "Never show users errors; all error handling should be",
        "User-facing for applications; technical details for logs and debugging"
      ],
      correctOption: 3,
      explanation: "The correct answer is user-facing for users, technical for logs. Users see: 'Configuration file not found. Using default settings.' Logs record: FileNotFoundError, file path, traceback. Users don't benefit from tracebacks (confusing, potentially security-revealing). Silent failures frustrate users (they don't know what's wrong). Full tracebacks are for developers, not end users. The pattern: catch exception, show user-friendly message to user, log technical details for debugging. This balances user experience (clear, actionable messages) with debugging needs (detailed context in logs).",
      source: "Lesson 5: Capstone: Robust CSV Parser"
    },
    {
      question: "You're code-reviewing exception handling. You see 'except Exception: pass'. What's the problem and how do you fix it?",
      options: [
        "Too specific; should use bare except to catch everything",
        "Silent failure; fix by logging exception or re-raising it",
        "Missing else clause; add else block for success path",
        "Nothing wrong; this is the standard exception handling pattern"
      ],
      correctOption: 1,
      explanation: "The correct answer is silent failure, fixed by logging or re-raising. `except Exception: pass` catches almost all errors and does nothing—errors occur invisibly. Minimum fix: `except Exception as e: logging.error(f'Error: {e}')` to record what happened. Better: catch specific exceptions you expect. Bare except is worse (catches system exits too). Missing else isn't the issue (pass is the issue). This pattern is never standard in production code. Rule: never catch exceptions silently. Always log, notify user, or re-raise. Silent failures make debugging impossible.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "What's the difference between error prevention (LBYL) and error handling (EAFP) in Python?",
      options: [
        "LBYL checks conditions first; EAFP handles exceptions after attempting operation",
        "LBYL uses exceptions; EAFP uses conditional checks for errors",
        "Both approaches are identical in Python with same performance",
        "LBYL is always faster and preferred in all situations"
      ],
      correctOption: 0,
      explanation: "The correct answer is LBYL checks first, EAFP handles after. LBYL (Look Before You Leap): `if key in dict: value = dict[key]` prevents KeyError. EAFP (Easier to Ask Forgiveness than Permission): `try: value = dict[key] except KeyError: ...` handles KeyError. Python prefers EAFP (exceptions are efficient when not raised, code is cleaner). The approaches aren't identical (different philosophy and code structure). LBYL isn't always faster (checking then acting can be slower than try-optimistic-path). EAFP is more Pythonic, but both have uses: LBYL for predictable conditions, EAFP for exceptional cases.",
      source: "Lesson 1: Exception Fundamentals"
    },
    {
      question: "When writing custom exceptions, should you inherit from Exception or from more specific built-in exceptions like ValueError?",
      options: [
        "Never inherit from built-ins; create independent exception classes always",
        "Always inherit from Exception for all custom exception types",
        "Inherit from ValueError for all custom exceptions since it's",
        "Inherit from Exception for domain errors; specific types for specialized"
      ],
      correctOption: 3,
      explanation: "The correct answer is Exception for domain, specific for specialized. For business errors (UserNotFoundError, OrderFailedError), inherit from Exception. For validation errors semantically similar to built-ins (WeakPasswordError), inherit from ValueError to signal 'this is a value validation error.' This communicates intent: ValueError subclasses are catchable as ValueError. Not all customs should inherit from Exception (loses semantic information). Not all should inherit from ValueError (not all are value validation). Custom exceptions should inherit from standard exception hierarchy but choose appropriate parent based on semantic meaning.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "You're implementing a retry loop with maximum attempts. What should happen when all retries are exhausted?",
      options: [
        "Keep retrying indefinitely until operation eventually succeeds completely",
        "Return None and let caller assume success occurred",
        "Raise the last exception to inform caller retry failed",
        "Log the failure but continue program execution silently"
      ],
      correctOption: 2,
      explanation: "The correct answer is raising the last exception. Pattern: `for attempt in range(max_retries): try: return operation() except TransientError as e: if attempt == max_retries - 1: raise`. This communicates: 'I tried max times, still failed, here's the error.' Returning None hides failure (caller thinks success). Infinite retry risks infinite loops. Logging without raising hides failure from caller. When retry exhausts attempts, the operation truly failed—propagate that information. Callers decide next steps: their own retry, fallback, user notification, etc.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "What makes a custom exception hierarchy valuable for a large application domain?",
      options: [
        "Enables catching errors at different granularity levels based context",
        "Reduces memory usage by sharing exception code across types",
        "Custom hierarchies execute faster than built-in exception types always",
        "Python requires hierarchies for applications with multiple exception types"
      ],
      correctOption: 0,
      explanation: "The correct answer is enabling granularity-based catching. Example: AuthenticationError (base) → UserNotFoundError, InvalidPasswordError. Callers choose: `except UserNotFoundError` (specific handling), `except AuthenticationError` (catch all auth errors), or both in different blocks. Hierarchies don't reduce memory (each instance allocates). Customs aren't faster (slightly slower due to subclassing). Python doesn't require hierarchies (you can inherit directly from Exception). The value: hierarchies organize exceptions by domain, enabling flexible error handling strategies. Handle broadly when context doesn't matter, specifically when recovery strategies differ.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "When should exceptions be logged at the level where they're caught versus propagated to a higher-level handler?",
      options: [
        "Always log at catch point regardless of propagation decisions",
        "Log when you handle and continue; propagate when caller should",
        "Never log at catch point; only log at top",
        "Log only unhandled exceptions at program exit boundary points"
      ],
      correctOption: 1,
      explanation: "The correct answer is log when handling, propagate when caller decides. If you catch and handle (use fallback, skip item, retry), log the exception (record what happened). If you catch, add context, and re-raise, don't log yet (let higher level log once with full context). Logging at every level creates duplicate log entries. Logging only at top loses intermediate context (where in the call chain did it fail?). Logging only unhandled misses handled errors. The pattern: log where you make recovery decision, propagate when passing decision upward. Avoids duplicate logging while capturing context.",
      source: "Lesson 4: Error Handling Strategies"
    },
    {
      question: "You're designing input validation that raises exceptions. How do you decide what information to include in the exception message?",
      options: [
        "Include what was invalid, what was expected, and rule",
        "Only include generic 'validation failed' message for all errors",
        "Include user's personal information to help debugging the issue",
        "Omit all context; exception type communicates everything needed already"
      ],
      correctOption: 0,
      explanation: "The correct answer is invalid value, expectation, rule. Example: `ValueError('Age must be 0-150, got -5')` tells: what field (age), what was wrong (-5), what's expected (0-150). This helps both debugging and user correction. Generic messages don't help ('validation failed' doesn't say what to fix). Never include sensitive information in exceptions (security/privacy violation). Exception type alone lacks context (ValueError doesn't say which field or why). Good exception messages are specific, actionable, and safe (no sensitive data). Format: f'{field} must be {constraint}, got {actual_value}'.",
      source: "Lesson 3: Raising and Custom Exceptions"
    },
    {
      question: "What's the purpose of the else block in try/except/else/finally, given that you could put the same code at the end of try?",
      options: [
        "Separates success-only code from risky code making exception source",
        "Else block executes faster than code at try block",
        "Python requires else for finally blocks to work correctly",
        "Else is deprecated; new code shouldn't use it"
      ],
      correctOption: 0,
      explanation: "The correct answer is separating success code from risky code. If else code raises an exception, you know it's from the success-path code, not from the risky operation in try. Example: `try: file = open() except FileNotFoundError: ... else: process(file)`. If process() raises, you know file opened successfully. If process() were in try, exceptions from process() would be caught by except (incorrect). Else isn't faster. Python doesn't require else for finally. Else isn't deprecated. The semantic value: else clarifies intent and exception scope.",
      source: "Lesson 2: Except, Else, and Finally"
    }
  ]}
/>
