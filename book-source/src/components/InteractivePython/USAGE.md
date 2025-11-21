# InteractivePython Component Usage Guide

The `InteractivePython` component enables interactive Python code execution directly in the browser using **Pyodide** (CPython compiled to WebAssembly).

## Quick Start

### Import the Component

```tsx
import InteractivePython from '@/components/InteractivePython';
```

### Basic Usage

```tsx
<InteractivePython
  initialCode={`print("Hello, World!")
print(2 + 2)`}
  title="Python Basics"
/>
```

## Component Props

| Prop | Type | Default | Description |
|------|------|---------|-------------|
| `initialCode` | `string` | `print("Hello, World!")` | The initial Python code to display in the editor |
| `language` | `string` | `"python"` | Language identifier (currently supports `python`) |
| `title` | `string` | `"Interactive Python"` | Title displayed at the top of the component |
| `showLineNumbers` | `boolean` | `true` | Show/hide line numbers (future feature) |

## Features

✅ **Live Code Editing** - Students can write and modify code
✅ **Execute Python** - Run code directly in the browser
✅ **Real Python Semantics** - Uses actual CPython via Pyodide
✅ **Output Display** - See results and errors in real-time
✅ **Loading State** - Graceful initialization of Pyodide
✅ **Error Handling** - Clear error messages for debugging
✅ **Reset Code** - Restore original code with one click
✅ **Clear Output** - Clear output without resetting code
✅ **Responsive Design** - Works on mobile and desktop

## Supported Python Features

### Standard Library
All standard Python libraries are available:
- `math`, `random`, `statistics`, `datetime`, `collections`, `itertools`, `functools`, etc.
- File I/O (in-memory)
- Regular expressions, string formatting, JSON parsing

### Built-in Functions
All Python built-ins work:
- `print()`, `input()`, `range()`, `len()`, `sum()`, `map()`, `filter()`, etc.

### Control Flow
- `if`/`elif`/`else` statements
- `for` and `while` loops
- `try`/`except`/`finally` error handling
- Functions and lambdas

### Data Structures
- Lists, tuples, sets, dictionaries
- List/dict/set comprehensions
- Slicing and unpacking

## Examples

### Example 1: Variables and Math

```tsx
<InteractivePython
  initialCode={`# Variables and Basic Math
x = 10
y = 20
print(f"x + y = {x + y}")
print(f"x * y = {x * y}")`}
  title="Variables and Math"
/>
```

### Example 2: Loops and Conditionals

```tsx
<InteractivePython
  initialCode={`# Fibonacci Sequence
def fibonacci(n):
    a, b = 0, 1
    for _ in range(n):
        print(a, end=' ')
        a, b = b, a + b
    print()

fibonacci(10)`}
  title="Fibonacci Sequence"
/>
```

### Example 3: String Operations

```tsx
<InteractivePython
  initialCode={`# String Manipulation
text = "Python Programming"
print(f"Original: {text}")
print(f"Length: {len(text)}")
print(f"Uppercase: {text.upper()}")
print(f"Reversed: {text[::-1]}")
print(f"Words: {text.split()}")`}
  title="String Operations"
/>
```

### Example 4: List Comprehensions

```tsx
<InteractivePython
  initialCode={`# List Comprehensions
numbers = [1, 2, 3, 4, 5]
squares = [x**2 for x in numbers]
evens = [x for x in numbers if x % 2 == 0]

print(f"Original: {numbers}")
print(f"Squares: {squares}")
print(f"Evens: {evens}")`}
  title="List Comprehensions"
/>
```

### Example 5: Using Standard Library

```tsx
<InteractivePython
  initialCode={`import math
import statistics

numbers = [2, 4, 6, 8, 10]
print(f"Sum: {sum(numbers)}")
print(f"Mean: {statistics.mean(numbers)}")
print(f"Median: {statistics.median(numbers)}")
print(f"Square root of 16: {math.sqrt(16)}")`}
  title="Using Standard Library"
/>
```

## Limitations & Notes

⚠️ **No External Packages** - PyPI packages (NumPy, Pandas, etc.) are not available by default
   - To enable: Use `micropip` to install packages from PyPI dynamically

⚠️ **No File System** - Cannot read/write actual files (in-memory only)

⚠️ **No Network Requests** - Cannot make HTTP requests from the browser

⚠️ **Single-threaded** - Executes in main thread (no multi-threading)

⚠️ **Performance** - Pyodide initialization takes ~2-5 seconds on first load

## Browser Support

| Browser | Version | Support |
|---------|---------|---------|
| Chrome | 57+ | ✅ Full |
| Firefox | 52+ | ✅ Full |
| Safari | 11+ | ✅ Full |
| Edge | 79+ | ✅ Full |

## Under the Hood

The `InteractivePython` component uses:

1. **Pyodide** - CPython compiled to WebAssembly
2. **Monaco Editor** - Code editor with syntax highlighting
3. **Custom Pyodide singleton** - Manages Python runtime initialization and execution

This means:
- No compilation needed on the user's machine
- No server-side execution (everything is client-side)
- Works offline once loaded
- Executes in the main thread (no separate workers)

## Tips for Educational Content

### Best Practices

1. **Start simple** - Show print statements before complex logic
2. **Incremental learning** - Build up concepts progressively
3. **Interactive exercises** - Let students modify code in real-time
4. **Error messages** - Show common mistakes and their errors
5. **Real Python** - Students learn actual Python, not a variant

### Classroom Use

```tsx
<InteractivePython
  initialCode={`# TODO: Write your code here
# Try printing your name!

`}
  title="Your First Python Program"
/>
```

Students can:
- Clear the code and write their own
- Click "Run" to execute
- See output immediately
- Debug errors on the spot
- Use "Reset" to go back

## Troubleshooting

### "Loading Pyodide..." takes too long
- First load initializes Pyodide (~2-5 seconds) - this is normal
- Subsequent loads are instant (cached)
- Refresh the page if stuck

### Code runs but no output appears
- Make sure you're using `print()` to display results
- Check the output panel below the editor

### Error: "ModuleNotFoundError"
- Standard library modules are available
- Third-party packages (NumPy, pandas) are not installed
- Use `import micropip; micropip.install("package-name")` if needed

## Future Enhancements

Planned features for the component:
- [ ] Code sharing/export
- [ ] Async/await support
- [ ] Package installation UI
- [ ] Collaborative editing

## Contributing

Found a bug or want to improve the component? The source code is at:
- Component: `src/components/InteractivePython/InteractivePython.tsx`
- Styles: `src/components/InteractivePython/styles.module.css`

Submit improvements via GitHub issues or pull requests!
