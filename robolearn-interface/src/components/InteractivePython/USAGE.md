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
/>
```

## Component Props

| Prop | Type | Default | Description |
|------|------|---------|-------------|
| `initialCode` | `string` | `print("Hello, World!")` | The initial Python code to display in the editor |
| `language` | `string` | `"python"` | Language identifier (currently supports `python`) |
| `showLineNumbers` | `boolean` | `true` | Show/hide line numbers |

## Features

✅ **Live Code Editing** - Students can write and modify code
✅ **Execute Python** - Run code directly in the browser with a prominent "Run" button
✅ **Real Python Semantics** - Uses actual CPython via Pyodide
✅ **Output Display** - See results and errors in real-time with labeled output sections
✅ **Loading State** - Graceful initialization of Pyodide
✅ **Error Handling** - Clear error messages for debugging with distinct styling
✅ **Reset Code** - Restore original code with improved reset icon
✅ **Copy Code** - Copy code to clipboard with proper copy icon
✅ **Copy Output** - Copy output/error text to clipboard
✅ **Clear Output** - Clear output without resetting code
✅ **Responsive Design** - Works on mobile and desktop
✅ **Dark Mode Support** - Full dark mode theming with contrasting colors
✅ **Professional Icons** - Using Lucide React icons for better visual clarity

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

### Pre-installed Packages
- **`requests`** (v2.32.4) - HTTP library for API interactions
  ```python
  import requests
  response = requests.get('https://api.example.com/data')
  data = response.json()
  ```
  Available for making GET/POST requests and working with REST APIs

## Examples

### Example 1: Variables and Math

```tsx
<InteractivePython
  initialCode={`# Variables and Basic Math
x = 10
y = 20
print(f"x + y = {x + y}")
print(f"x * y = {x * y}")`}
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
/>
```

## Limitations & Notes

-  Limited package availability (standard library + ~100 pre-built packages like numpy, pandas, matplotlib; see
  [Pyodide packages list](https://pyodide.org/en/stable/usage/packages-in-pyodide.html))
    - To enable other packages: Use `micropip` to install from PyPI dynamically
- **No File System** - Cannot read/write actual files (in-memory only)
- **Network Requests Supported** - You can make HTTP requests using the `requests` library
- **Single-threaded** - Executes in main thread (no multi-threading)
- **Performance** - Pyodide initialization takes ~2-5 seconds on first load

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

## UI & Keyboard Shortcuts

### Header Controls
- **Reset Button** (↻) - Restore code to initial state
- **Copy Button** (📋) - Copy current code to clipboard
- **Run Button** (⚡ Run) - Execute the code (or press Shift+Enter)

### Output Section
When code is executed, the output panel appears with:
- **Output/Error Label** - Indicates whether it's normal output or an error
- **Copy Button** - Copy the entire output to clipboard
- **Clear Button** - Clear the output panel

### Keyboard Shortcuts
| Shortcut | Action |
|----------|--------|
| `Shift + Enter` | Run the code |
| `Tab` | Indent code (within editor) |

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
