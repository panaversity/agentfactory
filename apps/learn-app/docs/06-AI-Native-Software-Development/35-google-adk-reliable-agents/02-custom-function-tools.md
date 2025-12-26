---
sidebar_position: 2
title: "Custom Function Tools"
description: "Create powerful function tools with Google ADK using type annotations and docstrings. Build a financial data tool to enrich agent capabilities."
keywords: [function_tool, type_annotations, docstrings, yfinance, tool_wrapping, external_apis, error_handling]
chapter: 35
lesson: 2
duration_minutes: 55

skills:
  - name: "Function Tool Creation with ADK"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can create functions with type hints and docstrings that ADK auto-wraps as tools"

  - name: "External API Integration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Data Integration"
    measurable_at_this_level: "Student can call external APIs (yfinance) from tool functions and handle errors gracefully"

  - name: "Tool Coordination in Agents"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design tools that work together to enrich agent capabilities"

learning_objectives:
  - objective: "Create function tools with type hints and comprehensive docstrings for agent discovery"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Tool executes and agent uses it correctly to answer questions"

  - objective: "Integrate external APIs (yfinance) into function tools with error handling"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Tool gracefully handles network errors and returns structured data"

  - objective: "Coordinate multiple tools to enrich agent outputs with complementary data"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Agent calls search tool then financial tool to produce enriched results"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (type hints, docstrings, auto-wrapping, yfinance API, error handling, return types, tool coordination, structured output) within B1 limit ✓"

differentiation:
  extension_for_advanced: "Build a tool that aggregates data from multiple APIs (yfinance + sentiment analysis) and handles rate limiting"
  remedial_for_struggling: "Start with a simple tool that calls one external API with minimal error handling, then add complexity"
---

# Custom Function Tools

In Lesson 1, you used the built-in `google_search` tool to help an agent find information. But what if you need to do something the built-in tools can't handle?

Think of an agent as someone with a set of skills. Google search is one skill. But what if you need that agent to look up stock prices, translate text, query a database, or analyze images? You need to add new tools—custom capabilities specific to your domain.

Google ADK makes this straightforward: Write a Python function with type hints and a docstring. ADK automatically wraps it as a tool and makes it available to your agent.

In this lesson, you'll learn to create custom tools that integrate external APIs. You'll build a financial data tool using yfinance, then combine it with the search tool to create a richer agent that finds news AND enriches it with stock prices.

## Understanding Function Tool Pattern

Google ADK's philosophy is simple: **If it's a function with type hints and a docstring, it's a tool.**

Here's the pattern:

```python
def tool_name(param1: type1, param2: type2) -> return_type:
    """
    Brief description of what the tool does.

    Args:
        param1: Description of param1.
        param2: Description of param2.

    Returns:
        Description of return type.
    """
    # Your implementation here
    return result
```

The three essential parts:
1. **Type hints** → Tell ADK what arguments to expect and what will be returned
2. **Docstring** → Becomes the tool's description; helps the agent decide when to use it
3. **Implementation** → Your actual code that executes when the agent calls the tool

Type hints are REQUIRED. Without them, ADK cannot generate the tool schema that the Gemini model needs to understand what arguments to provide.

Let's start with a simple example before building the financial tool:

```python
from google.generativeai.agents import Agent, Tool

def celsius_to_fahrenheit(celsius: float) -> float:
    """
    Convert temperature from Celsius to Fahrenheit.

    Args:
        celsius: Temperature in Celsius.

    Returns:
        Temperature in Fahrenheit.
    """
    return (celsius * 9/5) + 32

# Create agent with the tool
agent = Agent(
    name="TemperatureBot",
    instruction="Help users convert temperatures. Use the celsius_to_fahrenheit tool when asked.",
    tools=[
        Tool.from_function(celsius_to_fahrenheit)
    ]
)
```

**Output when you ask "What is 0 Celsius in Fahrenheit?"**:
```
0 Celsius is 32 Fahrenheit.
```

Notice what happened: The agent read the function signature and docstring, understood it could convert temperatures, called the function with the value 0, received 32 back, and incorporated it into its response. No decorators needed—just a function with types and documentation.

## Building a Financial Data Tool

Now let's build something more complex. You'll create a tool that fetches stock data using yfinance:

```python
from typing import Dict, List
import yfinance as yf
from google.generativeai.agents import Agent, Tool

def get_financial_context(tickers: List[str]) -> Dict[str, str]:
    """
    Fetches current stock price and daily change for stock tickers.

    Useful for enriching news articles with current market context.
    Returns data even if some tickers fail.

    Args:
        tickers: A list of stock market tickers (e.g., ["GOOG", "NVDA"]).

    Returns:
        A dictionary mapping each ticker to formatted financial data.
        Format: "ticker": "$price (±change%)"
        If data unavailable: "ticker": "Price data not available."
    """
    financial_data: Dict[str, str] = {}

    for ticker_symbol in tickers:
        try:
            stock = yf.Ticker(ticker_symbol)
            info = stock.info

            # Get price (primaryTradePrice is most recent, regularMarketPrice is fallback)
            price = info.get("currentPrice") or info.get("regularMarketPrice")
            change_percent = info.get("regularMarketChangePercent")

            if price is not None and change_percent is not None:
                change_str = f"{change_percent * 100:+.2f}%"
                financial_data[ticker_symbol] = f"${price:.2f} ({change_str})"
            else:
                financial_data[ticker_symbol] = "Price data not available."

        except Exception as e:
            # Graceful degradation: return informative error instead of crashing
            financial_data[ticker_symbol] = f"Unable to fetch data (API error)"

    return financial_data
```

Let's break down why this design matters:

**Return type consistency**: Always returns `Dict[str, str]`, never raises exceptions
**Graceful degradation**: If one ticker fails, others still work
**Informative format**: Stock data is human-readable ("$150.25 (+2.50%)")
**Error handling**: Try-catch around each ticker, not the whole function
**Docstring detail**: Explains what the tool returns AND when it fails

Now let's use it in an agent:

```python
agent = Agent(
    name="ai_news_chat_assistant",
    model="gemini-2.5-flash",
    instruction="""You are a financial news agent. Your job is to:

    1. Use google_search to find recent news about AI companies and tech stocks
    2. Extract company names and stock tickers from the news
    3. Use get_financial_context to fetch current stock prices for those companies
    4. Present the news with financial context: "Company X announced Y.
       Stock ticker: $price (change%)"

    This helps users understand news impact on stock prices.""",
    tools=[google_search, Tool.from_function(get_financial_context)]
)
```

When you run this:

```python
from google.generativeai.agents import Runner

response = Runner.run_sync(
    agent,
    "What's the latest on OpenAI and Google's AI competition? Show me their stock prices."
)
print(response.final_output)
```

**The agent's workflow**:
1. Calls `google_search("OpenAI latest news")`
2. Receives news articles mentioning "OpenAI" and "Google"
3. Extracts tickers: ["MSFT", "GOOGL"] (OpenAI is backed by Microsoft)
4. Calls `get_financial_context(["MSFT", "GOOGL"])`
5. Gets back: `{"MSFT": "$420.50 (+1.23%)", "GOOGL": "$185.30 (+0.45%)"}`
6. Synthesizes response: "OpenAI just announced... Microsoft (MSFT) is up 1.23% and Google (GOOGL) is up 0.45%..."

## Tool Coordination Patterns

### Pattern 1: Search + Enrich

Use one tool to gather raw information, another to add context:

```python
def search_research(query: str) -> str:
    """Search for information about a topic."""
    return google_search(query)

def get_financial_context(tickers: List[str]) -> Dict[str, str]:
    """Get stock prices for companies mentioned in research."""
    # Implementation above
    pass

agent = Agent(
    instruction="Search for research, then enrich with financial data.",
    tools=[search_research, get_financial_context]
)
```

The agent naturally chains these: search first → parse results → call financial tool.

### Pattern 2: Multiple Enrichment

One primary tool, multiple secondary tools for different contexts:

```python
def search_query(query: str) -> str:
    """Primary research tool."""
    pass

def get_stock_price(ticker: str) -> str:
    """Get stock price context."""
    pass

def get_news_sentiment(company: str) -> str:
    """Get sentiment analysis of recent news."""
    pass

agent = Agent(
    instruction="Search for information. If about companies, also get stock data and sentiment.",
    tools=[search_query, get_stock_price, get_news_sentiment]
)
```

### Pattern 3: Filtering + Validation

Use one tool to filter data, another to validate:

```python
def search_articles(topic: str) -> List[str]:
    """Search and return article URLs."""
    pass

def validate_source(url: str) -> bool:
    """Check if source is reputable (returns True/False)."""
    pass

# Agent calls search_articles, then for each result, calls validate_source
# to filter out low-quality sources
```

## Common Pitfalls & Solutions

### Pitfall 1: Missing or Incomplete Type Hints

**WRONG:**
```python
def get_price(ticker):  # No type hints!
    """Get stock price."""
    return yf.Ticker(ticker).info.get("currentPrice")
```

**RIGHT:**
```python
def get_price(ticker: str) -> float:
    """Get stock price.

    Args:
        ticker: Stock ticker symbol (e.g., "GOOG").

    Returns:
        Current price as float, or 0.0 if unavailable.
    """
    try:
        price = yf.Ticker(ticker).info.get("currentPrice")
        return price or 0.0
    except Exception:
        return 0.0
```

**Why it matters**: Without type hints, ADK cannot generate the tool schema. The agent won't know what type of arguments to pass.

### Pitfall 2: Raising Exceptions Instead of Returning Errors

**WRONG:**
```python
def get_stock_data(ticker: str) -> Dict:
    """Get stock data."""
    stock = yf.Ticker(ticker)  # Raises if network fails
    return stock.info
```

**RIGHT:**
```python
def get_stock_data(ticker: str) -> Dict[str, str]:
    """Get stock data.

    Returns structured data even on error (graceful degradation).
    """
    try:
        stock = yf.Ticker(ticker)
        return {
            "ticker": ticker,
            "price": str(stock.info.get("currentPrice", "unavailable")),
            "status": "success"
        }
    except Exception as e:
        return {
            "ticker": ticker,
            "price": "unavailable",
            "status": f"error: {str(e)}"
        }
```

**Why it matters**: If a tool raises an exception, the agent crashes. Graceful degradation means the agent can handle partial failures and continue working.

### Pitfall 3: Inconsistent Return Format

**WRONG:**
```python
def get_data(query: str):  # Sometimes returns str, sometimes dict
    """Get data."""
    if "price" in query:
        return {"price": 100}  # Returns dict
    else:
        return "No data"  # Returns string
```

**RIGHT:**
```python
def get_data(query: str) -> Dict[str, str]:
    """Get data in consistent format."""
    if "price" in query:
        return {"result": "price is $100"}  # Always dict
    else:
        return {"result": "No data found"}  # Always dict
```

**Why it matters**: Agents expect consistent return types. Inconsistency causes parsing errors.

### Pitfall 4: Overly Complex Logic in Tools

**WRONG:**
```python
def analyze_market(tickers: List[str]) -> str:
    # 50 lines of analysis logic
    # Complex calculations
    # Multiple API calls
    # Returns opaque string
    pass
```

**RIGHT:**
```python
def fetch_market_data(tickers: List[str]) -> Dict[str, float]:
    """Fetch raw market data."""
    # Simple: just get the data
    return {ticker: get_price(ticker) for ticker in tickers}

def analyze_market_trends(data: Dict[str, float]) -> str:
    """Analyze fetched data."""
    # Agent or separate function handles analysis
    # Tool stays focused: fetch or compute, not both
    pass
```

**Why it matters**: Tools should be focused. One tool fetches data, another analyzes. This makes tools reusable across agents.

## Type Hints for Tools

Google ADK supports these type hints:

```python
# Primitives
def example(a: int, b: str, c: float, d: bool) -> str:
    pass

# Collections
from typing import List, Dict, Tuple
def example(items: List[str], mapping: Dict[str, int]) -> Tuple[str, int]:
    pass

# Optional (can be None)
from typing import Optional
def example(name: Optional[str]) -> Optional[str]:
    pass

# Union (multiple types)
from typing import Union
def example(value: Union[str, int]) -> str:
    pass

# Nested structures
def example(data: List[Dict[str, int]]) -> Dict[str, List[str]]:
    pass
```

**Key rule**: Every parameter and return value must have a type hint. ADK uses these to generate the tool schema that Gemini uses to decide what arguments to pass.

## Docstring Format for Clarity

Your docstring becomes the agent's understanding of your tool. Make it clear:

```python
def process_data(items: List[str], threshold: float) -> List[str]:
    """
    Filter items by a numeric threshold.

    Use this tool when you need to filter a list of items
    based on some quantitative measure.

    Args:
        items: List of item names to filter.
        threshold: Numeric threshold for filtering (items with
                  values above this are included).

    Returns:
        Filtered list of items that meet the threshold.

    Raises:
        No exceptions raised; returns empty list if no items match.
    """
    # Implementation
    pass
```

The docstring tells the agent:
- **What it does**: Filter items by threshold
- **When to use it**: "when you need to filter"
- **What arguments it expects**: items (list), threshold (float)
- **What it returns**: filtered list
- **Edge cases**: Empty list if nothing matches

## Adding Tools to Agents

Once you've created your tools, register them with the agent:

```python
from google.generativeai.agents import Agent, Tool

# Define functions
def get_financial_data(tickers: List[str]) -> Dict[str, str]:
    pass

def search_news(query: str) -> str:
    pass

# Create agent with tools
agent = Agent(
    name="NewsAgent",
    instruction="Search for news and enrich with financial data.",
    tools=[
        Tool.from_function(get_financial_data),
        Tool.from_function(search_news),
        google_search  # Built-in tool
    ]
)
```

The agent now has three tools available. It will call whichever is most appropriate for the user's request.

---

## Try With AI

### Prompt 1: Create Your First Custom Tool

```
Write a function tool called get_crypto_price that:
- Takes a cryptocurrency symbol (e.g., "bitcoin", "ethereum") as input
- Returns a formatted string with current price and 24-hour change
- Handles errors gracefully (no crashing if API is down)
- Has proper type hints (str -> str) and a clear docstring

Use a real API like CoinGecko (free, no key needed) or mock the data for testing.
```

**What you're learning:** The pattern of wrapping functions as tools with type hints, error handling, and clear docstrings that help agents understand when to use them.

### Prompt 2: Build a Tool Chain

```
Create two function tools that work together:
1. search_products(category: str) -> str: Search for products in a category
2. compare_prices(product_names: List[str]) -> Dict[str, str]: Get prices for products

Then create an agent that can:
- Search for laptops
- Extract product names from the results
- Compare prices for those products

Show how the agent chains these tools automatically to answer:
"Find gaming laptops and show me the price comparison."
```

**What you're learning:** How tools coordinate to provide richer capabilities. The agent chains simple tools to accomplish complex tasks without you scripting each step.

### Prompt 3: Error Handling & Robustness

```
Enhance your price comparison tool from Prompt 2 to handle:
- Network failures (API timeouts)
- Missing data for some products
- Rate limiting (some API calls fail, others succeed)

Test it with: "Compare prices for [10 product names, some valid, some invalid]"

After running, answer:
- Which products returned valid prices?
- Which failed and why?
- Did the tool crash or degrade gracefully?
```

**What you're learning:** Production-grade tools don't assume everything works perfectly. Graceful degradation—returning partial results instead of crashing—is what separates fragile tools from reliable ones that agents can depend on.
