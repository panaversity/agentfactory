# Common API Troubleshooting Guide

This guide helps you solve the most common issues when working with the OpenAI API.

---

## Error: 401 Authentication Failed

**What it looks like:**
```
openai.AuthenticationError: Error code: 401 - {'error': {'message': 'Incorrect API key provided'}}
```

**What it means:** Your API key is missing, incorrect, or expired.

**How to fix:**
1. Check your `.env` file exists in your project folder
2. Verify the key format: `OPENAI_API_KEY=sk-...`
3. Make sure there are no extra spaces or quotes
4. Confirm your key is still valid at [platform.openai.com](https://platform.openai.com/api-keys)

---

## Error: Network Timeout / Connection Error

**What it looks like:**
```
openai.APIConnectionError: Connection error.
```

**What it means:** Your computer can't reach OpenAI's servers.

**How to fix:**
1. Check your internet connection
2. Try accessing [openai.com](https://openai.com) in your browser
3. If you're behind a corporate firewall, ask IT about API access
4. Try again in a few minutes (sometimes OpenAI has brief outages)

---

## Error: Invalid API Key Format

**What it looks like:**
```
openai.AuthenticationError: Invalid API key format
```

**What it means:** The key doesn't match OpenAI's expected format.

**How to fix:**
1. API keys should start with `sk-`
2. Check for copy/paste errors (extra characters, missing characters)
3. Generate a new key if needed: [platform.openai.com/api-keys](https://platform.openai.com/api-keys)

---

## Error: Module Not Found (openai)

**What it looks like:**
```
ModuleNotFoundError: No module named 'openai'
```

**What it means:** The OpenAI package isn't installed in your environment.

**How to fix:**

**If using UV:**
```bash
uv add openai
```

**If using pip:**
```bash
pip install openai
```

---

## Error: Rate Limit Exceeded

**What it looks like:**
```
openai.RateLimitError: Rate limit reached for default-gpt-3.5-turbo
```

**What it means:** You've made too many requests too quickly.

**How to fix:**
1. Wait a minute and try again
2. If on free tier, you have limited requests per minute
3. Consider adding a paid balance to your OpenAI account

---

## .env File Not Loading

**Symptoms:** Code runs but gets authentication error even though .env looks correct.

**Common causes:**
1. `.env` file is in the wrong folder (must be in project root)
2. Missing `python-dotenv` package
3. Variable name typo (must be exactly `OPENAI_API_KEY`)

**How to verify:**
```python
import os
print(os.getenv("OPENAI_API_KEY"))  # Should print your key
```

If it prints `None`, your .env isn't being loaded.

---

## OS-Specific Notes

### Windows
- Use PowerShell, not Command Prompt
- If UV install fails, try: `pip install openai` as fallback
- Path separators are `\` not `/`

### macOS/Linux
- If permission denied, don't use `sudo` with UV
- Try `pip install --user openai` if global install fails

---

## Still Stuck?

1. **Read the error message carefully** — it usually tells you what's wrong
2. **Search the error** — copy the key phrase into Google/ChatGPT
3. **Ask for help** — use ChatGPT web to troubleshoot your specific situation
