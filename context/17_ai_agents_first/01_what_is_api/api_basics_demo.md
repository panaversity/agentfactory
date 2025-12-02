# APIs — Hands‑on with Python
This Colab walks you through three live API calls:
1. Weather forecast
2. Cat facts
3. OpenAI Chat Completions

## 📦 installation

```python
!pip -q install requests openai
```

# 📦 Imports

```python
import requests, pprint
```

##  ⚙️ 1. Get current weather for London (Simple api call )

```python
url = "https://api.open-meteo.com/v1/forecast?latitude=51.5&longitude=-0.12&current_weather=true"
resp = requests.get(url, timeout=10)
resp.raise_for_status()
weather = resp.json()
pprint.pp(weather)
```

## ⚙️ 2. Swap endpoint: Cat Fact

```python
url = "https://catfact.ninja/fact"
resp = requests.get(url, timeout=10)
resp.raise_for_status()
print(resp.json()['fact'])
```

## 3. 🔐 Load API Key
Fill in your API key below. You can store it in an environment variable or directly in the code (not recommended for production).

```python
from google.colab import userdata
OPENAI_API_KEY = userdata.get('OPENAI_API_KEY')
```

## Checking Behavior - Stateless or Stateful

```python
import os, openai

# 🔑 TODO: Replace with your own key or set OPENAI_API_KEY in the environment
openai.api_key = OPENAI_API_KEY

response = openai.chat.completions.create(
    model="gpt-4o-mini",
    messages=[{"role": "user", "content": "Say hello! I'm Wania"}]
)
print(response.choices[0].message.content)
```

```python
response = openai.chat.completions.create(
    model="gpt-4o-mini",
    messages=[{"role": "user", "content": "What's my name?"}]
)
print(response.choices[0].message.content)
```

# Responses API

## Checking Behavior - Stateless or Stateful

```python
from openai import OpenAI
from google.colab import userdata

OPENAI_API_KEY = userdata.get('OPENAI_API_KEY')
client = OpenAI(api_key=OPENAI_API_KEY)

response = client.responses.create(
    model="gpt-4.1",
    input=[
        {
            "role": "user",
            "content": [
                { "type": "input_text", "text": "what is in this image?" },
                {
                    "type": "input_image",
                    "image_url": "https://upload.wikimedia.org/wikipedia/commons/thumb/d/dd/Gfp-wisconsin-madison-the-nature-boardwalk.jpg/2560px-Gfp-wisconsin-madison-the-nature-boardwalk.jpg"
                }
            ]
        }
    ]
)

print(response.output[0].content[0].text)
```

```python
from openai import OpenAI
from google.colab import userdata

OPENAI_API_KEY = userdata.get('OPENAI_API_KEY')
client = OpenAI(api_key=OPENAI_API_KEY)

response = client.responses.create(
    model="gpt-4.1",
    store=True,
    input=[
        {
            "role": "user",
            "content": [
                { "type": "input_text", "text": "hello, I'm Wania" },
            ]
        }
    ]
)

print(response.output[0].content[0].text)
```

```python
next_response = client.responses.create(
    model="gpt-4.1",
    store=True,
    previous_response_id=response.id,  # 👈 linking to earlier message
    input=[
        {
            "role": "user",
            "content": [
                { "type": "input_text", "text": "What is my name?" },
            ]
        }
    ]
)

print("Follow-up response:", next_response.output[0].content[0].text)
```

# 😊 Gemini Code

# 📦 Imports

```python
from openai import OpenAI
from google.colab import userdata
```

# 🔐 Load API Key

```python
api_key = userdata.get('GEMINI_API_KEY')
```

# 🤖 Initialize Gemini Client

```python
client = OpenAI(
    api_key=api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)
```

# 💬 Run Basic Chat Completion

```python
def main():
    print("🧠 Asking Gemini a question...\n")

    response = client.chat.completions.create(
        model="gemini-2.5-flash",
        messages=[
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user",   "content": "Explain how AI works in simple terms."}
        ]
    )

    message = response.choices[0].message.content
    print("💡 Gemini's Response:\n")
    print(message)
```

# 🚀 Entry Point

```python
if __name__ == "__main__":
    main()
```
