# 🪟 TutorGPT Backend - Windows Setup Guide

Quick setup guide for Windows users.

---

## 📋 **Prerequisites**

- ✅ Python 3.11+ installed
- ✅ Gemini API key from [Google AI Studio](https://aistudio.google.com/app/apikey)

---

## 🚀 **Option 1: Automated Setup (Recommended)**

### **Step 1: Run Setup Script**

Open PowerShell or Command Prompt in the `backend` folder and run:

```cmd
setup_windows.bat
```

This will:
- Create virtual environment (`.venv`)
- Install all dependencies
- Create `.env` file

### **Step 2: Add API Key**

Edit `.env` file and add your Gemini API key:

```env
GEMINI_API_KEY=your-actual-gemini-api-key-here
```

### **Step 3: Test the Agent**

```cmd
.venv\Scripts\activate
python test_agent_live.py
```

---

## 🔧 **Option 2: Manual Setup**

### **Step 1: Create Virtual Environment**

```cmd
python -m venv .venv
```

### **Step 2: Activate Virtual Environment**

```cmd
.venv\Scripts\activate
```

You should see `(.venv)` in your prompt.

### **Step 3: Install Dependencies**

```cmd
pip install --upgrade pip
pip install -r requirements.txt
```

### **Step 4: Create .env File**

```cmd
copy .env.example .env
```

Edit `.env` and add your GEMINI_API_KEY:

```env
GEMINI_API_KEY=your-actual-key-here
```

### **Step 5: Test the Agent**

```cmd
python test_agent_live.py
```

---

## 🧪 **Running Tests**

### **Unit Tests:**

```cmd
.venv\Scripts\activate
pytest tests\unit\agent\ -v
```

### **Integration Tests (with real LLM):**

```cmd
.venv\Scripts\activate
pytest tests\integration\test_agent_with_llm.py -v -s
```

The `-s` flag shows agent responses in real-time!

---

## ❓ **Common Issues**

### **Issue: "python is not recognized"**

**Solution:** Install Python from [python.org](https://www.python.org/downloads/) and check "Add Python to PATH" during installation.

### **Issue: "cannot import name 'SQLiteSession'"**

**Solution:** Make sure you activated the virtual environment and installed dependencies:

```cmd
.venv\Scripts\activate
pip install -r requirements.txt
```

### **Issue: "GEMINI_API_KEY not set"**

**Solution:** Edit `.env` file and add your actual API key (not the placeholder text).

---

## 📝 **Quick Commands Reference**

| Task | Windows Command |
|------|----------------|
| **Activate venv** | `.venv\Scripts\activate` |
| **Deactivate venv** | `deactivate` |
| **Install deps** | `pip install -r requirements.txt` |
| **Run live test** | `python test_agent_live.py` |
| **Run unit tests** | `pytest tests\unit\ -v` |
| **Run integration tests** | `pytest tests\integration\ -v -s` |

---

## ✅ **Verify Installation**

After setup, verify everything works:

```cmd
.venv\Scripts\activate
python -c "from app.agent.tutor_agent import create_tutor_agent; print('✅ All imports work!')"
```

If you see `✅ All imports work!`, you're ready to go!

---

## 🎯 **Next Steps**

1. ✅ Add your GEMINI_API_KEY to `.env`
2. ✅ Run `python test_agent_live.py`
3. ✅ Watch the autonomous agent in action! 🧠

---

Need help? Check the main [README.md](README.md) or [contact support](mailto:support@example.com).
