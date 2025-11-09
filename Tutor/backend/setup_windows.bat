@echo off
REM TutorGPT Backend Setup Script for Windows
REM This script sets up the virtual environment and installs dependencies

echo ========================================
echo TutorGPT Backend Setup (Windows)
echo ========================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH!
    echo Please install Python 3.11+ from https://python.org
    pause
    exit /b 1
)

echo [1/4] Creating virtual environment...
python -m venv .venv
if errorlevel 1 (
    echo ERROR: Failed to create virtual environment!
    pause
    exit /b 1
)
echo DONE!
echo.

echo [2/4] Activating virtual environment...
call .venv\Scripts\activate.bat
if errorlevel 1 (
    echo ERROR: Failed to activate virtual environment!
    pause
    exit /b 1
)
echo DONE!
echo.

echo [3/4] Installing dependencies...
pip install --upgrade pip
pip install -r requirements.txt
if errorlevel 1 (
    echo ERROR: Failed to install dependencies!
    pause
    exit /b 1
)
echo DONE!
echo.

echo [4/4] Creating .env file...
if not exist .env (
    copy .env.example .env
    echo DONE! Please edit .env and add your GEMINI_API_KEY
) else (
    echo .env already exists, skipping...
)
echo.

echo ========================================
echo Setup Complete!
echo ========================================
echo.
echo Next steps:
echo 1. Edit .env file and add your GEMINI_API_KEY
echo 2. Activate environment: .venv\Scripts\activate
echo 3. Run test: python test_agent_live.py
echo.
pause
