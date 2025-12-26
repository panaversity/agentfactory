---
sidebar_position: 25
title: "Chapter 25: IO and File Handling"
slides:
  source: "https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/slides/chapter-25-slides.pdf"
  title: "Chapter 25: IO and File Handling"
  height: 700
---

# Chapter 25: IO and File Handling

Programs need to interact with the outside world. They accept user input, display results, save data to files, load configurations, and process structured data. Input/output operations are the foundation of every useful program—without I/O, your code lives in isolation.

You'll learn how programs interact with users through the console, persist data to files for long-term storage, organize file systems using cross-platform paths, and work with structured data formats like CSV and JSON. By the end, you'll build a complete Note-Taking CLI application that integrates all these concepts. This chapter applies the **AI-Native Learning methodology** you've practiced since Chapter 1: you describe what you want your code to do, explore concepts with your AI companion, validate your understanding through interactive programs, and learn from errors by asking "why?"

This is the same pattern you'll use throughout your professional career. Your AI tool isn't a crutch—it's a partner in learning.

## Chapter Contribution to Todo App

This chapter contributes: **Task persistence (JSON)**

By the end of this chapter, your todo application will save tasks to JSON files and load them when the app starts—enabling persistent task storage that survives program restarts.

## 🎯 Before You Begin

---

## What You'll Learn

By the end of this chapter, you will be able to:

- **Gather and validate console input** with proper error handling and formatted output
- **Read and write files safely** using context managers and exception handling
- **Work with file paths** across Windows, Mac, and Linux using pathlib
- **Load and save structured data** in CSV and JSON formats with correct encoding
- **Build a complete CLI application** integrating all I/O concepts (Capstone)

---

## What's Next: Organizing Everything with Classes

Your TaskManager now saves and loads tasks from files. It works. But as the application grows—handling multiple projects, sharing tasks with others, filtering by due dates—the procedural approach of passing tasks around starts to become unwieldy. In **Chapter 27**, you'll wrap your task management logic inside a TaskManager class where data (task list, file path) and behavior (add, complete, save, load) live together in one cohesive object.

Instead of juggling lists and functions, you'll create a TaskManager instance that knows how to manage itself. This is Object-Oriented Programming: organizing your program around the objects that matter (in this case, a TaskManager that maintains its own state).
