# Fix Module Not Found Errors

## Problem
The new markdown rendering packages need to be installed on your local machine.

## Solution

Run this command in your terminal (in the book-source directory):

```bash
cd "P:\Ai native Book\ai-native-software-development\Tutor\book-source"
npm install
```

This will install all the required packages including:
- react-markdown
- rehype-highlight  
- remark-gfm
- highlight.js

## After Installation

Then restart the dev server:
```bash
npm start
```

The errors should be gone! 🎉
