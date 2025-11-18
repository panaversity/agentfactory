# Notes Feature Documentation

## Overview

The **Notes Feature** allows users to create, manage, and organize personal notes while reading documentation. Users can manually create notes or append selected text from the page content directly into their notes. This feature enhances the learning experience by providing a dedicated space for capturing thoughts, summaries, and key takeaways.

---

## Features

### 1. **Notes Panel**
The Notes panel is accessible from the navbar and provides two main views:
- **Notes List**: Display all saved notes with search functionality
- **Add/Edit Note**: Interface for creating new notes or editing existing ones

### 2. **Notes List View**

#### Search Functionality
- **Search Field**: Filter notes by title and content
- **Real-time filtering**: Results update as you type
- **Case-insensitive**: Search matches regardless of letter case

#### Note Cards
Each note card displays:
- **Title**: The note's title (clickable to edit)
- **Preview**: First 200 characters of the note content (with HTML preserved)
- **Timestamp**: Creation/modification date and time
- **Action Buttons**:
  - **Edit**: Opens the note in edit mode with prefilled content
  - **Delete**: Removes the note after confirmation

#### Empty State
- Displays a friendly message when no notes exist
- Provides an "Add Note" button to create the first note

---

### 3. **Add/Edit Note View**

#### Form Fields
- **Title Field**:
  - Default value: "New Note"
  - Required field
  - Placeholder: "Enter note title..."

- **Rich Textarea**:
  - Contenteditable div supporting rich text formatting
  - Preserves HTML formatting (bold, italic, line breaks, etc.)
  - Minimum height: 200px
  - Maximum height: 400px (scrollable)
  - Auto-scrolls when content exceeds max height
  - Placeholder: "Enter your notes here... You can also select text from the page and add it to your notes."

#### Action Buttons
- **Save**:
  - Saves the note (creates new or updates existing)
  - Returns to Notes List view
  - Validates that title is not empty

- **Clear**:
  - Clears all text in the textarea
  - Disabled when textarea is empty
  - Does not affect the title field

- **Cancel** (Edit mode only):
  - Discards changes and returns to Notes List view

#### Helper Text
- Displays a tip: "💡 Tip: Select text from the page and click 'Notes' to append it to your note."

---

### 4. **Text Selection Integration**

#### Selection Toolbar
When text is selected on the page:
1. A toolbar appears near the selection with various options
2. **Notes Button**: Clicking this button:
   - Opens the Notes panel (if not already open)
   - Switches to Add/Edit view
   - Appends the selected text to the active note's textarea

#### Appending Behavior
- **New Note**: If Add Note panel is active, selected text is appended to the textarea
- **Existing Note**: If editing an existing note, selected text is appended with proper spacing
- **Multiple Selections**: Users can select and append multiple text blocks sequentially
- **Formatting**: Selected text preserves its original formatting

#### Use Cases
- Collecting quotes from different sections of documentation
- Building study notes from multiple pages
- Creating summaries by selecting key points

---

### 5. **Opening Notes Panel**

#### Via Navbar Tab
- Click the **"Notes"** tab in the secondary navigation bar
- Default view: Notes List (if notes exist) or empty state (if no notes)

#### Via Text Selection
- Select text from page content
- Click **"Notes"** from the SelectionToolbar
- Panel opens in Add/Edit view with selected text pre-populated

---

## Technical Implementation

### Architecture

#### Context Management
- **NotesContext** (`src/contexts/NotesContext.tsx`)
  - Provides state management for all notes
  - Handles CRUD operations (Create, Read, Update, Delete)
  - Manages active note and view state
  - Persists notes to localStorage

#### Components
- **NotesContent** (`src/components/CustomNavbar/NotesContent.tsx`)
  - Main component rendering list and add/edit views
  - Handles search filtering
  - Manages form state and validation
  - Listens for text selection events

- **RightDrawer** (`src/components/CustomNavbar/RightDrawer.tsx`)
  - Container for all drawer panels (Bookmark, Mindmap, Notes, Assessment)
  - Handles drawer open/close state
  - Manages ESC key handling

#### Integration Points
- **Root Provider** (`src/theme/Root.tsx`)
  - Wraps entire app with NotesProvider
  - Ensures context is available throughout the application

- **Custom Navbar** (`src/components/CustomNavbar/index.tsx`)
  - Handles Notes button click in navbar
  - Manages drawer opening logic
  - Dispatches custom events for text appending

- **SelectionToolbar** (`src/components/CustomNavbar/SelectionToolbar.tsx`)
  - Provides Notes button in text selection toolbar
  - Triggers note creation/editing with selected text

---

### Data Structure

#### Note Interface
```typescript
interface Note {
  id: string;              // Unique identifier (generated)
  title: string;           // Note title
  content: string;         // Note content (HTML string)
  timestamp: number;       // Creation/update timestamp
}
```

#### Context State
```typescript
{
  notes: Record<string, Note>;        // All notes keyed by ID
  activeNoteId: string | null;        // Currently editing note
  view: 'list' | 'add' | 'edit';     // Current view mode
}
```

---

### Storage

- **Method**: Browser localStorage
- **Key**: `userNotes`
- **Format**: JSON serialized object
- **Persistence**: Automatic on every change
- **SSR Safe**: Uses `ExecutionEnvironment.canUseDOM` checks

---

### Event-Driven Communication

#### Custom Events

**`appendToNote`**
- Dispatched when: User selects text and clicks "Notes" button
- Payload: `{ selectedText: string }`
- Handler: NotesContent component
- Behavior: Appends text to active note's textarea

---

## User Workflows

### Workflow 1: Manual Note Creation

1. Click **"Notes"** tab in navbar
2. Click **"Add Note"** button
3. Enter note title
4. Type or paste content in textarea
5. Click **"Save"** to save the note
6. Note appears in the Notes List

### Workflow 2: Note Creation from Text Selection

1. Navigate to any documentation page
2. Select desired text
3. Click **"Notes"** from SelectionToolbar
4. Notes panel opens with text pre-populated in textarea
5. Add a descriptive title
6. Optionally select and append more text
7. Click **"Save"** to save the note

### Workflow 3: Editing an Existing Note

1. Open Notes panel via navbar
2. Locate the note in the list (use search if needed)
3. Click **"Edit"** button on the note card
4. Modify title and/or content
5. Click **"Save"** to update the note
6. Or click **"Cancel"** to discard changes

### Workflow 4: Deleting a Note

1. Open Notes panel
2. Locate the note to delete
3. Click **"Delete"** button on the note card
4. Confirm deletion in the confirmation dialog
5. Note is removed from the list

### Workflow 5: Searching Notes

1. Open Notes panel
2. Type search query in the search field
3. Notes are filtered in real-time by title and content
4. Clear search field to view all notes

---

## Styling

### CSS Architecture
- **File**: `src/components/CustomNavbar/notesContent.css`
- **Approach**: BEM methodology with modular classes
- **Theming**: Uses Docusaurus CSS variables for consistency
- **Dark Mode**: Fully supports light and dark themes
- **Responsive**: Adapts to different screen sizes

### Key Design Elements
- Clean, minimal interface
- Consistent spacing and typography
- Smooth transitions and hover effects
- Accessible color contrast ratios
- Custom scrollbar styling

---

## Accessibility

- **Keyboard Navigation**: All buttons are keyboard accessible
- **ARIA Labels**: Proper labeling for screen readers
- **Focus Management**: Clear focus indicators
- **Semantic HTML**: Proper heading hierarchy and structure
- **Color Contrast**: WCAG AA compliant color ratios

---

## Browser Compatibility

- **Modern Browsers**: Chrome, Firefox, Safari, Edge (latest versions)
- **Features Used**:
  - localStorage API
  - Custom Events
  - ContentEditable API
  - Flexbox Layout
  - CSS Variables

---

## Future Enhancements

Potential improvements for future releases:

1. **Rich Text Formatting Toolbar**
   - Add bold, italic, underline, lists
   - Code blocks and syntax highlighting
   - Link insertion

2. **Note Categories/Tags**
   - Organize notes by topics
   - Filter by tags
   - Color-coded categories

3. **Export Functionality**
   - Export notes as Markdown
   - Export as PDF
   - Copy to clipboard

4. **Note Templates**
   - Pre-defined note structures
   - Custom templates
   - Quick note types (summary, question, code snippet)

5. **Collaboration Features**
   - Share notes with others
   - Comment on notes
   - Collaborative editing

6. **Cloud Sync**
   - Sync notes across devices
   - Backup and restore
   - Version history

7. **Advanced Search**
   - Filter by date range
   - Sort by various criteria
   - Full-text search with highlighting

---

## Troubleshooting

### Issue: Notes not saving
**Solution**: Check browser localStorage is enabled and not full

### Issue: Selected text not appending
**Solution**: Ensure Notes panel is open in Add/Edit view, then try selecting text again

### Issue: Search not working
**Solution**: Refresh the page to reload notes from localStorage

### Issue: Notes lost after browser close
**Solution**: Check if browser is in private/incognito mode which clears localStorage on close

---

## Related Features

- **Bookmarks**: Save specific sections for quick reference
- **Mindmap**: Visualize content structure
- **Assessment**: Track learning progress
- **Highlight**: Mark important text with colors

---

## File Locations

### Core Files
```
book-source/
├── src/
│   ├── contexts/
│   │   └── NotesContext.tsx           # State management
│   ├── components/
│   │   └── CustomNavbar/
│   │       ├── NotesContent.tsx       # Main component
│   │       ├── notesContent.css       # Styles
│   │       ├── RightDrawer.tsx        # Updated to include Notes
│   │       ├── SelectionToolbar.tsx   # Text selection handler
│   │       └── index.tsx              # Navbar integration
│   └── theme/
│       └── Root.tsx                   # Provider integration
└── NOTES_FEATURE.md                   # This documentation
```

---

## Summary

The Notes feature provides a powerful, user-friendly way to capture and organize learning notes directly within the documentation. With seamless text selection integration, rich text support, and persistent storage, users can build a comprehensive personal knowledge base while exploring the AI Native Software Development book.

**Key Benefits:**
- ✅ Quick note-taking without leaving the documentation
- ✅ Text selection integration for rapid content capture
- ✅ Search and organization capabilities
- ✅ Persistent storage across sessions
- ✅ Clean, accessible interface

---

**Version**: 1.0.0
**Last Updated**: 2025-01-18
**Status**: Production Ready
