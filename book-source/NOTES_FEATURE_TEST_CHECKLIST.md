# Notes Feature - Testing Checklist

## Manual Testing Checklist

### ✅ Basic Functionality

#### Notes Panel Access
- [ ] Click "Notes" tab in navbar - panel opens
- [ ] Panel shows empty state when no notes exist
- [ ] "Add Note" button is visible in empty state
- [ ] Panel can be closed with X button
- [ ] Panel can be closed with ESC key

#### Add New Note (Manual)
- [ ] Click "Add Note" button - switches to Add view
- [ ] Default title is "New Note"
- [ ] Title field is editable
- [ ] Textarea accepts text input
- [ ] Textarea is contenteditable (supports rich text)
- [ ] Save button is enabled when title is not empty
- [ ] Clear button is disabled when textarea is empty
- [ ] Clear button clears textarea content
- [ ] Save creates note and returns to list view
- [ ] New note appears in the list

#### Edit Existing Note
- [ ] Click "Edit" button on note card
- [ ] Note opens in edit view with prefilled data
- [ ] Title field shows existing title
- [ ] Textarea shows existing content (with HTML preserved)
- [ ] Changes to title are saved
- [ ] Changes to content are saved
- [ ] Cancel button discards changes
- [ ] Save button updates note and returns to list

#### Delete Note
- [ ] Click "Delete" button on note card
- [ ] Confirmation dialog appears
- [ ] Confirming removes note from list
- [ ] Canceling keeps note in list
- [ ] Deleted note is removed from localStorage

#### Search Notes
- [ ] Search field is visible in list view
- [ ] Typing in search filters notes by title
- [ ] Search also filters by content
- [ ] Search is case-insensitive
- [ ] Clearing search shows all notes
- [ ] Search results update in real-time

---

### ✅ Text Selection Integration

#### Selection Toolbar
- [ ] Select text on documentation page
- [ ] Selection toolbar appears near selected text
- [ ] "Notes" button is visible in toolbar
- [ ] Clicking "Notes" opens Notes panel

#### Appending Selected Text
- [ ] With Notes panel closed:
  - [ ] Select text and click "Notes"
  - [ ] Panel opens in Add view
  - [ ] Selected text is appended to textarea
- [ ] With Notes panel open (Add view):
  - [ ] Select text and click "Notes"
  - [ ] Text is appended to existing content
- [ ] With Notes panel open (Edit view):
  - [ ] Select text and click "Notes"
  - [ ] Text is appended to existing note content
- [ ] Multiple selections can be appended sequentially
- [ ] Text is separated with proper spacing (line breaks)

#### Selection Behavior
- [ ] Selected text preserves formatting
- [ ] Long selections are appended completely
- [ ] Special characters are handled correctly
- [ ] HTML in selected text is preserved

---

### ✅ UI/UX

#### Visual Design
- [ ] Notes panel matches Bookmark panel styling
- [ ] Card design is consistent with existing patterns
- [ ] Colors follow theme variables
- [ ] Spacing and padding are consistent
- [ ] Buttons have clear hover states
- [ ] Scrollbars are styled

#### Responsive Design
- [ ] Panel width is appropriate
- [ ] Content is scrollable when it exceeds viewport
- [ ] Cards stack properly in list view
- [ ] Form fields are appropriately sized

#### Dark Mode
- [ ] All components work in dark mode
- [ ] Text is readable on dark backgrounds
- [ ] Card backgrounds are visible
- [ ] Input fields have correct dark mode colors
- [ ] Buttons have appropriate dark mode styling

#### Accessibility
- [ ] All buttons are keyboard accessible
- [ ] Tab navigation works correctly
- [ ] Focus indicators are visible
- [ ] Placeholders are clear and helpful
- [ ] Button labels are descriptive

---

### ✅ Data Persistence

#### localStorage
- [ ] New notes are saved to localStorage
- [ ] Edited notes persist changes
- [ ] Deleted notes are removed from storage
- [ ] Closing and reopening browser preserves notes
- [ ] Refreshing page shows all saved notes
- [ ] Multiple notes are stored correctly

#### State Management
- [ ] Context provides notes to components
- [ ] Adding note updates context immediately
- [ ] Editing note updates context
- [ ] Deleting note updates context
- [ ] Search state is maintained during session
- [ ] View state (list/add/edit) is managed correctly

---

### ✅ Edge Cases

#### Empty States
- [ ] No notes - shows empty message and "Add Note" button
- [ ] Search with no results - shows appropriate message
- [ ] Empty title - Save button validation prevents save
- [ ] Empty content - Clear button is disabled

#### Long Content
- [ ] Very long note titles wrap correctly
- [ ] Long note content shows preview (200 chars)
- [ ] Full content is visible in edit view
- [ ] Textarea scrolls when content exceeds max height

#### Special Characters
- [ ] Unicode characters are saved correctly
- [ ] Emojis are displayed properly
- [ ] HTML entities are handled correctly
- [ ] Code snippets preserve formatting

#### Performance
- [ ] List view renders quickly with many notes (50+)
- [ ] Search is responsive with large number of notes
- [ ] Appending text doesn't lag
- [ ] No memory leaks on repeated actions

---

### ✅ Integration Testing

#### With Other Features
- [ ] Notes panel doesn't interfere with Bookmark panel
- [ ] Selection toolbar shows both Bookmark and Notes buttons
- [ ] Sidebar collapse works when Notes panel is open
- [ ] TOC toggle works when Notes panel is open
- [ ] Chat panel can coexist with Notes panel

#### Navigation
- [ ] Opening Notes panel from different pages works
- [ ] Notes persist across page navigation
- [ ] Panel state is maintained during client-side routing
- [ ] External links don't break Notes functionality

---

### ✅ Browser Compatibility

- [ ] Chrome (latest)
- [ ] Firefox (latest)
- [ ] Safari (latest)
- [ ] Edge (latest)

---

### ✅ Build & Deployment

- [ ] `npm run build:fast` completes successfully
- [ ] No TypeScript errors
- [ ] No console errors in production build
- [ ] SSR doesn't break (localStorage errors are expected and handled)

---

## Automated Testing (Future)

### Unit Tests
- [ ] NotesContext CRUD operations
- [ ] Search filtering logic
- [ ] Note timestamp generation
- [ ] localStorage save/load functions

### Integration Tests
- [ ] Component rendering
- [ ] Event handlers
- [ ] Form validation
- [ ] Custom event dispatching

### E2E Tests
- [ ] Complete user workflows
- [ ] Text selection integration
- [ ] Multi-step interactions
- [ ] Cross-browser testing

---

## Known Issues

### None Currently Identified

---

## Test Environment

- **Browser**: Chrome/Firefox/Safari/Edge
- **OS**: Windows/macOS/Linux
- **Node Version**: >=20.0
- **npm Version**: Latest
- **Docusaurus Version**: 3.9.2

---

## Testing Notes

Record any issues or observations during testing:

1. **Test Date**: _____________
2. **Tester**: _____________
3. **Browser/OS**: _____________
4. **Issues Found**: _____________
5. **Additional Comments**: _____________

---

**Status**: Ready for Testing
**Version**: 1.0.0
**Last Updated**: 2025-01-18
