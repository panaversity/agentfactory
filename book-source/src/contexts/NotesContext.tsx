import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import React, { createContext, useCallback, useContext, useEffect, useState } from 'react';

export interface Note {
  id: string;
  name: string;
  content: string;
  pageTitle: string;
  pageUrl: string;
  timestamp: number;        // Creation timestamp
  lastModified: number;     // Last modification timestamp
  // Text-based anchoring (works for any text selection)
  textAnchor?: {
    selectedText: string;      // The exact text that was selected
    prefix: string;             // Text before selection (for context)
    suffix: string;             // Text after selection (for context)
    startOffset: number;        // Character offset within container
    endOffset: number;          // Character offset within container
  };
}

interface NotesContextValue {
  notes: Record<string, Note>;
  addNote: (note: Omit<Note, 'id' | 'timestamp' | 'lastModified'>) => void;
  updateNote: (id: string, updates: { name?: string; content?: string }) => void;
  deleteNote: (id: string) => void;
  getNotesByPage: (pageUrl: string) => Note[];
  getAllNotes: () => Note[];
  // Selected text for note creation
  selectedText: string;
  setSelectedText: (text: string) => void;
  // Initial view mode
  initialView: 'add' | 'view';
  setInitialView: (view: 'add' | 'view') => void;
  // Current note being viewed/edited
  currentNoteId: string | null;
  setCurrentNoteId: (id: string | null) => void;
}

const NotesContext = createContext<NotesContextValue | undefined>(undefined);

const STORAGE_KEY = 'pageNotes';

export const NotesProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [notes, setNotes] = useState<Record<string, Note>>({});
  const [selectedText, setSelectedText] = useState<string>('');
  const [initialView, setInitialView] = useState<'add' | 'view'>('view');
  const [currentNoteId, setCurrentNoteId] = useState<string | null>(null);

  // Load notes from localStorage on mount
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    try {
      const stored = localStorage.getItem(STORAGE_KEY);
      if (stored) {
        const parsed = JSON.parse(stored);
        setNotes(parsed);
      }
    } catch (error) {
      console.error('Error loading notes:', error);
    }
  }, []);

  // Save notes to localStorage whenever they change
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(notes));
    } catch (error) {
      console.error('Error saving notes:', error);
    }
  }, [notes]);

  const addNote = useCallback((note: Omit<Note, 'id' | 'timestamp' | 'lastModified'>) => {
    const id = `note_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    const now = Date.now();
    const newNote: Note = {
      ...note,
      id,
      timestamp: now,
      lastModified: now,
    };

    setNotes((prev) => ({
      ...prev,
      [id]: newNote,
    }));

    return id; // Return the ID for potential immediate use
  }, []);

  const updateNote = useCallback((id: string, updates: { name?: string; content?: string }) => {
    setNotes((prev) => {
      if (!prev[id]) return prev;

      return {
        ...prev,
        [id]: {
          ...prev[id],
          ...(updates.name !== undefined && { name: updates.name }),
          ...(updates.content !== undefined && { content: updates.content }),
          lastModified: Date.now(),
        },
      };
    });
  }, []);

  const deleteNote = useCallback((id: string) => {
    setNotes((prev) => {
      const updated = { ...prev };
      delete updated[id];
      return updated;
    });
  }, []);

  const getNotesByPage = useCallback(
    (pageUrl: string) => {
      return Object.values(notes).filter((note) => note.pageUrl === pageUrl);
    },
    [notes],
  );

  const getAllNotes = useCallback(() => {
    return Object.values(notes).sort((a, b) => b.lastModified - a.lastModified);
  }, [notes]);

  const value: NotesContextValue = {
    notes,
    addNote,
    updateNote,
    deleteNote,
    getNotesByPage,
    getAllNotes,
    selectedText,
    setSelectedText,
    initialView,
    setInitialView,
    currentNoteId,
    setCurrentNoteId,
  };

  return <NotesContext.Provider value={value}>{children}</NotesContext.Provider>;
};

export const useNotes = () => {
  const context = useContext(NotesContext);
  if (context === undefined) {
    throw new Error('useNotes must be used within a NotesProvider');
  }
  return context;
};
