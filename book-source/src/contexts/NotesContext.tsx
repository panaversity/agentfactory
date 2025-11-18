import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import React, { createContext, useCallback, useContext, useEffect, useState } from 'react';

export interface Note {
  id: string;
  title: string;
  content: string;
  timestamp: number;
}

interface NotesContextValue {
  notes: Record<string, Note>;
  addNote: (note: Omit<Note, 'id' | 'timestamp'>) => void;
  updateNote: (id: string, updates: Partial<Omit<Note, 'id' | 'timestamp'>>) => void;
  deleteNote: (id: string) => void;
  searchNotes: (query: string) => Note[];
  // Active note for editing
  activeNoteId: string | null;
  setActiveNoteId: (id: string | null) => void;
  // View mode
  view: 'list' | 'add' | 'edit';
  setView: (view: 'list' | 'add' | 'edit') => void;
}

const NotesContext = createContext<NotesContextValue | undefined>(undefined);

const STORAGE_KEY = 'userNotes';

export const NotesProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [notes, setNotes] = useState<Record<string, Note>>({});
  const [activeNoteId, setActiveNoteId] = useState<string | null>(null);
  const [view, setView] = useState<'list' | 'add' | 'edit'>('list');

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

  const addNote = useCallback((note: Omit<Note, 'id' | 'timestamp'>) => {
    const id = `note_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    const newNote: Note = {
      ...note,
      id,
      timestamp: Date.now(),
    };

    setNotes((prev) => ({
      ...prev,
      [id]: newNote,
    }));

    return id;
  }, []);

  const updateNote = useCallback((id: string, updates: Partial<Omit<Note, 'id' | 'timestamp'>>) => {
    setNotes((prev) => {
      if (!prev[id]) return prev;

      return {
        ...prev,
        [id]: {
          ...prev[id],
          ...updates,
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

  const searchNotes = useCallback(
    (query: string): Note[] => {
      if (!query.trim()) {
        return Object.values(notes).sort((a, b) => b.timestamp - a.timestamp);
      }

      const lowerQuery = query.toLowerCase();
      return Object.values(notes)
        .filter(
          (note) =>
            note.title.toLowerCase().includes(lowerQuery) ||
            note.content.toLowerCase().includes(lowerQuery)
        )
        .sort((a, b) => b.timestamp - a.timestamp);
    },
    [notes]
  );

  const value: NotesContextValue = {
    notes,
    addNote,
    updateNote,
    deleteNote,
    searchNotes,
    activeNoteId,
    setActiveNoteId,
    view,
    setView,
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
