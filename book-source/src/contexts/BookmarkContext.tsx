import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import React, { createContext, useCallback, useContext, useEffect, useState } from 'react';

// Controls visibility/behavior of the page Table of Contents
// - 'hidden'   => only the small "Show Table of Contents" button is visible
// - 'expanded' => full TOC panel is visible (default once user chooses to show)
// - 'collapsed' is kept for forward-compatibility but not currently used
export type TocMode = 'expanded' | 'collapsed' | 'hidden';

export interface Bookmark {
  id: string;
  name: string;
  pageTitle: string;
  pageUrl: string;
  note: string;
  timestamp: number;
  isEntirePage: boolean;
  elementId?: string; // ID of heading if available (for TOC items)
  // Text-based anchoring (works for any text, not just headings)
  textAnchor?: {
    selectedText: string;      // The exact text that was selected
    prefix: string;             // Text before selection (for context)
    suffix: string;             // Text after selection (for context)
    startOffset: number;        // Character offset within container
    endOffset: number;          // Character offset within container
  };
}

// Lightweight representation of the current doc's TOC + metadata,
// populated from the DocItem component via useDoc().
export interface CurrentDocInfo {
  toc: readonly {
    value: string;
    id: string;
    level: number;
    children: CurrentDocInfo['toc'];
  }[];
  metadata: {
    title: string;
    permalink: string;
  };
}

interface BookmarkContextValue {
  bookmarks: Record<string, Bookmark>;
  addBookmark: (bookmark: Omit<Bookmark, 'id' | 'timestamp'>) => void;
  deleteBookmark: (id: string) => void;
  updateBookmarkNote: (id: string, note: string) => void;
  getBookmarksByPage: (pageUrl: string) => Bookmark[];
  isBookmarked: (pageUrl: string) => boolean;
  // Global TOC visibility state
  hideTOC: boolean;
  setHideTOC: (hide: boolean) => void;
  tocMode: TocMode;
  setTocMode: (mode: TocMode) => void;
  // Current doc info (TOC + metadata)
  currentDoc: CurrentDocInfo | null;
  setCurrentDoc: (doc: CurrentDocInfo | null) => void;
  // Selected text for bookmarking
  selectedText: string;
  setSelectedText: (text: string) => void;
  // Selected element ID for bookmarking
  selectedElementId: string | undefined;
  setSelectedElementId: (elementId: string | undefined) => void;
  // Initial view mode
  initialView: 'add' | 'view';
  setInitialView: (view: 'add' | 'view') => void;
}

const BookmarkContext = createContext<BookmarkContextValue | undefined>(undefined);

const STORAGE_KEY = 'pageBookmarks';
const TOC_PREFS_KEY = 'tocPreferences';

export const BookmarkProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [bookmarks, setBookmarks] = useState<Record<string, Bookmark>>({});
  const [hideTOC, setHideTOC] = useState(false);
  // Start with TOC hidden by default; user can explicitly choose to show it
  const [tocMode, setTocMode] = useState<TocMode>('hidden');
  // Current doc info (set from DocItem via useDoc)
  const [currentDoc, setCurrentDoc] = useState<CurrentDocInfo | null>(null);
  // Selected text for bookmarking
  const [selectedText, setSelectedText] = useState<string>('');
  // Selected element ID for bookmarking
  const [selectedElementId, setSelectedElementId] = useState<string | undefined>(undefined);
  // Initial view mode
  const [initialView, setInitialView] = useState<'add' | 'view'>('view');

  // Load bookmarks from localStorage on mount
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    try {
      const stored = localStorage.getItem(STORAGE_KEY);
      if (stored) {
        const parsed = JSON.parse(stored);
        setBookmarks(parsed);
      }
    } catch (error) {
      console.error('Error loading bookmarks:', error);
    }

    try {
      const tocPrefsRaw = localStorage.getItem(TOC_PREFS_KEY);
      if (tocPrefsRaw) {
        const tocPrefs = JSON.parse(tocPrefsRaw) as { mode?: TocMode };
        if (
          tocPrefs.mode === 'expanded' ||
          tocPrefs.mode === 'collapsed' ||
          tocPrefs.mode === 'hidden'
        ) {
          setTocMode(tocPrefs.mode);
        }
      }
    } catch (error) {
      console.error('Error loading TOC preferences:', error);
    }
  }, []);

  // Save bookmarks to localStorage whenever they change
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(bookmarks));
    } catch (error) {
      console.error('Error saving bookmarks:', error);
    }
  }, [bookmarks]);

  // Persist TOC mode so users keep their preference
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    try {
      localStorage.setItem(TOC_PREFS_KEY, JSON.stringify({ mode: tocMode }));
    } catch (error) {
      console.error('Error saving TOC preferences:', error);
    }
  }, [tocMode]);

  const addBookmark = useCallback((bookmark: Omit<Bookmark, 'id' | 'timestamp'>) => {
    const id = `bookmark_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    const newBookmark: Bookmark = {
      ...bookmark,
      id,
      timestamp: Date.now(),
    };

    setBookmarks((prev) => ({
      ...prev,
      [id]: newBookmark,
    }));
  }, []);

  const deleteBookmark = useCallback((id: string) => {
    setBookmarks((prev) => {
      const updated = { ...prev };
      delete updated[id];
      return updated;
    });
  }, []);

  const updateBookmarkNote = useCallback((id: string, note: string) => {
    setBookmarks((prev) => ({
      ...prev,
      [id]: {
        ...prev[id],
        note,
      },
    }));
  }, []);

  const getBookmarksByPage = useCallback(
    (pageUrl: string) => {
      return Object.values(bookmarks).filter((bookmark) => bookmark.pageUrl === pageUrl);
    },
    [bookmarks],
  );

  const isBookmarked = useCallback(
    (pageUrl: string) => {
      return Object.values(bookmarks).some(
        (bookmark) => bookmark.pageUrl === pageUrl && bookmark.isEntirePage,
      );
    },
    [bookmarks],
  );

  const value: BookmarkContextValue = {
    bookmarks,
    addBookmark,
    deleteBookmark,
    updateBookmarkNote,
    getBookmarksByPage,
    isBookmarked,
    hideTOC,
    setHideTOC,
    tocMode,
    setTocMode,
    currentDoc,
    setCurrentDoc,
    selectedText,
    setSelectedText,
    selectedElementId,
    setSelectedElementId,
    initialView,
    setInitialView,
  };

  return <BookmarkContext.Provider value={value}>{children}</BookmarkContext.Provider>;
};

export const useBookmarks = () => {
  const context = useContext(BookmarkContext);
  if (context === undefined) {
    throw new Error('useBookmarks must be used within a BookmarkProvider');
  }
  return context;
};
