'use client';

import React from 'react';

export interface StatusBarProps {
    /** The programming language being displayed */
    language?: string;
    /** Current line count */
    lineCount: number;
    /** Git branch name */
    branch?: string;
    /** Number of errors */
    errors?: number;
    /** Number of warnings */
    warnings?: number;
}

/**
 * A VS Code-style status bar component.
 */
export function StatusBar({
    language = 'Python',
    lineCount,
    branch = 'main',
    errors = 0,
    warnings = 0,
}: StatusBarProps) {
    return (
        <div className="flex items-center justify-between px-3 py-1 bg-[#1e1e1e] border-t border-white/10 text-xs text-white/80 rounded-b-2xl shrink-0">
            <div className="flex items-center gap-4">
                <span>③ {branch}</span>
                <span>○ {errors} ⚠ {warnings}</span>
            </div>
            <div className="flex items-center gap-4">
                <span>{language}</span>
                <span>UTF-8</span>
                <span>Ln {lineCount}, Col 1</span>
            </div>
        </div>
    );
}
