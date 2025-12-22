import React from 'react';

interface WindowFrameProps {
    filename: string;
    language: 'python' | 'typescript';
    children: React.ReactNode;
}

const languageIcons: Record<string, string> = {
    python: '🐍',
    typescript: '📘',
};

export function WindowFrame({ filename, language, children }: WindowFrameProps) {
    return (
        <div className="rounded-xl overflow-hidden shadow-2xl border border-zinc-700/50 bg-zinc-900 dark:bg-zinc-950">
            {/* Title bar */}
            <div className="flex items-center gap-2 px-4 py-3 bg-zinc-800 dark:bg-zinc-900 border-b border-zinc-700/50">
                {/* Traffic lights */}
                <div className="flex gap-2">
                    <div className="w-3 h-3 rounded-full bg-red-500" />
                    <div className="w-3 h-3 rounded-full bg-yellow-500" />
                    <div className="w-3 h-3 rounded-full bg-green-500" />
                </div>

                {/* Filename */}
                <div className="flex-1 flex items-center justify-center gap-2">
                    <span className="text-base">{languageIcons[language]}</span>
                    <span className="text-sm font-mono text-zinc-400">{filename}</span>
                </div>

                {/* Spacer for symmetry */}
                <div className="w-14" />
            </div>

            {/* Content */}
            <div className="flex flex-col">
                {children}
            </div>
        </div>
    );
}
