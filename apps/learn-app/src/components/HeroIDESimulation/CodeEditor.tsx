import React from 'react';

interface CodeEditorProps {
    code: string;
    language: 'python' | 'typescript';
    showCursor?: boolean;
}

// Simple syntax highlighting using CSS classes
function highlightCode(code: string, language: string): React.ReactNode[] {
    const lines = code.split('\n');

    // Keywords for each language
    const pythonKeywords = ['from', 'import', 'def', 'async', 'await', 'return', 'class', 'if', 'else', 'for', 'in', 'None', 'True', 'False', 'Optional'];
    const tsKeywords = ['import', 'export', 'from', 'const', 'let', 'async', 'await', 'return', 'function', 'type', 'interface', 'class', 'if', 'else', 'for', 'of', 'in', 'null', 'undefined', 'true', 'false'];

    const keywords = language === 'python' ? pythonKeywords : tsKeywords;

    return lines.map((line, lineIndex) => {
        // Simple tokenization
        const tokens: React.ReactNode[] = [];
        let remaining = line;
        let tokenIndex = 0;

        while (remaining.length > 0) {
            let matched = false;

            // Check for strings (single or double quotes)
            const stringMatch = remaining.match(/^(['"`])(.*?)\1/) || remaining.match(/^(['"`])([^'"`]*)/);
            if (stringMatch) {
                tokens.push(
                    <span key={tokenIndex++} className="text-green-400">
                        {stringMatch[0]}
                    </span>
                );
                remaining = remaining.slice(stringMatch[0].length);
                matched = true;
                continue;
            }

            // Check for decorators (Python) or decorators-like patterns
            const decoratorMatch = remaining.match(/^@\w+/);
            if (decoratorMatch) {
                tokens.push(
                    <span key={tokenIndex++} className="text-yellow-400">
                        {decoratorMatch[0]}
                    </span>
                );
                remaining = remaining.slice(decoratorMatch[0].length);
                matched = true;
                continue;
            }

            // Check for comments
            const commentMatch = remaining.match(/^(#|\/\/).*/);
            if (commentMatch) {
                tokens.push(
                    <span key={tokenIndex++} className="text-zinc-500 italic">
                        {commentMatch[0]}
                    </span>
                );
                remaining = '';
                matched = true;
                continue;
            }

            // Check for keywords
            for (const keyword of keywords) {
                const keywordRegex = new RegExp(`^\\b${keyword}\\b`);
                if (keywordRegex.test(remaining)) {
                    tokens.push(
                        <span key={tokenIndex++} className="text-purple-400 font-medium">
                            {keyword}
                        </span>
                    );
                    remaining = remaining.slice(keyword.length);
                    matched = true;
                    break;
                }
            }
            if (matched) continue;

            // Check for function definitions
            const funcMatch = remaining.match(/^(\w+)(?=\()/);
            if (funcMatch) {
                tokens.push(
                    <span key={tokenIndex++} className="text-blue-400">
                        {funcMatch[0]}
                    </span>
                );
                remaining = remaining.slice(funcMatch[0].length);
                matched = true;
                continue;
            }

            // Check for types (capitalized words)
            const typeMatch = remaining.match(/^[A-Z]\w*/);
            if (typeMatch) {
                tokens.push(
                    <span key={tokenIndex++} className="text-teal-400">
                        {typeMatch[0]}
                    </span>
                );
                remaining = remaining.slice(typeMatch[0].length);
                matched = true;
                continue;
            }

            // Default: take one character
            tokens.push(remaining[0]);
            remaining = remaining.slice(1);
        }

        return (
            <div key={lineIndex} className="flex">
                <span className="w-8 text-right pr-4 text-zinc-600 select-none text-xs">
                    {lineIndex + 1}
                </span>
                <span className="flex-1">{tokens}</span>
            </div>
        );
    });
}

export function CodeEditor({ code, language, showCursor = false }: CodeEditorProps) {
    const highlightedCode = highlightCode(code, language);

    return (
        <div className="bg-zinc-900 dark:bg-zinc-950 p-4 font-mono text-sm min-h-[200px] overflow-x-auto">
            <pre className="leading-relaxed text-zinc-100">
                {code ? (
                    <code>{highlightedCode}</code>
                ) : (
                    <span className="text-zinc-600 italic">// Code will appear here...</span>
                )}
                {showCursor && (
                    <span className="inline-block w-2 h-4 bg-teal-400 animate-pulse ml-0.5 align-middle" />
                )}
            </pre>
        </div>
    );
}
