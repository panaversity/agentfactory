'use client';

import React from 'react';
import { useAnimationSequence } from './useAnimationSequence';
import { pythonSequence } from './sequences';
import { TrafficLights } from './TrafficLightDot';
import { Sidebar } from './SidebarIcon';
import { StatusBar } from './StatusBar';
import { ChatPanel, ChatMessage } from './ChatPanel';

// Authentic macOS IDE styles - Xcode/VSCode inspired
const macOSStyles = `
  @keyframes fadeIn {
    from { opacity: 0; transform: translateY(8px); }
    to { opacity: 1; transform: translateY(0); }
  }
  
  @keyframes blink {
    0%, 50% { opacity: 1; }
    51%, 100% { opacity: 0; }
  }
  
  @keyframes typing-dots {
    0%, 20% { opacity: 0.3; }
    50% { opacity: 1; }
    80%, 100% { opacity: 0.3; }
  }

  @keyframes pulseGlow {
    0%, 100% { box-shadow: 0 0 8px rgba(59, 130, 246, 0.4); }
    50% { box-shadow: 0 0 16px rgba(59, 130, 246, 0.6); }
  }
  
  .animate-fadeIn {
    animation: fadeIn 0.3s ease-out forwards;
  }
  
  .cursor-blink {
    animation: blink 1s step-end infinite;
  }

  /* Authentic macOS window shadow - matches Sonoma/Ventura */
  .macos-window {
    box-shadow: 
      0 0 0 0.5px rgba(0, 0, 0, 0.15),
      0 1px 1px rgba(0, 0, 0, 0.1),
      0 4px 8px rgba(0, 0, 0, 0.12),
      0 16px 32px rgba(0, 0, 0, 0.15),
      0 32px 64px rgba(0, 0, 0, 0.2);
    border-radius: 16px;
  }

  /* macOS title bar vibrancy effect */
  .macos-titlebar {
    background: linear-gradient(180deg, #3d3d3d 0%, #323232 50%, #2d2d2d 100%);
    border-bottom: 1px solid rgba(0, 0, 0, 0.4);
    border-top-left-radius: 16px;
    border-top-right-radius: 16px;
  }

  /* macOS sidebar vibrancy */
  .macos-sidebar {
    background: linear-gradient(180deg, #2a2a2a 0%, #262626 100%);
  }

  /* Custom scrollbar for code editor - macOS style */
  .code-scrollbar::-webkit-scrollbar {
    width: 10px;
    height: 10px;
  }
  
  .code-scrollbar::-webkit-scrollbar-track {
    background: transparent;
  }
  
  .code-scrollbar::-webkit-scrollbar-thumb {
    background: rgba(255, 255, 255, 0.15);
    border-radius: 5px;
    border: 2px solid transparent;
    background-clip: content-box;
  }
  
  .code-scrollbar::-webkit-scrollbar-thumb:hover {
    background: rgba(255, 255, 255, 0.25);
    background-clip: content-box;
  }

  .code-scrollbar::-webkit-scrollbar-corner {
    background: transparent;
  }

  /* Typing dots animation */
  .typing-dot {
    animation: typing-dots 1.4s infinite ease-in-out;
  }
  .typing-dot:nth-child(1) { animation-delay: 0s; }
  .typing-dot:nth-child(2) { animation-delay: 0.2s; }
  .typing-dot:nth-child(3) { animation-delay: 0.4s; }

  /* Traffic light hover container */
  .traffic-lights:hover .traffic-light-icon {
    opacity: 1;
  }
  .traffic-light-icon {
    opacity: 0;
    transition: opacity 0.15s ease;
  }
`;

// Python keywords for syntax highlighting
const PYTHON_KEYWORDS = [
    'from', 'import', 'def', 'async', 'await', 'return', 'class',
    'if', 'else', 'for', 'in', 'None', 'True', 'False', 'Optional',
    'try', 'except', 'with', 'as'
];

/**
 * Syntax highlighting with One Dark Pro theme colors.
 */
function highlightCode(code: string): React.ReactNode[] {
    const lines = code.split('\n');

    return lines.map((line, lineIndex) => {
        const tokens: React.ReactNode[] = [];
        let remaining = line;
        let tokenIndex = 0;

        while (remaining.length > 0) {
            // Strings (green)
            const stringMatch = remaining.match(/^(['"`])(.*?)\1/) || remaining.match(/^(['"`])([^'"`]*)/);
            if (stringMatch) {
                tokens.push(<span key={tokenIndex++} className="text-[#98c379]">{stringMatch[0]}</span>);
                remaining = remaining.slice(stringMatch[0].length);
                continue;
            }

            // Decorators (yellow)
            const decoratorMatch = remaining.match(/^@\w+/);
            if (decoratorMatch) {
                tokens.push(<span key={tokenIndex++} className="text-[#e5c07b]">{decoratorMatch[0]}</span>);
                remaining = remaining.slice(decoratorMatch[0].length);
                continue;
            }

            // Comments (gray italic)
            const commentMatch = remaining.match(/^#.*/);
            if (commentMatch) {
                tokens.push(<span key={tokenIndex++} className="text-[#5c6370] italic">{commentMatch[0]}</span>);
                remaining = '';
                continue;
            }

            // Keywords (purple)
            let keywordMatched = false;
            for (const keyword of PYTHON_KEYWORDS) {
                const keywordRegex = new RegExp(`^\\b${keyword}\\b`);
                if (keywordRegex.test(remaining)) {
                    tokens.push(<span key={tokenIndex++} className="text-[#c678dd]">{keyword}</span>);
                    remaining = remaining.slice(keyword.length);
                    keywordMatched = true;
                    break;
                }
            }
            if (keywordMatched) continue;

            // Function calls (blue)
            const funcMatch = remaining.match(/^(\w+)(?=\()/);
            if (funcMatch) {
                tokens.push(<span key={tokenIndex++} className="text-[#61afef]">{funcMatch[0]}</span>);
                remaining = remaining.slice(funcMatch[0].length);
                continue;
            }

            // Types/classes (cyan)
            const typeMatch = remaining.match(/^[A-Z]\w*/);
            if (typeMatch) {
                tokens.push(<span key={tokenIndex++} className="text-[#56b6c2]">{typeMatch[0]}</span>);
                remaining = remaining.slice(typeMatch[0].length);
                continue;
            }

            // Numbers (orange)
            const numberMatch = remaining.match(/^\d+(\.\d+)?/);
            if (numberMatch) {
                tokens.push(<span key={tokenIndex++} className="text-[#d19a66]">{numberMatch[0]}</span>);
                remaining = remaining.slice(numberMatch[0].length);
                continue;
            }

            // Default (light gray)
            tokens.push(<span key={tokenIndex++} className="text-[#abb2bf]">{remaining[0]}</span>);
            remaining = remaining.slice(1);
        }

        return (
            <div key={lineIndex} className="flex min-h-[1.5em]">
                <span className="w-[45px] text-right pr-4 text-[#4b5263] select-none text-[13px] shrink-0">
                    {lineIndex + 1}
                </span>
                <span className="flex-1">{tokens.length > 0 ? tokens : '\u00A0'}</span>
            </div>
        );
    });
}

// Language configuration
const LANGUAGE_CONFIG = {
    python: { icon: '🐍', name: 'Python' },
} as const;

/**
 * Main IDE Showcase Section component.
 * Simulates a macOS-style IDE with AI coding assistant.
 */
export function IDEShowcaseSection() {
    const { state, currentSequence } = useAnimationSequence([pythonSequence], {
        autoStart: true,
        pauseOnHidden: true,
    });

    // Check for reduced motion preference
    const [prefersReducedMotion, setPrefersReducedMotion] = React.useState(false);

    React.useEffect(() => {
        const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
        setPrefersReducedMotion(mediaQuery.matches);

        const handler = (e: MediaQueryListEvent) => setPrefersReducedMotion(e.matches);
        mediaQuery.addEventListener('change', handler);
        return () => mediaQuery.removeEventListener('change', handler);
    }, []);

    // Static content for reduced motion
    const staticCode = pythonSequence.steps.find(s => s.type === 'code')?.content || '';
    const displayedCode = prefersReducedMotion ? staticCode : state.displayedCode;

    const messages: ChatMessage[] = prefersReducedMotion
        ? [
            { id: '1', role: 'user', content: 'Create a REST API for user management' },
            { id: '2', role: 'agent', content: '✅ FastAPI endpoint ready with Pydantic validation!' },
        ]
        : state.messages;

    const lang = LANGUAGE_CONFIG.python;
    const lineCount = displayedCode.split('\n').length;

    return (
        <>
            <style>{macOSStyles}</style>

            <section className="min-h-[90vh] flex items-center justify-center py-6 pb-12 md:pb-20 rounded-xl allow-rounded bg-background">
                <div className="w-[95%] max-w-[1800px] mx-auto px-6 h-full">
                    {/* macOS Window */}
                    <div className="allow-rounded macos-window w-full overflow-hidden bg-[#1e1e1e] h-[calc(90vh-48px)] flex flex-col rounded-xl">

                        {/* Title Bar */}
                        <div className="macos-titlebar flex items-center px-3 py-2.5 shrink-0 rounded-t-xl">
                            <TrafficLights size={14} gap={8} />

                            {/* Window Title & Tab */}
                            <div className="flex-1 flex items-center justify-center">
                                <div className="flex items-center gap-2 px-3 py-1.5 bg-[#2d2d2d] border border-[#000000]/20 rounded-md relative group select-none">
                                    <span className="text-sm opacity-90">{lang.icon}</span>
                                    <span className="font-mono text-[13px] text-[#e4e4e4] opacity-90">
                                        {currentSequence.filename}
                                    </span>
                                    <div className="flex items-center justify-center w-4 h-4 ml-1">
                                        <span
                                            className={`w-2 h-2 rounded-full ${state.isTyping ? 'bg-[#febc2e]' : 'bg-[#28c840]'} group-hover:hidden transition-opacity`}
                                        />
                                        <svg
                                            className="hidden group-hover:block text-white/60 hover:text-white"
                                            width="10"
                                            height="10"
                                            viewBox="0 0 24 24"
                                            fill="none"
                                            stroke="currentColor"
                                            strokeWidth="3.5"
                                            strokeLinecap="round"
                                            strokeLinejoin="round"
                                        >
                                            <path d="M18 6L6 18M6 6l12 12" />
                                        </svg>
                                    </div>
                                </div>
                            </div>

                            {/* Spacer for symmetry */}
                            <div className="w-[60px]" />
                        </div>

                        {/* Main Content Area */}
                        <div className="flex flex-1 min-h-0">
                            <Sidebar activeIcon="files" />

                            {/* Code Editor */}
                            <div className="code-scrollbar flex-1 bg-[#1e1e1e] font-mono text-[13px] leading-relaxed py-4 overflow-y-auto min-h-0">
                                {displayedCode ? (
                                    <div>
                                        {highlightCode(displayedCode)}
                                        {state.showCursor && !prefersReducedMotion && (
                                            <span className="cursor-blink inline-block w-0.5 h-4 bg-[#528bff] ml-0.5 align-middle" />
                                        )}
                                    </div>
                                ) : (
                                    <div className="flex min-h-[1.5em] pl-4">
                                        <span className="w-[45px] text-right pr-4 text-[#4b5263] text-[13px]">1</span>
                                        <span className="text-[#5c6370] italic">// Waiting for AI agent...</span>
                                    </div>
                                )}
                            </div>

                            <ChatPanel
                                messages={messages}
                                isTyping={state.isTyping}
                                typingRole={state.typingRole}
                                currentTypingText={state.currentTypingText}
                            />
                        </div>

                        <StatusBar language={lang.name} lineCount={lineCount} />
                    </div>
                </div>
            </section>
        </>
    );
}

// Backward compatibility exports
export { IDEShowcaseSection as HeroIDESimulation };
export default IDEShowcaseSection;
