'use client';

import React from 'react';

export interface ChatMessage {
    id: string;
    role: 'user' | 'agent' | 'system';
    content: string;
}

export interface ChatPanelProps {
    /** List of messages to display */
    messages: ChatMessage[];
    /** Whether someone is currently typing */
    isTyping?: boolean;
    /** Who is typing (user or agent) */
    typingRole?: 'user' | 'agent' | null;
    /** The text being typed (shown with cursor) */
    currentTypingText?: string;
    /** Panel width (default: 320px) */
    width?: string;
}

/**
 * AI Chat Panel component for the IDE simulation.
 * Shows messages, typing indicator, and input area.
 */
export function ChatPanel({
    messages,
    isTyping = false,
    typingRole = null,
    currentTypingText,
    width = 'w-80',
}: ChatPanelProps) {
    return (
        <div className={`${width} bg-[#252526] border-l border-[#1a1a1a] flex flex-col shrink-0`}>
            {/* Chat Header */}
            <div className="px-4 py-3 border-b border-[#1a1a1a] flex items-center gap-2">
                <div className="w-7 h-7 rounded-md bg-gradient-to-br from-[#667eea] to-[#764ba2] flex items-center justify-center text-sm">
                    🤖
                </div>
                <div className="flex-1">
                    <div className="text-[13px] font-semibold text-[#e4e4e4]">
                        AI Copilot
                    </div>
                    <div className="text-[11px] text-[#3fb950] flex items-center gap-1">
                        <span className="w-1.5 h-1.5 rounded-full bg-[#3fb950]" />
                        Online
                    </div>
                </div>
            </div>

            {/* Messages */}
            <div className="flex-1 p-4 overflow-y-auto flex flex-col gap-3">
                {messages.length === 0 && !isTyping && (
                    <div className="text-center py-8 px-4 text-[#5c6370] text-[13px]">
                        <div className="text-[32px] mb-2">💬</div>
                        Start a conversation...
                    </div>
                )}

                {messages.map((message) => (
                    <ChatMessageBubble key={message.id} message={message} />
                ))}

                {/* Currently typing */}
                {isTyping && currentTypingText && typingRole && (
                    <div className={`animate-fadeIn flex gap-2 items-start ${typingRole === 'user' ? 'flex-row-reverse' : 'flex-row'}`}>
                        <div
                            className={`w-6 h-6 rounded-full flex items-center justify-center text-xs shrink-0 ${typingRole === 'user'
                                ? 'bg-gradient-to-br from-[#667eea] to-[#764ba2]'
                                : 'bg-gradient-to-br from-[#10b981] to-[#059669]'
                                }`}
                        >
                            {typingRole === 'user' ? '👤' : '🤖'}
                        </div>
                        <div
                            className={`rounded-xl max-w-[85%] leading-relaxed text-[13px] ${typingRole === 'user'
                                    ? 'bg-[#3c3c3c] text-[#e4e4e4] py-2.5 px-3.5'
                                    : 'text-[#e4e4e4] py-2 px-1'
                                }`}
                        >
                            {currentTypingText}
                            <span className="cursor-blink inline-block w-0.5 h-3.5 bg-[#528bff] ml-0.5 align-middle" />
                        </div>
                    </div>
                )}
            </div>

            {/* Input Area */}
            <div className="px-4 py-3 border-t border-[#1a1a1a]">
                <div className="flex items-center gap-2 py-2.5 px-3.5 bg-[#1e1e1e] rounded-lg border border-[#3c3c3c]">
                    <span className="text-[#5c6370] text-[13px]">
                        Ask AI Copilot...
                    </span>
                    <span className="ml-auto text-[#5c6370] text-[11px]">
                        ⌘K
                    </span>
                </div>
            </div>
        </div>
    );
}

interface ChatMessageBubbleProps {
    message: ChatMessage;
}

function ChatMessageBubble({ message }: ChatMessageBubbleProps) {
    const isUser = message.role === 'user';
    const isSystem = message.role === 'system';

    return (
        <div
            className={`animate-fadeIn flex gap-2 items-start ${isUser ? 'flex-row-reverse' : 'flex-row'}`}
        >
            {!isSystem && (
                <div
                    className={`w-6 h-6 rounded-full flex items-center justify-center text-xs shrink-0 ${isUser
                        ? 'bg-gradient-to-br from-[#667eea] to-[#764ba2]'
                        : 'bg-gradient-to-br from-[#10b981] to-[#059669]'
                        }`}
                >
                    {isUser ? '👤' : '🤖'}
                </div>
            )}
            <div
                className={`rounded-xl max-w-[85%] leading-relaxed text-[13px] ${isUser
                        ? 'bg-[#3c3c3c] text-[#e4e4e4] py-2.5 px-3.5'
                        : isSystem
                            ? 'bg-white/5 text-[#858585] text-[11px] py-1 px-3'
                            : 'text-[#e4e4e4] py-2 px-1'
                    }`}
            >
                {message.content}
            </div>
        </div>
    );
}
