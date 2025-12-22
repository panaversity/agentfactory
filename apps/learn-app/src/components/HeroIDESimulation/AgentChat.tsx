import React from 'react';
import { cn } from '@/lib/utils';

export interface Message {
    id: string;
    role: 'user' | 'agent' | 'system';
    content: string;
    isTyping?: boolean;
}

interface AgentChatProps {
    messages: Message[];
    currentTypingText?: string;
}

export function AgentChat({ messages, currentTypingText }: AgentChatProps) {
    return (
        <div className="bg-zinc-800 dark:bg-zinc-900 border-t border-zinc-700/50">
            {/* Chat header */}
            <div className="px-4 py-2 border-b border-zinc-700/50 flex items-center gap-2">
                <span className="text-lg">💬</span>
                <span className="text-sm font-medium text-zinc-400">Agent Chat</span>
            </div>

            {/* Messages container */}
            <div className="p-4 min-h-[120px] max-h-[180px] overflow-y-auto space-y-3">
                {messages.length === 0 && !currentTypingText && (
                    <div className="text-zinc-500 text-sm italic text-center py-4">
                        Start a conversation with the AI agent...
                    </div>
                )}

                {messages.map((message) => (
                    <ChatMessage key={message.id} message={message} />
                ))}

                {currentTypingText && (
                    <div className="flex items-start gap-2 animate-fadeIn">
                        <span className="text-lg flex-shrink-0">
                            {messages.length > 0 && messages[messages.length - 1]?.role === 'user' ? '🤖' : '👤'}
                        </span>
                        <div className="text-sm text-zinc-300">
                            {currentTypingText}
                            <span className="inline-block w-1.5 h-3 bg-teal-400 animate-pulse ml-0.5 align-middle" />
                        </div>
                    </div>
                )}
            </div>
        </div>
    );
}

function ChatMessage({ message }: { message: Message }) {
    if (message.role === 'system') {
        return (
            <div className="flex justify-center animate-fadeIn">
                <span className="text-xs text-zinc-500 bg-zinc-700/50 px-3 py-1 rounded-full">
                    {message.content}
                </span>
            </div>
        );
    }

    const isUser = message.role === 'user';

    return (
        <div
            className={cn(
                'flex items-start gap-2 animate-fadeIn',
                isUser && 'flex-row-reverse'
            )}
        >
            <span className="text-lg flex-shrink-0">
                {isUser ? '👤' : '🤖'}
            </span>
            <div
                className={cn(
                    'text-sm px-3 py-2 rounded-lg max-w-[85%]',
                    isUser
                        ? 'bg-teal-600 text-white'
                        : 'bg-zinc-700 text-zinc-200'
                )}
            >
                {message.content}
            </div>
        </div>
    );
}
