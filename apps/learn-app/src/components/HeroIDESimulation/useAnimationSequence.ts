import { useState, useEffect, useCallback, useRef } from 'react';
import type { AnimationSequence, AnimationStep } from './sequences';
import type { Message } from './AgentChat';

interface AnimationState {
    currentStepIndex: number;
    displayedCode: string;
    messages: Message[];
    currentTypingText: string;
    typingRole: 'user' | 'agent' | null;
    isTyping: boolean;
    isPaused: boolean;
    showCursor: boolean;
}

const initialState: AnimationState = {
    currentStepIndex: 0,
    displayedCode: '',
    messages: [],
    currentTypingText: '',
    typingRole: null,
    isTyping: false,
    isPaused: false,
    showCursor: false,
};

export function useAnimationSequence(
    sequences: AnimationSequence[],
    options: { autoStart?: boolean; pauseOnHidden?: boolean } = {}
) {
    const { autoStart = true, pauseOnHidden = true } = options;

    const [sequenceIndex, setSequenceIndex] = useState(0);
    const [state, setState] = useState<AnimationState>(initialState);

    const timerRef = useRef<NodeJS.Timeout | null>(null);
    const charTimerRef = useRef<NodeJS.Timeout | null>(null);
    const isRunningRef = useRef(false);

    const currentSequence = sequences[sequenceIndex];

    // Clear all timers
    const clearTimers = useCallback(() => {
        if (timerRef.current) clearTimeout(timerRef.current);
        if (charTimerRef.current) clearTimeout(charTimerRef.current);
    }, []);

    // Type text character by character
    const typeText = useCallback((
        text: string,
        onUpdate: (text: string) => void,
        onComplete: () => void,
        speed: number = 25
    ) => {
        let index = 0;

        const type = () => {
            if (index < text.length) {
                onUpdate(text.slice(0, index + 1));
                index++;
                charTimerRef.current = setTimeout(type, speed);
            } else {
                onComplete();
            }
        };

        type();
    }, []);

    // Process a single step
    const processStep = useCallback((step: AnimationStep, onComplete: () => void) => {
        const messageId = `msg-${Date.now()}`;

        switch (step.type) {
            case 'user-message':
                setState(prev => ({ ...prev, isTyping: true, typingRole: 'user', currentTypingText: '' }));
                typeText(
                    step.content,
                    (text) => setState(prev => ({ ...prev, currentTypingText: text })),
                    () => {
                        setState(prev => ({
                            ...prev,
                            isTyping: false,
                            typingRole: null,
                            currentTypingText: '',
                            messages: [...prev.messages, { id: messageId, role: 'user', content: step.content }],
                        }));
                        timerRef.current = setTimeout(onComplete, 300);
                    },
                    35
                );
                break;

            case 'agent-message':
                setState(prev => ({ ...prev, isTyping: true, typingRole: 'agent', currentTypingText: '' }));
                typeText(
                    step.content,
                    (text) => setState(prev => ({ ...prev, currentTypingText: text })),
                    () => {
                        setState(prev => ({
                            ...prev,
                            isTyping: false,
                            typingRole: null,
                            currentTypingText: '',
                            messages: [...prev.messages, { id: messageId, role: 'agent', content: step.content }],
                        }));
                        timerRef.current = setTimeout(onComplete, 300);
                    },
                    20
                );
                break;

            case 'system-message':
                setState(prev => ({
                    ...prev,
                    messages: [...prev.messages, { id: messageId, role: 'system', content: step.content }],
                }));
                timerRef.current = setTimeout(onComplete, step.delay);
                break;

            case 'code':
                setState(prev => ({ ...prev, showCursor: true }));
                typeText(
                    step.content,
                    (text) => setState(prev => ({ ...prev, displayedCode: text })),
                    () => {
                        setState(prev => ({
                            ...prev,
                            showCursor: false,
                            // Remove system messages (like "Generating code...") when code is done
                            messages: prev.messages.filter(m => m.role !== 'system'),
                        }));
                        timerRef.current = setTimeout(onComplete, 500);
                    },
                    15 // Faster for code
                );
                break;
        }
    }, [typeText]);

    // Run the animation sequence
    const runSequence = useCallback(() => {
        if (!currentSequence || isRunningRef.current) return;

        isRunningRef.current = true;
        const steps = currentSequence.steps;
        let stepIndex = 0;

        const nextStep = () => {
            if (stepIndex >= steps.length) {
                // Sequence complete - wait and then restart
                timerRef.current = setTimeout(() => {
                    // Reset and switch to next sequence
                    setState(initialState);
                    setSequenceIndex(prev => (prev + 1) % sequences.length);
                    isRunningRef.current = false;

                    // Start next sequence after delay
                    timerRef.current = setTimeout(runSequence, 1500);
                }, 3000);
                return;
            }

            setState(prev => ({ ...prev, currentStepIndex: stepIndex }));
            processStep(steps[stepIndex], () => {
                stepIndex++;
                nextStep();
            });
        };

        // Start with a small delay
        timerRef.current = setTimeout(nextStep, 500);
    }, [currentSequence, sequences.length, processStep]);

    // Start/restart animation
    const start = useCallback(() => {
        clearTimers();
        setState(initialState);
        isRunningRef.current = false;
        runSequence();
    }, [clearTimers, runSequence]);

    // Pause animation
    const pause = useCallback(() => {
        clearTimers();
        setState(prev => ({ ...prev, isPaused: true }));
    }, [clearTimers]);

    // Handle visibility change
    useEffect(() => {
        if (!pauseOnHidden) return;

        const handleVisibility = () => {
            if (document.hidden) {
                pause();
            } else if (state.isPaused) {
                // Resume by restarting
                start();
            }
        };

        document.addEventListener('visibilitychange', handleVisibility);
        return () => document.removeEventListener('visibilitychange', handleVisibility);
    }, [pauseOnHidden, pause, start, state.isPaused]);

    // Auto-start
    useEffect(() => {
        if (autoStart) {
            start();
        }

        return () => {
            clearTimers();
            isRunningRef.current = false;
        };
    }, []); // Only on mount

    return {
        state,
        currentSequence,
        start,
        pause,
    };
}
