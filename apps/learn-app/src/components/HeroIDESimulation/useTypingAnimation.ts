import { useState, useEffect, useCallback, useRef } from 'react';

interface UseTypingAnimationOptions {
    speed?: number; // ms per character
    onComplete?: () => void;
}

export function useTypingAnimation(
    text: string,
    options: UseTypingAnimationOptions = {}
) {
    const { speed = 30, onComplete } = options;
    const [displayedText, setDisplayedText] = useState('');
    const [isComplete, setIsComplete] = useState(false);
    const [isTyping, setIsTyping] = useState(false);
    const timerRef = useRef<NodeJS.Timeout | null>(null);
    const indexRef = useRef(0);

    const startTyping = useCallback(() => {
        setDisplayedText('');
        setIsComplete(false);
        setIsTyping(true);
        indexRef.current = 0;

        const type = () => {
            if (indexRef.current < text.length) {
                setDisplayedText(text.slice(0, indexRef.current + 1));
                indexRef.current++;
                timerRef.current = setTimeout(type, speed);
            } else {
                setIsComplete(true);
                setIsTyping(false);
                onComplete?.();
            }
        };

        timerRef.current = setTimeout(type, speed);
    }, [text, speed, onComplete]);

    const reset = useCallback(() => {
        if (timerRef.current) {
            clearTimeout(timerRef.current);
        }
        setDisplayedText('');
        setIsComplete(false);
        setIsTyping(false);
        indexRef.current = 0;
    }, []);

    // Cleanup on unmount
    useEffect(() => {
        return () => {
            if (timerRef.current) {
                clearTimeout(timerRef.current);
            }
        };
    }, []);

    return {
        displayedText,
        isComplete,
        isTyping,
        startTyping,
        reset,
    };
}

// Simpler version that auto-starts
export function useAutoTyping(
    text: string,
    speed: number = 30,
    startDelay: number = 0
) {
    const [displayedText, setDisplayedText] = useState('');
    const [isComplete, setIsComplete] = useState(false);

    useEffect(() => {
        if (!text) {
            setDisplayedText('');
            setIsComplete(false);
            return;
        }

        let index = 0;
        let timer: NodeJS.Timeout;

        const startTimeout = setTimeout(() => {
            const type = () => {
                if (index < text.length) {
                    setDisplayedText(text.slice(0, index + 1));
                    index++;
                    timer = setTimeout(type, speed);
                } else {
                    setIsComplete(true);
                }
            };
            type();
        }, startDelay);

        return () => {
            clearTimeout(startTimeout);
            clearTimeout(timer);
        };
    }, [text, speed, startDelay]);

    return { displayedText, isComplete };
}
