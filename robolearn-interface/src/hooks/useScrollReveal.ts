import { useEffect, useRef, useState, RefObject } from 'react';

interface UseScrollRevealOptions {
  /** Threshold for intersection observer (0-1). Default: 0.1 */
  threshold?: number;
  /** Root margin for earlier/later triggering. Default: '0px 0px -50px 0px' */
  rootMargin?: string;
  /** Whether to only trigger once. Default: true */
  triggerOnce?: boolean;
  /** Delay before applying visible class (ms). Default: 0 */
  delay?: number;
}

interface UseScrollRevealReturn<T extends HTMLElement> {
  ref: RefObject<T | null>;
  isVisible: boolean;
}

/**
 * Hook for triggering scroll-reveal animations using Intersection Observer.
 *
 * Usage:
 * ```tsx
 * const { ref, isVisible } = useScrollReveal<HTMLDivElement>();
 * return (
 *   <div ref={ref} className={`ifk-reveal ${isVisible ? 'visible' : ''}`}>
 *     Content
 *   </div>
 * );
 * ```
 *
 * Or with stagger delay for multiple items:
 * ```tsx
 * {items.map((item, index) => {
 *   const { ref, isVisible } = useScrollReveal<HTMLDivElement>({ delay: index * 100 });
 *   return <div ref={ref} className={isVisible ? 'visible' : ''} key={item.id}>{item.content}</div>;
 * })}
 * ```
 */
export function useScrollReveal<T extends HTMLElement = HTMLDivElement>(
  options: UseScrollRevealOptions = {}
): UseScrollRevealReturn<T> {
  const {
    threshold = 0.1,
    rootMargin = '0px 0px -50px 0px',
    triggerOnce = true,
    delay = 0,
  } = options;

  const ref = useRef<T | null>(null);
  const [isVisible, setIsVisible] = useState(false);
  const hasTriggered = useRef(false);

  useEffect(() => {
    // Check for reduced motion preference
    const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

    // If user prefers reduced motion, show content immediately
    if (prefersReducedMotion) {
      setIsVisible(true);
      return;
    }

    const element = ref.current;
    if (!element) return;

    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            // Apply delay if specified
            if (delay > 0) {
              setTimeout(() => {
                setIsVisible(true);
              }, delay);
            } else {
              setIsVisible(true);
            }

            // Unobserve if triggerOnce is enabled
            if (triggerOnce) {
              hasTriggered.current = true;
              observer.unobserve(entry.target);
            }
          } else if (!triggerOnce && !hasTriggered.current) {
            // Reset visibility when not intersecting (only if triggerOnce is false)
            setIsVisible(false);
          }
        });
      },
      {
        threshold,
        rootMargin,
      }
    );

    observer.observe(element);

    return () => {
      observer.disconnect();
    };
  }, [threshold, rootMargin, triggerOnce, delay]);

  return { ref, isVisible };
}

/**
 * Hook for batch scroll reveal with stagger effect.
 * Returns refs and visibility states for multiple elements.
 *
 * Usage:
 * ```tsx
 * const items = useScrollRevealGroup(4, { staggerDelay: 100 });
 * return (
 *   <div>
 *     {items.map((item, index) => (
 *       <div ref={item.ref} className={item.isVisible ? 'visible' : ''} key={index}>
 *         Card {index + 1}
 *       </div>
 *     ))}
 *   </div>
 * );
 * ```
 */
export function useScrollRevealGroup<T extends HTMLElement = HTMLDivElement>(
  count: number,
  options: UseScrollRevealOptions & { staggerDelay?: number } = {}
): UseScrollRevealReturn<T>[] {
  const { staggerDelay = 100, ...baseOptions } = options;

  // Create array of hooks - this is safe because count is fixed per component instance
  // eslint-disable-next-line react-hooks/rules-of-hooks
  return Array.from({ length: count }, (_, index) =>
    useScrollReveal<T>({ ...baseOptions, delay: (baseOptions.delay || 0) + index * staggerDelay })
  );
}

export default useScrollReveal;
