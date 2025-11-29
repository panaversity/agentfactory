import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';

interface Stat {
  value: number;
  label: string;
  suffix?: string;
}

const STATS: Stat[] = [
  { value: 4, label: 'Modules' },
  { value: 48, label: 'Lessons' },
  { value: 12, label: 'Labs' },
  { value: 24, label: 'AI Chat', suffix: '/7' },
];

/**
 * PlatformStats - Animated counters showing platform metrics
 */
export function PlatformStats(): React.ReactElement {
  const [counts, setCounts] = useState<number[]>(STATS.map(() => 0));
  const [hasAnimated, setHasAnimated] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (hasAnimated) return;

    const observer = new IntersectionObserver(
      (entries) => {
        if (entries[0].isIntersecting) {
          setHasAnimated(true);
          animateCounters();
        }
      },
      { threshold: 0.5 }
    );

    if (containerRef.current) {
      observer.observe(containerRef.current);
    }

    return () => observer.disconnect();
  }, [hasAnimated]);

  const animateCounters = () => {
    const duration = 1500;
    const steps = 30;
    const interval = duration / steps;

    let step = 0;
    const timer = setInterval(() => {
      step++;
      const progress = step / steps;
      // Ease out cubic
      const eased = 1 - Math.pow(1 - progress, 3);

      setCounts(STATS.map((stat) => Math.round(stat.value * eased)));

      if (step >= steps) {
        clearInterval(timer);
      }
    }, interval);
  };

  return (
    <div ref={containerRef} className={styles.statsContainer}>
      {STATS.map((stat, index) => (
        <div key={stat.label} className={styles.statItem}>
          <span className={styles.statValue}>
            {counts[index]}
            {stat.suffix && <span className={styles.statSuffix}>{stat.suffix}</span>}
          </span>
          <span className={styles.statLabel}>{stat.label}</span>
        </div>
      ))}
    </div>
  );
}

export default PlatformStats;
