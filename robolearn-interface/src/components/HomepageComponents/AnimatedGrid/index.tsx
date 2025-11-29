import React from 'react';
import styles from './styles.module.css';

interface AnimatedGridProps {
  /** Additional CSS classes */
  className?: string;
  /** Disable animation (for reduced motion preference) */
  disableAnimation?: boolean;
}

/**
 * AnimatedGrid - Industrial blueprint-style animated background grid
 *
 * Features:
 * - SVG pattern grid with subtle blue lines
 * - Animated scan-line effect sweeping across
 * - Glow overlay at intersection points
 * - Respects reduced-motion preferences
 *
 * Usage:
 * ```tsx
 * <div className="hero-container">
 *   <AnimatedGrid />
 *   <div className="hero-content">...</div>
 * </div>
 * ```
 */
export function AnimatedGrid({
  className = '',
  disableAnimation = false,
}: AnimatedGridProps): React.ReactElement {
  return (
    <div
      className={`${styles.gridContainer} ${className}`}
      aria-hidden="true"
      role="presentation"
    >
      {/* SVG Grid Pattern */}
      <svg
        className={styles.gridSvg}
        xmlns="http://www.w3.org/2000/svg"
        width="100%"
        height="100%"
        preserveAspectRatio="none"
      >
        <defs>
          {/* Grid pattern */}
          <pattern
            id="gridPattern"
            width="60"
            height="60"
            patternUnits="userSpaceOnUse"
          >
            {/* Horizontal line */}
            <line
              x1="0"
              y1="60"
              x2="60"
              y2="60"
              stroke="var(--ifk-cyan)"
              strokeOpacity="0.08"
              strokeWidth="1"
            />
            {/* Vertical line */}
            <line
              x1="60"
              y1="0"
              x2="60"
              y2="60"
              stroke="var(--ifk-cyan)"
              strokeOpacity="0.08"
              strokeWidth="1"
            />
          </pattern>

          {/* Gradient for scan line */}
          <linearGradient id="scanGradient" x1="0%" y1="0%" x2="0%" y2="100%">
            <stop offset="0%" stopColor="var(--ifk-cyan)" stopOpacity="0" />
            <stop offset="50%" stopColor="var(--ifk-cyan)" stopOpacity="0.3" />
            <stop offset="100%" stopColor="var(--ifk-cyan)" stopOpacity="0" />
          </linearGradient>
        </defs>

        {/* Grid fill */}
        <rect width="100%" height="100%" fill="url(#gridPattern)" />
      </svg>

      {/* Scan line animation */}
      {!disableAnimation && (
        <div className={styles.scanLine} />
      )}

      {/* Corner glow accents */}
      <div className={styles.glowTopLeft} />
      <div className={styles.glowBottomRight} />

      {/* Vignette overlay for depth */}
      <div className={styles.vignette} />
    </div>
  );
}

export default AnimatedGrid;
