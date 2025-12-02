import React from 'react';
import { useScrollReveal } from '../../../hooks/useScrollReveal';
import styles from './styles.module.css';

interface HardwareTierProps {
  /** Tier number (1-3) */
  tierNumber: number;
  /** Tier title (e.g., "Workstation") */
  title: string;
  /** Tier subtitle (e.g., "Full Local Setup") */
  subtitle: string;
  /** Tier description */
  description: string;
  /** Cost estimate text */
  costEstimate: string;
  /** Impact label (e.g., "Best Experience", "Flexible") */
  impactLabel: string;
  /** Whether this is the recommended tier */
  recommended?: boolean;
  /** Animation delay for stagger effect (ms) */
  animationDelay?: number;
}

/**
 * HardwareTier - Hardware setup option card with Industrial-Kinetic styling
 *
 * Features:
 * - Tier number badge with cyan/amber border
 * - RECOMMENDED badge for highlighted tier
 * - Impact label pill
 * - Cost estimate display
 * - Scroll-triggered reveal animation
 */
export function HardwareTier({
  tierNumber,
  title,
  subtitle,
  description,
  costEstimate,
  impactLabel,
  recommended = false,
  animationDelay = 0,
}: HardwareTierProps): React.ReactElement {
  const { ref, isVisible } = useScrollReveal<HTMLDivElement>({ delay: animationDelay });

  return (
    <article
      ref={ref}
      className={`
        ${styles.hardwareTier}
        ${recommended ? styles.recommended : ''}
        ${isVisible ? styles.visible : ''}
      `}
      tabIndex={0}
      role="article"
      aria-labelledby={`tier-${tierNumber}-title`}
    >
      {/* RECOMMENDED badge */}
      {recommended && (
        <span className={styles.recommendedBadge}>
          RECOMMENDED
        </span>
      )}

      {/* Tier number badge */}
      <div className={`${styles.tierNumber} ${recommended ? styles.tierNumberHighlight : ''}`}>
        {tierNumber}
      </div>

      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerText}>
          <h3 id={`tier-${tierNumber}-title`} className={styles.title}>
            {title}
          </h3>
          <div className={styles.subtitle}>{subtitle}</div>
        </div>
        <div className={styles.impactLabel}>
          {impactLabel}
        </div>
      </div>

      {/* Description */}
      <p className={styles.description}>{description}</p>

      {/* Cost estimate */}
      <div className={styles.costEstimate}>
        <span className={styles.costLabel}>Cost:</span> {costEstimate}
      </div>

      {/* Glow effect for recommended tier */}
      {recommended && <div className={styles.glowEffect} aria-hidden="true" />}
    </article>
  );
}

export default HardwareTier;
