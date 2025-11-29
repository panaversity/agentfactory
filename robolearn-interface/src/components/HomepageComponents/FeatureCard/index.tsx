import React from 'react';
import { useScrollReveal } from '../../../hooks/useScrollReveal';
import styles from './styles.module.css';

interface FeatureCardProps {
  /** Feature icon (SVG component or emoji) */
  icon: React.ReactNode;
  /** Feature title */
  title: string;
  /** Feature description */
  description: string;
  /** Whether this is a featured/core item */
  featured?: boolean;
  /** Animation delay for stagger effect (ms) */
  animationDelay?: number;
}

/**
 * FeatureCard - Feature highlight card with Industrial-Kinetic styling
 *
 * Features:
 * - Icon wrapper with cyan border and hover rotation
 * - Core badge for featured items
 * - Scroll-triggered reveal animation
 */
export function FeatureCard({
  icon,
  title,
  description,
  featured = false,
  animationDelay = 0,
}: FeatureCardProps): React.ReactElement {
  const { ref, isVisible } = useScrollReveal<HTMLDivElement>({ delay: animationDelay });

  return (
    <article
      ref={ref}
      className={`
        ${styles.featureCard}
        ${featured ? styles.featured : ''}
        ${isVisible ? styles.visible : ''}
      `}
      tabIndex={0}
      role="article"
    >
      {/* Core badge for featured items */}
      {featured && (
        <span className={styles.coreBadge}>Core</span>
      )}

      {/* Icon wrapper */}
      <div className={styles.iconWrapper}>
        <span className={styles.icon} aria-hidden="true">{icon}</span>
      </div>

      {/* Title */}
      <h3 className={styles.title}>{title}</h3>

      {/* Description */}
      <p className={styles.description}>{description}</p>

      {/* Bottom accent line */}
      <div className={styles.accentLine} aria-hidden="true" />
    </article>
  );
}

export default FeatureCard;
