import React from 'react';
import { useScrollReveal } from '../../../hooks/useScrollReveal';
import styles from './styles.module.css';

interface ModuleCardProps {
  /** Module number (1-4) */
  moduleNumber: number;
  /** Module icon (emoji or SVG component) */
  icon: React.ReactNode;
  /** Module title (e.g., "ROS 2 Fundamentals") */
  title: string;
  /** Short subtitle (e.g., "Robot Middleware") */
  subtitle: string;
  /** Module description */
  description: string;
  /** List of topics covered */
  topics: string[];
  /** Week range (e.g., "Weeks 1-5") */
  weekRange: string;
  /** Whether this is a highlighted/advanced module */
  highlighted?: boolean;
  /** Optional badge text (e.g., "AI-Powered", "Capstone") */
  badge?: string;
  /** Animation delay for stagger effect (ms) */
  animationDelay?: number;
}

/**
 * ModuleCard - Course curriculum module card with Industrial-Kinetic styling
 *
 * Features:
 * - Dark secondary background with subtle border
 * - Cyan glow on hover for modules 1-2, amber for 3-4
 * - Badge support for advanced modules
 * - Scroll-triggered reveal animation
 * - Topic list with accent bullet styling
 */
export function ModuleCard({
  moduleNumber,
  icon,
  title,
  subtitle,
  description,
  topics,
  weekRange,
  highlighted = false,
  badge,
  animationDelay = 0,
}: ModuleCardProps): React.ReactElement {
  const { ref, isVisible } = useScrollReveal<HTMLDivElement>({ delay: animationDelay });

  // Determine glow variant based on module number
  const glowVariant = moduleNumber <= 2 ? 'cyan' : 'amber';

  return (
    <article
      ref={ref}
      className={`
        ${styles.moduleCard}
        ${highlighted ? styles.highlighted : ''}
        ${styles[glowVariant]}
        ${isVisible ? styles.visible : ''}
      `}
      tabIndex={0}
      role="article"
      aria-labelledby={`module-${moduleNumber}-title`}
    >
      {/* Badge */}
      {badge && (
        <span className={styles.badge} aria-label={`${badge} module`}>
          {badge}
        </span>
      )}

      {/* Card Header */}
      <div className={styles.header}>
        <div className={styles.iconWrapper}>
          <span className={styles.icon} aria-hidden="true">{icon}</span>
        </div>
        <div className={styles.headerText}>
          <h3 id={`module-${moduleNumber}-title`} className={styles.title}>
            Module {moduleNumber}
          </h3>
          <div className={styles.subtitle}>{subtitle}</div>
        </div>
      </div>

      {/* Title as secondary heading */}
      <h4 className={styles.moduleTitle}>{title}</h4>

      {/* Description */}
      <p className={styles.description}>{description}</p>

      {/* Topics List */}
      <ul className={styles.topicsList} role="list">
        {topics.map((topic, index) => (
          <li key={index} className={styles.topic}>
            <span className={styles.topicBullet} aria-hidden="true">▸</span>
            {topic}
          </li>
        ))}
      </ul>

      {/* Week Range */}
      <div className={styles.weekRange}>
        <strong>{weekRange}</strong>
      </div>

      {/* Hover glow effect element */}
      <div className={styles.glowEffect} aria-hidden="true" />
    </article>
  );
}

export default ModuleCard;
