import React from 'react';
import styles from './styles.module.css';

interface JourneyStep {
  icon: string;
  label: string;
  description: string;
}

const JOURNEY_STEPS: JourneyStep[] = [
  {
    icon: '💻',
    label: 'Browser',
    description: 'Start with just a laptop',
  },
  {
    icon: '🎮',
    label: 'Simulate',
    description: 'Build in virtual worlds',
  },
  {
    icon: '🔧',
    label: 'Edge',
    description: 'Deploy to real sensors',
  },
  {
    icon: '🤖',
    label: 'Robot',
    description: 'Control real hardware',
  },
];

/**
 * LearningJourney - Visual progression from laptop to real robot
 *
 * Shows visitors the path: Browser → Simulation → Edge → Robot
 * Communicates "start with nothing, end with a real robot"
 */
export function LearningJourney(): React.ReactElement {
  return (
    <div className={styles.journeyContainer}>
      <span className={styles.journeyLabel}>Your Learning Journey</span>
      <div className={styles.journeySteps}>
        {JOURNEY_STEPS.map((step, index) => (
          <React.Fragment key={step.label}>
            <div className={styles.step} style={{ '--delay': `${index * 0.15}s` } as React.CSSProperties}>
              <div className={styles.stepIcon}>{step.icon}</div>
              <span className={styles.stepLabel}>{step.label}</span>
              <span className={styles.stepDescription}>{step.description}</span>
            </div>
            {index < JOURNEY_STEPS.length - 1 && (
              <div className={styles.connector} style={{ '--delay': `${index * 0.15 + 0.1}s` } as React.CSSProperties}>
                <svg viewBox="0 0 40 20" className={styles.arrow}>
                  <path d="M0 10 L30 10 M25 5 L30 10 L25 15" fill="none" stroke="currentColor" strokeWidth="2" />
                </svg>
              </div>
            )}
          </React.Fragment>
        ))}
      </div>
    </div>
  );
}

export default LearningJourney;
