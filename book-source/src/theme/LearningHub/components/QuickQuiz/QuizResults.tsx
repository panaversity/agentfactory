/**
 * QuizResults Component
 * Displays quiz score summary and retake button
 */

import React, { type JSX } from 'react';
import styles from './QuizResults.module.css';

interface QuizResultsProps {
  score: {
    correct: number;
    total: number;
    percentage: number;
  };
  totalQuestions: number;
  onRetake: () => void;
}

export default function QuizResults({
  score,
  totalQuestions,
  onRetake,
}: QuizResultsProps): JSX.Element {
  // Determine performance level
  const getPerformanceMessage = (): { emoji: string; title: string; message: string } => {
    if (score.percentage >= 90) {
      return {
        emoji: '🌟',
        title: 'Excellent!',
        message: 'You have mastered this content.',
      };
    } else if (score.percentage >= 70) {
      return {
        emoji: '👍',
        title: 'Great Job!',
        message: 'You have a solid understanding of the material.',
      };
    } else if (score.percentage >= 50) {
      return {
        emoji: '📚',
        title: 'Good Effort!',
        message: 'Consider reviewing the content to strengthen your understanding.',
      };
    } else {
      return {
        emoji: '💪',
        title: 'Keep Learning!',
        message: 'Try reading the chapter again and then retake the quiz.',
      };
    }
  };

  const performance = getPerformanceMessage();

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <span className={styles.emoji}>{performance.emoji}</span>
        <h3>{performance.title}</h3>
      </div>

      <div className={styles.scoreDisplay}>
        <div className={styles.scoreCircle}>
          <div className={styles.scoreNumber}>{score.percentage}%</div>
          <div className={styles.scoreLabel}>
            {score.correct}/{score.total}
          </div>
        </div>
      </div>

      <p className={styles.message}>{performance.message}</p>

      <div className={styles.details}>
        <div className={styles.stat}>
          <div className={styles.statValue}>✓ {score.correct}</div>
          <div className={styles.statLabel}>Correct</div>
        </div>
        <div className={styles.stat}>
          <div className={styles.statValue}>✗ {score.total - score.correct}</div>
          <div className={styles.statLabel}>Incorrect</div>
        </div>
        <div className={styles.stat}>
          <div className={styles.statValue}>{totalQuestions}</div>
          <div className={styles.statLabel}>Total Questions</div>
        </div>
      </div>

      <button onClick={onRetake} className={styles.retakeButton}>
        🔄 Retake Quiz
      </button>

      <p className={styles.note}>
        Note: Retaking the quiz will generate new questions from the same content.
      </p>
    </div>
  );
}
