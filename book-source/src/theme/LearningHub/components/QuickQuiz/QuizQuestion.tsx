/**
 * QuizQuestion Component
 * Displays a single question with 4 radio button choices and feedback
 */

import React, { useState, type JSX } from 'react';
import type { QuizQuestion as QuizQuestionType } from '../../types';
import styles from './QuizQuestion.module.css';

interface QuizQuestionProps {
  question: QuizQuestionType;
  selectedChoice?: number;
  isAnswered: boolean;
  onSelectChoice: (choice: number) => void;
}

export default function QuizQuestion({
  question,
  selectedChoice,
  isAnswered,
  onSelectChoice,
}: QuizQuestionProps): JSX.Element {
  const [showFeedback, setShowFeedback] = useState(false);

  // Handle answer selection
  const handleChoiceSelect = (choiceIndex: number) => {
    if (isAnswered) return; // Don't allow changing answers
    
    onSelectChoice(choiceIndex);
    setShowFeedback(true);
  };

  // Check if a choice is correct/incorrect
  const getChoiceStatus = (choiceIndex: number): 'correct' | 'incorrect' | 'neutral' => {
    if (!isAnswered || selectedChoice === undefined) return 'neutral';
    
    if (choiceIndex === question.correctAnswer) return 'correct';
    if (choiceIndex === selectedChoice && selectedChoice !== question.correctAnswer) return 'incorrect';
    return 'neutral';
  };

  return (
    <div className={styles.container}>
      <div className={styles.questionText}>
        <h4>{question.question}</h4>
        {question.difficulty && (
          <span className={`${styles.difficulty} ${styles[question.difficulty]}`}>
            {question.difficulty}
          </span>
        )}
      </div>

      <div className={styles.choices}>
        {question.choices.map((choice, index) => {
          const status = getChoiceStatus(index);
          const isSelected = selectedChoice === index;

          return (
            <label
              key={index}
              className={`${styles.choice} ${styles[status]} ${isSelected ? styles.selected : ''}`}
            >
              <input
                type="radio"
                name={`question-${question.id}`}
                value={index}
                checked={isSelected}
                onChange={() => handleChoiceSelect(index)}
                disabled={isAnswered}
                className={styles.radio}
              />
              <span className={styles.choiceLabel}>
                {String.fromCharCode(65 + index)}. {choice}
              </span>
              {status === 'correct' && (
                <span className={styles.icon}>✓</span>
              )}
              {status === 'incorrect' && (
                <span className={styles.icon}>✗</span>
              )}
            </label>
          );
        })}
      </div>

      {isAnswered && showFeedback && (
        <div className={`${styles.feedback} ${selectedChoice === question.correctAnswer ? styles.correct : styles.incorrect}`}>
          <div className={styles.feedbackHeader}>
            {selectedChoice === question.correctAnswer ? (
              <>
                <span className={styles.feedbackIcon}>✓</span>
                <strong>Correct!</strong>
              </>
            ) : (
              <>
                <span className={styles.feedbackIcon}>✗</span>
                <strong>Incorrect</strong>
              </>
            )}
          </div>
          <p className={styles.explanation}>{question.explanation}</p>
          {selectedChoice !== question.correctAnswer && (
            <p className={styles.correctAnswerNote}>
              Correct answer: <strong>{String.fromCharCode(65 + question.correctAnswer)}. {question.choices[question.correctAnswer]}</strong>
            </p>
          )}
        </div>
      )}
    </div>
  );
}
