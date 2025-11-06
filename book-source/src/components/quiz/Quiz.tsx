import React, { useState, useMemo } from 'react';
import styles from './Quiz.module.css';

export interface QuizQuestion {
  question: string;
  options: [string, string, string, string];
  correctOption: 0 | 1 | 2 | 3;
  explanation?: string;
  source?: string; // e.g., "Lesson 1: Understanding Mutability"
}

export interface QuizProps {
  title?: string;
  questions: QuizQuestion[];
  questionsPerBatch?: number; // Number of questions to show per batch (default: 15-20)
}

// Utility function to shuffle array
const shuffleArray = <T,>(array: T[]): T[] => {
  const shuffled = [...array];
  for (let i = shuffled.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    [shuffled[i], shuffled[j]] = [shuffled[j], shuffled[i]];
  }
  return shuffled;
};

const Quiz: React.FC<QuizProps> = ({
  title = "Quiz",
  questions,
  questionsPerBatch = 15
}) => {
  // Initialize displayed questions on mount
  const [displayedQuestions, setDisplayedQuestions] = useState<QuizQuestion[]>(() => {
    const shuffled = shuffleArray(questions);
    return shuffled.slice(0, questionsPerBatch);
  });

  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [selectedAnswers, setSelectedAnswers] = useState<(number | null)[]>(
    new Array(displayedQuestions.length).fill(null)
  );
  const [showFeedback, setShowFeedback] = useState(false);
  const [showResults, setShowResults] = useState(false);
  const [answeredQuestions, setAnsweredQuestions] = useState<Set<number>>(new Set());

  const handleAnswerSelect = (optionIndex: number) => {
    // Only allow selection if not already answered
    if (answeredQuestions.has(currentQuestion)) {
      return;
    }

    const newAnswers = [...selectedAnswers];
    newAnswers[currentQuestion] = optionIndex;
    setSelectedAnswers(newAnswers);

    const newAnswered = new Set(answeredQuestions);
    newAnswered.add(currentQuestion);
    setAnsweredQuestions(newAnswered);

    // Show feedback immediately after answer
    setShowFeedback(true);
  };

  const handleNext = () => {
    if (currentQuestion < displayedQuestions.length - 1) {
      setCurrentQuestion(currentQuestion + 1);
      setShowFeedback(false);
    }
  };

  const handleBack = () => {
    if (currentQuestion > 0) {
      setCurrentQuestion(currentQuestion - 1);
      // Preserve feedback for previously answered questions
      setShowFeedback(answeredQuestions.has(currentQuestion - 1));
    }
  };

  const handleSubmit = () => {
    setShowResults(true);
  };

  const handleReset = () => {
    // Shuffle questions again for new batch
    const newShuffled = shuffleArray(questions);
    const newBatch = newShuffled.slice(0, questionsPerBatch);

    setDisplayedQuestions(newBatch);
    setCurrentQuestion(0);
    setSelectedAnswers(new Array(newBatch.length).fill(null));
    setShowResults(false);
    setShowFeedback(false);
    setAnsweredQuestions(new Set());
  };

  const calculateScore = () => {
    let correct = 0;
    selectedAnswers.forEach((answer, index) => {
      if (answer === displayedQuestions[index].correctOption) {
        correct++;
      }
    });
    return {
      correct,
      total: displayedQuestions.length,
      percentage: Math.round((correct / displayedQuestions.length) * 100)
    };
  };

  const score = calculateScore();
  const allAnswered = selectedAnswers.every(answer => answer !== null);

  if (showResults) {
    return (
      <div className={styles.quizContainer}>
        <div className={styles.resultsCard}>
          <div className={styles.resultsHeader}>
            <h2 className={styles.resultsTitle}>Quiz Complete</h2>
            <div className={styles.scoreCircle}>
              <div className={styles.scorePercentage}>{score.percentage}%</div>
              <div className={styles.scoreLabel}>Your Score</div>
            </div>
          </div>

          <div className={styles.resultsStats}>
            <div className={styles.statItem}>
              <span className={styles.statValue}>{score.correct}</span>
              <span className={styles.statLabel}>Correct</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statValue}>{score.total - score.correct}</span>
              <span className={styles.statLabel}>Incorrect</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statValue}>{score.total}</span>
              <span className={styles.statLabel}>Total</span>
            </div>
          </div>

          <div className={styles.resultMessage}>
            <span className={styles.resultIcon}>📚</span>
            <strong>Great effort!</strong> You answered {score.correct} out of {score.total} questions correctly.
          </div>

          <div className={styles.detailedResults}>
            <h3 className={styles.detailedResultsTitle}>Question Review</h3>
            {displayedQuestions.map((q, index) => {
              const userAnswer = selectedAnswers[index];
              const isCorrect = userAnswer === q.correctOption;

              return (
                <div key={index} className={styles.reviewItem}>
                  <div className={styles.reviewHeader}>
                    <span className={styles.reviewQuestionNumber}>Question {index + 1}</span>
                    <span className={`${styles.reviewBadge} ${isCorrect ? styles.correctBadge : styles.incorrectBadge}`}>
                      {isCorrect ? '✓ Correct' : '✗ Incorrect'}
                    </span>
                  </div>
                  <p className={styles.reviewQuestion}>{q.question}</p>
                  <div className={styles.reviewAnswers}>
                    <div className={styles.reviewAnswer}>
                      <strong>Your answer:</strong>{' '}
                      <span className={isCorrect ? styles.correctText : styles.incorrectText}>
                        {userAnswer !== null ? q.options[userAnswer] : 'Not answered'}
                      </span>
                    </div>
                    {!isCorrect && (
                      <div className={styles.reviewAnswer}>
                        <strong>Correct answer:</strong>{' '}
                        <span className={styles.correctText}>
                          {q.options[q.correctOption]}
                        </span>
                      </div>
                    )}
                    {q.explanation && (
                      <div className={styles.explanation}>
                        <strong>Explanation:</strong> {q.explanation}
                      </div>
                    )}
                  </div>
                </div>
              );
            })}
          </div>

          <button
            onClick={handleReset}
            className={styles.resetButton}
          >
            Retake Quiz
          </button>
        </div>
      </div>
    );
  }

  const question = displayedQuestions[currentQuestion];
  const selectedAnswer = selectedAnswers[currentQuestion];
  const progress = ((currentQuestion + 1) / displayedQuestions.length) * 100;
  const isAnswerCorrect = selectedAnswer === question.correctOption;

  return (
    <div className={styles.quizContainer}>
      <div className={styles.quizCard}>
        <div className={styles.quizHeader}>
          <h2 className={styles.quizTitle}>{title}</h2>
          <div className={styles.questionCounter}>
            Question {currentQuestion + 1} of {displayedQuestions.length}
          </div>
        </div>

        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${progress}%` }}
          />
        </div>

        <div className={styles.questionSection}>
          <h3 className={styles.questionText}>{question.question}</h3>

          <div className={styles.optionsGrid}>
            {question.options.map((option, index) => (
              <button
                key={index}
                className={`${styles.optionButton} ${
                  selectedAnswer === index ? styles.selected : ''
                } ${
                  showFeedback && index === question.correctOption ? styles.correctOption : ''
                } ${
                  showFeedback && selectedAnswer === index && !isAnswerCorrect ? styles.incorrectOption : ''
                }`}
                onClick={() => handleAnswerSelect(index)}
                disabled={answeredQuestions.has(currentQuestion)}
              >
                <span className={styles.optionLetter}>
                  {String.fromCharCode(65 + index)}
                </span>
                <span className={styles.optionText}>{option}</span>
                {showFeedback && index === question.correctOption && (
                  <span className={styles.correctCheckmark}>✓</span>
                )}
                {showFeedback && selectedAnswer === index && !isAnswerCorrect && (
                  <span className={styles.incorrectMark}>✗</span>
                )}
              </button>
            ))}
          </div>

          {/* Immediate Feedback Section */}
          {showFeedback && (
            <div className={`${styles.feedbackSection} ${isAnswerCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect}`}>
              <div className={styles.feedbackHeader}>
                {isAnswerCorrect ? (
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

              {!isAnswerCorrect && (
                <div className={styles.feedbackYourAnswer}>
                  <strong>Why your answer was wrong:</strong>
                  <p>
                    You selected: <span className={styles.incorrectText}>{question.options[selectedAnswer]}</span>
                  </p>
                  <p className={styles.feedbackExplanationText}>
                    This answer is incorrect. The correct answer is:{' '}
                    <span className={styles.correctText}>{question.options[question.correctOption]}</span>
                  </p>
                </div>
              )}

              {question.explanation && (
                <div className={styles.feedbackExplanation}>
                  <strong>Explanation:</strong>
                  <p>{question.explanation}</p>
                </div>
              )}

              {question.source && (
                <div className={styles.feedbackSource}>
                  <strong>Source:</strong> {question.source}
                </div>
              )}
            </div>
          )}
        </div>

        <div className={styles.navigationSection}>
          <div className={styles.answeredIndicator}>
            Answered: {answeredQuestions.size} / {displayedQuestions.length}
          </div>

          <div className={styles.navigationButtons}>
            <button
              onClick={handleBack}
              disabled={currentQuestion === 0}
              className={`${styles.navButton} ${styles.backButton}`}
            >
              ← Back
            </button>

            {currentQuestion === displayedQuestions.length - 1 ? (
              <button
                onClick={handleSubmit}
                disabled={!allAnswered}
                className={`${styles.navButton} ${styles.submitButton}`}
                title={!allAnswered ? 'Please answer all questions' : ''}
              >
                Submit Quiz
              </button>
            ) : (
              <button
                onClick={handleNext}
                disabled={!showFeedback}
                className={`${styles.navButton} ${styles.nextButton}`}
                title={!showFeedback ? 'Please answer the question first' : ''}
              >
                Next →
              </button>
            )}
          </div>
        </div>

        <div className={styles.questionDots}>
          {displayedQuestions.map((_, index) => (
            <button
              key={index}
              className={`${styles.dot} ${
                index === currentQuestion ? styles.activeDot : ''
              } ${answeredQuestions.has(index) ? styles.answeredDot : ''}`}
              onClick={() => {
                if (answeredQuestions.has(index)) {
                  setCurrentQuestion(index);
                  setShowFeedback(true);
                }
              }}
              title={`Question ${index + 1}`}
            />
          ))}
        </div>
      </div>
    </div>
  );
};

export default Quiz;
