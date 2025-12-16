import React, { useState, useRef, useEffect, useLayoutEffect } from 'react';
import ReactMarkdown from 'react-markdown';
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
  // Create a ref for smooth scrolling
  const quizRef = useRef<HTMLDivElement>(null);


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
  const [shouldScroll, setShouldScroll] = useState(false);

  // Custom easing function for ultra-smooth scrolling (easeOutCubic - slower and smoother)
  const easeOutCubic = (t: number): number => {
    return 1 - Math.pow(1 - t, 3);
  };

  // Smooth scroll with custom easing
  const smoothScrollToQuiz = () => {
    if (!quizRef.current) return;

    const startY = window.scrollY;
    const quizPosition = quizRef.current.getBoundingClientRect().top + window.scrollY;
    const targetY = quizPosition - 20;
    const distance = targetY - startY;

    // Duration in milliseconds (1200ms for very smooth, slower feel)
    const duration = 1200;
    const startTime = performance.now();

    const scroll = (currentTime: number) => {
      const elapsed = currentTime - startTime;
      const progress = Math.min(elapsed / duration, 1);

      // Apply easing function for smoother deceleration
      const easeProgress = easeOutCubic(progress);
      const currentY = startY + distance * easeProgress;

      window.scrollTo(0, currentY);

      // Continue animation if not complete
      if (progress < 1) {
        requestAnimationFrame(scroll);
      }
    };

    requestAnimationFrame(scroll);
  };

  // Scroll when question or results change
  // Use useLayoutEffect for immediate scroll after DOM mutations but before paint
  useLayoutEffect(() => {
    if (shouldScroll) {
      smoothScrollToQuiz();
      setShouldScroll(false);
    }
  }, [shouldScroll]);

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
      // Update state first
      setCurrentQuestion(currentQuestion + 1);
      // Preserve feedback for previously answered questions
      setShowFeedback(answeredQuestions.has(currentQuestion + 1));
      // Trigger scroll after React renders
      setShouldScroll(true);
    }
  };

  const handleBack = () => {
    if (currentQuestion > 0) {
      // Update state first
      setCurrentQuestion(currentQuestion - 1);
      // Preserve feedback for previously answered questions
      setShowFeedback(answeredQuestions.has(currentQuestion - 1));
      // Trigger scroll after React renders
      setShouldScroll(true);
    }
  };

  const handleSubmit = () => {
    // Update state first
    setShowResults(true);
    // Trigger scroll after React renders
    setShouldScroll(true);
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
    // Trigger scroll after React renders
    setShouldScroll(true);
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
      <div className={styles.quizContainer} ref={quizRef}>
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
                  <div className={styles.reviewQuestion}>
                    <ReactMarkdown components={{ p: ({node, ...props}) => <p {...props} />, a: ({node, ...props}) => <a {...props} target="_blank" rel="noopener noreferrer" /> }}>
                      {q.question}
                    </ReactMarkdown>
                  </div>
                  <div className={styles.reviewAnswers}>
                    <div className={styles.reviewAnswer}>
                      <strong>Your answer:</strong>{' '}
                      <span className={isCorrect ? styles.correctText : styles.incorrectText}>
                        {userAnswer !== null ? (
                          <ReactMarkdown components={{ p: ({node, ...props}) => <span {...props} />, a: ({node, ...props}) => <a {...props} target="_blank" rel="noopener noreferrer" /> }}>
                            {q.options[userAnswer]}
                          </ReactMarkdown>
                        ) : (
                          'Not answered'
                        )}
                      </span>
                    </div>
                    {!isCorrect && (
                      <div className={styles.reviewAnswer}>
                        <strong>Correct answer:</strong>{' '}
                        <span className={styles.correctText}>
                          <ReactMarkdown components={{ p: ({node, ...props}) => <span {...props} />, a: ({node, ...props}) => <a {...props} target="_blank" rel="noopener noreferrer" /> }}>
                            {q.options[q.correctOption]}
                          </ReactMarkdown>
                        </span>
                      </div>
                    )}
                    {q.explanation && (
                      <div className={styles.explanation}>
                        <strong>Explanation:</strong>
                        <div className={styles.explanationContent}>
                          <ReactMarkdown components={{ p: ({node, ...props}) => <p {...props} />, a: ({node, ...props}) => <a {...props} target="_blank" rel="noopener noreferrer" /> }}>
                            {q.explanation}
                          </ReactMarkdown>
                        </div>
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
    <div className={styles.quizContainer} ref={quizRef}>
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
          <h3 className={styles.questionText}>
            <ReactMarkdown components={{ p: ({node, ...props}) => <span {...props} />, a: ({node, ...props}) => <a {...props} target="_blank" rel="noopener noreferrer" /> }}>
              {question.question}
            </ReactMarkdown>
          </h3>

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
                <span className={styles.optionText}>
                  <ReactMarkdown components={{ p: ({node, ...props}) => <span {...props} />, a: ({node, ...props}) => <a {...props} target="_blank" rel="noopener noreferrer" /> }}>
                    {option}
                  </ReactMarkdown>
                </span>
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
            <div
              className={`${styles.feedbackSection} ${isAnswerCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect}`}
              role="region"
              aria-live="polite"
              aria-label="Question feedback"
            >
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

              {!isAnswerCorrect && selectedAnswer !== null && (
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
                  <div className={styles.explanationContent}>
                    <ReactMarkdown components={{ p: ({node, ...props}) => <p {...props} />, a: ({node, ...props}) => <a {...props} target="_blank" rel="noopener noreferrer" /> }}>
                      {question.explanation}
                    </ReactMarkdown>
                  </div>
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
              aria-disabled={currentQuestion === 0}
              aria-describedby={currentQuestion === 0 ? "back-button-help" : undefined}
              className={`${styles.navButton} ${styles.backButton}`}
              title={currentQuestion === 0 ? 'You are on the first question' : ''}
            >
              ← Back
            </button>
            {currentQuestion === 0 && (
              <span id="back-button-help" className={styles.srOnly}>
                You are on the first question. Cannot go back.
              </span>
            )}

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
              <>
                <button
                  onClick={handleNext}
                  disabled={!showFeedback}
                  aria-disabled={!showFeedback}
                  aria-describedby={!showFeedback ? "next-button-help" : undefined}
                  className={`${styles.navButton} ${styles.nextButton}`}
                  title={!showFeedback ? 'Please answer the question first' : ''}
                >
                  Next →
                </button>
                {!showFeedback && (
                  <span id="next-button-help" className={styles.srOnly}>
                    Please answer the question first to proceed to the next question.
                  </span>
                )}
              </>
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
                  setShouldScroll(true);
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
