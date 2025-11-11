import React, { useState, useRef, useEffect } from 'react';
import styles from './Quiz.module.css';

// --- UPDATED INTERFACE to match JSON data structure ---
export interface QuizQuestion {
  question: string;
  options: { [key: string]: string }; // Was an array, now an object
  correct_answer: string; // Was correctOption: number
  explanation?: string;
  source?: string;
}

export interface QuizProps {
  title?: string;
  questions: QuizQuestion[];
  questionsPerBatch?: number;
}

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
  const quizRef = useRef<HTMLDivElement>(null);

  const scrollToTop = () => {
    if (quizRef.current) {
      quizRef.current.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
  };

  const [displayedQuestions, setDisplayedQuestions] = useState<QuizQuestion[]>(() => {
    const shuffled = shuffleArray(questions);
    return shuffled.slice(0, questionsPerBatch);
  });

  const [currentQuestion, setCurrentQuestion] = useState(0);
  // --- UPDATED: Store answer keys ('a', 'b', etc.) instead of index ---
  const [selectedAnswers, setSelectedAnswers] = useState<(string | null)[]>(
    new Array(displayedQuestions.length).fill(null)
  );
  const [showFeedback, setShowFeedback] = useState(false);
  const [showResults, setShowResults] = useState(false);
  const [answeredQuestions, setAnsweredQuestions] = useState<Set<number>>(new Set());
  const [shouldScroll, setShouldScroll] = useState(false);

  useEffect(() => {
    if (shouldScroll) {
      scrollToTop();
      setShouldScroll(false);
    }
  }, [shouldScroll]);

  // --- UPDATED: handleAnswerSelect now takes a string key ---
  const handleAnswerSelect = (optionKey: string) => {
    if (answeredQuestions.has(currentQuestion)) {
      return;
    }
    const newAnswers = [...selectedAnswers];
    newAnswers[currentQuestion] = optionKey;
    setSelectedAnswers(newAnswers);
    const newAnswered = new Set(answeredQuestions);
    newAnswered.add(currentQuestion);
    setAnsweredQuestions(newAnswered);
    setShowFeedback(true);
  };

  const handleNext = () => {
    if (currentQuestion < displayedQuestions.length - 1) {
      setCurrentQuestion(currentQuestion + 1);
      setShowFeedback(answeredQuestions.has(currentQuestion + 1));
      setShouldScroll(true);
    }
  };

  const handleBack = () => {
    if (currentQuestion > 0) {
      setCurrentQuestion(currentQuestion - 1);
      setShowFeedback(answeredQuestions.has(currentQuestion - 1));
      setShouldScroll(true);
    }
  };

  const handleSubmit = () => {
    setShowResults(true);
    setShouldScroll(true);
  };

  const handleReset = () => {
    const newShuffled = shuffleArray(questions);
    const newBatch = newShuffled.slice(0, questionsPerBatch);
    setDisplayedQuestions(newBatch);
    setCurrentQuestion(0);
    setSelectedAnswers(new Array(newBatch.length).fill(null));
    setShowResults(false);
    setShowFeedback(false);
    setAnsweredQuestions(new Set());
  };

  // --- UPDATED: calculateScore compares keys ---
  const calculateScore = () => {
    let correct = 0;
    selectedAnswers.forEach((answer, index) => {
      if (answer === displayedQuestions[index].correct_answer) {
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
            <div className={styles.statItem}><span className={styles.statValue}>{score.correct}</span><span className={styles.statLabel}>Correct</span></div>
            <div className={styles.statItem}><span className={styles.statValue}>{score.total - score.correct}</span><span className={styles.statLabel}>Incorrect</span></div>
            <div className={styles.statItem}><span className={styles.statValue}>{score.total}</span><span className={styles.statLabel}>Total</span></div>
          </div>
          <div className={styles.resultMessage}><span className={styles.resultIcon}>📚</span><strong>Great effort!</strong> You answered {score.correct} out of {score.total} questions correctly.</div>
          <div className={styles.detailedResults}>
            <h3 className={styles.detailedResultsTitle}>Question Review</h3>
            {/* --- UPDATED: Results display logic --- */}
            {displayedQuestions.map((q, index) => {
              const userAnswerKey = selectedAnswers[index];
              const isCorrect = userAnswerKey === q.correct_answer;
              return (
                <div key={index} className={styles.reviewItem}>
                  <div className={styles.reviewHeader}><span className={styles.reviewQuestionNumber}>Question {index + 1}</span><span className={`${styles.reviewBadge} ${isCorrect ? styles.correctBadge : styles.incorrectBadge}`}>{isCorrect ? '✓ Correct' : '✗ Incorrect'}</span></div>
                  <p className={styles.reviewQuestion}>{q.question}</p>
                  <div className={styles.reviewAnswers}>
                    <div className={styles.reviewAnswer}><strong>Your answer:</strong>{' '}<span className={isCorrect ? styles.correctText : styles.incorrectText}>{userAnswerKey !== null ? q.options[userAnswerKey] : 'Not answered'}</span></div>
                    {!isCorrect && (<div className={styles.reviewAnswer}><strong>Correct answer:</strong>{' '}<span className={styles.correctText}>{q.options[q.correct_answer]}</span></div>)}
                    {q.explanation && (<div className={styles.explanation}><strong>Explanation:</strong> {q.explanation}</div>)}
                  </div>
                </div>
              );
            })}
          </div>
          <button onClick={() => { handleReset(); setShouldScroll(true); }} className={styles.resetButton}>Retake Quiz</button>
        </div>
      </div>
    );
  }

  const question = displayedQuestions[currentQuestion];
  const selectedAnswerKey = selectedAnswers[currentQuestion];
  const progress = ((currentQuestion + 1) / displayedQuestions.length) * 100;
  const isAnswerCorrect = selectedAnswerKey === question.correct_answer;

  // --- Defensive check for options ---
  const options = question?.options;
  const areOptionsValid = options && typeof options === 'object' && !Array.isArray(options);

  return (
    <div className={styles.quizContainer} ref={quizRef}>
      <div className={styles.quizCard}>
        <div className={styles.quizHeader}><h2 className={styles.quizTitle}>{title}</h2><div className={styles.questionCounter}>Question {currentQuestion + 1} of {displayedQuestions.length}</div></div>
        <div className={styles.progressBar}><div className={styles.progressFill} style={{ width: `${progress}%` }} /></div>
        <div className={styles.questionSection}>
          <h3 className={styles.questionText}>{question.question}</h3>
          {/* --- UPDATED: Rendering logic for options --- */}
          <div className={styles.optionsGrid}>
            {areOptionsValid ? Object.entries(options).map(([key, value]) => (
              <button
                key={key}
                className={`${styles.optionButton} ${selectedAnswerKey === key ? styles.selected : ''} ${showFeedback && key === question.correct_answer ? styles.correctOption : ''} ${showFeedback && selectedAnswerKey === key && !isAnswerCorrect ? styles.incorrectOption : ''}`}
                onClick={() => handleAnswerSelect(key)}
                disabled={answeredQuestions.has(currentQuestion)}
              >
                <span className={styles.optionLetter}>{key.toUpperCase()}</span>
                <span className={styles.optionText}>{value}</span>
                {showFeedback && key === question.correct_answer && (<span className={styles.correctCheckmark}>✓</span>)}
                {showFeedback && selectedAnswerKey === key && !isAnswerCorrect && (<span className={styles.incorrectMark}>✗</span>)}
              </button>
            )) : <p style={{color: 'red'}}>Error: Options for this question are invalid.</p>}
          </div>
          {showFeedback && (
            <div className={`${styles.feedbackSection} ${isAnswerCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect}`} role="region" aria-live="polite" aria-label="Question feedback">
              <div className={styles.feedbackHeader}>{isAnswerCorrect ? (<><span className={styles.feedbackIcon}>✓</span><strong>Correct!</strong></>) : (<><span className={styles.feedbackIcon}>✗</span><strong>Incorrect</strong></>)}</div>
              {!isAnswerCorrect && selectedAnswerKey !== null && (
                <div className={styles.feedbackYourAnswer}>
                  <strong>Why your answer was wrong:</strong>
                  <p>You selected: <span className={styles.incorrectText}>{question.options[selectedAnswerKey]}</span></p>
                  <p className={styles.feedbackExplanationText}>This answer is incorrect. The correct answer is:{' '}<span className={styles.correctText}>{question.options[question.correct_answer]}</span></p>
                </div>
              )}
              {question.explanation && (<div className={styles.feedbackExplanation}><strong>Explanation:</strong><p>{question.explanation}</p></div>)}
              {question.source && (<div className={styles.feedbackSource}><strong>Source:</strong> {question.source}</div>)}
            </div>
          )}
        </div>
        <div className={styles.navigationSection}>
          <div className={styles.answeredIndicator}>Answered: {answeredQuestions.size} / {displayedQuestions.length}</div>
          <div className={styles.navigationButtons}>
            <button onClick={handleBack} disabled={currentQuestion === 0} aria-disabled={currentQuestion === 0} aria-describedby={currentQuestion === 0 ? "back-button-help" : undefined} className={`${styles.navButton} ${styles.backButton}`} title={currentQuestion === 0 ? 'You are on the first question' : ''}>← Back</button>
            {currentQuestion === 0 && (<span id="back-button-help" className={styles.srOnly}>You are on the first question. Cannot go back.</span>)}
            {currentQuestion === displayedQuestions.length - 1 ? (
              <button onClick={handleSubmit} disabled={!allAnswered} className={`${styles.navButton} ${styles.submitButton}`} title={!allAnswered ? 'Please answer all questions' : ''}>Submit Quiz</button>
            ) : (
              <>
                <button onClick={handleNext} disabled={!showFeedback} aria-disabled={!showFeedback} aria-describedby={!showFeedback ? "next-button-help" : undefined} className={`${styles.navButton} ${styles.nextButton}`} title={!showFeedback ? 'Please answer the question first' : ''}>Next →</button>
                {!showFeedback && (<span id="next-button-help" className={styles.srOnly}>Please answer the question first to proceed to the next question.</span>)}
              </>
            )}
          </div>
        </div>
        <div className={styles.questionDots}>
          {displayedQuestions.map((_, index) => (
            <button
              key={index}
              className={`${styles.dot} ${index === currentQuestion ? styles.activeDot : ''} ${answeredQuestions.has(index) ? styles.answeredDot : ''}`}
              onClick={() => { if (answeredQuestions.has(index)) { setCurrentQuestion(index); setShowFeedback(true); setShouldScroll(true); } }}
              title={`Question ${index + 1}`}
            />
          ))}
        </div>
      </div>
    </div>
  );
};

export default Quiz;
