/**
 * QuizInterface Component
 * Main quiz UI with question navigation and score display
 */

import React, { type JSX } from 'react';
import { useQuiz } from '../../hooks/useQuiz';
import { usePageContent } from '../../hooks/usePageContent';
import QuizQuestion from './QuizQuestion';
import QuizResults from './QuizResults';
import styles from './QuizInterface.module.css';

export default function QuizInterface(): JSX.Element {
  const { session, isLoading, error, generateQuiz, answerQuestion, nextQuestion, previousQuestion, retakeQuiz, clearQuiz } = useQuiz();
  const pageContent = usePageContent();
  const [lastPageUrl, setLastPageUrl] = React.useState(pageContent.url);

  // Clear quiz when page changes
  React.useEffect(() => {
    if (pageContent.url !== lastPageUrl && lastPageUrl !== '') {
      console.log('[QuizInterface] Page changed, clearing quiz');
      clearQuiz();
    }
    setLastPageUrl(pageContent.url);
  }, [pageContent.url, lastPageUrl, clearQuiz]);

  // Handle generate quiz button click
  const handleGenerateQuiz = async () => {
    if (!pageContent) {
      console.error('[QuizInterface] No page content available');
      return;
    }

    // Check if content is too short
    const wordCount = pageContent.content.split(/\s+/).length;
    if (wordCount < 200) {
      console.warn('[QuizInterface] Content too brief for quiz');
      return;
    }

    await generateQuiz({
      url: pageContent.url,
      title: pageContent.title,
      content: pageContent.content,
    });
  };

  // Handle answer selection
  const handleAnswerSelect = (selectedChoice: number) => {
    if (!session) return;
    answerQuestion(session.currentQuestionIndex, selectedChoice);
  };

  // Handle retake button
  const handleRetake = async () => {
    await retakeQuiz();
  };

  // Check if content is too short
  const contentTooShort = pageContent && pageContent.content.split(/\s+/).length < 200;

  // Show loading state
  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>
          <div className={styles.spinner}></div>
          <p>Generating quiz questions...</p>
        </div>
      </div>
    );
  }

  // Show error state
  if (error) {
    return (
      <div className={styles.container}>
        <div className={styles.error}>
          <h3>⚠️ Quiz Generation Failed</h3>
          <p>{error}</p>
          <button onClick={handleGenerateQuiz} className={styles.retryButton}>
            Try Again
          </button>
        </div>
      </div>
    );
  }

  // Show content too short message
  if (contentTooShort && !session) {
    return (
      <div className={styles.container}>
        <div className={styles.emptyState}>
          <h3>📝 Content Too Brief</h3>
          <p>Content too brief for quiz - try a longer chapter</p>
        </div>
      </div>
    );
  }

  // Show empty state (no quiz generated yet)
  if (!session) {
    return (
      <div className={styles.container}>
        <div className={styles.emptyState}>
          <h3>📝 Quick Quiz</h3>
          <p>Test your understanding of this chapter with AI-generated questions.</p>
          <button onClick={handleGenerateQuiz} className={styles.generateButton}>
            Generate Quiz
          </button>
        </div>
      </div>
    );
  }

  // Show results if quiz is complete
  if (session.isComplete) {
    return (
      <div className={styles.container}>
        <QuizResults
          score={session.score}
          totalQuestions={session.questions.length}
          onRetake={handleRetake}
        />
      </div>
    );
  }

  // Show current question
  const currentQuestion = session.questions[session.currentQuestionIndex];
  const currentAnswer = session.answers.find(
    (a) => a.questionIndex === session.currentQuestionIndex
  );

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h3>📝 Quick Quiz</h3>
        <div className={styles.progress}>
          Question {session.currentQuestionIndex + 1} of {session.questions.length}
        </div>
      </div>

      <div className={styles.content}>
        <QuizQuestion
          question={currentQuestion}
          selectedChoice={currentAnswer?.selectedChoice}
          isAnswered={currentAnswer !== undefined}
          onSelectChoice={handleAnswerSelect}
        />
      </div>

      <div className={styles.navigation}>
        <button
          onClick={previousQuestion}
          disabled={session.currentQuestionIndex === 0}
          className={styles.navButton}
        >
          ← Previous
        </button>

        <div className={styles.score}>
          Score: {session.score.correct}/{session.score.total} ({session.score.percentage}%)
        </div>

        {session.currentQuestionIndex < session.questions.length - 1 ? (
          <button
            onClick={nextQuestion}
            disabled={!currentAnswer}
            className={styles.navButton}
          >
            Next →
          </button>
        ) : (
          <button
            onClick={() => {
              // Auto-complete when last question is answered
              if (currentAnswer && !session.isComplete) {
                console.log('[QuizInterface] Auto-completing quiz');
              }
            }}
            disabled={!currentAnswer}
            className={styles.completeButton}
          >
            Finish Quiz
          </button>
        )}
      </div>
    </div>
  );
}
