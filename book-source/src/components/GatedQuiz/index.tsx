import React from 'react';
import Quiz, { QuizProps } from '@/components/quiz/Quiz';
import ContentGate from '@/components/ContentGate';

interface GatedQuizProps extends QuizProps {
  /** Override the default gate title */
  gateTitle?: string;
  /** Override the default gate description */
  gateDescription?: string;
}

/**
 * GatedQuiz - A Quiz component wrapped with authentication gate
 *
 * Users must be signed in to access the quiz.
 * When not authenticated, shows a preview with sign-in prompt.
 */
export function GatedQuiz({
  gateTitle,
  gateDescription,
  ...quizProps
}: GatedQuizProps) {
  return (
    <ContentGate
      type="quiz"
      title={gateTitle}
      description={gateDescription}
    >
      <Quiz {...quizProps} />
    </ContentGate>
  );
}

export default GatedQuiz;
