/**
 * useQuiz Hook
 * Manages quiz session state, answer tracking, and score calculation
 */

import { useState, useCallback, useEffect } from 'react';
import type { QuizQuestion, QuizDifficulty } from '../types';
import { geminiService } from '../services/geminiService';
import { errorLogger } from '../services/errorLogger';

interface QuizAnswer {
  questionIndex: number;
  selectedChoice: number;
  isCorrect: boolean;
}

interface QuizSession {
  questions: QuizQuestion[];
  answers: QuizAnswer[];
  currentQuestionIndex: number;
  isComplete: boolean;
  score: {
    correct: number;
    total: number;
    percentage: number;
  };
}

interface UseQuizResult {
  session: QuizSession | null;
  isLoading: boolean;
  error: string | null;
  generateQuiz: (pageContext: { url: string; title: string; content: string }) => Promise<void>;
  answerQuestion: (questionIndex: number, selectedChoice: number) => void;
  nextQuestion: () => void;
  previousQuestion: () => void;
  retakeQuiz: () => Promise<void>;
  clearQuiz: () => void;
}

export function useQuiz(): UseQuizResult {
  const [session, setSession] = useState<QuizSession | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [lastPageContext, setLastPageContext] = useState<{
    url: string;
    title: string;
    content: string;
  } | null>(null);
  const [currentPageUrl, setCurrentPageUrl] = useState<string>(() => 
    typeof window !== 'undefined' ? window.location.pathname : ''
  );

  // Clear quiz state when page changes
  useEffect(() => {
    const checkPageChange = () => {
      const pageUrl = typeof window !== 'undefined' ? window.location.pathname : '';
      
      if (currentPageUrl !== pageUrl) {
        console.log('[useQuiz] Page changed from', currentPageUrl, 'to', pageUrl);
        console.log('[useQuiz] Clearing quiz state');
        setSession(null);
        setError(null);
        setLastPageContext(null);
        setCurrentPageUrl(pageUrl);
      }
    };

    // Poll for page changes every 200ms
    const intervalId = setInterval(checkPageChange, 200);
    
    // Also check immediately
    checkPageChange();
    
    return () => clearInterval(intervalId);
  }, [currentPageUrl]);

  /**
   * Generate a new quiz from page content
   */
  const generateQuiz = useCallback(async (pageContext: {
    url: string;
    title: string;
    content: string;
  }) => {
    try {
      setIsLoading(true);
      setError(null);
      setLastPageContext(pageContext);

      console.log('[useQuiz] Generating quiz for:', pageContext.title);

      // Generate 4 questions with medium difficulty
      const rawQuestions = await geminiService.generateQuiz(pageContext, 4, 'medium');

      console.log('[useQuiz] Quiz generated:', rawQuestions.length, 'questions');

      // Map raw questions to QuizQuestion type with required fields
      const questions: QuizQuestion[] = rawQuestions.map((q, index) => ({
        id: `quiz-${Date.now()}-${index}`,
        pageUrl: pageContext.url,
        question: q.question,
        choices: q.choices as [string, string, string, string],
        correctAnswer: q.correctAnswer as 0 | 1 | 2 | 3,
        explanation: q.explanation,
        difficulty: q.difficulty as QuizDifficulty,
        generatedAt: Date.now(),
      }));

      // Initialize session
      const newSession: QuizSession = {
        questions,
        answers: [],
        currentQuestionIndex: 0,
        isComplete: false,
        score: {
          correct: 0,
          total: questions.length,
          percentage: 0,
        },
      };

      setSession(newSession);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to generate quiz';
      console.error('[useQuiz] Error generating quiz:', err);
      errorLogger.logError(err as Error, { context: 'useQuiz.generateQuiz' });
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  }, []);

  /**
   * Record an answer for a question
   */
  const answerQuestion = useCallback((questionIndex: number, selectedChoice: number) => {
    if (!session) return;

    console.log('[useQuiz] Answering question', questionIndex, 'with choice', selectedChoice);

    const question = session.questions[questionIndex];
    const isCorrect = selectedChoice === question.correctAnswer;

    // Check if this question was already answered
    const existingAnswerIndex = session.answers.findIndex(
      (a) => a.questionIndex === questionIndex
    );

    let newAnswers: QuizAnswer[];
    if (existingAnswerIndex >= 0) {
      // Update existing answer
      newAnswers = [...session.answers];
      newAnswers[existingAnswerIndex] = {
        questionIndex,
        selectedChoice,
        isCorrect,
      };
    } else {
      // Add new answer
      newAnswers = [
        ...session.answers,
        {
          questionIndex,
          selectedChoice,
          isCorrect,
        },
      ];
    }

    // Calculate score
    const correct = newAnswers.filter((a) => a.isCorrect).length;
    const total = session.questions.length;
    const percentage = Math.round((correct / total) * 100);

    // Check if quiz is complete
    const isComplete = newAnswers.length === session.questions.length;

    setSession({
      ...session,
      answers: newAnswers,
      isComplete,
      score: {
        correct,
        total,
        percentage,
      },
    });

    console.log('[useQuiz] Answer recorded:', {
      isCorrect,
      score: `${correct}/${total} (${percentage}%)`,
      isComplete,
    });
  }, [session]);

  /**
   * Navigate to next question
   */
  const nextQuestion = useCallback(() => {
    if (!session) return;

    const nextIndex = session.currentQuestionIndex + 1;
    if (nextIndex < session.questions.length) {
      setSession({
        ...session,
        currentQuestionIndex: nextIndex,
      });
      console.log('[useQuiz] Navigated to question', nextIndex + 1);
    }
  }, [session]);

  /**
   * Navigate to previous question
   */
  const previousQuestion = useCallback(() => {
    if (!session) return;

    const prevIndex = session.currentQuestionIndex - 1;
    if (prevIndex >= 0) {
      setSession({
        ...session,
        currentQuestionIndex: prevIndex,
      });
      console.log('[useQuiz] Navigated to question', prevIndex + 1);
    }
  }, [session]);

  /**
   * Retake the quiz (generate new questions from same content)
   */
  const retakeQuiz = useCallback(async () => {
    if (!lastPageContext) {
      console.error('[useQuiz] Cannot retake: no previous page context');
      return;
    }

    console.log('[useQuiz] Retaking quiz');
    await generateQuiz(lastPageContext);
  }, [lastPageContext, generateQuiz]);

  /**
   * Clear quiz session
   */
  const clearQuiz = useCallback(() => {
    console.log('[useQuiz] Clearing quiz session');
    setSession(null);
    setError(null);
    setLastPageContext(null);
  }, []);

  return {
    session,
    isLoading,
    error,
    generateQuiz,
    answerQuestion,
    nextQuestion,
    previousQuestion,
    retakeQuiz,
    clearQuiz,
  };
}
