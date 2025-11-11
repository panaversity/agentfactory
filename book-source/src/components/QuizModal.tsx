import React, { useState, useEffect } from 'react';

// --- (Interfaces are the same) ---
interface Option { a: string; b: string; c: string; d: string; }
interface Question { question: string; options: Option; correct_answer: 'a' | 'b' | 'c' | 'd'; explanation: string; }
interface ChapterQuizData { chapter_id: number; chapter_title: string; skipped: boolean; questions: Question[]; }
interface QuizModalProps { chapterId: number; onClose: () => void; quizData: ChapterQuizData[]; }
interface AnswerRecord { question: string; selectedAnswer: string; correctAnswer: string; explanation: string; isCorrect: boolean; }


const QuizModal: React.FC<QuizModalProps> = ({ chapterId, onClose, quizData }) => {
  const [currentQuestionIndex, setCurrentQuestionIndex] = useState(0);
  const [selectedAnswer, setSelectedAnswer] = useState<string | null>(null);
  const [quizCompleted, setQuizCompleted] = useState(false);
  const [answers, setAnswers] = useState<AnswerRecord[]>([]);

  const chapterQuiz = quizData.find(chapter => chapter.chapter_id === chapterId);

  useEffect(() => {
    setCurrentQuestionIndex(0);
    setSelectedAnswer(null);
    setQuizCompleted(false);
    setAnswers([]);
  }, [chapterId]);

  if (!chapterQuiz || chapterQuiz.skipped || chapterQuiz.questions.length === 0) {
    return (
      <div className="quiz-modal-overlay">
        <div className="quiz-modal-content">
          <button onClick={onClose} className="quiz-modal-close-button">×</button>
          <h2>Quiz Not Available</h2>
          <p>No quiz found for this chapter or it has been skipped.</p>
        </div>
      </div>
    );
  }

  // --- NEW: Add a hard check for currentQuestion ---
  if (currentQuestionIndex >= chapterQuiz.questions.length) {
      // This case should not happen with correct logic, but as a safeguard:
      return (
          <div className="quiz-modal-overlay">
              <div className="quiz-modal-content">
                  <button onClick={onClose} className="quiz-modal-close-button">×</button>
                  <h2>Error</h2>
                  <p>An unexpected error occurred while loading the quiz question.</p>
              </div>
          </div>
      );
  }

  const currentQuestion = chapterQuiz.questions[currentQuestionIndex];

  // --- NEW: Add a hard check for currentQuestion being undefined ---
  if (!currentQuestion) {
    return (
        <div className="quiz-modal-overlay">
            <div className="quiz-modal-content">
                <button onClick={onClose} className="quiz-modal-close-button">×</button>
                <h2>Error Loading Question</h2>
                <p>Could not load the current question. The quiz data might be incomplete.</p>
            </div>
        </div>
    );
  }


  const handleAnswerSelect = (option: string) => {
    setSelectedAnswer(option);
  };

  const handleNextQuestion = () => {
    if (selectedAnswer) {
      const isCorrect = selectedAnswer === currentQuestion.correct_answer;
      setAnswers(prevAnswers => [
        ...prevAnswers,
        {
          question: currentQuestion.question,
          selectedAnswer: selectedAnswer,
          correctAnswer: currentQuestion.correct_answer,
          explanation: currentQuestion.explanation,
          isCorrect: isCorrect,
        },
      ]);

      setSelectedAnswer(null);

      if (currentQuestionIndex < chapterQuiz.questions.length - 1) {
        setCurrentQuestionIndex(prevIndex => prevIndex + 1);
      } else {
        setQuizCompleted(true);
      }
    }
  };

  const score = answers.filter(answer => answer.isCorrect).length;

  const options = currentQuestion.options;
  const areOptionsValid = options && typeof options === 'object' && !Array.isArray(options);


  return (
    <div className="quiz-modal-overlay">
      <div className="quiz-modal-content">
        <button onClick={onClose} className="quiz-modal-close-button">×</button>

        {!quizCompleted ? (
          <>
            <h2>{chapterQuiz.chapter_title}</h2>
            <p>Question {currentQuestionIndex + 1} of {chapterQuiz.questions.length}</p>
            <h3>{currentQuestion.question}</h3>
            <div className="options">
              {areOptionsValid ? (
                Object.entries(options).map(([key, value]) => (
                  <div key={key}>
                    <input
                      type="radio"
                      id={`option-${key}`}
                      name="quiz-option"
                      value={key}
                      checked={selectedAnswer === key}
                      onChange={() => handleAnswerSelect(key)}
                    />
                    <label htmlFor={`option-${key}`}>{key.toUpperCase()}: {value}</label>
                  </div>
                ))
              ) : (
                <p style={{color: 'red'}}>Error: This question's options are missing or invalid.</p>
              )}
            </div>
            <button onClick={handleNextQuestion} disabled={!selectedAnswer || !areOptionsValid}>
              {currentQuestionIndex < chapterQuiz.questions.length - 1 ? 'Next Question' : 'Submit'}
            </button>
          </>
        ) : (
          // ... (rest of the component is the same)
          <>
            <h2>Quiz Completed!</h2>
            <p>Your Score: {score} out of {chapterQuiz.questions.length}</p>
            <hr />
            <h3>Review Your Answers</h3>
            <div className="quiz-review">
              {answers.map((answer, index) => (
                <div key={index} className="review-item">
                  <h4>{index + 1}. {answer.question}</h4>
                  <p style={{ color: answer.isCorrect ? 'green' : 'red' }}>
                    Your answer: {answer.selectedAnswer.toUpperCase()} (Correct: {answer.correctAnswer.toUpperCase()})
                  </p>
                  {!answer.isCorrect && (
                    <p className="explanation">
                      <strong>Explanation:</strong> {answer.explanation}
                    </p>
                  )}
                </div>
              ))}
            </div>
          </>
        )}
      </div>
    </div>
  );
};

export default QuizModal;
