import React, { useMemo, useState } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { jsPDF } from 'jspdf';
import './assessmentContent.css';

let useDocSafe: any = null;

if (ExecutionEnvironment.canUseDOM) {
  try {
    const docModule = require('@docusaurus/plugin-content-docs/client');
    useDocSafe = docModule.useDoc;
  } catch (error) {
    // ignore
  }
}

type Difficulty = 'easy' | 'medium' | 'hard' | 'professional';

type AssessmentQuestion = {
  id: string;
  question: string;
  options: string[];
  answerIndex: number;
  explanation?: string;
};

type Phase = 'setup' | 'loading' | 'quiz' | 'results';

const questionPresets = [5, 10, 15, 20];

const difficultyOptions: { label: string; value: Difficulty; description: string }[] = [
  { label: 'Easy', value: 'easy', description: 'Fundamental recall and definitions' },
  { label: 'Medium', value: 'medium', description: 'Concept links + light reasoning' },
  { label: 'Hard', value: 'hard', description: 'Multi-step reasoning & applied use' },
  { label: 'Professional', value: 'professional', description: 'Advanced vocabulary & nuance' },
];

const examTypes = ['General Assessment', 'Coding Interview', 'Certification Prep', 'Rapid Review'];

const resolveApiBase = () => {
  if (ExecutionEnvironment.canUseDOM) {
    const host = window.location.hostname;
    if (host === 'localhost' || host === '127.0.0.1') {
      return 'http://localhost:3001/api';
    }
  }
  return '/api';
};

interface AssessmentContentProps {
  onClose?: () => void;
}

const AssessmentContent: React.FC<AssessmentContentProps> = ({ onClose }) => {
  let docContext: any = null;

  try {
    if (useDocSafe) {
      docContext = useDocSafe();
    }
  } catch {
    docContext = null;
  }

  const apiBase = useMemo(() => resolveApiBase(), []);
  const defaultTopic = docContext?.metadata?.title || 'AI Native Software Development';

  const [phase, setPhase] = useState<Phase>('setup');
  const [questionCount, setQuestionCount] = useState(5);
  const [customQuestionCount, setCustomQuestionCount] = useState('');
  const [difficulty, setDifficulty] = useState<Difficulty>('medium');
  const [topic, setTopic] = useState(defaultTopic);
  const [examType, setExamType] = useState(examTypes[0]);
  const [loadingHint, setLoadingHint] = useState('Preparing your adaptive assessment…');
  const [error, setError] = useState<string | null>(null);

  const [questions, setQuestions] = useState<AssessmentQuestion[]>([]);
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [answers, setAnswers] = useState<(number | null)[]>([]);
  const [resultSummary, setResultSummary] = useState<{
    correct: number;
    total: number;
  } | null>(null);

  const [breakdown, setBreakdown] = useState<
    { question: AssessmentQuestion; userAnswer: number | null }[]
  >([]);

  const finalQuestionCount = customQuestionCount
    ? Math.max(1, Number(customQuestionCount))
    : questionCount;

  const selectedQuestion = questions[currentQuestion];
  const currentAnswer = answers[currentQuestion];

  const handlePresetSelect = (count: number) => {
    setQuestionCount(count);
    setCustomQuestionCount('');
  };

  const handleCustomCountChange = (value: string) => {
    setCustomQuestionCount(value);
    if (!value) {
      return;
    }
    const parsed = Number(value);
    if (!Number.isNaN(parsed) && parsed > 0) {
      setQuestionCount(parsed);
    }
  };

  const handleOptionSelect = (index: number) => {
    const updated = [...answers];
    updated[currentQuestion] = index;
    setAnswers(updated);
  };

  const moveToNextQuestion = () => {
    if (currentQuestion < questions.length - 1) {
      setCurrentQuestion((prev) => prev + 1);
    } else {
      finalizeAssessment();
    }
  };

  const moveToPrevQuestion = () => {
    if (currentQuestion > 0) {
      setCurrentQuestion((prev) => prev - 1);
    }
  };

  const finalizeAssessment = () => {
    const correct = answers.reduce((acc, ans, index) => {
      if (ans === questions[index].answerIndex) {
        return acc + 1;
      }
      return acc;
    }, 0);

    setResultSummary({ correct, total: questions.length });
    setBreakdown(
      questions.map((question, index) => ({
        question,
        userAnswer: answers[index],
      }))
    );
    setPhase('results');
  };

  const resetAssessment = () => {
    setPhase('setup');
    setQuestions([]);
    setAnswers([]);
    setCurrentQuestion(0);
    setResultSummary(null);
    setBreakdown([]);
    setError(null);
  };

  const downloadPdf = (generated: AssessmentQuestion[], meta?: { topic?: string }) => {
    const doc = new jsPDF();
    const title = `Assessment - ${meta?.topic || topic}`;

    doc.setFontSize(16);
    doc.text(title, 10, 20);

    let y = 30;
    generated.forEach((item, index) => {
      const questionLabel = `${index + 1}. ${item.question}`;
      const lines = doc.splitTextToSize(questionLabel, 180);

      if (y + lines.length * 8 > 270) {
        doc.addPage();
        y = 20;
      }

      doc.setFontSize(12);
      doc.text(lines, 10, y);
      y += lines.length * 6 + 2;

      item.options.forEach((option, optIndex) => {
        const optionLabel = `${String.fromCharCode(65 + optIndex)}. ${option}`;
        const optionLines = doc.splitTextToSize(optionLabel, 170);

        if (y + optionLines.length * 6 > 280) {
          doc.addPage();
          y = 20;
        }

        doc.text(optionLines, 14, y);
        y += optionLines.length * 6 + 1;
      });

      if (item.explanation) {
        const explanationLines = doc.splitTextToSize(`Explanation: ${item.explanation}`, 170);
        if (y + explanationLines.length * 6 > 280) {
          doc.addPage();
          y = 20;
        }
        doc.setFontSize(11);
        doc.text(explanationLines, 14, y);
        y += explanationLines.length * 5 + 4;
      }

      y += 4;
    });

    doc.save(`${title.replace(/\s+/g, '-').toLowerCase()}.pdf`);
  };

  const requestAssessment = async (mode: 'quiz' | 'pdf') => {
    try {
      setError(null);
      setPhase('loading');
      setLoadingHint(mode === 'pdf' ? 'Building a printable PDF…' : 'Generating your adaptive test…');

      const response = await fetch(`${apiBase}/assessment/generate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          questionCount: finalQuestionCount,
          difficulty,
          topic,
          examType,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to generate assessment. Please try again.');
      }

      const data = await response.json();
      const generatedQuestions: AssessmentQuestion[] = Array.isArray(data.questions) ? data.questions : [];

      if (!generatedQuestions.length) {
        throw new Error('The AI did not return any questions. Please try again.');
      }

      if (mode === 'pdf') {
        downloadPdf(generatedQuestions, data.meta);
        setPhase('setup');
        return;
      }

      setQuestions(generatedQuestions);
      setAnswers(new Array(generatedQuestions.length).fill(null));
      setCurrentQuestion(0);
      setPhase('quiz');
    } catch (err: any) {
      console.error(err);
      setError(err.message || 'Something went wrong while generating the assessment.');
      setPhase('setup');
    }
  };

  const renderSetup = () => (
    <div className="assessment-content">
      <div className="assessment-section">
        <p className="assessment-eyebrow">Step 1 · Question Volume</p>
        <h3>How many questions would you like?</h3>
        <div className="count-options">
          {questionPresets.map((count) => (
            <button
              key={count}
              type="button"
              className={`count-option ${finalQuestionCount === count ? 'active' : ''}`}
              onClick={() => handlePresetSelect(count)}
            >
              {count}
            </button>
          ))}
        </div>
        <label className="assessment-field">
          <span>Or enter a custom number</span>
          <input
            type="number"
            min={1}
            inputMode="numeric"
            value={customQuestionCount}
            onChange={(event) => handleCustomCountChange(event.target.value)}
            placeholder="e.g. 25"
          />
        </label>
      </div>

      <div className="assessment-section">
        <p className="assessment-eyebrow">Step 2 · Difficulty</p>
        <h3>Select a difficulty level</h3>
        <div className="difficulty-grid">
          {difficultyOptions.map((option) => (
            <button
              key={option.value}
              type="button"
              className={`difficulty-card ${difficulty === option.value ? 'active' : ''}`}
              onClick={() => setDifficulty(option.value)}
            >
              <span className="difficulty-label">{option.label}</span>
              <span className="difficulty-caption">{option.description}</span>
            </button>
          ))}
        </div>
      </div>

      <div className="assessment-section">
        <p className="assessment-eyebrow">Step 3 · Context</p>
        <div className="assessment-field-group">
          <label className="assessment-field">
            <span>Exam type</span>
            <select value={examType} onChange={(event) => setExamType(event.target.value)}>
              {examTypes.map((type) => (
                <option key={type} value={type}>
                  {type}
                </option>
              ))}
            </select>
          </label>

          <label className="assessment-field">
            <span>Topic</span>
            <input
              type="text"
              value={topic}
              onChange={(event) => setTopic(event.target.value)}
              placeholder="e.g. Zooming in SQL"
            />
            <small className="field-hint">
              Leave blank to automatically use “{defaultTopic}”
            </small>
          </label>
        </div>
      </div>

      {error && <p className="assessment-error">{error}</p>}

      <div className="assessment-actions">
        <button
          type="button"
          className="assessment-primary"
          onClick={() => requestAssessment('quiz')}
        >
          Generate &amp; Take Test Here
        </button>
        <button
          type="button"
          className="assessment-secondary"
          onClick={() => requestAssessment('pdf')}
        >
          Generate &amp; Download PDF
        </button>
      </div>
    </div>
  );

  const renderLoading = () => (
    <div className="assessment-loading">
      <div className="assessment-spinner" aria-hidden="true" />
      <p>{loadingHint}</p>
      <small>We’re working with the LLM to craft fresh questions…</small>
    </div>
  );

  const renderQuiz = () => {
    if (!selectedQuestion) {
      return null;
    }

    const progress = ((currentQuestion + 1) / questions.length) * 100;

    return (
      <div className="assessment-quiz">
        <div className="quiz-progress">
          <div className="quiz-progress-bar" style={{ width: `${progress}%` }} />
          <span>
            Question {currentQuestion + 1} / {questions.length}
          </span>
        </div>

        <h3 className="quiz-question">{selectedQuestion.question}</h3>

        <div className="quiz-options">
          {selectedQuestion.options.map((option, index) => (
            <button
              key={option}
              type="button"
              className={`quiz-option ${currentAnswer === index ? 'selected' : ''}`}
              onClick={() => handleOptionSelect(index)}
            >
              <span className="quiz-option-letter">{String.fromCharCode(65 + index)}</span>
              <span className="quiz-option-text">{option}</span>
            </button>
          ))}
        </div>

        <div className="quiz-navigation">
          <button
            type="button"
            className="quiz-nav quiz-nav--ghost"
            onClick={moveToPrevQuestion}
            disabled={currentQuestion === 0}
          >
            ← Back
          </button>
          <button
            type="button"
            className="quiz-nav"
            onClick={moveToNextQuestion}
            disabled={currentAnswer === null}
          >
            {currentQuestion === questions.length - 1 ? 'Submit' : 'Next →'}
          </button>
        </div>
      </div>
    );
  };

  const renderResults = () => {
    if (!resultSummary) {
      return null;
    }

    return (
      <div className="assessment-results">
        <div className="results-hero">
          <div>
            <p className="assessment-eyebrow">Assessment complete</p>
            <h3>Great job! You made it to the finish line.</h3>
            <p className="results-caption">
              You answered {resultSummary.correct} out of {resultSummary.total} questions correctly.
            </p>
          </div>
          <div className="results-score">
            <span>{Math.round((resultSummary.correct / resultSummary.total) * 100)}%</span>
            <small>Score</small>
          </div>
        </div>

        <div className="results-breakdown">
          {breakdown.map(({ question, userAnswer }, index) => {
            const isCorrect = userAnswer === question.answerIndex;

            return (
              <div key={question.id} className="breakdown-card">
                <div className="breakdown-header">
                  <span>Question {index + 1}</span>
                  <span className={`badge ${isCorrect ? 'badge--success' : 'badge--danger'}`}>
                    {isCorrect ? 'Correct' : 'Review'}
                  </span>
                </div>
                <p className="breakdown-question">{question.question}</p>
                <p className="breakdown-answer">
                  <strong>Your answer:</strong>{' '}
                  {userAnswer !== null ? question.options[userAnswer] : 'Not answered'}
                </p>
                {!isCorrect && (
                  <p className="breakdown-answer">
                    <strong>Correct answer:</strong> {question.options[question.answerIndex]}
                  </p>
                )}
                {question.explanation && (
                  <p className="breakdown-explanation">
                    <strong>Why:</strong> {question.explanation}
                  </p>
                )}
              </div>
            );
          })}
        </div>

        <div className="results-actions">
          <button type="button" className="assessment-secondary" onClick={resetAssessment}>
            Create Another Test
          </button>
          <button type="button" className="assessment-primary" onClick={onClose}>
            Done
          </button>
        </div>
      </div>
    );
  };

  if (phase === 'loading') {
    return renderLoading();
  }

  if (phase === 'quiz') {
    return renderQuiz();
  }

  if (phase === 'results') {
    return renderResults();
  }

  return renderSetup();
};

export default AssessmentContent;


