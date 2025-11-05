# API Contract: Gemini Quiz Generation

**Feature**: 012-learning-hub-sidebar  
**Service**: Google Gemini 2.0 Flash  
**Purpose**: Generate multiple-choice quiz questions from page content

---

## Endpoint Information

**Provider**: Google AI (Generative Language API)  
**Model**: `gemini-2.0-flash-exp`  
**Method**: `generateContent` (non-streaming)  
**Rate Limits**: 15 RPM, 1M TPM, 1500 RPD

---

## Request Specification

### Input Parameters

```typescript
interface GenerateQuizRequest {
  pageContext: {
    url: string;                 // Page URL for quiz
    title: string;               // Page title
    content: string;             // Full page content (max 20000 chars)
  };
  questionCount: number;         // Number of questions (3-5)
  difficulty?: 'easy' | 'medium' | 'hard';  // Optional, defaults to 'medium'
}
```

### System Prompt Template

```typescript
const QUIZ_SYSTEM_PROMPT = `
You are an educational assessment expert creating quiz questions for "${bookTitle}".

## Task
Generate {questionCount} multiple-choice quiz questions based on the page content below.

## Requirements
1. Each question must test understanding of key concepts from the page
2. Provide exactly 4 answer choices per question
3. Clearly mark the correct answer
4. Include a brief explanation for why the answer is correct
5. Difficulty level: {difficulty}
6. Questions should be varied (cover different topics from the page)

## Output Format (JSON)
Return ONLY valid JSON with this structure:
{
  "questions": [
    {
      "question": "What is...?",
      "choices": ["Option A", "Option B", "Option C", "Option D"],
      "correctAnswer": 0,
      "explanation": "This is correct because...",
      "difficulty": "medium"
    }
  ]
}

## Page Content
Title: {pageTitle}
Content: {pageContent}

Generate the quiz now:
`;
```

### API Call Structure

```typescript
import { GoogleGenerativeAI } from '@google/generative-ai';

async function generateQuiz(request: GenerateQuizRequest): Promise<QuizQuestion[]> {
  const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY!);
  const model = genAI.getGenerativeModel({ model: 'gemini-2.0-flash-exp' });

  const prompt = QUIZ_SYSTEM_PROMPT
    .replace('{questionCount}', request.questionCount.toString())
    .replace('{difficulty}', request.difficulty || 'medium')
    .replace('{pageTitle}', request.pageContext.title)
    .replace('{pageContent}', request.pageContext.content.slice(0, 20000));

  const result = await model.generateContent({
    contents: [{ role: 'user', parts: [{ text: prompt }] }],
    generationConfig: {
      temperature: 0.8,        // Higher for question variety
      topK: 40,
      topP: 0.95,
      maxOutputTokens: 2048,
    }
  });

  const response = await result.response;
  const jsonText = extractJSON(response.text());
  const parsed = JSON.parse(jsonText);

  // Transform to internal format
  return parsed.questions.map((q: any) => ({
    id: crypto.randomUUID(),
    pageUrl: request.pageContext.url,
    question: q.question,
    choices: q.choices,
    correctAnswer: q.correctAnswer,
    explanation: q.explanation,
    difficulty: q.difficulty || request.difficulty || 'medium',
    generatedAt: Date.now()
  }));
}

// Helper to extract JSON from potentially wrapped text
function extractJSON(text: string): string {
  // Remove markdown code blocks if present
  const jsonMatch = text.match(/```json\s*([\s\S]*?)\s*```/) ||
                    text.match(/```\s*([\s\S]*?)\s*```/) ||
                    [null, text];
  return jsonMatch[1].trim();
}
```

---

## Response Specification

### Success Response

```typescript
interface QuizQuestion {
  id: string;
  pageUrl: string;
  question: string;
  choices: [string, string, string, string];  // Exactly 4
  correctAnswer: 0 | 1 | 2 | 3;
  explanation: string;
  difficulty: 'easy' | 'medium' | 'hard';
  generatedAt: number;
}

type GenerateQuizResponse = QuizQuestion[];

// Example response
[
  {
    id: "a1b2c3d4-...",
    pageUrl: "/python-basics",
    question: "What does the 'def' keyword do in Python?",
    choices: [
      "Defines a variable",
      "Defines a function",
      "Defines a class",
      "Defines a module"
    ],
    correctAnswer: 1,
    explanation: "'def' is the keyword used to define functions in Python. Functions are reusable blocks of code.",
    difficulty: "easy",
    generatedAt: 1699123456789
  },
  // ... 2-4 more questions
]
```

### Response Validation

```typescript
function validateQuizResponse(questions: any[]): boolean {
  if (!Array.isArray(questions)) return false;
  if (questions.length < 3 || questions.length > 5) return false;

  return questions.every(q =>
    typeof q.question === 'string' &&
    q.question.length >= 10 &&
    q.question.length <= 500 &&
    Array.isArray(q.choices) &&
    q.choices.length === 4 &&
    q.choices.every((c: string) => typeof c === 'string' && c.length > 0) &&
    typeof q.correctAnswer === 'number' &&
    q.correctAnswer >= 0 &&
    q.correctAnswer <= 3 &&
    typeof q.explanation === 'string' &&
    q.explanation.length >= 10 &&
    ['easy', 'medium', 'hard'].includes(q.difficulty)
  );
}

// Handle invalid responses
const questions = await generateQuiz(request);
if (!validateQuizResponse(questions)) {
  throw new QuizError(
    'INVALID_RESPONSE',
    'Generated quiz failed validation. Please try again.'
  );
}
```

---

## Error Handling

| Error Code | Status | User Message | Recovery Action |
|------------|--------|--------------|-----------------|
| 429 | Too Many Requests | "Quiz generation limit reached. Please wait before generating another quiz." | Disable "Generate Quiz" button temporarily |
| 400 | Bad Request | "Could not generate quiz for this page. Content may be too short or complex." | Show fallback message |
| 401 | Unauthorized | "AI service authentication failed. Please refresh the page." | Reload page |
| 503 | Service Unavailable | "Quiz service temporarily unavailable." | Retry button with delay |
| PARSE_ERROR | JSON Parse Error | "Generated quiz format was invalid. Please try again." | Retry button |
| VALIDATION_ERROR | Invalid Questions | "Generated quiz failed quality checks. Please try again." | Retry button |

```typescript
class QuizError extends Error {
  constructor(
    public code: string,
    message: string,
    public userMessage?: string
  ) {
    super(message);
    this.name = 'QuizError';
  }
}

async function generateQuizWithErrorHandling(
  request: GenerateQuizRequest
): Promise<QuizQuestion[]> {
  try {
    await sharedLimiter.checkLimit('quiz');
    const questions = await generateQuiz(request);
    
    if (!validateQuizResponse(questions)) {
      throw new QuizError(
        'VALIDATION_ERROR',
        'Quiz validation failed',
        'Generated quiz did not meet quality standards. Please try again.'
      );
    }
    
    return questions;
  } catch (error) {
    if (error instanceof QuizError) {
      throw error;
    } else if (error.status === 429) {
      throw new QuizError(
        'RATE_LIMIT',
        'Rate limit exceeded',
        'Too many quiz generation requests. Please wait a moment.'
      );
    } else if (error instanceof SyntaxError) {
      throw new QuizError(
        'PARSE_ERROR',
        'JSON parse error',
        'Quiz response format was invalid. Please try again.'
      );
    } else {
      throw new QuizError(
        'UNKNOWN_ERROR',
        error.message,
        'An unexpected error occurred. Please try again.'
      );
    }
  }
}
```

---

## Caching Strategy

**Not Cached**: Quizzes are regenerated on each attempt to provide variety and freshness.

**Rationale**:
- Users expect different questions on each attempt ("Retake Quiz")
- Page content may change (cache invalidation complexity)
- Quiz generation is fast (~2-3 seconds)

---

## Rate Limiting Strategy

**Shared Limiter**: Uses SharedRateLimiter (combined 15 RPM with other AI features)

### User-Facing Feedback

- Disable "Generate Quiz" button when limit reached
- Show countdown: "Quiz available again in {X} seconds"
- Display remaining requests: "5 AI requests remaining this minute"

---

## Performance Targets

- **Generation Time**: < 3 seconds for 5 questions
- **Success Rate**: > 95% (valid JSON with correct structure)
- **Question Quality**: Human-evaluated periodically (relevance, difficulty, clarity)

---

## Testing Strategy

### Unit Tests

```typescript
describe('Generate Quiz API', () => {
  it('should generate 3-5 valid questions', async () => {
    const request: GenerateQuizRequest = {
      pageContext: {
        url: '/python-basics',
        title: 'Python Basics',
        content: 'Python is a high-level programming language...'
      },
      questionCount: 4,
      difficulty: 'medium'
    };
    
    const questions = await generateQuiz(request);
    expect(questions).toHaveLength(4);
    expect(validateQuizResponse(questions)).toBe(true);
  });

  it('should validate question structure', () => {
    const invalidQuestions = [
      { question: 'What?', choices: ['A'], correctAnswer: 0 }  // Only 1 choice
    ];
    
    expect(validateQuizResponse(invalidQuestions)).toBe(false);
  });

  it('should extract JSON from markdown code blocks', () => {
    const wrappedJSON = '```json\n{"questions": []}\n```';
    const extracted = extractJSON(wrappedJSON);
    expect(JSON.parse(extracted)).toEqual({ questions: [] });
  });
});
```

### Integration Tests

```typescript
describe('Generate Quiz Integration', () => {
  it('should generate real quiz from Gemini API', async () => {
    const request: GenerateQuizRequest = {
      pageContext: {
        url: '/test-page',
        title: 'Test Chapter',
        content: `
          JavaScript is a programming language. 
          Variables are declared with let, const, or var.
          Functions are defined with the function keyword.
          Arrays store multiple values in a single variable.
        `
      },
      questionCount: 3,
      difficulty: 'easy'
    };
    
    const questions = await generateQuiz(request);
    
    expect(questions).toHaveLength(3);
    expect(questions[0].question).toContain('JavaScript' || 'variable' || 'function');
    expect(validateQuizResponse(questions)).toBe(true);
  }, 15000);  // 15s timeout
});
```

### Mock Service Worker (MSW)

```typescript
export const geminiQuizHandler = http.post(
  'https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:generateContent',
  () => {
    return HttpResponse.json({
      candidates: [{
        content: {
          parts: [{
            text: JSON.stringify({
              questions: [
                {
                  question: "What is Python?",
                  choices: ["A snake", "A programming language", "A game", "A database"],
                  correctAnswer: 1,
                  explanation: "Python is a high-level programming language.",
                  difficulty: "easy"
                },
                // ... more mock questions
              ]
            })
          }]
        }
      }]
    });
  }
);
```

---

## Prompt Engineering Notes

### Difficulty Calibration

- **Easy**: Recall of facts, definitions, basic concepts
- **Medium**: Application of concepts, comparisons, reasoning
- **Hard**: Synthesis, evaluation, edge cases, advanced scenarios

### Quality Guidelines (for prompt)

- Questions should have only one unambiguous correct answer
- Distractors (wrong answers) should be plausible but clearly incorrect
- Avoid "all of the above" or "none of the above" options
- Questions should test understanding, not trick the user
- Explanations should reference page content directly

---

## Security Considerations

1. **Input Validation**: Validate pageContent length (max 20000 chars)
2. **Output Sanitization**: Strip any HTML/scripts from generated questions
3. **Content Filtering**: Rely on Gemini's built-in safety filters
4. **Rate Limiting**: Prevent quiz generation spam
5. **No Persistence**: Questions stored in session only (privacy-first)

---

Contract complete. Ready for implementation.
