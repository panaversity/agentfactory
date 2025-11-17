const { GoogleGenerativeAI } = require('@google/generative-ai');

class AssessmentService {
  constructor() {
    const apiKey = process.env.GEMINI_API_KEY;
    if (!apiKey) {
      throw new Error('GEMINI_API_KEY is not set in environment variables');
    }

    this.genAI = new GoogleGenerativeAI(apiKey);
    this.model = this.genAI.getGenerativeModel({ model: 'gemini-2.0-flash' });
  }

  async generateAssessment({ questionCount, difficulty, topic, examType }) {
    const sanitizedTopic = topic?.trim() || 'AI Native Software Development';
    const sanitizedExamType = examType?.trim() || 'General Assessment';
    const clampedCount = Math.max(1, Number(questionCount) || 5);

    const prompt = `You are an assessment generator for an AI engineering course.
Create ${clampedCount} multiple-choice questions for the "${sanitizedExamType}" exam.

Parameters:
- Topic: ${sanitizedTopic}
- Difficulty: ${difficulty}

Output strict JSON with this shape (NO prose, markdown, or code fences):
{
  "questions": [
    {
      "question": "Question text",
      "options": ["Option A", "Option B", "Option C", "Option D"],
      "answerIndex": 0,
      "explanation": "Why the answer is correct"
    }
  ]
}

Rules:
- Always provide exactly 4 unique options per question.
- Use advanced vocabulary only if difficulty is "professional".
- Keep explanations concise (1-2 sentences).`;

    const result = await this.model.generateContent(prompt);
    const response = await result.response;
    const text = response.text();
    const payload = this.extractJson(text);

    const questions = Array.isArray(payload.questions)
      ? payload.questions.map((question, index) => this.normalizeQuestion(question, index))
      : [];

    if (!questions.length) {
      throw new Error('The AI did not return any questions. Please try again.');
    }

    return {
      questions,
      meta: {
        questionCount: questions.length,
        difficulty,
        topic: sanitizedTopic,
        examType: sanitizedExamType,
      },
    };
  }

  extractJson(text) {
    try {
      const jsonMatch = text.match(/\{[\s\S]*\}/);
      if (jsonMatch) {
        return JSON.parse(jsonMatch[0]);
      }
      return JSON.parse(text);
    } catch (error) {
      console.error('Failed to parse assessment JSON:', text);
      throw new Error('Could not parse AI response. Please try again.');
    }
  }

  normalizeQuestion(question, index) {
    const fallbackQuestion = `Question ${index + 1}`;
    const fallbackOptions = ['Option A', 'Option B', 'Option C', 'Option D'];

    let options = Array.isArray(question.options) ? question.options.slice(0, 4) : [];
    while (options.length < 4) {
      options.push(`Option ${String.fromCharCode(65 + options.length)}`);
    }

    return {
      id: `q-${index}`,
      question: question.question?.trim() || fallbackQuestion,
      options,
      answerIndex:
        typeof question.answerIndex === 'number' && question.answerIndex >= 0 && question.answerIndex < 4
          ? question.answerIndex
          : 0,
      explanation: question.explanation?.trim() || 'Review the associated chapter to reinforce the concept.',
    };
  }
}

const assessmentService = new AssessmentService();

async function generateAssessment(payload) {
  return assessmentService.generateAssessment(payload);
}

module.exports = { generateAssessment };


