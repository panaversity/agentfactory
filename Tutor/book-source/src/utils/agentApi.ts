/**
 * Agent API Client
 *
 * Handles communication with the TutorGPT backend API
 * Supports RAG search and agent chat interactions
 */

export interface AgentAction {
  action: 'summary' | 'explain' | 'main_points' | 'example' | 'ask';
  text: string;
  cursorContext?: string;
  userId?: string;
  uiHints?: {
    tone: 'student-friendly' | 'technical';
    length: 'short' | 'medium' | 'long';
  };
}

export interface AgentResponse {
  success: boolean;
  message: string;
  preview?: string; // Short preview for inline display
  fullResponse?: string; // Full response for chat window
  error?: string;
}

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: number;
  action?: AgentAction['action'];
}

export interface CoLearningAction {
  action: 'lesson_step' | 'explain' | 'summary' | 'quiz_prepare' | 'quiz_grade' | 'task' | 'greeting';
  chapter: number;
  section?: string;
  text?: string;
  language?: 'en' | 'roman_ur' | 'es';
  userId: string;
  uiHints?: {
    tone: string;
    length: string;
  };
  studentAnswer?: string;
  quizAnswers?: any[];
}

export interface QuizQuestion {
  id: string;
  question: string;
  type: 'multiple_choice' | 'short_answer' | 'true_false';
  options?: string[];
  correctAnswer?: string | number;
  explanation?: string;
}

export interface QuizResult {
  score: number;
  totalQuestions: number;
  percentage: number;
  answers: {
    questionId: string;
    userAnswer: string;
    correct: boolean;
    feedback: string;
  }[];
  weakTopics?: string[];
  needsRemedial: boolean;
}

// Configuration
// Note: To change API URL, edit this value directly or use browser localStorage
const getApiBaseUrl = () => {
  if (typeof window !== 'undefined') {
    return localStorage.getItem('tutorgpt_api_url') || 'http://localhost:8000';
  }
  return 'http://localhost:8000';
};
const API_BASE_URL = getApiBaseUrl();

class AgentApiClient {
  private baseUrl: string;
  private userId: string;

  constructor() {
    this.baseUrl = API_BASE_URL;
    this.userId = this.getUserId();
  }

  /**
   * Get or create user ID from localStorage
   */
  private getUserId(): string {
    // SSR guard
    if (typeof window === 'undefined') {
      return 'ssr-user-temp';
    }

    let userId = localStorage.getItem('tutorgpt_user_id');
    if (!userId) {
      userId = `student-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('tutorgpt_user_id', userId);
    }
    return userId;
  }

  /**
   * Send an action to the agent
   */
  async sendAction(actionData: AgentAction): Promise<AgentResponse> {
    try {
      const payload = {
        ...actionData,
        userId: this.userId,
        uiHints: actionData.uiHints || {
          tone: 'student-friendly',
          length: 'short'
        }
      };

      const response = await fetch(`${this.baseUrl}/api/agent/action`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      return {
        success: true,
        message: data.message || data.response || '',
        preview: data.preview || this.generatePreview(data.message || data.response || ''),
        fullResponse: data.message || data.response || '',
      };
    } catch (error) {
      console.error('Error sending action to agent:', error);
      return {
        success: false,
        message: '',
        error: error instanceof Error ? error.message : 'Unknown error occurred',
      };
    }
  }

  /**
   * Send a chat message to the agent
   */
  async sendChatMessage(message: string, context?: string): Promise<AgentResponse> {
    return this.sendAction({
      action: 'ask',
      text: message,
      cursorContext: context,
    });
  }

  /**
   * Search the book content using RAG
   */
  async searchBook(query: string, scope: 'current_lesson' | 'current_chapter' | 'entire_book' = 'current_chapter'): Promise<any> {
    try {
      const response = await fetch(`${this.baseUrl}/api/rag/search`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          scope,
          top_k: 5,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error searching book:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred',
      };
    }
  }

  /**
   * Generate a short preview from a longer response
   */
  private generatePreview(text: string, maxLength: number = 100): string {
    if (text.length <= maxLength) return text;
    return text.substring(0, maxLength).trim() + '...';
  }

  /**
   * Get health status of the API
   */
  async getHealth(): Promise<{ status: string; details?: any }> {
    try {
      const response = await fetch(`${this.baseUrl}/api/rag/health`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      return await response.json();
    } catch (error) {
      console.error('Error checking API health:', error);
      return {
        status: 'error',
        details: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * Send co-learning action (lesson step, quiz, etc.)
   */
  async sendCoLearningAction(actionData: CoLearningAction): Promise<AgentResponse> {
    try {
      const payload = {
        ...actionData,
        userId: actionData.userId || this.userId,
        uiHints: actionData.uiHints || {
          tone: 'professional+funny',
          length: 'short'
        }
      };

      const response = await fetch(`${this.baseUrl}/api/colearn/action`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      return {
        success: true,
        message: data.message || data.response || '',
        preview: data.preview,
        fullResponse: data.message || data.response || '',
      };
    } catch (error) {
      console.error('Error sending co-learning action:', error);
      return {
        success: false,
        message: '',
        error: error instanceof Error ? error.message : 'Unknown error occurred',
      };
    }
  }

  /**
   * Get chapter content from live website or backend
   */
  async getChapterContent(chapterNumber: number): Promise<any> {
    try {
      const response = await fetch(`${this.baseUrl}/api/colearn/chapter/${chapterNumber}`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      return await response.json();
    } catch (error) {
      console.error('Error fetching chapter content:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred',
      };
    }
  }

  /**
   * Prepare quiz for a chapter
   */
  async prepareQuiz(chapterNumber: number, language: string = 'en'): Promise<QuizQuestion[]> {
    try {
      const response = await fetch(`${this.baseUrl}/api/colearn/quiz/prepare`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter: chapterNumber,
          language,
          questionCount: 10
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data.questions || [];
    } catch (error) {
      console.error('Error preparing quiz:', error);
      return [];
    }
  }

  /**
   * Grade quiz answers
   */
  async gradeQuiz(chapterNumber: number, answers: any[]): Promise<QuizResult> {
    try {
      const response = await fetch(`${this.baseUrl}/api/colearn/quiz/grade`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter: chapterNumber,
          answers,
          userId: this.userId
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error grading quiz:', error);
      return {
        score: 0,
        totalQuestions: answers.length,
        percentage: 0,
        answers: [],
        needsRemedial: true
      };
    }
  }
}

// Export singleton instance
export const agentApi = new AgentApiClient();

// Mock responses for development (when backend is not available)
// To disable mock mode, set in browser console: localStorage.setItem('tutorgpt_use_mock', 'false')
// Default: false (use real backend, fall back to mock if unavailable)
export const useMockResponses = typeof window !== 'undefined'
  ? localStorage.getItem('tutorgpt_use_mock') === 'true' // Changed to default false
  : false;

export const mockAgentResponse = async (action: AgentAction): Promise<AgentResponse> => {
  // Simulate network delay
  await new Promise(resolve => setTimeout(resolve, 500 + Math.random() * 1000));

  const responses: Record<AgentAction['action'], string> = {
    summary: `**Summary:** ${action.text.substring(0, 50)}...\n\nThis section explains the key concepts with practical examples.`,
    explain: `Let me explain this in simple terms:\n\n${action.text}\n\nThink of it like this: it's similar to how you organize your thoughts when writing an essay. Each part has a specific purpose and they work together to achieve the goal.`,
    main_points: `**Main Points:**\n\n1. First key concept from the selected text\n2. Second important idea to remember\n3. Third practical application\n\nThese are the core takeaways you should focus on!`,
    example: `Here's a practical example:\n\n\`\`\`python\n# Example code demonstrating the concept\ndef example_function():\n    print("This shows how the concept works")\n\`\`\`\n\nThis illustrates the idea by showing a real-world use case.`,
    ask: `Great question! Based on the context:\n\n"${action.text}"\n\nThe answer is that this concept is fundamental to understanding how modern AI-driven development works. It builds on the principles we discussed earlier and sets the foundation for what comes next.`,
  };

  return {
    success: true,
    message: responses[action.action],
    preview: responses[action.action].substring(0, 100) + '...',
    fullResponse: responses[action.action],
  };
};

/**
 * Mock co-learning responses for development
 */
export const mockCoLearningResponse = async (action: CoLearningAction): Promise<AgentResponse> => {
  // Simulate network delay
  await new Promise(resolve => setTimeout(resolve, 800 + Math.random() * 1200));

  const language = action.language || 'en';

  const greetings = {
    en: `Hello! 👋 I'm your AI Tutor for this AI-Native Development course.\n\nI'm here to guide you through the entire book step-by-step. Would you like to:\n\n1. **Start from Chapter 1** (recommended for beginners)\n2. **Jump to a specific chapter** (if you have some background)\n\nAlso, which language do you prefer?\n- 🇬🇧 English\n- 🇵🇰 Roman Urdu  \n- 🇪🇸 Spanish\n\nLet me know and we'll get started!`,
    roman_ur: `Hello! 👋 Main aapka AI Tutor hoon is AI-Native Development course ke liye.\n\nMain aapko poori book step-by-step parhaonga. Aap kya chahte hain:\n\n1. **Chapter 1 se shuru karein** (beginners ke liye best)\n2. **Kisi specific chapter pe jayen**\n\nAur aap konsi language prefer karte hain?\n- 🇬🇧 English\n- 🇵🇰 Roman Urdu  \n- 🇪🇸 Spanish\n\nBatayen aur shuru karte hain!`,
    es: `¡Hola! 👋 Soy tu tutor de IA para este curso de desarrollo nativo de IA.\n\nEstoy aquí para guiarte paso a paso. ¿Te gustaría:\n\n1. **Comenzar desde el Capítulo 1** (recomendado para principiantes)\n2. **Ir a un capítulo específico**\n\n¿Qué idioma prefieres?\n- 🇬🇧 Inglés\n- 🇵🇰 Urdu Romano\n- 🇪🇸 Español\n\n¡Dime y empecemos!`,
  };

  const lessonSteps = {
    en: `📚 **Chapter ${action.chapter}: Introduction to AI-Native Development**\n\n**Quick Note:** AI is transforming how we build software.\n\n**Explanation:** Think of AI-native development as building applications where AI isn't just a feature—it's woven into the core architecture. Just like mobile-first changed web development, AI-native changes everything about how we design, build, and deploy software.\n\n**Quick Check:** Can you think of one way AI might improve your current development workflow? (Just answer in one sentence!)`,
    roman_ur: `📚 **Chapter ${action.chapter}: AI-Native Development ka Introduction**\n\n**Quick Note:** AI software development ko transform kar raha hai.\n\n**Explanation:** AI-native development ka matlab hai ke hum aise applications banate hain jahan AI sirf ek feature nahi hai—balke core architecture mein integrated hai. Jaise mobile-first ne web development ko badla, waise hi AI-native sab kuch badal deta hai.\n\n**Quick Check:** Ek tareeka batayein ke AI aapke current development workflow ko kaise improve kar sakta hai? (Sirf ek sentence mein!)`,
    es: `📚 **Capítulo ${action.chapter}: Introducción al Desarrollo Nativo de IA**\n\n**Nota Rápida:** La IA está transformando cómo construimos software.\n\n**Explicación:** Piensa en el desarrollo nativo de IA como construir aplicaciones donde la IA no es solo una característica, sino que está tejida en la arquitectura central.\n\n**Verificación Rápida:** ¿Puedes pensar en una forma en que la IA podría mejorar tu flujo de trabajo actual? (¡Solo responde en una oración!)`,
  };

  const responses: Record<CoLearningAction['action'], string> = {
    greeting: greetings[language] || greetings.en,
    lesson_step: lessonSteps[language] || lessonSteps.en,
    explain: `Great question! Let me break this down in simpler terms...\n\n${action.text || 'This concept'} is like building with LEGO blocks. Each piece has a specific purpose, and when you put them together correctly, you create something amazing!\n\nDoes this make sense? Feel free to ask if you need more clarification! 😊`,
    summary: `📝 **Summary of this section:**\n\n• Main concept: ${action.text?.substring(0, 30) || 'AI development patterns'}\n• Key takeaway: Understanding these fundamentals is crucial\n• Next step: We'll apply this in practical examples\n\nReady to move forward?`,
    quiz_prepare: `📝 **Chapter ${action.chapter} Quiz Time!**\n\nGreat work completing the chapter! Let's test your understanding with 10 questions.\n\nDon't worry—this helps identify what you've mastered and what might need a quick review. Ready? Click "Start Quiz" when you're ready!`,
    quiz_grade: `🎉 **Quiz Results:**\n\nYou scored 7/10 (70%)—Good job!\n\n**What you nailed:** \n✅ Core concepts\n✅ Practical applications\n\n**Areas to review:**\n📌 Advanced patterns (Questions 3, 8, 10)\n\nWould you like a quick review of those topics, or shall we move to the next chapter?`,
    task: `💡 **Practice Task:**\n\nNow let's apply what you learned! Here's a small task:\n\n\`\`\`\nTask: Write a simple function that demonstrates the concept we just learned.\nTime: 5-10 minutes\n\`\`\`\n\nWhen you're done, paste your code and I'll give you feedback! Or if you get stuck, just ask for hints. 🚀`,
  };

  return {
    success: true,
    message: responses[action.action] || 'I didn\'t quite understand that. Can you rephrase?',
    preview: responses[action.action]?.substring(0, 100) + '...' || '',
    fullResponse: responses[action.action] || '',
  };
};

/**
 * Mock quiz generation
 */
export const mockGenerateQuiz = async (chapterNumber: number, language: string = 'en'): Promise<QuizQuestion[]> => {
  await new Promise(resolve => setTimeout(resolve, 1000));

  const quizzes: QuizQuestion[] = [
    {
      id: 'q1',
      question: 'What is the main benefit of AI-native development?',
      type: 'multiple_choice',
      options: [
        'Faster coding',
        'AI integrated into core architecture',
        'Less bugs',
        'Cheaper hosting'
      ],
      correctAnswer: 1,
      explanation: 'AI-native development means AI is woven into the core architecture, not just added as a feature.'
    },
    {
      id: 'q2',
      question: 'True or False: AI can replace all human developers.',
      type: 'true_false',
      options: ['True', 'False'],
      correctAnswer: 1,
      explanation: 'False. AI augments developers but cannot replace human creativity, judgment, and problem-solving.'
    },
    {
      id: 'q3',
      question: 'Name one key pillar of AI-driven development.',
      type: 'short_answer',
      correctAnswer: 'prompt engineering',
      explanation: 'Key pillars include: Prompt Engineering, RAG, Agent Systems, and more.'
    },
    {
      id: 'q4',
      question: 'What does RAG stand for?',
      type: 'multiple_choice',
      options: [
        'Rapid Application Generation',
        'Retrieval Augmented Generation',
        'Random Access Gateway',
        'Real Agent Generator'
      ],
      correctAnswer: 1,
      explanation: 'RAG stands for Retrieval Augmented Generation—a technique to enhance AI with external knowledge.'
    },
    {
      id: 'q5',
      question: 'AI agents can work autonomously without any human guidance.',
      type: 'true_false',
      options: ['True', 'False'],
      correctAnswer: 1,
      explanation: 'False. AI agents need proper prompts, guardrails, and human oversight.'
    },
    {
      id: 'q6',
      question: 'Which is NOT a component of an AI agent system?',
      type: 'multiple_choice',
      options: [
        'Memory',
        'Tools',
        'Quantum processor',
        'Planning'
      ],
      correctAnswer: 2,
      explanation: 'AI agents typically have memory, tools, and planning capabilities. Quantum processors are not required.'
    },
    {
      id: 'q7',
      question: 'What is the purpose of vector databases in RAG?',
      type: 'short_answer',
      correctAnswer: 'store embeddings',
      explanation: 'Vector databases store embeddings for semantic search and retrieval.'
    },
    {
      id: 'q8',
      question: 'Prompt engineering is only about writing good questions.',
      type: 'true_false',
      options: ['True', 'False'],
      correctAnswer: 1,
      explanation: 'False. Prompt engineering involves crafting effective instructions, context, examples, and constraints.'
    },
    {
      id: 'q9',
      question: 'Which language model architecture is most common today?',
      type: 'multiple_choice',
      options: [
        'RNN',
        'CNN',
        'Transformer',
        'Decision Tree'
      ],
      correctAnswer: 2,
      explanation: 'Transformers are the dominant architecture for modern language models like GPT and Claude.'
    },
    {
      id: 'q10',
      question: 'What is one risk of using AI in production?',
      type: 'short_answer',
      correctAnswer: 'hallucination',
      explanation: 'Key risks include hallucinations, bias, security issues, and unpredictable behavior.'
    }
  ];

  return quizzes;
};
