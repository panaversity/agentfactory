const { GoogleGenerativeAI } = require('@google/generative-ai');
const { BrowserSearch } = require('../utils/browserSearch');

class AnswerAgent {
  constructor() {
    const apiKey = process.env.GEMINI_API_KEY;
    if (!apiKey) {
      throw new Error('GEMINI_API_KEY is not set in environment variables');
    }
    this.genAI = new GoogleGenerativeAI(apiKey);
    this.model = this.genAI.getGenerativeModel({ model: 'gemini-2.0-flash' });
    this.browserSearch = new BrowserSearch();
  }

  async generateAnswer({ query, structuredQuery, toneResult, projectContext, conversationHistory, isSummary = false }) {
    try {
      let contextToUse = projectContext.content || '';
      let usedBrowserSearch = false;
      let externalInfo = '';

      // If out-of-tone and not a summary, perform browser search
      if (!toneResult.isInTone && toneResult.confidence < 0.5 && !isSummary) {
        console.log('Query is out-of-tone, performing browser search...');
        externalInfo = await this.browserSearch.search(query, {
          projectDomain: 'AI Native Software Development',
          projectTopics: ['AI-driven development', 'Python', 'TypeScript', 'Agentic AI', 'Spec-driven development']
        });
        usedBrowserSearch = true;
      }

      // Build conversation history context (skip for summaries to keep focused)
      const historyContext = !isSummary
        ? conversationHistory
            .slice(-5) // Last 5 messages
            .map(msg => `${msg.isBot ? 'Assistant' : 'User'}: ${msg.text}`)
            .join('\n')
        : '';

      // Use specialized prompt for summaries
      const systemPrompt = isSummary
        ? this.buildSummaryPrompt(query)
        : `You are an intelligent AI assistant for the "AI Native Software Development" book and documentation platform.

PROJECT CONTEXT:
This is a comprehensive book and learning platform about:
- AI-Driven Development (AIDD) and AI-Native Development
- Python programming for AI agents
- TypeScript for realtime and interaction layers
- Spec-Driven Development methodology
- OpenAI Agents SDK, Google Gemini, MCP (Model Context Protocol)
- Building agentic AI systems, realtime agents, voice agents
- Containerization, orchestration, event-driven architectures

Your role is to:
1. Answer questions based on the project's codebase, documentation, and content
2. Provide accurate, helpful, and contextually relevant answers
3. When information is not in the project, use external knowledge but relate it back to the project's domain
4. Be conversational, clear, and educational

${toneResult.isInTone 
  ? 'This query is RELATED to the project context. Use the provided project context to answer.'
  : 'This query is LESS RELATED to the project. Use external information but connect it back to AI Native Software Development concepts when possible.'
}

PROJECT CONTEXT DATA:
${contextToUse || 'No specific project context found for this query.'}

${externalInfo ? `\nEXTERNAL INFORMATION (for context):\n${externalInfo}\n\nRemember to relate this back to the project domain when relevant.` : ''}

${historyContext ? `\nCONVERSATION HISTORY:\n${historyContext}\n` : ''}

STRUCTURED QUERY ANALYSIS:
- Intent: ${structuredQuery.intent}
- Topics: ${structuredQuery.topics.join(', ') || 'General'}
- Complexity: ${structuredQuery.complexity}
- Expected Response: ${structuredQuery.expectedResponseType}

Now, provide a helpful, accurate answer to the user's query: "${query}"

Format your response naturally and conversationally. If you reference external information, explain how it relates to AI Native Software Development.`;

      const result = await this.model.generateContent(systemPrompt);
      const response = await result.response;
      const answerText = response.text();

      // Extract sources if mentioned
      const sources = this.extractSources(answerText, projectContext);

      return {
        text: answerText,
        sources,
        usedBrowserSearch
      };
    } catch (error) {
      console.error('Error in AnswerAgent:', error);
      throw new Error(`Failed to generate answer: ${error.message}`);
    }
  }

  extractSources(text, projectContext) {
    const sources = [];
    
    // Check if project context has file references
    if (projectContext.files && projectContext.files.length > 0) {
      projectContext.files.slice(0, 3).forEach(file => {
        if (file.path) {
          sources.push({
            type: 'project_file',
            path: file.path,
            title: file.title || file.path
          });
        }
      });
    }

    return sources;
  }

  buildSummaryPrompt(query) {
    // Extract the summary request details from the query
    const summaryStyleMatch = query.match(/Provide a (concise|comprehensive|detailed) ([\w\s-]+) summary/i);
    const style = summaryStyleMatch ? summaryStyleMatch[1].toLowerCase() : 'medium';

    // Extract selected text
    const textMatch = query.match(/Selected Text:\s*"""\s*([\s\S]+?)\s*"""/);
    const selectedText = textMatch ? textMatch[1].trim() : '';

    // Extract context metadata
    const contextMatch = query.match(/Context: This text is from "([^"]+)" \(([^)]+)\), section: "([^"]+)"/);
    const pageTitle = contextMatch ? contextMatch[1] : 'Unknown Page';
    const pageUrl = contextMatch ? contextMatch[2] : '';
    const section = contextMatch ? contextMatch[3] : 'Unknown Section';

    // Build style-specific instructions
    const styleInstructions = {
      concise: 'Provide a concise 2-3 sentence summary that captures the key takeaway. Be direct and clear.',
      comprehensive: 'Provide a comprehensive paragraph summary covering all main points. Explain the concepts thoroughly while staying focused.',
      detailed: 'Provide a detailed summary with key points listed as bullets. Cover all important concepts, examples, and implications.'
    };

    const instruction = styleInstructions[style] || styleInstructions.comprehensive;

    return `You are an expert AI summarizer for educational content.

TASK: Summarize the following text from the "AI Native Software Development" book.

SOURCE CONTEXT:
- Page: ${pageTitle}
- URL: ${pageUrl}
- Section: ${section}

SUMMARY STYLE: ${style.toUpperCase()}
${instruction}

TEXT TO SUMMARIZE:
"""
${selectedText}
"""

INSTRUCTIONS:
1. Focus on the core concepts and key information
2. Maintain technical accuracy
3. Keep the summary style consistent with the requested format
4. Preserve important technical terms and examples
5. Make it clear and easy to understand
6. Do not add information not present in the original text
7. Do not include phrases like "this text discusses" or "the passage explains" - just provide the summary directly

Now provide the summary:`;
  }
}

module.exports = { AnswerAgent };

