const { GoogleGenerativeAI } = require('@google/generative-ai');
const fs = require('fs').promises;
const path = require('path');

class SummaryService {
  constructor() {
    const apiKey = process.env.GEMINI_API_KEY;
    if (!apiKey) {
      throw new Error('GEMINI_API_KEY is not set in environment variables');
    }
    this.genAI = new GoogleGenerativeAI(apiKey);
    this.model = this.genAI.getGenerativeModel({ model: 'gemini-2.0-flash' });
    this.summaryDir = path.join(__dirname, '../../summary');
  }

  async ensureSummaryDirectory() {
    try {
      await fs.mkdir(this.summaryDir, { recursive: true });
    } catch (error) {
      console.error('Error creating summary directory:', error);
    }
  }

  getSummaryFilePath(pagePath, size = 'medium') {
    // Convert page path to a safe filename
    // e.g., "docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/readme.md"
    // becomes "01-Introducing-AI-Driven-Development__01-ai-development-revolution__readme__medium.json"
    const sanitized = pagePath
      .replace(/^docs\//, '') // Remove leading "docs/"
      .replace(/\//g, '__')    // Replace / with __
      .replace(/\.md$/, '')    // Remove .md extension
      + `__${size}.json`;

    return path.join(this.summaryDir, sanitized);
  }

  async getSummary(pagePath, size = 'medium') {
    try {
      await this.ensureSummaryDirectory();
      const filePath = this.getSummaryFilePath(pagePath, size);

      const content = await fs.readFile(filePath, 'utf-8');
      const data = JSON.parse(content);

      return data.summary;
    } catch (error) {
      if (error.code === 'ENOENT') {
        // File doesn't exist, return null
        return null;
      }
      throw error;
    }
  }

  async saveSummary(pagePath, summary, size = 'medium') {
    try {
      await this.ensureSummaryDirectory();
      const filePath = this.getSummaryFilePath(pagePath, size);

      const data = {
        pagePath,
        summary,
        size,
        generatedAt: new Date().toISOString()
      };

      await fs.writeFile(filePath, JSON.stringify(data, null, 2), 'utf-8');

      return true;
    } catch (error) {
      console.error('Error saving summary:', error);
      throw error;
    }
  }

  async readMarkdownFile(pagePath) {
    try {
      // pagePath is like "docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/readme.md"
      const fullPath = path.join(__dirname, '../../', pagePath);
      const content = await fs.readFile(fullPath, 'utf-8');
      return content;
    } catch (error) {
      console.error('Error reading markdown file:', error);
      throw new Error(`Failed to read file: ${error.message}`);
    }
  }

  async generateSummary(pagePath, pageTitle, size = 'medium') {
    try {
      console.log('\n🎯 [Service] generateSummary called with:', {
        pagePath,
        pageTitle,
        size,
      });

      // Map size to exact sentence counts with examples
      const sizeConfig = {
        short: {
          sentences: 2,
          example: 'This technology revolutionizes software development through AI collaboration. It enables faster iteration and better code quality.'
        },
        medium: {
          sentences: 4,
          example: 'This technology revolutionizes software development through AI collaboration. It enables faster iteration and better code quality. Developers write specifications first before generating code. The approach reduces errors and improves maintainability.'
        },
        long: {
          sentences: 6,
          example: 'This technology revolutionizes software development through AI collaboration. It enables faster iteration and better code quality. Developers write specifications first before generating code. The approach reduces errors and improves maintainability. Teams can scale development while maintaining high standards. The methodology has proven effective across various project sizes.'
        }
      };

      const config = sizeConfig[size] || sizeConfig['medium'];

      console.log('⚙️ [Service] Using config:', {
        size,
        targetSentences: config.sentences,
      });

      // Check if summary already exists for this size
      const existingSummary = await this.getSummary(pagePath, size);
      if (existingSummary) {
        console.log(`✅ [Service] Summary already exists for ${pagePath} (${size}), returning cached version`);
        console.log(`📊 [Service] Cached summary has ${this.countSentences(existingSummary)} sentences`);
        return existingSummary;
      }

      console.log('🆕 [Service] No cache found, generating new summary...');

      // Read the raw markdown file from filesystem
      const pageContent = await this.readMarkdownFile(pagePath);

      const prompt = `You are a summarization expert. Your task is to create a summary with EXACTLY ${config.sentences} sentences.

===== CRITICAL RULES (MUST FOLLOW) =====

1. OUTPUT FORMAT:
   - Plain text ONLY
   - NO markdown (#, ##, *, -, etc.)
   - NO bullet points or lists
   - NO tables
   - NO code blocks
   - Just plain sentences with periods

2. SENTENCE COUNT (MOST IMPORTANT):
   - Write EXACTLY ${config.sentences} sentences
   - Each sentence must end with a period (.)
   - Count before submitting: ${Array.from({length: config.sentences}, (_, i) => i + 1).join(', ')}

3. EXAMPLE OF ${config.sentences}-SENTENCE OUTPUT:
   "${config.example}"

4. CONTENT RULES:
   - Focus on key concepts only
   - No meta-commentary
   - Professional tone
   - Direct and clear

===== PAGE CONTENT TO SUMMARIZE =====
${pageContent}

===== YOUR TASK =====
Write exactly ${config.sentences} plain text sentences summarizing the above content.
Do NOT write anything except the ${config.sentences} sentences.

OUTPUT (${config.sentences} sentences):`;

      console.log('🤖 [Service] Calling Gemini AI...');

      const result = await this.model.generateContent(prompt);
      const response = result.response;
      let summary = response.text().trim();

      console.log('📝 [Service] Raw AI response length:', summary.length);
      console.log('📝 [Service] Raw AI response preview:', summary.substring(0, 200) + '...');

      // Strip any markdown that might have slipped through
      summary = this.stripMarkdown(summary);
      console.log('🧹 [Service] After markdown strip length:', summary.length);

      // Enforce exact sentence count
      const beforeEnforce = this.countSentences(summary);
      summary = this.enforceSentenceCount(summary, config.sentences);
      const afterEnforce = this.countSentences(summary);

      console.log(`📊 [Service] Sentence count: ${beforeEnforce} → ${afterEnforce} (target: ${config.sentences})`);
      console.log('✅ [Service] Final summary:', summary);

      // Save the generated summary
      await this.saveSummary(pagePath, summary, size);

      console.log(`✓ Generated and saved ${size} summary (${config.sentences} sentences) for ${pagePath}`);

      return summary;
    } catch (error) {
      console.error('Error generating summary with Gemini:', error);
      throw new Error(`Failed to generate summary: ${error.message}`);
    }
  }

  stripMarkdown(text) {
    let clean = text;

    // Remove headings (# ## ###)
    clean = clean.replace(/^#+\s+/gm, '');

    // Remove bold/italic (** __ * _)
    clean = clean.replace(/(\*\*|__)(.*?)\1/g, '$2');
    clean = clean.replace(/(\*|_)(.*?)\1/g, '$2');

    // Remove bullet points (- * +)
    clean = clean.replace(/^[\s]*[-*+]\s+/gm, '');

    // Remove numbered lists (1. 2. etc)
    clean = clean.replace(/^\d+\.\s+/gm, '');

    // Remove code blocks (``` or `)
    clean = clean.replace(/`{3}[\s\S]*?`{3}/g, '');
    clean = clean.replace(/`([^`]+)`/g, '$1');

    // Remove links [text](url)
    clean = clean.replace(/\[([^\]]+)\]\([^\)]+\)/g, '$1');

    // Remove tables
    clean = clean.replace(/\|.*\|/g, '');
    clean = clean.replace(/^\|.*$/gm, '');

    // Remove multiple newlines
    clean = clean.replace(/\n{2,}/g, ' ');

    // Remove extra whitespace
    clean = clean.replace(/\s+/g, ' ');

    return clean.trim();
  }

  countSentences(text) {
    const sentences = text.match(/[^.!?]+[.!?]+/g) || [];
    return sentences.length;
  }

  enforceSentenceCount(text, targetCount) {
    // Split by sentence-ending punctuation (., !, ?)
    const sentences = text.match(/[^.!?]+[.!?]+/g) || [];

    // Remove empty or whitespace-only sentences
    const validSentences = sentences
      .map(s => s.trim())
      .filter(s => s.length > 0);

    // Truncate to target count if too long
    if (validSentences.length > targetCount) {
      return validSentences.slice(0, targetCount).join(' ');
    }

    // Return as-is if correct count or too short (we trust AI for correct generation)
    return validSentences.join(' ');
  }
}

const summaryService = new SummaryService();

async function generateSummary(pagePath, pageTitle, size = 'medium') {
  return await summaryService.generateSummary(pagePath, pageTitle, size);
}

async function getSummary(pagePath, size = 'medium') {
  return await summaryService.getSummary(pagePath, size);
}

module.exports = { generateSummary, getSummary, SummaryService };
