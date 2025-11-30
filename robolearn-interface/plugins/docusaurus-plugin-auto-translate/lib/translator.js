/**
 * Translator Module
 * 
 * Supports multiple translation providers:
 * - Gemini API (paid/free tier)
 * - LibreTranslate (free, open-source)
 */

const { GoogleGenerativeAI } = require('@google/generative-ai');

/**
 * Initialize Gemini client with configurable temperature
 * Lower temperature (0.1-0.3) = more deterministic, less creative, better for code preservation
 * Higher temperature (0.7-1.0) = more creative, but may introduce errors
 */
function createGeminiClient(apiKey, model = 'gemini-flash-lite-latest', temperature = 0.1) {
  if (!apiKey) {
    throw new Error('GEMINI_API_KEY is required');
  }
  
  const genAI = new GoogleGenerativeAI(apiKey);
  return genAI.getGenerativeModel({ 
    model,
    generationConfig: {
      temperature: temperature, // Low temperature for more deterministic translations
    },
  });
}

/**
 * Create translation prompt that preserves code blocks and technical terms
 */
function createTranslationPrompt(sourceContent, sourceLocale, targetLocale) {
  const localeNames = {
    en: 'English',
    ur: 'Urdu',
  };

  return `Translate the following ${localeNames[sourceLocale]} educational content to ${localeNames[targetLocale]}.

CRITICAL INSTRUCTIONS - DO NOT VIOLATE THESE RULES:
1. Preserve ALL code blocks exactly as-is (do not translate code, comments, code syntax, or any content inside code fences)
2. Preserve ALL MDX/JSX syntax exactly: <Component>, {variable}, import statements, export statements, function calls, etc.
3. Preserve ALL JavaScript/TypeScript syntax: variable names, function names, object properties, array syntax, etc.
4. Keep technical terms in English: ROS, URDF, API, SDK, GPU, CPU, etc. Add Urdu explanation in parentheses if helpful
5. Preserve frontmatter metadata exactly (do not translate YAML frontmatter)
6. Use conversational Urdu (not formal literary style)
7. Maintain markdown formatting (headers, lists, links, etc.)
8. Preserve all special characters and symbols
9. DO NOT translate anything that looks like code, variables, or technical syntax - even if it appears outside code blocks
10. If you see patterns like: <Component prop={value}>, import X from Y, export const Z, function name(), variable names, etc. - preserve them EXACTLY

Content to translate:

${sourceContent}

Translated ${localeNames[targetLocale]} content:`;
}

/**
 * Translate content using LibreTranslate (FREE, open-source)
 * Public instance: https://libretranslate.com (no API key needed)
 */
async function translateWithLibreTranslate(content, sourceLocale, targetLocale, apiUrl = 'https://libretranslate.com') {
  // Map locale codes
  const localeMap = {
    en: 'en',
    ur: 'ur',
  };
  
  const sourceLang = localeMap[sourceLocale] || sourceLocale;
  const targetLang = localeMap[targetLocale] || targetLocale;
  
  try {
    // LibreTranslate has a 5000 character limit, so we need to chunk
    const maxLength = 4000; // Leave buffer for API overhead
    const chunks = [];
    
    if (content.length <= maxLength) {
      chunks.push(content);
    } else {
      // Split by paragraphs to preserve structure
      const paragraphs = content.split(/\n\n+/);
      let currentChunk = '';
      
      for (const para of paragraphs) {
        if ((currentChunk + para).length > maxLength && currentChunk) {
          chunks.push(currentChunk);
          currentChunk = para;
        } else {
          currentChunk += (currentChunk ? '\n\n' : '') + para;
        }
      }
      if (currentChunk) chunks.push(currentChunk);
    }
    
    // Translate each chunk with retry logic and rate limiting
    const translatedChunks = [];
    for (let i = 0; i < chunks.length; i++) {
      const chunk = chunks[i];
      let retries = 3;
      let lastError = null;
      
      while (retries > 0) {
        try {
          const response = await fetch(`${apiUrl}/translate`, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              q: chunk,
              source: sourceLang,
              target: targetLang,
              format: 'text',
            }),
          });
          
          if (response.status === 429) {
            // Rate limited - wait longer and retry
            const waitTime = Math.pow(2, 4 - retries) * 2000; // Exponential backoff: 2s, 4s, 8s
            console.warn(`[LibreTranslate] Rate limited, waiting ${waitTime}ms before retry...`);
            await new Promise(resolve => setTimeout(resolve, waitTime));
            retries--;
            continue;
          }
          
          if (!response.ok) {
            throw new Error(`LibreTranslate API error: ${response.status} ${response.statusText}`);
          }
          
          const data = await response.json();
          translatedChunks.push(data.translatedText);
          
          // Rate limiting: longer delay between requests to avoid 429
          // Public API limit is ~5 requests/minute, so wait 15 seconds between files
          if (i < chunks.length - 1) {
            await new Promise(resolve => setTimeout(resolve, 15000)); // 15 second delay
          }
          
          break; // Success, exit retry loop
        } catch (error) {
          lastError = error;
          retries--;
          if (retries > 0) {
            const waitTime = Math.pow(2, 4 - retries) * 1000;
            await new Promise(resolve => setTimeout(resolve, waitTime));
          }
        }
      }
      
      if (retries === 0 && lastError) {
        throw lastError;
      }
    }
    
    return translatedChunks.join('\n\n');
  } catch (error) {
    throw new Error(`LibreTranslate translation failed: ${error.message}`);
  }
}

/**
 * Translate content using Gemini API
 */
async function translateWithGemini(model, content, sourceLocale, targetLocale) {
  try {
    const prompt = createTranslationPrompt(content, sourceLocale, targetLocale);
    
    const result = await model.generateContent(prompt);
    const response = await result.response;
    const translatedText = response.text();
    
    return translatedText;
  } catch (error) {
    throw new Error(`Gemini translation failed: ${error.message}`);
  }
}

/**
 * Translate content - routes to appropriate provider
 */
async function translateContent(model, content, sourceLocale, targetLocale, apiProvider = 'gemini', libreTranslateUrl) {
  if (apiProvider === 'libretranslate') {
    return await translateWithLibreTranslate(content, sourceLocale, targetLocale, libreTranslateUrl);
  } else {
    return await translateWithGemini(model, content, sourceLocale, targetLocale);
  }
}

/**
 * Chunk content for large files (if needed)
 * Gemini has token limits, so we may need to chunk very large files
 */
function chunkContent(content, maxChunkSize = 50000) {
  if (content.length <= maxChunkSize) {
    return [content];
  }

  // Simple chunking by paragraphs (in production, use smarter markdown-aware chunking)
  const paragraphs = content.split(/\n\n+/);
  const chunks = [];
  let currentChunk = '';

  for (const para of paragraphs) {
    if ((currentChunk + para).length > maxChunkSize && currentChunk) {
      chunks.push(currentChunk);
      currentChunk = para;
    } else {
      currentChunk += (currentChunk ? '\n\n' : '') + para;
    }
  }

  if (currentChunk) {
    chunks.push(currentChunk);
  }

  return chunks;
}

module.exports = {
  createGeminiClient,
  translateContent,
  chunkContent,
};

