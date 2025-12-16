/**
 * MCP HTTP Client for Docusaurus Plugin
 *
 * Communicates with PanaversityFS MCP server via Streamable HTTP transport.
 * Does NOT spawn a server process - expects server to be running externally.
 */

class MCPHttpClient {
  constructor(config = {}) {
    this.serverUrl = config.serverUrl || 'http://localhost:8000/mcp';
    this.bookId = config.bookId || 'ai-native-dev';
    this.apiKey = config.apiKey || null; // API key for authenticated requests
    this.timeoutMs = config.timeoutMs || 120000; // 2 minutes default (book fetch can be slow)
    this.messageId = 0;
  }

  /**
   * Call an MCP tool via HTTP POST
   * @param {string} toolName - Name of the tool to call
   * @param {Object} params - Tool parameters (wrapped in params object)
   * @param {number} timeoutMs - Optional timeout override in milliseconds
   * @returns {Promise<Object>} Tool result
   */
  async callTool(toolName, params = {}, timeoutMs = null) {
    const messageId = ++this.messageId;
    const timeout = timeoutMs || this.timeoutMs;

    const request = {
      jsonrpc: '2.0',
      id: messageId,
      method: 'tools/call',
      params: {
        name: toolName,
        arguments: { params },  // Server expects params wrapper (Pydantic model)
      },
    };

    console.log(`[MCP HTTP] Calling ${toolName} (timeout: ${timeout}ms)...`);

    const headers = {
      'Content-Type': 'application/json',
      Accept: 'application/json',
    };

    // Add Authorization header if API key is configured
    if (this.apiKey) {
      headers['Authorization'] = `Bearer ${this.apiKey}`;
    }

    // Create AbortController for timeout
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), timeout);

    try {
      const response = await fetch(this.serverUrl, {
        method: 'POST',
        headers,
        body: JSON.stringify(request),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP error: ${response.status} ${response.statusText}`);
      }

      const result = await response.json();

      if (result.error) {
        throw new Error(result.error.message || JSON.stringify(result.error));
      }

      // Extract content from MCP response format
      if (result.result?.content?.[0]?.text) {
        return JSON.parse(result.result.content[0].text);
      }

      return result.result;
    } catch (error) {
      clearTimeout(timeoutId);
      if (error.name === 'AbortError') {
        throw new Error(`Tool call '${toolName}' timed out after ${timeout}ms`);
      }
      throw error;
    }
  }

  /**
   * List all books from server
   * @param {Object} options - List options
   * @returns {Promise<Array>} Array of book entries
   */
  async listBooks(options = {}) {
    return this.callTool('list_books', options);
  }

  /**
   * Read all content from a book using scope=book
   * @param {string} bookId - Book identifier
   * @returns {Promise<Array>} Array of content objects with path, content, file_size, etc.
   */
  async readBookContent(bookId) {
    return this.callTool('read_content', {
      book_id: bookId || this.bookId,
      scope: 'book',
    });
  }

  /**
   * Read a single file from the book
   * @param {string} bookId - Book identifier
   * @param {string} path - File path relative to book root
   * @returns {Promise<Object>} Content object with content, metadata, etc.
   */
  async readContent(bookId, path) {
    return this.callTool('read_content', {
      book_id: bookId || this.bookId,
      path: path,
      scope: 'file',
    });
  }

  /**
   * Search for files matching glob pattern
   * @param {string} bookId - Book identifier
   * @param {string} pattern - Glob pattern (e.g., '**\/*.md')
   * @returns {Promise<Array>} Array of matching file paths
   */
  async globSearch(bookId, pattern) {
    return this.callTool('glob_search', {
      book_id: bookId || this.bookId,
      pattern: pattern || '**/*.md',
    });
  }

  /**
   * Get book archive download URL
   * @param {string} bookId - Book identifier
   * @param {string} scope - Archive scope: 'content', 'assets', or 'all'
   * @returns {Promise<Object>} Archive URL and metadata
   */
  async getBookArchive(bookId, scope = 'content') {
    return this.callTool('get_book_archive', {
      book_id: bookId || this.bookId,
      scope: scope,
    });
  }

  /**
   * Check if server is available with retry logic
   * @param {number} retries - Number of retry attempts (default: 3)
   * @param {number} timeoutMs - Timeout per attempt in ms (default: 10000)
   * @param {number} delayMs - Delay between retries in ms (default: 2000)
   * @returns {Promise<boolean>} True if server responds
   */
  async ping(retries = 3, timeoutMs = 10000, delayMs = 2000) {
    for (let attempt = 1; attempt <= retries; attempt++) {
      try {
        // Create AbortController for timeout
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), timeoutMs);

        const pingHeaders = {
          'Content-Type': 'application/json',
          Accept: 'application/json',
        };

        if (this.apiKey) {
          pingHeaders['Authorization'] = `Bearer ${this.apiKey}`;
        }

        const response = await fetch(this.serverUrl, {
          method: 'POST',
          headers: pingHeaders,
          body: JSON.stringify({
            jsonrpc: '2.0',
            id: 0,
            method: 'tools/list',
            params: {},
          }),
          signal: controller.signal,
        });

        clearTimeout(timeout);

        if (response.ok) {
          if (attempt > 1) {
            console.log(`[MCP HTTP] Server responded on attempt ${attempt}/${retries}`);
          }
          return true;
        }

        console.log(`[MCP HTTP] Ping attempt ${attempt}/${retries} failed: HTTP ${response.status}`);
      } catch (error) {
        const errorMsg = error.name === 'AbortError' ? 'timeout' : error.message;
        console.log(`[MCP HTTP] Ping attempt ${attempt}/${retries} failed: ${errorMsg}`);
      }

      // Wait before retry (except on last attempt)
      if (attempt < retries) {
        console.log(`[MCP HTTP] Waiting ${delayMs}ms before retry...`);
        await new Promise(resolve => setTimeout(resolve, delayMs));
      }
    }

    console.error(`[MCP HTTP] Server unavailable after ${retries} attempts`);
    return false;
  }
}

module.exports = MCPHttpClient;
