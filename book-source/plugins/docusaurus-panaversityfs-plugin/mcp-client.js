// MCP Client for Docusaurus Plugin
// Spawns PanaversityFS MCP server and communicates via stdio

const { spawn } = require('child_process');
const readline = require('readline');

class MCPClient {
  constructor(config = {}) {
    this.serverCommand = config.serverCommand || 'uv';
    this.serverArgs = config.serverArgs || ['run', 'python', '-m', 'panaversity_fs.server'];
    this.serverPath = config.serverPath || '../../panaversity-fs';
    this.env = {
      ...process.env,
      PANAVERSITY_STORAGE_BACKEND: config.storageBackend || 'fs',
      PANAVERSITY_STORAGE_ROOT: config.storageRoot || '/tmp/panaversity-test',
    };

    this.serverProcess = null;
    this.messageId = 0;
    this.pendingRequests = new Map();
  }

  // Start the MCP server process
  async start() {
    console.log('[MCP Client] Starting PanaversityFS server...');
    console.log('[MCP Client] Command:', this.serverCommand, this.serverArgs.join(' '));
    console.log('[MCP Client] Storage:', this.env.PANAVERSITY_STORAGE_BACKEND);

    return new Promise((resolve, reject) => {
      this.serverProcess = spawn(this.serverCommand, this.serverArgs, {
        cwd: this.serverPath,
        env: this.env,
        stdio: ['pipe', 'pipe', 'pipe'],
      });

      const rl = readline.createInterface({
        input: this.serverProcess.stdout,
        crlfDelay: Infinity,
      });

      rl.on('line', (line) => {
        try {
          const message = JSON.parse(line);
          this._handleMessage(message);
        } catch (error) {
          console.error('[MCP Client] Failed to parse message:', line);
        }
      });

      this.serverProcess.stderr.on('data', (data) => {
        const message = data.toString();
        if (message.includes('Server started')) {
          console.log('[MCP Client] Server ready');
          resolve();
        }
        if (!message.includes('INFO') && !message.includes('DEBUG')) {
          console.error('[MCP Client] Server stderr:', message);
        }
      });

      this.serverProcess.on('error', (error) => {
        console.error('[MCP Client] Server error:', error);
        reject(error);
      });

      this.serverProcess.on('exit', (code) => {
        console.log('[MCP Client] Server exited with code', code);
        if (code !== 0 && code !== null) {
          reject(new Error('Server exited with code ' + code));
        }
      });

      setTimeout(() => {
        if (!this.serverProcess.killed) {
          console.log('[MCP Client] Server started (timeout reached)');
          resolve();
        }
      }, 5000);
    });
  }

  // Stop the MCP server process
  async stop() {
    if (this.serverProcess && !this.serverProcess.killed) {
      console.log('[MCP Client] Stopping server...');
      this.serverProcess.kill();
      return new Promise((resolve) => {
        this.serverProcess.on('exit', () => {
          console.log('[MCP Client] Server stopped');
          resolve();
        });
      });
    }
  }

  // Call an MCP tool
  async callTool(toolName, params = {}) {
    const messageId = ++this.messageId;

    const request = {
      jsonrpc: '2.0',
      id: messageId,
      method: 'tools/call',
      params: {
        name: toolName,
        arguments: params,
      },
    };

    console.log('[MCP Client] Calling tool:', toolName);

    return new Promise((resolve, reject) => {
      this.pendingRequests.set(messageId, { resolve, reject });
      const requestLine = JSON.stringify(request) + '\n';
      this.serverProcess.stdin.write(requestLine);

      setTimeout(() => {
        if (this.pendingRequests.has(messageId)) {
          this.pendingRequests.delete(messageId);
          reject(new Error('Tool call timeout: ' + toolName));
        }
      }, 30000);
    });
  }

  // Handle incoming messages from server
  _handleMessage(message) {
    if (message.id && this.pendingRequests.has(message.id)) {
      const { resolve, reject } = this.pendingRequests.get(message.id);
      this.pendingRequests.delete(message.id);

      if (message.error) {
        reject(new Error(message.error.message || 'Tool call failed'));
      } else {
        resolve(message.result);
      }
    }
  }

  // Search for content files
  async globSearch(bookId, pattern) {
    pattern = pattern || '**/*.md';
    const result = await this.callTool('glob_search', {
      book_id: bookId,
      pattern: pattern,
    });
    return result.content[0].text.split('\n').filter(Boolean);
  }

  // Read content from a file
  async readContent(bookId, path) {
    const result = await this.callTool('read_content', {
      book_id: bookId,
      path: path,
    });
    const contentData = JSON.parse(result.content[0].text);
    return contentData;
  }

  // Read a chapter summary
  async readSummary(bookId, chapterId) {
    const result = await this.callTool('read_summary', {
      book_id: bookId,
      chapter_id: chapterId,
    });
    const summary = JSON.parse(result.content[0].text);
    return summary;
  }

  // Write a chapter summary
  async writeSummary(bookId, chapterId, content) {
    const result = await this.callTool('write_summary', {
      book_id: bookId,
      chapter_id: chapterId,
      content: content,
    });
    return JSON.parse(result.content[0].text);
  }

  // Delete a chapter summary
  async deleteSummary(bookId, chapterId) {
    const result = await this.callTool('delete_summary', {
      book_id: bookId,
      chapter_id: chapterId,
    });
    return JSON.parse(result.content[0].text);
  }
}

module.exports = MCPClient;
