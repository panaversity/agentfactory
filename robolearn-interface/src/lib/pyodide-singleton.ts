/**
 * Pyodide Singleton
 * Manages a single instance of Pyodide for the entire application
 * Provides methods to run Python code with output capture
 *
 * NOTE: Pyodide is loaded dynamically from CDN to avoid webpack bundling issues.
 * The loadPyodide function is loaded from the CDN, not from npm.
 */

// Use any type to avoid webpack import issues with Pyodide
type PyodideInterface = any;

export class PyodideRunner {
  private static instance: PyodideRunner | null = null;
  private pyodide: PyodideInterface | null = null;
  private initPromise: Promise<void> | null = null;
  private isRunning: boolean = false;
  private loadedPackages: Set<string> = new Set();

  // Map of package names to their Pyodide package names
  private packageMap: Record<string, string> = {
    'requests': 'requests',
    'httpx': 'httpx',
    'pydantic': 'pydantic',
    'numpy': 'numpy',
    'pandas': 'pandas',
    'matplotlib': 'matplotlib',
  };

  /**
   * Private constructor - use getInstance() instead
   */
  private constructor() {}

  /**
   * Get or create the singleton instance
   */
  static getInstance(): PyodideRunner {
    if (!this.instance) {
      this.instance = new PyodideRunner();
    }
    return this.instance;
  }

  /**
   * Initialize Pyodide (lazy loading)
   * Only loads once, subsequent calls return cached promise
   *
   * Loads Pyodide from CDN to avoid webpack bundling issues with the npm package
   */
  async init(): Promise<void> {
    if (!this.initPromise) {
      this.initPromise = (async () => {
        try {
          // Ensure the CDN script is loaded
          await this.loadPyodideCDN();

          // Access loadPyodide from global window object
          const loadPyodide = (window as any).loadPyodide;

          if (!loadPyodide) {
            throw new Error('Failed to load Pyodide from CDN');
          }

          this.pyodide = await loadPyodide({
            indexURL: 'https://cdn.jsdelivr.net/pyodide/v0.29.0/full/'
          });

          // Don't pre-load packages - use lazy loading instead
          // Packages will be loaded on-demand when detected in user code
        } catch (error) {
          this.initPromise = null; // Reset on error so it can be retried
          throw error;
        }
      })();
    }
    return this.initPromise;
  }

  /**
   * Load Pyodide script from CDN if not already loaded
   */
  private loadPyodideCDN(): Promise<void> {
    return new Promise((resolve, reject) => {
      // Check if already loaded
      if ((window as any).loadPyodide) {
        resolve();
        return;
      }

      // Create script tag to load Pyodide from CDN
      const script = document.createElement('script');
      script.src = 'https://cdn.jsdelivr.net/pyodide/v0.29.0/full/pyodide.js';
      // TODO: Verify SRI hash against official Pyodide v0.29.0 documentation
      // Reference: https://pyodide.org/ or https://www.jsdelivr.com/package/npm/pyodide
      script.integrity = 'sha384-l95tshxQlbjf4kdyWZf10uUL5Dw8/iN9q16SQ+ttOEWA8SN0cLG6BGDGY17GxToh';
      script.crossOrigin = 'anonymous';
      script.async = true;
      script.onload = () => resolve();
      script.onerror = () => reject(new Error('Failed to load Pyodide from CDN'));
      document.head.appendChild(script);
    });
  }

  /**
   * Detect required packages from import statements in code
   * @param code - Python code to analyze
   * @returns Array of package names that need to be loaded
   */
  private detectRequiredPackages(code: string): string[] {
    const packages: string[] = [];
    const lines = code.split('\n');

    for (const line of lines) {
      const trimmed = line.trim();

      // Match: import package or from package import ...
      const importMatch = trimmed.match(/^(?:import|from)\s+([a-zA-Z_][a-zA-Z0-9_]*)/);

      if (importMatch) {
        const packageName = importMatch[1];

        // Check if this is a known package we can load
        if (this.packageMap[packageName] && !this.loadedPackages.has(packageName)) {
          packages.push(packageName);
        }
      }
    }

    return [...new Set(packages)]; // Remove duplicates
  }

  /**
   * Load required packages on-demand
   * @param packages - Array of package names to load
   * @param onProgress - Optional callback for loading progress
   */
  private async loadRequiredPackages(
    packages: string[],
    onProgress?: (packageName: string, loaded: number, total: number) => void
  ): Promise<void> {
    if (packages.length === 0) return;

    const packagesToLoad = packages.filter(pkg => !this.loadedPackages.has(pkg));

    if (packagesToLoad.length === 0) return;

    for (let i = 0; i < packagesToLoad.length; i++) {
      const pkg = packagesToLoad[i];
      const pyodidePackage = this.packageMap[pkg];

      if (onProgress) {
        onProgress(pkg, i + 1, packagesToLoad.length);
      }

      await this.pyodide.loadPackage([pyodidePackage]);
      this.loadedPackages.add(pkg);
    }
  }

  /**
   * Run Python code with direct output callbacks
   * @param code - Python code to execute
   * @param onOutput - Callback for stdout (print statements)
   * @param onError - Callback for stderr (errors and tracebacks)
   * @param onPackageLoad - Optional callback for package loading progress
   */
  async run(
    code: string,
    onOutput: (text: string) => void,
    onError: (text: string) => void,
    onPackageLoad?: (packageName: string, loaded: number, total: number) => void
  ): Promise<void> {
    // Prevent concurrent executions
    if (this.isRunning) {
      throw new Error('Another code execution is already in progress. Please wait for it to complete.');
    }

    // Ensure Pyodide is initialized
    await this.init();

    if (!this.pyodide) {
      throw new Error('Pyodide failed to initialize');
    }

    // Mark as running before setting up handlers
    this.isRunning = true;

    try {
      // Detect and load required packages before execution
      const requiredPackages = this.detectRequiredPackages(code);
      if (requiredPackages.length > 0) {
        await this.loadRequiredPackages(requiredPackages, onPackageLoad);
      }

      // Set up stdout/stderr capture with batched callbacks
      // batched: Called when newline appears or on flush
      // This provides real-time output as code executes
      this.pyodide.setStdout({
        batched: (msg: string) => {
          if (msg) {
            onOutput(msg + '\n');
          }
        }
      });

      this.pyodide.setStderr({
        batched: (msg: string) => {
          if (msg) {
            onError(msg + '\n');
          }
        }
      });

      // Override Python's built-in input() to use browser dialog
      // This prevents input prompts from appearing in stdout
      const setupInputCode = `
import builtins
_original_input = builtins.input

def _custom_input(prompt=''):
    # Use browser's prompt dialog to get user input
    # The prompt message is shown in the dialog, not in stdout
    from js import window
    result = window.prompt(prompt)
    if result is None:  # User clicked Cancel
        raise EOFError('User cancelled input')
    return str(result)

builtins.input = _custom_input
`;

      // Run the input override setup first
      await this.pyodide.runPythonAsync(setupInputCode);

      // Use runPythonAsync to support both sync and async code
      const result = await this.pyodide.runPythonAsync(code);

      // If code ends with an expression (not assigned, not print),
      // the result is returned. Display it if not None/null
      if (result !== undefined && result !== null) {
        const resultStr = String(result);
        if (resultStr && resultStr !== 'None') {
          onOutput(resultStr + '\n');
        }
      }
    } catch (error) {
      // Python errors are caught here and sent to onError
      const errorMsg = error instanceof Error ? error.message : String(error);
      onError(errorMsg);
    } finally {
      // Always release the lock, even if an error occurred
      this.isRunning = false;
    }
  }

  /**
   * Check if Pyodide is initialized
   */
  isReady(): boolean {
    return this.pyodide !== null;
  }

  /**
   * Optional: Reset Python state (clear user-defined variables)
   * Keeps built-ins and standard library intact
   */
  reset(): void {
    if (this.pyodide) {
      try {
        this.pyodide.runPython(`
import sys
# Clear all user-defined globals except built-ins and modules
for name in list(globals().keys()):
  if not name.startswith('_') and name not in sys.modules:
    try:
      del globals()[name]
    except:
      pass
        `);
      } catch (error) {
        console.warn('Error resetting Python state:', error);
      }
    }
  }

  /**
   * Optional: Load additional packages (e.g., NumPy, Pandas)
   * Useful for advanced educational content
   */
  async loadPackage(packageNames: string | string[]): Promise<void> {
    await this.init();

    if (!this.pyodide) {
      throw new Error('Pyodide not initialized');
    }

    const packages = Array.isArray(packageNames) ? packageNames : [packageNames];

    try {
      await this.pyodide.loadPackage(packages);
    } catch (error) {
      throw new Error(`Failed to load package(s) ${packages.join(', ')}: ${error}`);
    }
  }
}
