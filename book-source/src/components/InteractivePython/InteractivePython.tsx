/**
 * InteractivePython Component
 *
 * Provides interactive Python code execution in the browser using Pyodide.
 * Allows students to write and run Python code directly in their browser,
 * making it perfect for educational content.
 *
 * Features:
 * - Live code editing with syntax highlighting (Monaco Editor)
 * - Execute Python code directly in the browser
 * - Display output and errors in real-time
 * - Loading state management while Pyodide initializes
 * - Icon-based controls (refresh, copy, play)
 * - Full CPython support with standard library
 *
 * Usage:
 * <InteractivePython
 *   initialCode="print('Hello, World!')"
 * />
 */

import React, { useState, useRef, useEffect, useCallback, useMemo } from 'react';
import Editor from '@monaco-editor/react';
import { RotateCcw, Copy, Zap, Check } from 'lucide-react';
import { usePyodide } from '@/contexts/PyodideContext';
import { ErrorDisplay } from './ErrorDisplay';
import styles from './styles.module.css';

export interface InteractivePythonProps {
  initialCode?: string;
  showLineNumbers?: boolean;
}

export const InteractivePython: React.FC<InteractivePythonProps> = ({
  initialCode = 'print("Hello, World!")',
  showLineNumbers = true,
}) => {
  const [code, setCode] = useState(initialCode);
  const [output, setOutput] = useState<string>('');
  const [error, setError] = useState<string>('');
  const [isRunning, setIsRunning] = useState(false);
  const [codeCopied, setCodeCopied] = useState(false);
  const [outputCopied, setOutputCopied] = useState(false);
  const [loadingPackage, setLoadingPackage] = useState<string | null>(null);
  const [packageProgress, setPackageProgress] = useState<{ loaded: number; total: number } | null>(null);
  const { pyodide, isLoading } = usePyodide();
  const outputRef = useRef<HTMLDivElement>(null);
  
  // Refs to track current state for keyboard shortcut (avoid stale closure)
  const isLoadingRef = useRef(isLoading);
  const isRunningRef = useRef(isRunning);
  
  // Keep refs in sync with state
  useEffect(() => {
    isLoadingRef.current = isLoading;
  }, [isLoading]);
  
  useEffect(() => {
    isRunningRef.current = isRunning;
  }, [isRunning]);

  // Calculate dynamic editor height based on code lines
  const editorHeight = useMemo(() => {
    const lineCount = code.split('\n').length;
    const lineHeight = 19; // Monaco default line height in pixels
    const padding = 32; // Top + bottom padding
    const minHeight = 51;
    const maxHeight = 400;
    return Math.min(
      Math.max(lineCount * lineHeight + padding, minHeight),
      maxHeight
    );
  }, [code]);

  // Auto-scroll output to bottom when new content is added
  useEffect(() => {
    if (outputRef.current) {
      outputRef.current.scrollTop = outputRef.current.scrollHeight;
    }
  }, [output, error]);

  const handleRun = useCallback(async () => {
    setIsRunning(true);
    setOutput('');
    setError('');
    setLoadingPackage(null);
    setPackageProgress(null);

    try {
      // Run Python code with direct output callbacks and package loading progress
      await pyodide.run(
        code,
        (text) => setOutput(prev => prev + text),  // stdout callback
        (text) => setError(prev => prev + text),   // stderr callback
        (packageName, loaded, total) => {          // package loading callback
          setLoadingPackage(packageName);
          setPackageProgress({ loaded, total });
        }
      );
    } catch (err) {
      // Preserve any stderr output that was already captured
      setError(prev => prev + `Error: ${err instanceof Error ? err.message : String(err)}`);
    } finally {
      setIsRunning(false);
      setLoadingPackage(null);
      setPackageProgress(null);
    }
  }, [code, pyodide]);

  // Keep a ref to the latest handleRun so keyboard shortcuts always run fresh code
  const handleRunRef = useRef(handleRun);
  useEffect(() => {
    handleRunRef.current = handleRun;
  }, [handleRun]);

  const handleReset = () => {
    setCode(initialCode);
    setOutput('');
    setError('');
  };

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(code);
      setCodeCopied(true);
      setTimeout(() => setCodeCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy code:', err);
    }
  };

  const handleClearOutput = () => {
    setOutput('');
    setError('');
  };

  const handleCopyOutput = async () => {
    const outputText = error || output;
    try {
      await navigator.clipboard.writeText(outputText);
      setOutputCopied(true);
      setTimeout(() => setOutputCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy output:', err);
    }
  };

  return (
    <div className={styles.container}>
      {/* Header with title and icon buttons */}
      <div className={styles.header}>
        {/* macOS-style window controls */}
        <div className={styles.windowControls}>
          <div className={`${styles.windowControl} ${styles.windowControlRed}`} title="Close" />
          <div className={`${styles.windowControl} ${styles.windowControlYellow}`} title="Minimize" />
          <div className={`${styles.windowControl} ${styles.windowControlGreen}`} title="Maximize" />
        </div>
        <div className={styles.buttonGroup}>
          <button
            className={styles.iconButton}
            onClick={handleReset}
            disabled={isLoading || isRunning}
            title="Reset code to initial state"
            aria-label="Reset code"
          >
            <RotateCcw size={16} />
          </button>
          <button
            className={`${styles.iconButton} ${codeCopied ? styles.copied : ''}`}
            onClick={handleCopy}
            disabled={isLoading || isRunning}
            title={codeCopied ? "Copied!" : "Copy code to clipboard"}
            aria-label="Copy code"
          >
            {codeCopied ? <Check size={16} /> : <Copy size={16} />}
          </button>
          <button
            className={styles.runButton}
            onClick={handleRun}
            disabled={isLoading || isRunning}
            title="Run code (or press Shift+Enter)"
            aria-label="Run code"
          >
            <Zap size={16} />
            <span>Run</span>
          </button>
        </div>
      </div>

      {/* Monaco Editor with syntax highlighting */}
      <div className={styles.editorWrapper}>
        {isLoading && (
          <div className={styles.loadingOverlay}>
            <div className={styles.spinner} />
            <p>Loading Python environment...</p>
          </div>
        )}
        {loadingPackage && packageProgress && (
          <div className={styles.packageLoadingOverlay}>
            <div className={styles.packageLoadingContent}>
              <div className={styles.packageSpinner} />
              <p className={styles.packageLoadingText}>
                Loading <strong>{loadingPackage}</strong>...
              </p>
              <p className={styles.packageLoadingProgress}>
                ({packageProgress.loaded} of {packageProgress.total} packages)
              </p>
            </div>
          </div>
        )}
        <Editor
          height={`${editorHeight}px`}
          defaultLanguage="python"
          value={code}
          onChange={(value) => setCode(value || '')}
          theme="vs-dark"
          options={{
            minimap: { enabled: false },
            lineNumbers: showLineNumbers ? 'on' : 'off',
            fontSize: 13,
            scrollBeyondLastLine: false,
            automaticLayout: true,
            padding: { top: 16, bottom: 16 },
            fontFamily: "'Monaco', 'Menlo', 'Consolas', monospace",
            quickSuggestions: false,
            parameterHints: { enabled: false },
            suggestOnTriggerCharacters: false,
            acceptSuggestionOnCommitCharacter: false,
          }}
          onMount={(editor, monaco) => {
            // Add keyboard shortcut: Shift+Enter to run code
            // Use addAction() API - recommended approach for custom keyboard shortcuts
            // Use refs to avoid stale closure capturing initial state values
            editor.addAction({
              id: 'run-python-code',
              label: 'Run Python Code',
              keybindings: [
                monaco.KeyMod.Shift | monaco.KeyCode.Enter
              ],
              run: () => {
                if (!isLoadingRef.current && !isRunningRef.current) {
                  handleRunRef.current?.();
                }
              }
            });
          }}
          loading={<div />}
        />
      </div>

      {/* Output Section - only shows when there's output or error */}
      {(output || error) && (
        <div className={`${styles.outputSection} ${error ? styles.hasError : ''}`}>
          <div className={styles.outputHeader}>
            <span className={styles.outputLabel}>
              {error ? 'Error' : 'Output'}
            </span>
            <div className={styles.outputActions}>
              <button
                className={`${styles.outputActionBtn} ${outputCopied ? styles.copied : ''}`}
                onClick={handleCopyOutput}
                title={outputCopied ? "Copied!" : "Copy output"}
                aria-label="Copy output"
              >
                {outputCopied ? <Check size={14} /> : <Copy size={14} />}
              </button>
              <button
                className={styles.outputActionBtn}
                onClick={handleClearOutput}
                title="Clear output"
                aria-label="Clear output"
              >
                <RotateCcw size={14} />
              </button>
            </div>
          </div>
          <div className={styles.outputContent} ref={outputRef}>
            {error ? (
              <ErrorDisplay errorText={error} sourceCode={code} />
            ) : (
              <div className={styles.outputText}>{output}</div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default InteractivePython;
