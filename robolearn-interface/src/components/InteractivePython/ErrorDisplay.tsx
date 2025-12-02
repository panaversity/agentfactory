/**
 * Enhanced Python Error Display
 *
 * Parses Python tracebacks and presents them in a learner-friendly format
 * with syntax highlighting, line highlighting, and educational hints.
 */

import React from 'react';
import styles from './ErrorDisplay.module.css';

interface ParsedError {
  type: string;
  message: string;
  line?: number;
  file?: string;
  traceback?: string[];
  hint?: string;
}

/**
 * Parse Python error message into structured format
 */
function parsePythonError(errorText: string): ParsedError {
  const lines = errorText.split('\n');

  // Common Python error patterns
  const errorTypeMatch = errorText.match(/(\w+Error|Exception): (.+)$/m);
  const lineMatch = errorText.match(/line (\d+)/i);
  const fileMatch = errorText.match(/File "([^"]+)"/);

  // Extract error type and message
  const type = errorTypeMatch?.[1] || 'Error';
  const message = errorTypeMatch?.[2] || errorText;

  // Extract line number
  const line = lineMatch ? parseInt(lineMatch[1], 10) : undefined;

  // Extract file name
  const file = fileMatch?.[1];

  // Extract traceback lines (lines starting with "  File" or spaces)
  const traceback = lines.filter(line =>
    line.trim().startsWith('File "') ||
    line.match(/^\s{2,}/)
  );

  // Generate helpful hint based on error type
  const hint = getErrorHint(type, message);

  return { type, message, line, file, traceback, hint };
}

/**
 * Provide educational hints for common Python errors
 */
function getErrorHint(errorType: string, message: string): string | undefined {
  const hints: Record<string, string> = {
    'SyntaxError': 'Check for missing colons, unmatched parentheses, or incorrect indentation.',
    'IndentationError': 'Python uses indentation to define code blocks. Make sure your indentation is consistent (use spaces or tabs, not both).',
    'NameError': 'This variable or function hasn\'t been defined yet. Check for typos or make sure you define it before using it.',
    'TypeError': 'You\'re trying to use a value in a way that doesn\'t match its type. For example, you can\'t add a string to a number.',
    'ValueError': 'The value you provided is the right type but has an invalid value for this operation.',
    'IndexError': 'You\'re trying to access an index that doesn\'t exist in your list or string.',
    'KeyError': 'This key doesn\'t exist in your dictionary. Use `.get()` or check if the key exists first.',
    'AttributeError': 'This object doesn\'t have the attribute or method you\'re trying to use.',
    'ZeroDivisionError': 'You\'re trying to divide by zero, which is mathematically undefined.',
    'ImportError': 'The module or package you\'re trying to import isn\'t available in this environment.',
    'ModuleNotFoundError': 'The module you\'re trying to import doesn\'t exist or isn\'t installed.',
  };

  // Special case hints based on message content
  if (message.includes('unexpected EOF')) {
    return 'Your code ended unexpectedly. Check for unclosed parentheses, brackets, or quotes.';
  }
  if (message.includes('invalid syntax')) {
    return 'There\'s a syntax error in your code. Common causes: missing colons after if/for/def, mismatched quotes, or typos.';
  }
  if (message.includes('not defined')) {
    return 'Make sure you define variables and functions before using them. Check for typos in names.';
  }

  return hints[errorType];
}

interface ErrorDisplayProps {
  errorText: string;
  sourceCode?: string;
}

export const ErrorDisplay: React.FC<ErrorDisplayProps> = ({ errorText, sourceCode }) => {
  const parsed = parsePythonError(errorText);

  return (
    <div className={styles.errorContainer}>
      {/* Error header with type badge */}
      <div className={styles.errorHeader}>
        <div className={styles.errorBadge}>
          <span className={styles.errorIcon}>⚠</span>
          <span className={styles.errorType}>{parsed.type}</span>
        </div>
        {parsed.line && (
          <div className={styles.lineIndicator}>
            <span className={styles.lineLabel}>Line</span>
            <span className={styles.lineNumber}>{parsed.line}</span>
          </div>
        )}
      </div>

      {/* Error message */}
      <div className={styles.errorMessage}>
        {parsed.message}
      </div>

      {/* Educational hint */}
      {parsed.hint && (
        <div className={styles.hint}>
          <div className={styles.hintHeader}>
            <span className={styles.hintIcon}>💡</span>
            <span className={styles.hintLabel}>Hint</span>
          </div>
          <div className={styles.hintText}>{parsed.hint}</div>
        </div>
      )}

      {/* Traceback (collapsible for cleaner view) */}
      {parsed.traceback && parsed.traceback.length > 0 && (
        <details className={styles.tracebackDetails}>
          <summary className={styles.tracebackSummary}>
            <span>View Full Traceback</span>
            <span className={styles.tracebackCount}>
              ({parsed.traceback.length} lines)
            </span>
          </summary>
          <div className={styles.traceback}>
            {parsed.traceback.map((line, i) => (
              <div key={i} className={styles.tracebackLine}>
                {line}
              </div>
            ))}
          </div>
        </details>
      )}
    </div>
  );
};

export default ErrorDisplay;
