/**
 * ErrorBoundary Component
 * Catches component errors without breaking entire UI
 */

import React, { Component, ErrorInfo, ReactNode } from 'react';
import { errorLogger } from '../services/errorLogger';
import styles from '../styles/ErrorBoundary.module.css';

interface Props {
  children: ReactNode;
  fallback?: ReactNode;
}

interface State {
  hasError: boolean;
  error: Error | null;
}

export class ErrorBoundary extends Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
    };
  }

  static getDerivedStateFromError(error: Error): State {
    return {
      hasError: true,
      error,
    };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo): void {
    // Log error to our error logger
    errorLogger.logError(error, {
      componentStack: errorInfo.componentStack,
      feature: 'LearningHub',
    });
  }

  handleReset = (): void => {
    this.setState({
      hasError: false,
      error: null,
    });
  };

  render(): ReactNode {
    if (this.state.hasError) {
      // Custom fallback if provided
      if (this.props.fallback) {
        return this.props.fallback;
      }

      // Default error UI
      return (
        <div className={styles.errorContainer}>
          <div className={styles.errorIcon}>⚠️</div>
          <h3 className={styles.errorTitle}>Something went wrong</h3>
          <p className={styles.errorMessage}>
            The Learning Hub encountered an error. Don't worry, your progress is saved.
          </p>
          <button className={styles.resetButton} onClick={this.handleReset}>
            Try Again
          </button>
          {process.env.NODE_ENV === 'development' && this.state.error && (
            <details className={styles.errorDetails}>
              <summary>Error Details (Development Only)</summary>
              <pre>{this.state.error.message}</pre>
              <pre>{this.state.error.stack}</pre>
            </details>
          )}
        </div>
      );
    }

    return this.props.children;
  }
}
