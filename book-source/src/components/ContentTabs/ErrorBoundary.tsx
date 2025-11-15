/**
 * ErrorBoundary Component - catches React errors in children components
 */
import React, { Component, ReactNode, ErrorInfo } from 'react';
import styles from './styles.module.css';

interface ErrorBoundaryProps {
  children: ReactNode;
}

interface ErrorBoundaryState {
  hasError: boolean;
  error: Error | null;
  errorInfo: ErrorInfo | null;
}

export default class ErrorBoundary extends Component<ErrorBoundaryProps, ErrorBoundaryState> {
  constructor(props: ErrorBoundaryProps) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
      errorInfo: null,
    };
  }

  static getDerivedStateFromError(error: Error): Partial<ErrorBoundaryState> {
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo): void {
    console.error('ErrorBoundary caught an error:', error, errorInfo);
    this.setState({
      error,
      errorInfo,
    });
  }

  handleReset = (): void => {
    this.setState({
      hasError: false,
      error: null,
      errorInfo: null,
    });
  };

  render(): ReactNode {
    if (this.state.hasError) {
      return (
        <div className={styles.errorBoundary}>
          <div className={styles.errorBoundaryContent}>
            <h2 className={styles.errorBoundaryTitle}>⚠️ Something went wrong</h2>
            <p className={styles.errorBoundaryMessage}>
              The content tabs encountered an unexpected error. Please try refreshing the page.
            </p>
            {this.state.error && (
              <details className={styles.errorDetails}>
                <summary>Error details</summary>
                <pre className={styles.errorStack}>
                  <code>{this.state.error.toString()}</code>
                  {this.state.errorInfo && (
                    <code>{this.state.errorInfo.componentStack}</code>
                  )}
                </pre>
              </details>
            )}
            <button 
              onClick={this.handleReset} 
              className={styles.errorBoundaryButton}
              type="button"
            >
              Try Again
            </button>
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}
