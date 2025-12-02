/**
 * Pyodide Context Provider
 * Provides Pyodide instance and loading state to all child components
 * Ensures Pyodide is initialized once and shared across the app
 */

import React, { createContext, useContext, useEffect, useState, useMemo } from 'react';
import { PyodideRunner } from '@/lib/pyodide-singleton';

/**
 * Context value type
 */
interface PyodideContextType {
  pyodide: PyodideRunner;
  isLoading: boolean;
  error: Error | null;
}

/**
 * Create the context
 */
const PyodideContext = createContext<PyodideContextType | null>(null);

/**
 * Provider component - wrap your app with this to enable Pyodide
 * Usage: <PyodideProvider><App /></PyodideProvider>
 */
export const PyodideProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);
  const pyodide = PyodideRunner.getInstance();

  /**
   * Initialize Pyodide on component mount
   */
  useEffect(() => {
    pyodide
      .init()
      .then(() => {
        setIsLoading(false);
        setError(null);
      })
      .catch((err) => {
        setError(err instanceof Error ? err : new Error(String(err)));
        setIsLoading(false);
      });
  }, [pyodide]);

  // Memoize the context value to prevent unnecessary re-renders of consumers
  const contextValue = useMemo(
    () => ({ pyodide, isLoading, error }),
    [pyodide, isLoading, error]
  );

  return (
    <PyodideContext.Provider value={contextValue}>
      {children}
    </PyodideContext.Provider>
  );
};

/**
 * Custom hook to use Pyodide in any component
 * Must be used within a PyodideProvider
 * Usage: const { pyodide, isLoading } = usePyodide();
 */
export const usePyodide = (): PyodideContextType => {
  const context = useContext(PyodideContext);

  if (!context) {
    throw new Error(
      'usePyodide must be used within a PyodideProvider. ' +
      'Wrap your app with <PyodideProvider> in Root.tsx'
    );
  }

  return context;
};
