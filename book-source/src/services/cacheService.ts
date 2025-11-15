/**
 * Cache Service - sessionStorage wrapper for managing cached data
 * 
 * Provides type-safe methods for storing and retrieving data from sessionStorage
 * with automatic JSON serialization/deserialization.
 */

/**
 * Get item from sessionStorage
 * @param key - Storage key
 * @returns Parsed value or null if not found
 */
export function get<T>(key: string): T | null {
  try {
    const item = sessionStorage.getItem(key);
    if (!item) {
      return null;
    }
    return JSON.parse(item) as T;
  } catch (error) {
    console.error(`Error reading from sessionStorage key "${key}":`, error);
    return null;
  }
}

/**
 * Set item in sessionStorage
 * @param key - Storage key
 * @param value - Value to store (will be JSON stringified)
 */
export function set<T>(key: string, value: T): void {
  try {
    const serialized = JSON.stringify(value);
    sessionStorage.setItem(key, serialized);
  } catch (error) {
    console.error(`Error writing to sessionStorage key "${key}":`, error);
  }
}

/**
 * Remove item from sessionStorage
 * @param key - Storage key
 */
export function remove(key: string): void {
  try {
    sessionStorage.removeItem(key);
  } catch (error) {
    console.error(`Error removing sessionStorage key "${key}":`, error);
  }
}

/**
 * Clear all items from sessionStorage
 */
export function clear(): void {
  try {
    sessionStorage.clear();
  } catch (error) {
    console.error('Error clearing sessionStorage:', error);
  }
}

/**
 * Check if a key exists in sessionStorage
 * @param key - Storage key
 * @returns true if key exists
 */
export function has(key: string): boolean {
  return sessionStorage.getItem(key) !== null;
}
