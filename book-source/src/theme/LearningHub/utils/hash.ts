/**
 * MD5 Hash Utility
 * Content hashing for cache invalidation
 */

import MD5 from 'crypto-js/md5';

/**
 * Compute MD5 hash of content
 * @param content String to hash
 * @returns 32-character hex string
 */
export function computeHash(content: string): string {
  return MD5(content).toString();
}

/**
 * Compute hash of page content (normalized)
 * Removes whitespace variations for stable hashing
 * @param content Page content
 * @returns MD5 hash
 */
export function computeContentHash(content: string): string {
  // Normalize: trim, collapse multiple spaces, lowercase
  const normalized = content
    .trim()
    .replace(/\s+/g, ' ')
    .toLowerCase();
  
  return computeHash(normalized);
}

/**
 * Verify if hash matches content
 * @param content Content to verify
 * @param expectedHash Expected MD5 hash
 * @returns true if hashes match
 */
export function verifyHash(content: string, expectedHash: string): boolean {
  const actualHash = computeHash(content);
  return actualHash === expectedHash;
}
