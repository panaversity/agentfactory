/**
 * Translation Cache Module
 * 
 * Manages translation cache to avoid re-translating unchanged content.
 * Uses file hashing (SHA256) to detect changes.
 */

const crypto = require('crypto');
const fs = require('fs');
const path = require('path');

/**
 * Generate SHA256 hash of file content
 */
function generateFileHash(content) {
  return crypto.createHash('sha256').update(content, 'utf8').digest('hex');
}

/**
 * Get cache file path for a source file
 */
function getCacheFilePath(cacheDir, sourcePath, targetLocale) {
  const hash = generateFileHash(sourcePath); // Use path as part of hash key
  return path.join(cacheDir, `${hash}-${targetLocale}.json`);
}

/**
 * Check if translation exists in cache and is up-to-date
 */
function isCacheValid(cacheDir, sourcePath, sourceContent, targetLocale) {
  const cacheFile = getCacheFilePath(cacheDir, sourcePath, targetLocale);
  
  if (!fs.existsSync(cacheFile)) {
    return false;
  }

  try {
    const cacheData = JSON.parse(fs.readFileSync(cacheFile, 'utf8'));
    const currentHash = generateFileHash(sourceContent);
    
    // Cache is valid if hash matches
    return cacheData.sourceHash === currentHash && 
           cacheData.targetLocale === targetLocale &&
           fs.existsSync(cacheData.translationPath);
  } catch (error) {
    // Cache file corrupted or invalid
    return false;
  }
}

/**
 * Get cached translation path
 */
function getCachedTranslationPath(cacheDir, sourcePath, targetLocale) {
  const cacheFile = getCacheFilePath(cacheDir, sourcePath, targetLocale);
  
  if (!fs.existsSync(cacheFile)) {
    return null;
  }

  try {
    const cacheData = JSON.parse(fs.readFileSync(cacheFile, 'utf8'));
    return cacheData.translationPath;
  } catch (error) {
    return null;
  }
}

/**
 * Store translation in cache
 */
function storeCache(cacheDir, sourcePath, sourceContent, targetLocale, translationPath) {
  const cacheFile = getCacheFilePath(cacheDir, sourcePath, targetLocale);
  const cacheData = {
    sourcePath,
    sourceHash: generateFileHash(sourceContent),
    targetLocale,
    translationPath,
    timestamp: new Date().toISOString(),
  };

  // Ensure cache directory exists
  if (!fs.existsSync(cacheDir)) {
    fs.mkdirSync(cacheDir, { recursive: true });
  }

  fs.writeFileSync(cacheFile, JSON.stringify(cacheData, null, 2), 'utf8');
}

/**
 * Invalidate cache for a source file
 */
function invalidateCache(cacheDir, sourcePath, targetLocale) {
  const cacheFile = getCacheFilePath(cacheDir, sourcePath, targetLocale);
  if (fs.existsSync(cacheFile)) {
    fs.unlinkSync(cacheFile);
  }
}

module.exports = {
  generateFileHash,
  isCacheValid,
  getCachedTranslationPath,
  storeCache,
  invalidateCache,
};

