/**
 * URL utilities for handling baseUrl in Docusaurus deployments
 * Works for dev, GitHub Pages, and custom domains
 * 
 * Simple approach: Detect the base path from the current URL
 * No hardcoding - just use whatever path the user is currently on
 */

/**
 * Detects the base path from the current URL
 * 
 * Docusaurus respects baseUrl even in dev mode.
 * If baseUrl is "/robolearn/", you access the site at http://localhost:3000/robolearn/
 * 
 * So we detect the base path from the current URL pathname.
 */
function detectBasePath(): string {
  if (typeof window === 'undefined') return '';
  
  const pathname = window.location.pathname;
  
  // Extract the first path segment (e.g., /robolearn/docs -> /robolearn)
  const match = pathname.match(/^\/([^/]+)/);
  
  if (match) {
    const firstSegment = match[1];
    // If first segment is a known app/content path, we're at root
    // These are Docusaurus routes that don't indicate a baseUrl
    // Add custom routes like 'labs', 'chat' to this list
    if (['auth', 'api', 'docs', 'blog', 'search', 'labs', 'chat'].includes(firstSegment)) {
      return '';
    }
    // Otherwise, we're in a subpath (e.g., /robolearn/)
    return `/${firstSegment}`;
  }
  
  return ''; // Root (pathname is just "/")
}

/**
 * Gets the home URL (with baseUrl if applicable)
 */
export function getHomeUrl(): string {
  const basePath = detectBasePath();
  return basePath ? `${basePath}/` : '/';
}

/**
 * Constructs a redirect URI for OAuth callback
 * Uses the detected base path from current URL - no hardcoding needed
 */
export function getRedirectUri(): string {
  if (typeof window === 'undefined') {
    return 'http://localhost:3000/auth/callback';
  }
  
  const basePath = detectBasePath();
  return `${window.location.origin}${basePath}/auth/callback`;
}

