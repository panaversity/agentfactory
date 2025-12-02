/**
 * Search utilities for integrating with @easyops-cn/docusaurus-search-local
 * 
 * The plugin creates search index files that we can load and search.
 */

export interface SearchResult {
  title: string;
  url: string;
  text?: string;
  type?: string;
}

let searchIndex: any = null;
let searchIndexLoaded = false;
let lunrLoaded = false;

/**
 * Load Lunr.js library for full-text search
 */
async function loadLunr(): Promise<boolean> {
  if (lunrLoaded || (typeof window !== 'undefined' && (window as any).lunr)) {
    return true;
  }

  return new Promise((resolve) => {
    if (typeof window === 'undefined') {
      resolve(false);
      return;
    }

    // Check if already loaded
    if ((window as any).lunr) {
      lunrLoaded = true;
      resolve(true);
      return;
    }

    // Load Lunr.js from CDN
    const script = document.createElement('script');
    script.src = 'https://unpkg.com/lunr@2.3.9/lunr.min.js';
    script.onload = () => {
      lunrLoaded = true;
      resolve(true);
    };
    script.onerror = () => {
      console.warn('Failed to load Lunr.js, using simple search');
      resolve(false);
    };
    document.head.appendChild(script);
  });
}

/**
 * Load the search index from the plugin's generated files
 * Note: Search index is only available in production builds, not in dev mode (npm start)
 * But it IS available when serving the build locally (npm run serve)
 */
async function loadSearchIndex(): Promise<any> {
  if (searchIndexLoaded && searchIndex) {
    return searchIndex;
  }

  try {
    const baseUrl = window.location.origin;
    const pathname = window.location.pathname;
    
    // Detect base path (e.g., /robolearn/)
    // Check if we're in a subpath by looking at the first segment
    let basePath = '';
    const pathSegments = pathname.split('/').filter(Boolean);
    
    // If first segment is NOT a known Docusaurus route, it's likely the basePath
    if (pathSegments.length > 0) {
      const firstSegment = pathSegments[0];
      const knownRoutes = ['docs', 'blog', 'search', 'auth', 'api', 'code'];
      if (!knownRoutes.includes(firstSegment)) {
        basePath = `/${firstSegment}`;
      }
    }
    
    // Try different possible paths for the search index
    // The plugin puts search-index.json at the root of the build output
    const possiblePaths = [
      `${baseUrl}${basePath}/search-index.json`,
      `${baseUrl}/search-index.json`,
    ];

    for (const path of possiblePaths) {
      try {
        const response = await fetch(path, {
          headers: {
            'Accept': 'application/json',
          },
        });
        
        if (response.ok) {
          const contentType = response.headers.get('content-type');
          if (contentType && contentType.includes('application/json')) {
            const data = await response.json();
            searchIndex = data;
            searchIndexLoaded = true;
            console.log('✅ Search index loaded from:', path);
            return data;
          }
        }
      } catch (e) {
        // Silently continue to next path
        continue;
      }
    }

    // If no index found, return null
    return null;
  } catch (error) {
    // Silently fail - search will be disabled
    return null;
  }
}

/**
 * Perform a search query using the loaded index
 * Uses Lunr.js if available, otherwise falls back to simple text search
 */
export async function searchContent(query: string): Promise<SearchResult[]> {
  if (!query.trim()) {
    return [];
  }

  const index = await loadSearchIndex();
  if (!index) {
    // In dev mode, search index is not available (only generated during build)
    // Return empty results silently
    return [];
  }

  const searchTerms = query.toLowerCase().split(/\s+/).filter(term => term.length > 0);
  const results: SearchResult[] = [];

  // The plugin uses an array format: [{ documents: [...], index: {...} }]
  // We need to get documents and Lunr index from the array items
  let documents: any[] = [];
  let lunrIndex: any = null;
  
  if (Array.isArray(index)) {
    // The index is an array of language objects: [{ documents: [...], index: {...} }]
    // Get documents and Lunr index from first language object (usually 'en')
    const langObj = index.find((obj: any) => obj.documents) || index[0];
    if (langObj) {
      if (langObj.documents && Array.isArray(langObj.documents)) {
        documents = langObj.documents;
      }
      if (langObj.index) {
        lunrIndex = langObj.index;
      }
    }
  } else if (index.documents && Array.isArray(index.documents)) {
    documents = index.documents;
    lunrIndex = index.index;
  } else if (index.docs && Array.isArray(index.docs)) {
    documents = index.docs;
  }
  
  if (documents.length === 0) {
    console.warn('No documents found in search index');
    return [];
  }

  // Try to use Lunr.js for full-text search
  if (lunrIndex) {
    const lunrAvailable = await loadLunr();
    
    if (lunrAvailable && typeof window !== 'undefined' && (window as any).lunr) {
      try {
        const lunr = (window as any).lunr;
        const idx = lunr.Index.load(lunrIndex);
        const searchResults = idx.search(query);
        
        // Map Lunr results to our format
        searchResults.forEach((result: any) => {
          // result.ref is the document ID (string), match with doc.i (also string)
          const doc = documents.find((d: any) => String(d.i) === String(result.ref));
          if (doc) {
            const title = doc.t || '';
            const url = doc.u || '';
            // Breadcrumbs can be array or JSON string, parse if needed
            let breadcrumbs: string[] = [];
            if (Array.isArray(doc.b)) {
              breadcrumbs = doc.b;
            } else if (typeof doc.b === 'string' && doc.b.trim().startsWith('[')) {
              try {
                breadcrumbs = JSON.parse(doc.b);
              } catch (e) {
                breadcrumbs = [];
              }
            }
            
            results.push({
              title: title || url.split('/').pop() || 'Untitled',
              url: url,
              text: breadcrumbs.length > 0 ? breadcrumbs.join(' › ') : '',
              type: 'doc',
              score: result.score || 0,
            } as any);
          }
        });
        
        // Sort by score
        results.sort((a: any, b: any) => (b.score || 0) - (a.score || 0));
        console.log(`Lunr.js search for "${query}" found ${results.length} results`);
        return results.slice(0, 8);
      } catch (error) {
        console.warn('Lunr.js search failed, falling back to simple search:', error);
      }
    }
  }

  // Fallback: Simple text search through title and breadcrumbs
  // Note: This only searches titles/breadcrumbs, not full content
  // For full-content search, Lunr.js is needed
  documents.forEach((doc: any) => {
    const title = doc.t || doc.title || '';
    const url = doc.u || doc.url || doc.href || '';
    // Breadcrumbs can be array or JSON string, parse if needed
    let breadcrumbs: string[] = [];
    if (Array.isArray(doc.b)) {
      breadcrumbs = doc.b;
    } else if (typeof doc.b === 'string' && doc.b.trim().startsWith('[')) {
      try {
        breadcrumbs = JSON.parse(doc.b);
      } catch (e) {
        breadcrumbs = [];
      }
    }
    
    // Combine title and breadcrumbs for searching
    const searchableText = [
      title,
      ...breadcrumbs
    ].filter(Boolean).join(' ').toLowerCase();

    const titleLower = title.toLowerCase();

    // Check if search terms match (use .some() for OR matching - any term matches)
    const matches = searchTerms.some(term => 
      searchableText.includes(term)
    );

    if (matches) {
      // Calculate relevance score
      let score = 0;
      searchTerms.forEach(term => {
        if (titleLower.includes(term)) {
          score += 10; // Title match is high priority
        }
        if (titleLower === term) {
          score += 20; // Exact title match is highest priority
        }
        if (searchableText.includes(term)) {
          score += 1; // Any match
        }
      });

      results.push({
        title: title || url.split('/').pop() || 'Untitled',
        url: url,
        text: breadcrumbs.length > 0 ? breadcrumbs.join(' › ') : '',
        type: 'doc',
        score: score,
      } as any);
    }
  });
  
  console.log(`Search for "${query}" found ${results.length} results in ${documents.length} documents`);

  // Sort by relevance score (higher score first)
  results.sort((a: any, b: any) => {
    const aScore = a.score || 0;
    const bScore = b.score || 0;
    return bScore - aScore;
  });

  return results.slice(0, 8);
}

