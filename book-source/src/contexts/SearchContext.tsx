import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';

export interface SearchResult {
  type: 'page' | 'heading';
  title: string;
  breadcrumb: string;
  description: string;
  url: string;
  level?: number; // Only for headings (2-6)
  _filePath?: string;
}

interface SearchContextType {
  searchIndex: SearchResult[];
  isLoading: boolean;
  search: (query: string) => SearchResult[];
}

const SearchContext = createContext<SearchContextType | undefined>(undefined);

// Fuzzy search scoring function
function fuzzyScore(text: string, query: string): number {
  const textLower = text.toLowerCase();
  const queryLower = query.toLowerCase();

  // Exact match gets highest score
  if (textLower === queryLower) return 1000;

  // Starts with query gets high score
  if (textLower.startsWith(queryLower)) return 500;

  // Contains whole query gets good score
  if (textLower.includes(queryLower)) return 100;

  // Fuzzy match - check if all characters in query appear in order
  let queryIndex = 0;
  let matchCount = 0;

  for (let i = 0; i < textLower.length && queryIndex < queryLower.length; i++) {
    if (textLower[i] === queryLower[queryIndex]) {
      matchCount++;
      queryIndex++;
    }
  }

  // If all query characters were found in order
  if (queryIndex === queryLower.length) {
    return matchCount * 10;
  }

  return 0;
}

// Search function with scoring and ranking
function performSearch(searchIndex: SearchResult[], query: string): SearchResult[] {
  if (!query || query.trim().length === 0) {
    return [];
  }

  const queryTrimmed = query.trim();
  const queryWords = queryTrimmed.toLowerCase().split(/\s+/);

  const results: Array<SearchResult & { score: number }> = [];

  for (const item of searchIndex) {
    let totalScore = 0;

    // Score based on title match (highest weight)
    const titleScore = fuzzyScore(item.title, queryTrimmed);
    totalScore += titleScore * 10;

    // Score based on breadcrumb match (medium weight)
    const breadcrumbScore = fuzzyScore(item.breadcrumb, queryTrimmed);
    totalScore += breadcrumbScore * 3;

    // Bonus points for matching multiple words
    if (queryWords.length > 1) {
      const allText = `${item.title} ${item.breadcrumb}`.toLowerCase();
      let wordsMatched = 0;

      for (const word of queryWords) {
        if (allText.includes(word)) {
          wordsMatched++;
        }
      }

      if (wordsMatched === queryWords.length) {
        totalScore += 50; // Bonus for matching all words
      }
    }

    // Boost pages over headings slightly (pages are more important)
    if (item.type === 'page') {
      totalScore *= 1.2;
    }

    // Only include results with a score above threshold
    if (totalScore > 0) {
      results.push({
        ...item,
        score: totalScore,
      });
    }
  }

  // Sort by score descending
  results.sort((a, b) => b.score - a.score);

  // Return top 20 results without the score
  return results.slice(0, 20).map(({ score, ...item }) => item);
}

export const SearchProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [searchIndex, setSearchIndex] = useState<SearchResult[]>([]);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Load search index
    const loadSearchIndex = async () => {
      try {
        setIsLoading(true);

        // Fetch the search index
        const response = await fetch('/search-index.json');

        if (!response.ok) {
          throw new Error(`Failed to load search index: ${response.statusText}`);
        }

        const data: SearchResult[] = await response.json();
        setSearchIndex(data);

        console.log(`✅ Search index loaded: ${data.length} documents`);
      } catch (error) {
        console.error('❌ Error loading search index:', error);
        // Set empty index on error
        setSearchIndex([]);
      } finally {
        setIsLoading(false);
      }
    };

    loadSearchIndex();
  }, []);

  const search = useCallback(
    (query: string): SearchResult[] => {
      return performSearch(searchIndex, query);
    },
    [searchIndex]
  );

  return (
    <SearchContext.Provider value={{ searchIndex, isLoading, search }}>
      {children}
    </SearchContext.Provider>
  );
};

export const useSearch = (): SearchContextType => {
  const context = useContext(SearchContext);
  if (context === undefined) {
    throw new Error('useSearch must be used within a SearchProvider');
  }
  return context;
};
