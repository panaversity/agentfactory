/**
 * Summary Service - API client for streaming summary generation
 */

/**
 * Fetch summary with Server-Sent Events (SSE) streaming
 * 
 * @param pageId - Unique identifier for the content page
 * @param content - Full page content text
 * @param token - Authentication token
 * @param onChunk - Callback for each received chunk
 * @param onComplete - Callback when streaming completes
 * @param onError - Callback for errors
 */
export async function fetchSummary(
  pageId: string,
  content: string,
  token: string,
  onChunk: (chunk: string) => void,
  onComplete: () => void,
  onError: (error: string) => void
): Promise<void> {
  let timeoutId: NodeJS.Timeout | null = null;

  try {
    const apiUrl = process.env.NODE_ENV === 'production' 
      ? '/api/v1/summarize' 
      : 'http://localhost:8000/api/v1/summarize';

    const url = new URL(apiUrl, window.location.origin);
    url.searchParams.append('pageId', pageId);
    url.searchParams.append('token', token);
    url.searchParams.append('content', content);

    const es = new EventSource(url.toString());

    // Cleanup function
    const cleanup = () => {
      if (timeoutId) {
        clearTimeout(timeoutId);
        timeoutId = null;
      }
      es.close();
    };

    es.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);

        if (data.error) {
          cleanup();
          onError(data.error);
          return;
        }

        if (data.chunk) {
          onChunk(data.chunk);
        }

        if (data.done) {
          console.log('✅ Summary streaming completed successfully');
          cleanup();
          onComplete();
        }
      } catch (err) {
        console.error('Error parsing SSE data:', err);
        cleanup();
        onError('Failed to parse server response');
      }
    };

    es.onerror = (error) => {
      console.error('EventSource error:', error);
      cleanup();
      onError('Connection error. Please try again.');
    };

    // Timeout after 60 seconds (increased for AI generation)
    timeoutId = setTimeout(() => {
      console.warn('⏰ Request timeout after 60 seconds');
      cleanup();
      onError('Request timeout. Please try again.');
    }, 60000);

  } catch (error) {
    if (timeoutId) {
      clearTimeout(timeoutId);
    }
    console.error('Fetch summary error:', error);
    onError(error instanceof Error ? error.message : 'Unknown error occurred');
  }
}
