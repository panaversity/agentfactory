/**
 * T051-T057: Personalization service with SSE streaming
 * 
 * Provides functions for streaming personalized content from the API
 * using Server-Sent Events (SSE), mirroring summarizationService pattern.
 */

import { ProficiencyLevel } from '../types/contentTabs';

// T051: API configuration
const API_BASE_URL = typeof window !== 'undefined' 
  ? (window as any).REACT_APP_API_BASE_URL || 'http://localhost:8000'
  : 'http://localhost:8000';

/**
 * T052: SSE event interface for personalized content
 */
export interface PersonalizationEvent {
  chunk: string;
  done: boolean;
  error?: string;
}

/**
 * T053: PersonalizationService class with SSE EventSource client
 */
export class PersonalizationService {
  private eventSource: EventSource | null = null;
  private abortController: AbortController | null = null;
  private isCompleted: boolean = false; // Track if stream completed successfully

  /**
   * T054: Stream personalized content from API
   * 
   * Connects to /api/v1/personalize endpoint and streams personalized content
   * chunks using Server-Sent Events (SSE).
   * 
   * @param pageId - Unique identifier for the content page
   * @param content - Full page content text to personalize
   * @param token - Authentication token
   * @param programmingLevel - User's programming proficiency
   * @param aiLevel - User's AI knowledge proficiency
   * @param onChunk - Callback for each content chunk
   * @param onComplete - Callback when streaming completes
   * @param onError - Callback for errors
   */
  streamPersonalizedContent(
    pageId: string,
    content: string,
    token: string,
    programmingLevel: ProficiencyLevel,
    aiLevel: ProficiencyLevel,
    onChunk: (chunk: string) => void,
    onComplete: () => void,
    onError: (error: string) => void
  ): void {
    // Reset completion flag for new stream
    this.isCompleted = false;
    
    // T055: Build query params
    const params = new URLSearchParams({
      pageId,
      content,
      token,
      programmingLevel,
      aiLevel,
    });

    const url = `${API_BASE_URL}/api/v1/personalize?${params.toString()}`;

    // T056: Create EventSource for SSE connection
    this.eventSource = new EventSource(url);

    // Handle incoming messages
    this.eventSource.onmessage = (event) => {
      try {
        const data: PersonalizationEvent = JSON.parse(event.data);

        if (data.error) {
          onError(data.error);
          this.close();
          return;
        }

        if (data.done) {
          this.isCompleted = true; // Mark as successfully completed
          onComplete();
          this.close();
          return;
        }

        if (data.chunk) {
          onChunk(data.chunk);
        }
      } catch (error) {
        console.error('Error parsing SSE event:', error);
        onError('Failed to parse server response');
        this.close();
      }
    };

    // Handle errors
    this.eventSource.onerror = (error) => {
      // Ignore errors if stream already completed successfully
      if (this.isCompleted) {
        console.log('EventSource closed after successful completion (expected behavior)');
        this.close();
        return;
      }
      
      console.error('EventSource error:', error);
      onError('Connection to server failed');
      this.close();
    };
  }

  /**
   * T057: Cancel ongoing personalization stream
   */
  cancel(): void {
    this.close();
  }

  /**
   * Close EventSource connection
   */
  private close(): void {
    if (this.eventSource) {
      this.eventSource.close();
      this.eventSource = null;
    }
  }
}

// Export singleton instance
export const personalizationService = new PersonalizationService();
