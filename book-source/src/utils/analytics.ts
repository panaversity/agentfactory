/**
 * Analytics Utility Module
 *
 * Custom event tracking for GA4
 * Track chapter views, external resource clicks, scroll depth, and engagement
 *
 * Usage:
 * import { trackChapterView, trackResourceClick, trackScrollDepth } from '@/utils/analytics';
 * trackChapterView(1, 'Introducing AI-Driven Development', 'Part 1');
 */

declare global {
  interface Window {
    gtag: Function;
  }
}

// Helper to check if we're in development mode
const isDevelopment = () => process.env.NODE_ENV === "development";

// Helper for debug logging (only in development)
const debugLog = (message: string) => {
  if (isDevelopment()) {
    console.log(message);
  }
};

/**
 * Track when a chapter/lesson is viewed
 * @param chapterNumber - The chapter number (1, 2, 3, etc.)
 * @param chapterTitle - The title of the chapter
 * @param part - The part number or name (e.g., "Part 1", "Preface")
 */
export const trackChapterView = (
  chapterNumber: number,
  chapterTitle: string,
  part: string
) => {
  if (typeof window !== "undefined" && window.gtag) {
    window.gtag("event", "chapter_view", {
      chapter_number: chapterNumber,
      chapter_title: chapterTitle,
      part: part,
      timestamp: new Date().toISOString(),
    });
    debugLog(`📊 Analytics: Chapter viewed - ${part}: ${chapterTitle}`);
  }
};

/**
 * Track when external resources are clicked
 * @param resourceType - Type of resource ('github', 'youtube', 'docs', 'external')
 * @param url - The URL of the resource
 * @param label - Optional label for the resource
 */
export const trackResourceClick = (
  resourceType: "github" | "youtube" | "docs" | "external",
  url: string,
  label?: string
) => {
  if (typeof window !== "undefined" && window.gtag) {
    window.gtag("event", "resource_click", {
      resource_type: resourceType,
      url: url,
      label: label || url,
      timestamp: new Date().toISOString(),
    });
    debugLog(`📊 Analytics: ${resourceType} clicked - ${label || url}`);
  }
};

/**
 * Track scroll depth as percentage of page scrolled
 * @param depthPercentage - Percentage of page scrolled (0-100)
 */
export const trackScrollDepth = (depthPercentage: number) => {
  if (typeof window !== "undefined" && window.gtag) {
    // Only log significant scroll milestones to reduce noise
    const milestone = Math.floor(depthPercentage / 25) * 25;
    if (depthPercentage >= milestone) {
      window.gtag("event", "scroll_depth", {
        scroll_percentage: Math.min(depthPercentage, 100),
        page_title: document.title,
        timestamp: new Date().toISOString(),
      });
      debugLog(
        `📊 Analytics: Scroll depth - ${Math.min(depthPercentage, 100)}%`
      );
    }
  }
};

/**
 * Track time spent on a specific page
 * @param pageTitle - Title of the page
 * @param secondsSpent - Number of seconds spent on the page
 * @param chapter - Optional chapter identifier
 */
export const trackTimeOnPage = (
  pageTitle: string,
  secondsSpent: number,
  chapter?: string
) => {
  if (typeof window !== "undefined" && window.gtag) {
    // Only track if user spent meaningful time (>5 seconds)
    if (secondsSpent > 5) {
      window.gtag("event", "page_engagement", {
        page_title: pageTitle,
        seconds_spent: Math.round(secondsSpent),
        chapter: chapter,
        timestamp: new Date().toISOString(),
      });
      debugLog(
        `📊 Analytics: Page engagement - ${pageTitle} (${Math.round(
          secondsSpent
        )}s)`
      );
    }
  }
};

/**
 * Track code block interactions (copy, view, etc.)
 * @param action - Action taken ('copy', 'expand', 'view')
 * @param codeLanguage - Programming language of the code block
 * @param chapter - Chapter where code block is located
 */
export const trackCodeInteraction = (
  action: "copy" | "expand" | "view",
  codeLanguage: string,
  chapter?: string
) => {
  if (typeof window !== "undefined" && window.gtag) {
    window.gtag("event", "code_interaction", {
      action: action,
      language: codeLanguage,
      chapter: chapter,
      timestamp: new Date().toISOString(),
    });
    debugLog(`📊 Analytics: Code ${action} - ${codeLanguage}`);
  }
};

/**
 * Track quiz or assessment interactions
 * @param action - Action taken ('start', 'complete', 'answer')
 * @param quizTitle - Title of the quiz
 * @param score - Optional score achieved
 * @param chapter - Chapter containing the quiz
 */
export const trackQuizInteraction = (
  action: "start" | "complete" | "answer",
  quizTitle: string,
  score?: number,
  chapter?: string
) => {
  if (typeof window !== "undefined" && window.gtag) {
    window.gtag("event", "quiz_interaction", {
      action: action,
      quiz_title: quizTitle,
      score: score,
      chapter: chapter,
      timestamp: new Date().toISOString(),
    });
    debugLog(`📊 Analytics: Quiz ${action} - ${quizTitle}`);
  }
};

/**
 * Track file or resource downloads
 * @param fileName - Name of the file downloaded
 * @param fileType - Type of file (pdf, code, notebook, etc.)
 * @param chapter - Chapter where download was initiated
 */
export const trackDownload = (
  fileName: string,
  fileType: "pdf" | "code" | "notebook" | "image" | "other",
  chapter?: string
) => {
  if (typeof window !== "undefined" && window.gtag) {
    window.gtag("event", "file_download", {
      file_name: fileName,
      file_type: fileType,
      chapter: chapter,
      timestamp: new Date().toISOString(),
    });
    debugLog(`📊 Analytics: Downloaded - ${fileName}`);
  }
};

/**
 * Track learner segment/cohort
 * Helps segment users (e.g., 'beginner', 'experienced', 'educator')
 * @param segment - Learner segment identifier
 */
export const trackLearnerSegment = (segment: string) => {
  if (typeof window !== "undefined" && window.gtag) {
    window.gtag("set", { user_segment: segment });
    window.gtag("event", "learner_segment", {
      segment: segment,
      timestamp: new Date().toISOString(),
    });
    debugLog(`📊 Analytics: Learner segment - ${segment}`);
  }
};

/**
 * Track custom event with flexible properties
 * Use this for tracking events not covered by other functions
 * @param eventName - Name of the custom event
 * @param eventData - Object containing event properties
 */
export const trackCustomEvent = (
  eventName: string,
  eventData: Record<string, any>
) => {
  if (typeof window !== "undefined" && window.gtag) {
    window.gtag("event", eventName, {
      ...eventData,
      timestamp: new Date().toISOString(),
    });
    debugLog(`📊 Analytics: ${eventName} - ${JSON.stringify(eventData)}`);
  }
};
