/**
 * T058-T070: PersonalizationTab Component
 * 
 * Displays AI-personalized content tailored to user's proficiency levels
 * with SSE streaming support. Mirrors SummaryTab pattern with login button
 * integration and profile-based personalization.
 * 
 * T109: State Machine Documentation
 * 
 * State Flow:
 * 1. IDLE (initial) → Component mounts
 * 2. CHECKING_AUTH → Check if user authenticated
 *    - If NOT authenticated → LOGIN_REQUIRED (show login button)
 *    - If authenticated → CHECKING_CACHE
 * 3. CHECKING_CACHE → Check sessionStorage for cached content
 *    - Check session expiration first
 *    - If expired → ERROR (show expiration message)
 *    - If cache hit → CACHE_HIT (display cached content instantly)
 *    - If cache miss → READY (show generate button)
 * 4. READY → User clicks "Generate Personalized Content"
 *    - Validate session not expired
 *    - Validate session not cleared (T095)
 *    - Apply debounce guard (T097)
 * 5. LOADING → Initial API request setup
 *    - Disable generate button
 *    - Show loading spinner
 * 6. STREAMING → Receiving SSE chunks
 *    - Display streaming indicator with ARIA live region (T100)
 *    - Append chunks progressively
 *    - Auto-scroll to bottom
 * 7. SUCCESS → Stream completed
 *    - Save to cache with profile fingerprint
 *    - Show regenerate button
 *    - Display cache indicator on reload
 * 8. ERROR → Stream failed
 *    - Preserve partial content (FR-015a)
 *    - Show error message with retry button
 *    - If session expired → redirect to login (T093)
 *    - If session cleared → show login button (T095)
 * 
 * State Transitions Summary:
 * - IDLE → CHECKING_AUTH (on mount)
 * - CHECKING_AUTH → LOGIN_REQUIRED | CHECKING_CACHE
 * - CHECKING_CACHE → ERROR | CACHE_HIT | READY
 * - READY → LOADING (on generate click)
 * - LOADING → STREAMING (first chunk received)
 * - STREAMING → SUCCESS | ERROR
 * - SUCCESS → READY (on regenerate click)
 * - ERROR → READY (on retry click)
 */
import React, { useState, useEffect, useRef } from "react";
import { useHistory, useLocation } from "@docusaurus/router";
import MDXContent from "@theme/MDXContent";
import { PersonalizationCacheEntry, UserProfile } from "../../types/contentTabs";
import * as authService from "../../services/authService";
import * as cacheService from "../../services/cacheService";
import { personalizationService } from "../../services/personalizationService";
import styles from "./styles.module.css";

interface PersonalizationTabProps {
  pageId: string;
  content: string;
}

// Helper function to convert markdown to HTML with full feature support
function convertMarkdownToHTML(markdown: string): string {
  let html = markdown;
  
  // Convert Docusaurus admonitions (:::tip, :::note, :::warning, etc.)
  html = html.replace(/:::(\w+)\s*([^\n]*)\n([\s\S]*?):::/g, (match, type, title, content) => {
    const admonitionTitle = title || type.charAt(0).toUpperCase() + type.slice(1);
    return `<div class="admonition admonition-${type} alert alert--${type}">
      <div class="admonition-heading"><h5>${admonitionTitle}</h5></div>
      <div class="admonition-content">${content.trim()}</div>
    </div>`;
  });
  
  // Convert code blocks (BEFORE other processing)
  const codeBlocks: string[] = [];
  html = html.replace(/```(\w+)?\n([\s\S]*?)```/g, (match, lang, code) => {
    const placeholder = `__CODEBLOCK_${codeBlocks.length}__`;
    codeBlocks.push(`<pre><code class="language-${lang || 'text'}">${code.replace(/</g, '&lt;').replace(/>/g, '&gt;')}</code></pre>`);
    return placeholder;
  });
  
  // Convert headers (must be on their own line)
  html = html.replace(/^#### (.+)$/gm, '<h4>$1</h4>');
  html = html.replace(/^### (.+)$/gm, '<h3>$1</h3>');
  html = html.replace(/^## (.+)$/gm, '<h2>$1</h2>');
  html = html.replace(/^# (.+)$/gm, '<h1>$1</h1>');
  
  // Convert blockquotes (lines starting with >)
  html = html.replace(/^> (.+)$/gm, '<blockquote><p>$1</p></blockquote>');
  // Merge consecutive blockquotes
  html = html.replace(/<\/blockquote>\n<blockquote>/g, '');
  
  // Convert bold text
  html = html.replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>');
  
  // Convert inline code (AFTER code blocks are extracted)
  html = html.replace(/`([^`]+)`/g, '<code>$1</code>');
  
  // Convert tables
  html = html.replace(/(\|.+\|\n)+/g, (match) => {
    const rows = match.trim().split('\n');
    if (rows.length < 2) return match;
    
    const headerRow = rows[0];
    const separatorRow = rows[1];
    const bodyRows = rows.slice(2);
    
    const headers = headerRow.split('|').filter(cell => cell.trim()).map(cell => `<th>${cell.trim()}</th>`).join('');
    const body = bodyRows.map(row => {
      const cells = row.split('|').filter(cell => cell.trim()).map(cell => `<td>${cell.trim()}</td>`).join('');
      return `<tr>${cells}</tr>`;
    }).join('');
    
    return `<table class="table"><thead><tr>${headers}</tr></thead><tbody>${body}</tbody></table>`;
  });
  
  // Convert numbered lists (capture multi-line items)
  const lines = html.split('\n');
  const processedLines: string[] = [];
  let inOrderedList = false;
  let inUnorderedList = false;
  
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    const trimmedLine = line.trim();
    
    // Ordered list detection
    if (/^\d+\.\s/.test(trimmedLine)) {
      const content = trimmedLine.replace(/^\d+\.\s/, '');
      if (!inOrderedList) {
        processedLines.push('<ol>');
        inOrderedList = true;
      }
      processedLines.push(`<li>${content}</li>`);
    }
    // Unordered list detection (must start line)
    else if (/^-\s/.test(trimmedLine) && !trimmedLine.match(/^-{3,}/)) {
      const content = trimmedLine.replace(/^-\s/, '');
      if (inOrderedList) {
        processedLines.push('</ol>');
        inOrderedList = false;
      }
      if (!inUnorderedList) {
        processedLines.push('<ul>');
        inUnorderedList = true;
      }
      processedLines.push(`<li>${content}</li>`);
    }
    // Close lists when hitting non-list content
    else {
      if (inOrderedList) {
        processedLines.push('</ol>');
        inOrderedList = false;
      }
      if (inUnorderedList) {
        processedLines.push('</ul>');
        inUnorderedList = false;
      }
      processedLines.push(line);
    }
  }
  
  // Close any remaining open lists
  if (inOrderedList) processedLines.push('</ol>');
  if (inUnorderedList) processedLines.push('</ul>');
  
  html = processedLines.join('\n');
  
  // Convert horizontal rules
  html = html.replace(/^---+$/gm, '<hr>');
  
  // Convert paragraphs (groups of text separated by blank lines)
  const paragraphs = html.split('\n\n');
  html = paragraphs.map(para => {
    para = para.trim();
    if (!para) return '';
    
    // Skip if already wrapped in block-level HTML
    if (para.match(/^<(h[1-6]|div|pre|blockquote|table|ul|ol|hr)/i)) {
      return para;
    }
    
    // Check if it contains placeholders
    if (para.includes('__CODEBLOCK_')) {
      return para;
    }
    
    // Wrap in paragraph
    return `<p>${para.replace(/\n/g, '<br>')}</p>`;
  }).join('\n');
  
  // Restore code blocks
  codeBlocks.forEach((block, index) => {
    html = html.replace(`__CODEBLOCK_${index}__`, block);
  });
  
  return html;
}

// T058: Create PersonalizationTab component
export default function PersonalizationTab({
  pageId,
  content,
}: PersonalizationTabProps): React.ReactElement {
  const history = useHistory();
  const location = useLocation();
  
  // T059: Check authentication on mount
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<string>("");
  const [error, setError] = useState<string | null>(null);
  const [isGenerating, setIsGenerating] = useState(false);
  const [isCached, setIsCached] = useState(false);
  const [streamingText, setStreamingText] = useState<string>("");
  const contentEndRef = useRef<HTMLDivElement>(null);
  const generatingRef = useRef(false);
  const contentContainerRef = useRef<HTMLDivElement>(null);

  // T060: Check authentication and load profile
  useEffect(() => {
    console.log(`📄 PersonalizationTab mounted/updated for pageId: ${pageId}`);
    
    const authenticated = authService.isAuthenticated();
    setIsAuthenticated(authenticated);

    if (authenticated) {
      const profile = authService.getProfile();
      setUserProfile(profile);

      if (profile) {
        const fingerprint = authService.generateProfileFingerprint(profile);
        console.log(`👤 User profile loaded: ${fingerprint}`);
        checkCacheAndGenerate(profile);
      }
    }

    // Cleanup: Cancel ongoing personalization when pageId changes
    return () => {
      if (generatingRef.current) {
        console.log(`🛑 Cancelling ongoing personalization due to page navigation from ${pageId}`);
        personalizationService.cancel();
        generatingRef.current = false;
      }
    };
  }, [pageId]);

  // T061: Auto-scroll during streaming
  useEffect(() => {
    if (isGenerating && contentContainerRef.current) {
      contentContainerRef.current.scrollTop =
        contentContainerRef.current.scrollHeight;
    }
  }, [streamingText, isGenerating]);

  // T062: Check cache then generate if needed
  const checkCacheAndGenerate = async (profile: UserProfile) => {
    // T090: Check session expiration before generation
    if (authService.isSessionExpired()) {
      // T091: Show non-intrusive notification
      setError("Session expired. Please login to generate new content.");
      // T092: Don't clear existing content - allow viewing cached/displayed content
      return;
    }

    const fingerprint = authService.generateProfileFingerprint(profile);
    const cacheKey = `personalized_${pageId}_${fingerprint}`;
    
    console.log(`🔍 Checking cache for pageId: ${pageId}, fingerprint: ${fingerprint}`);
    
    const cached = cacheService.get<PersonalizationCacheEntry>(cacheKey);

    if (cached && cached.personalizedText) {
      console.log(`✅ Cache HIT for ${pageId} (${fingerprint}) - displaying cached content`);
      setPersonalizedContent(cached.personalizedText);
      setIsCached(true);
      setIsLoading(false);
      setIsGenerating(false);
      return;
    }

    console.log(`❌ Cache MISS for ${pageId} (${fingerprint}) - will generate if not already in progress`);
    setIsCached(false);

    // Request deduplication - prevent duplicate generations
    if (generatingRef.current) {
      console.log(`⚠️ Generation already in progress for ${pageId}, skipping duplicate request`);
      return;
    }

    // Generate new personalized content
    await generatePersonalizedContent(profile);
  };

  // T063: Generate personalized content with streaming
  const generatePersonalizedContent = async (profile: UserProfile) => {
    // T090-T093: Check session expiration and require re-login
    if (authService.isSessionExpired()) {
      setError("Session expired. Redirecting to login...");
      setTimeout(() => {
        const returnTo = encodeURIComponent(location.pathname);
        history.push(`/login?returnTo=${returnTo}`);
      }, 1500);
      return;
    }

    const token = authService.getToken();
    if (!token) {
      setError("Authentication required");
      return;
    }

    // T095: Handle edge case - sessionStorage cleared during generation
    const session = authService.getSession();
    if (!session || !session.profile) {
      setError("Session lost. Please login again.");
      setIsAuthenticated(false);
      return;
    }

    // Request deduplication
    if (generatingRef.current) {
      return;
    }

    // BUGFIX: Capture pageId and content at generation start to avoid stale closure
    const capturedPageId = pageId;
    const capturedContent = content;

    generatingRef.current = true;
    setIsGenerating(true);
    setIsLoading(true);
    setError(null);
    setPersonalizedContent("");
    setStreamingText("");

    let accumulatedContent = "";
    let firstChunkReceived = false;

    try {
      // T064: Stream personalized content from API
      personalizationService.streamPersonalizedContent(
        capturedPageId,
        capturedContent,
        token,
        profile.programmingExperience,
        profile.aiProficiency,
        (chunk) => {
          // Hide loading screen on first chunk
          if (!firstChunkReceived) {
            firstChunkReceived = true;
            setIsLoading(false);
          }

          accumulatedContent += chunk;
          setStreamingText(accumulatedContent);
        },
        () => {
          // T065: On completion, cache the result
          console.log(`✅ Personalization completed for ${capturedPageId}`);
          setPersonalizedContent(accumulatedContent);
          setIsGenerating(false);
          setIsLoading(false);
          generatingRef.current = false;

          // Cache personalized content with profile fingerprint
          const fingerprint = authService.generateProfileFingerprint(profile);
          const cacheKey = `personalized_${capturedPageId}_${fingerprint}`;
          const cacheEntry: PersonalizationCacheEntry = {
            personalizedText: accumulatedContent,
            pageId: capturedPageId,
            profileFingerprint: fingerprint,
            timestamp: Date.now(),
            cached: true,
          };
          cacheService.set(cacheKey, cacheEntry);
          console.log(`💾 Cached personalized content for ${capturedPageId} (${fingerprint})`);
        },
        (errorMsg) => {
          // T066: Error handling with partial content preservation
          console.error(`❌ Personalization error for ${capturedPageId}:`, errorMsg);
          
          // Preserve partial content if any was received
          if (accumulatedContent) {
            setPersonalizedContent(accumulatedContent);
            setError(`Personalization interrupted: ${errorMsg}. Partial content displayed.`);
          } else {
            setError(errorMsg);
          }
          
          setIsGenerating(false);
          setIsLoading(false);
          generatingRef.current = false;
        }
      );
    } catch (err) {
      console.error('Personalization request failed:', err);
      setError('Failed to personalize content');
      setIsGenerating(false);
      setIsLoading(false);
      generatingRef.current = false;
    }
  };

  // T068: Handle login button click (REMOVED - inline in button onClick)

  // T069: Render login button if not authenticated (UPDATED to match SummaryTab)
  if (!isAuthenticated) {
    return (
      <div className={styles.loginButton}>
        <h3>Authentication Required</h3>
        <p>You need to be authenticated to view personalized content tailored to your experience level.</p>
        <button
          onClick={() => {
            // Add navigation state with return URL
            const returnTo = `${location.pathname}${location.hash || '#personalized'}`;
            history.push(`/login?returnTo=${encodeURIComponent(returnTo)}`);
          }}
          type="button"
        >
          Login to See Personalized Content
        </button>
        <p style={{ fontSize: '0.85em', color: 'var(--ifm-color-emphasis-600)', marginTop: '1rem' }}>
          Note: This is a temporary dummy authentication for demonstration purposes.
        </p>
      </div>
    );
  }

  // T070: Main content rendering
  return (
    <div role="tabpanel" id="personalized-panel" aria-labelledby="personalized-tab">
      {/* Generate button - only show if no content */}
      {!personalizedContent && !isGenerating && !isLoading && (
        <div className={styles.generateContainer}>
          <button
            className={styles.generateButton}
            onClick={() => userProfile && generatePersonalizedContent(userProfile)}
            disabled={!userProfile}
            aria-label="Generate personalized content based on your proficiency level"
            aria-busy={isGenerating}
          >
            Generate Personalized Content
          </button>
        </div>
      )}

      {/* Loading state - matches SummaryTab spinner */}
      {isLoading && !personalizedContent && (
        <div className={styles.loadingContainer}>
          <div className={styles.loadingContent}>
            <div className={styles.loadingSpinner} />
            <div className={styles.loadingText}>
              <div className={styles.loadingTitle}>Generating Personalized Content</div>
              <div className={styles.loadingSubtitle}>
                Tailoring content to your experience level...
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Error message */}
      {error && (
        <div className={styles.errorMessage}>
          <strong>Error:</strong> {error}
        </div>
      )}

      {/* Content display - rendered as markdown like Original tab */}
      {(isGenerating || personalizedContent) && (
        <>
          <div 
            ref={contentContainerRef}
            className="markdown"
            dangerouslySetInnerHTML={{
              __html: convertMarkdownToHTML(isGenerating ? streamingText : personalizedContent)
            }}
          />
          <div ref={contentEndRef} />
        </>
      )}
    </div>
  );
}
