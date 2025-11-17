/**
 * T058-T070: PersonalizationTab Component
 * 
 * Displays AI-personalized content tailored to user's proficiency levels
 * with SSE streaming support. Mirrors SummaryTab pattern with login button
 * integration and profile-based personalization.
 */
import React, { useState, useEffect, useRef } from "react";
import { useHistory, useLocation } from "@docusaurus/router";
import { PersonalizationCacheEntry, UserProfile } from "../../types/contentTabs";
import * as authService from "../../services/authService";
import * as cacheService from "../../services/cacheService";
import { personalizationService } from "../../services/personalizationService";
import styles from "./styles.module.css";

interface PersonalizationTabProps {
  pageId: string;
  content: string;
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
    const authenticated = authService.isAuthenticated();
    setIsAuthenticated(authenticated);

    if (authenticated) {
      const profile = authService.getProfile();
      setUserProfile(profile);

      if (profile) {
        checkCacheAndGenerate(profile);
      }
    }
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
    const fingerprint = authService.generateProfileFingerprint(profile);
    const cacheKey = `personalized_${pageId}_${fingerprint}`;
    const cached = cacheService.get<PersonalizationCacheEntry>(cacheKey);

    if (cached && cached.personalizedContent) {
      console.log(`✅ Cache hit for ${pageId} (${fingerprint}) - displaying cached content`);
      setPersonalizedContent(cached.personalizedContent);
      setIsCached(true);
      return;
    }

    console.log(`❌ Cache miss for ${pageId} (${fingerprint}) - generating personalized content`);
    setIsCached(false);

    // Request deduplication
    if (generatingRef.current) {
      return;
    }

    // Generate new personalized content
    await generatePersonalizedContent(profile);
  };

  // T063: Generate personalized content with streaming
  const generatePersonalizedContent = async (profile: UserProfile) => {
    const token = authService.getToken();
    if (!token) {
      setError("Authentication required");
      return;
    }

    // Request deduplication
    if (generatingRef.current) {
      return;
    }

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
        pageId,
        content,
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
          console.log(`✅ Personalization completed for ${pageId}`);
          setPersonalizedContent(accumulatedContent);
          setIsGenerating(false);
          setIsLoading(false);
          generatingRef.current = false;

          // Cache personalized content with profile fingerprint
          const fingerprint = authService.generateProfileFingerprint(profile);
          const cacheKey = `personalized_${pageId}_${fingerprint}`;
          const cacheEntry: PersonalizationCacheEntry = {
            personalizedContent: accumulatedContent,
            pageId,
            profileFingerprint: fingerprint,
            timestamp: Date.now(),
          };
          cacheService.set(cacheKey, cacheEntry);
          console.log(`💾 Cached personalized content for ${pageId} (${fingerprint})`);
        },
        (errorMsg) => {
          // T066: Error handling with partial content preservation
          console.error(`❌ Personalization error for ${pageId}:`, errorMsg);
          
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

  // T067: Handle regenerate button click
  const handleRegenerate = () => {
    if (userProfile) {
      setIsCached(false);
      generatePersonalizedContent(userProfile);
    }
  };

  // T068: Handle login button click
  const handleLoginClick = () => {
    const returnTo = encodeURIComponent(location.pathname);
    history.push(`/login?returnTo=${returnTo}`);
  };

  // T069: Render login button if not authenticated
  if (!isAuthenticated) {
    return (
      <div className={styles.tabContent}>
        <div className={styles.loginContainer}>
          <p>Sign in to view personalized content tailored to your experience level.</p>
          <button 
            className={styles.loginButton}
            onClick={handleLoginClick}
          >
            Login to See Personalized Content
          </button>
        </div>
      </div>
    );
  }

  // T070: Main content rendering
  return (
    <div className={styles.tabContent}>
      {/* Profile info badge */}
      {userProfile && (
        <div className={styles.profileBadge}>
          <span className={styles.badgeLabel}>Personalized for:</span>
          <span className={styles.badgeValue}>
            {userProfile.programmingExperience} Programming, {userProfile.aiProficiency} AI
          </span>
        </div>
      )}

      {/* Generate button */}
      {!personalizedContent && !isGenerating && !isLoading && (
        <div className={styles.generateContainer}>
          <button
            className={styles.generateButton}
            onClick={() => userProfile && generatePersonalizedContent(userProfile)}
            disabled={!userProfile}
          >
            Generate Personalized Content
          </button>
        </div>
      )}

      {/* Loading state */}
      {isLoading && (
        <div className={styles.loadingContainer}>
          <div className={styles.spinner}></div>
          <p>Personalizing content for your experience level...</p>
        </div>
      )}

      {/* Error message */}
      {error && (
        <div className={styles.errorMessage}>
          <strong>Error:</strong> {error}
        </div>
      )}

      {/* Streaming content */}
      {(isGenerating || personalizedContent) && (
        <div className={styles.contentWrapper}>
          {/* Cache indicator */}
          {isCached && !isGenerating && (
            <div className={styles.cacheIndicator}>
              📋 Cached content
            </div>
          )}

          {/* Streaming indicator */}
          {isGenerating && (
            <div className={styles.streamingIndicator}>
              ✨ Generating personalized content...
            </div>
          )}

          {/* Content display */}
          <div
            ref={contentContainerRef}
            className={`${styles.contentContainer} ${isGenerating ? styles.streaming : ''}`}
          >
            <div className={styles.personalizedContent}>
              {isGenerating ? streamingText : personalizedContent}
            </div>
            <div ref={contentEndRef} />
          </div>

          {/* Regenerate button */}
          {!isGenerating && personalizedContent && (
            <div className={styles.regenerateContainer}>
              <button
                className={styles.regenerateButton}
                onClick={handleRegenerate}
              >
                🔄 Regenerate
              </button>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
