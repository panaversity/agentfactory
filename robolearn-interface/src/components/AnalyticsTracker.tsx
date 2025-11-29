/**
 * AnalyticsTracker Component
 *
 * A React component that automatically tracks user interactions:
 * - Page views and navigation
 * - Scroll depth (how far users read)
 * - External link clicks (GitHub, YouTube, etc.)
 * - Time spent on page
 *
 * Usage:
 * Wrap your app or page layout with this component:
 * <AnalyticsTracker>
 *   <YourPageContent />
 * </AnalyticsTracker>
 */

import React, { useEffect, useRef, useState } from "react";
import { useLocation } from "@docusaurus/router";
import {
  trackScrollDepth,
  trackResourceClick,
  trackTimeOnPage,
  trackChapterView,
} from "@/utils/analytics";

interface AnalyticsTrackerProps {
  children: React.ReactNode;
  chapterTitle?: string;
  part?: string;
  chapterNumber?: number;
}

export const AnalyticsTracker: React.FC<AnalyticsTrackerProps> = ({
  children,
  chapterTitle,
  part,
  chapterNumber,
}) => {
  const location = useLocation();
  const startTimeRef = useRef<number>(Date.now());
  const [lastScrollDepth, setLastScrollDepth] = useState<number>(0);
  const [trackedScrollMilestones, setTrackedScrollMilestones] = useState<
    Set<number>
  >(new Set());

  // Track page view and chapter view on mount/navigation
  useEffect(() => {
    startTimeRef.current = Date.now();
    setLastScrollDepth(0);
    setTrackedScrollMilestones(new Set());

    // Announce page view to GA4
    if (typeof window !== "undefined" && window.gtag) {
      window.gtag("event", "page_view", {
        page_path: location.pathname,
        page_title: document.title,
      });
    }

    // Track chapter view if metadata provided
    if (chapterTitle && part && chapterNumber !== undefined) {
      trackChapterView(chapterNumber, chapterTitle, part);
    }

    // Debug log only in development
    if (process.env.NODE_ENV === "development") {
      console.log(`📍 Navigated to: ${location.pathname}`);
    }
  }, [location, chapterTitle, part, chapterNumber]);

  // Handle scroll tracking
  useEffect(() => {
    let scrollTimeout: NodeJS.Timeout;

    const handleScroll = () => {
      clearTimeout(scrollTimeout);

      scrollTimeout = setTimeout(() => {
        const scrollHeight =
          document.documentElement.scrollHeight - window.innerHeight;
        const scrollPercentage =
          scrollHeight > 0 ? (window.scrollY / scrollHeight) * 100 : 0;

        const roundedPercentage = Math.round(scrollPercentage / 10) * 10; // Round to nearest 10%

        // Track at 25%, 50%, 75%, 100% milestones
        if (
          !trackedScrollMilestones.has(roundedPercentage) &&
          roundedPercentage % 25 === 0
        ) {
          const newMilestones = new Set(trackedScrollMilestones);
          newMilestones.add(roundedPercentage);
          setTrackedScrollMilestones(newMilestones);

          trackScrollDepth(Math.min(roundedPercentage, 100));
        }

        setLastScrollDepth(roundedPercentage);
      }, 300); // Debounce scroll tracking
    };

    window.addEventListener("scroll", handleScroll, { passive: true });

    return () => {
      window.removeEventListener("scroll", handleScroll);
      clearTimeout(scrollTimeout);
    };
  }, [trackedScrollMilestones]);

  // Handle outbound link tracking
  useEffect(() => {
    const handleClick = (event: MouseEvent) => {
      const target = event.target as HTMLElement;
      const link = target.closest("a") as HTMLAnchorElement;

      if (link && link.href) {
        const href = link.href;
        const isExternal = !href.includes(window.location.hostname);

        if (isExternal) {
          // Determine resource type
          let resourceType: "github" | "youtube" | "docs" | "external" =
            "external";

          if (href.includes("github.com")) {
            resourceType = "github";
          } else if (
            href.includes("youtube.com") ||
            href.includes("youtu.be")
          ) {
            resourceType = "youtube";
          }

          trackResourceClick(resourceType, href, link.textContent || undefined);

          // For outbound links, add a small delay to ensure GA tracks before navigation
          if (!link.getAttribute("target")) {
            event.preventDefault();
            setTimeout(() => {
              window.location.href = href;
            }, 200);
          }
        }
      }
    };

    document.addEventListener("click", handleClick);
    return () => document.removeEventListener("click", handleClick);
  }, []);

  // Track time on page on component unmount
  useEffect(() => {
    return () => {
      const timeSpent = (Date.now() - startTimeRef.current) / 1000; // Convert to seconds
      if (timeSpent > 0) {
        trackTimeOnPage(document.title, timeSpent, chapterTitle);
      }
    };
  }, [chapterTitle]);

  // Render scroll depth indicator in development mode
  const isDevelopment = process.env.NODE_ENV === "development";

  return (
    <>
      {children}
      {isDevelopment && (
        <div
          style={{
            position: "fixed",
            bottom: "20px",
            right: "20px",
            backgroundColor: "rgba(0, 0, 0, 0.7)",
            color: "#fff",
            padding: "10px 15px",
            borderRadius: "5px",
            fontSize: "12px",
            zIndex: 9999,
            fontFamily: "monospace",
            pointerEvents: "none",
          }}
        >
          📊 Scroll: {Math.round(lastScrollDepth)}%
        </div>
      )}
    </>
  );
};

export default AnalyticsTracker;
