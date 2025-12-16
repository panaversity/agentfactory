import React, { ReactNode, useState, useEffect } from 'react';
import { useAuth } from '@/contexts/AuthContext';
import { getOAuthAuthorizationUrl } from '@/lib/auth-client';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './ContentGate.module.css';

export type GateType = 'quiz' | 'summary' | 'exercise' | 'premium';

interface ContentGateProps {
  children?: ReactNode;
  type: GateType;
  /** Preview content to show in blurred state (optional - defaults to children) */
  preview?: ReactNode;
  /** Custom title for the gate */
  title?: string;
  /** Custom description for the gate */
  description?: string;
  /** Whether to show the gate even when authenticated (for testing) */
  forceGate?: boolean;
}

// Gate configuration per content type
const gateConfig: Record<GateType, {
  title: string;
  description: string;
  icon: string;
  badge: string;
  benefits: string[];
}> = {
  quiz: {
    title: 'Unlock Chapter Quiz',
    description: 'Test your understanding with interactive questions and get instant feedback on your progress.',
    icon: '🎯',
    badge: 'Chapter Quiz',
    benefits: [
      'Track your learning progress',
      'Get detailed explanations',
      'Retake anytime to improve',
    ],
  },
  summary: {
    title: 'Unlock Lesson Summary',
    description: 'Access condensed key takeaways and quick reference notes for efficient review.',
    icon: '📋',
    badge: 'Quick Reference',
    benefits: [
      'Key concepts at a glance',
      'Perfect for revision',
      'Save study time',
    ],
  },
  exercise: {
    title: 'Unlock Practice Exercise',
    description: 'Get hands-on with coding exercises and guided solutions to build real skills.',
    icon: '💻',
    badge: 'Hands-On Practice',
    benefits: [
      'Real-world scenarios',
      'Step-by-step guidance',
      'Build portfolio projects',
    ],
  },
  premium: {
    title: 'Unlock Premium Content',
    description: 'Access exclusive learning materials designed to accelerate your journey.',
    icon: '✨',
    badge: 'Premium',
    benefits: [
      'Exclusive resources',
      'Advanced techniques',
      'Expert insights',
    ],
  },
};

// Lock SVG icon
const LockIcon = () => (
  <svg
    className={styles.lockIcon}
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <rect x="3" y="11" width="18" height="11" rx="2" ry="2" />
    <path d="M7 11V7a5 5 0 0 1 10 0v4" />
  </svg>
);

// Check SVG icon
const CheckIcon = () => (
  <svg
    className={styles.checkIcon}
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2.5"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <polyline points="20 6 9 17 4 12" />
  </svg>
);

// Arrow icon for button
const ArrowIcon = () => (
  <svg
    className={styles.buttonIcon}
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <path d="M5 12h14" />
    <path d="m12 5 7 7-7 7" />
  </svg>
);

export function ContentGate({
  children,
  type,
  preview,
  title,
  description,
  forceGate = false,
}: ContentGateProps) {
  const { session, isLoading } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3001';
  const oauthClientId = (siteConfig.customFields?.oauthClientId as string) || 'ai-native-public-client';

  const [isUnlocking, setIsUnlocking] = useState(false);
  const [isSigningIn, setIsSigningIn] = useState(false);

  const config = gateConfig[type];
  const displayTitle = title || config.title;
  const displayDescription = description || config.description;

  // OAuth config
  const oauthConfig = {
    authUrl,
    clientId: oauthClientId,
  };

  // Handle sign in
  const handleSignIn = async () => {
    setIsSigningIn(true);
    try {
      const authorizationUrl = await getOAuthAuthorizationUrl('signin', oauthConfig);
      await new Promise(resolve => setTimeout(resolve, 50));
      window.location.href = authorizationUrl;
    } catch (error) {
      console.error('Sign in failed:', error);
      setIsSigningIn(false);
    }
  };

  // Handle sign up
  const handleSignUp = async () => {
    setIsSigningIn(true);
    try {
      const oauthUrl = await getOAuthAuthorizationUrl('signup', oauthConfig);
      const signupUrl = `${authUrl}/auth/sign-up?redirect=${encodeURIComponent(oauthUrl)}`;
      window.location.href = signupUrl;
    } catch (error) {
      console.error('Sign up failed:', error);
      setIsSigningIn(false);
    }
  };

  // Unlock animation when session becomes available
  useEffect(() => {
    if (session && !isLoading) {
      setIsUnlocking(true);
      const timer = setTimeout(() => setIsUnlocking(false), 600);
      return () => clearTimeout(timer);
    }
  }, [session, isLoading]);

  // Show loading state
  if (isLoading) {
    return (
      <div className={styles.gateContainer}>
        <div className={styles.loadingState}>
          <div className={styles.loadingSpinner} />
          <span>Checking access...</span>
        </div>
      </div>
    );
  }

  // User is authenticated - show content (with optional unlock animation)
  if (session && !forceGate) {
    return (
      <div className={`${styles.contentWrapper} ${isUnlocking ? styles.unlocking : ''}`}>
        {children}
      </div>
    );
  }

  // User is not authenticated - show gate
  return (
    <div className={styles.gateContainer}>
      <div className={styles.gateLayout}>
        {/* Left side - Blurred preview */}
        <div className={styles.previewSection}>
          <div className={styles.blurredContent} aria-hidden="true">
            {preview || children}
          </div>
          <div className={styles.previewOverlay}>
            <div className={styles.lockBadge}>
              <LockIcon />
            </div>
          </div>
        </div>

        {/* Right side - CTA */}
        <div className={styles.ctaSection}>
          <div className={styles.contentBadge}>
            <span className={styles.badgeIcon}>{config.icon}</span>
            <span>{config.badge}</span>
          </div>

          <h3 className={styles.gateTitle}>{displayTitle}</h3>

          <p className={styles.gateDescription}>{displayDescription}</p>

          <ul className={styles.benefitsList}>
            {config.benefits.map((benefit, index) => (
              <li key={index} className={styles.benefitItem}>
                <CheckIcon />
                <span>{benefit}</span>
              </li>
            ))}
          </ul>

          <div className={styles.gateActions}>
            <button
              onClick={handleSignIn}
              className={styles.signInButton}
              disabled={isSigningIn}
            >
              {isSigningIn ? (
                <>
                  <div className={styles.buttonSpinner} />
                  Signing in...
                </>
              ) : (
                <>
                  Sign In to Unlock
                  <ArrowIcon />
                </>
              )}
            </button>

            <button
              onClick={handleSignUp}
              className={styles.signUpButton}
              disabled={isSigningIn}
            >
              Create Free Account
            </button>
          </div>

          <p className={styles.gateFooter}>
            Free forever. No credit card required.
          </p>
        </div>
      </div>
    </div>
  );
}

export default ContentGate;
