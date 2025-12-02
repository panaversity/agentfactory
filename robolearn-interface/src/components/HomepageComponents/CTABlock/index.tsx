import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from '@/contexts/AuthContext';
import { getHomeUrl } from '@/lib/url-utils';
import { useScrollReveal } from '../../../hooks/useScrollReveal';
import styles from './styles.module.css';

interface CTABlockProps {
  /** Section label text */
  label?: string;
  /** Main headline */
  title?: string;
  /** Title accent/subtitle */
  titleAccent?: string;
  /** Description text */
  description?: string;
  /** CTA icon (emoji or component) */
  ctaIcon?: React.ReactNode;
  /** CTA card title */
  ctaTitle?: string;
  /** CTA card description */
  ctaDescription?: string;
  /** CTA button text */
  buttonText?: string;
  /** CTA button link */
  buttonLink?: string;
}

/**
 * CTABlock - Final call-to-action section with Industrial-Kinetic styling
 *
 * Features:
 * - Gradient background with grid pattern overlay
 * - Glow button with cyan → amber transition
 * - Scroll-triggered reveal animation
 */
export function CTABlock({
  label = 'Ready to Begin?',
  title = 'The Future is Physical AI',
  titleAccent = 'Robots That Think, Move, and Collaborate',
  description = 'Join the transition from AI confined to screens to AI that shapes the physical world alongside us.',
  ctaIcon = '🤖',
  ctaTitle = 'Start Your Physical AI Journey',
  ctaDescription = 'From ROS 2 basics to autonomous humanoids with voice control',
  buttonText = 'Get Started Free',
  buttonLink,
}: CTABlockProps): React.ReactElement {
  const { siteConfig } = useDocusaurusContext();
  const { session } = useAuth();
  const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3001';
  
  // Determine button link based on auth status
  const getButtonLink = () => {
    if (buttonLink) return buttonLink;
    // If user is logged in, redirect to docs
    if (session?.user) {
      const homeUrl = getHomeUrl();
      return `${homeUrl}docs/preface-agent-native`;
    }
    // If not logged in, go to sign-up
    return `${authUrl}/auth/sign-up`;
  };
  
  const finalButtonLink = getButtonLink();

  const { ref, isVisible } = useScrollReveal<HTMLElement>();

  return (
    <section
      ref={ref}
      className={`${styles.ctaSection} ${isVisible ? styles.visible : ''}`}
      aria-labelledby="cta-title"
    >
      {/* Grid pattern overlay */}
      <div className={styles.gridOverlay} aria-hidden="true" />

      {/* Gradient background */}
      <div className={styles.gradientBg} aria-hidden="true" />

      <div className={styles.container}>
        {/* Header */}
        <div className={styles.header}>
          <span className={styles.label}>{label}</span>
          <h2 id="cta-title" className={styles.title}>
            {title}
            <br />
            <span className={styles.titleAccent}>{titleAccent}</span>
          </h2>
          <p className={styles.description}>{description}</p>
        </div>

        {/* CTA Card */}
        <div className={styles.ctaCard}>
          <div className={styles.ctaContent}>
            <span className={styles.ctaIcon} aria-hidden="true">{ctaIcon}</span>
            <div className={styles.ctaText}>
              <h3 className={styles.ctaTitle}>{ctaTitle}</h3>
              <p className={styles.ctaDescription}>{ctaDescription}</p>
            </div>
            <a href={finalButtonLink} className={styles.ctaButton}>
              <span className={styles.buttonText}>{buttonText}</span>
              <span className={styles.buttonShine} aria-hidden="true" />
            </a>
          </div>
        </div>
      </div>
    </section>
  );
}

export default CTABlock;
