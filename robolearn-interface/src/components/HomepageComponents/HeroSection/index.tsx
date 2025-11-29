import React, { useEffect, useState, useRef } from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

/**
 * HeroSection - Industrial Confidence Design
 *
 * Premium, engineering-focused hero that communicates:
 * - Serious platform for serious builders
 * - The tech stack (ROS 2, Isaac, Gazebo, VLA)
 * - Free but not cheap - confident value proposition
 */
export function HeroSection(): React.ReactElement {
  const [isVisible, setIsVisible] = useState(false);
  const [mousePos, setMousePos] = useState({ x: 0, y: 0 });
  const heroRef = useRef<HTMLElement>(null);

  useEffect(() => {
    setIsVisible(true);
  }, []);

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (!heroRef.current) return;
      const rect = heroRef.current.getBoundingClientRect();
      setMousePos({
        x: (e.clientX - rect.left) / rect.width,
        y: (e.clientY - rect.top) / rect.height,
      });
    };

    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, []);

  return (
    <section
      ref={heroRef}
      className={`${styles.hero} ${isVisible ? styles.visible : ''}`}
      aria-labelledby="hero-title"
    >
      {/* Gradient orb that follows mouse subtly */}
      <div
        className={styles.gradientOrb}
        style={{
          transform: `translate(${mousePos.x * 100 - 50}px, ${mousePos.y * 50 - 25}px)`,
        }}
      />

      {/* Grid pattern background */}
      <div className={styles.gridPattern} aria-hidden="true" />

      <div className={styles.heroContent}>
        {/* Left: Value Proposition */}
        <div className={styles.textBlock}>
          {/* Brand mark */}
          <div className={styles.brandMark}>
            <span className={styles.brandLogo}>ROBOLEARN</span>
            <span className={styles.brandTag}>PLATFORM</span>
          </div>

          {/* Headline */}
          <h1 id="hero-title" className={styles.headline}>
            <span className={styles.headlineLine}>Build robots that</span>
            <span className={styles.headlineLine}>
              <span className={styles.headlineAccent}>understand</span>
            </span>
            <span className={styles.headlineLine}>the physical world.</span>
          </h1>

          {/* Subheadline */}
          <p className={styles.subheadline}>
            Master Physical AI from browser to production.
            ROS 2, Isaac Sim, and Vision-Language-Action models.
            <span className={styles.freeTag}>Free forever.</span>
          </p>

          {/* CTAs */}
          <div className={styles.ctaGroup}>
            <Link to="/docs/preface-agent-native" className={styles.primaryCta}>
              <span>Begin Building</span>
              <svg className={styles.ctaIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M5 12h14M12 5l7 7-7 7" />
              </svg>
            </Link>
            <a href="#hardware-heading" className={styles.secondaryCta}>
              <span>Explore the Stack</span>
            </a>
          </div>

          {/* Tech stack pills */}
          <div className={styles.techStack}>
            <div className={styles.stackItem}>
              <span className={styles.stackDot} />
              <span>ROS 2</span>
            </div>
            <div className={styles.stackItem}>
              <span className={styles.stackDot} />
              <span>Isaac Sim</span>
            </div>
            <div className={styles.stackItem}>
              <span className={styles.stackDot} />
              <span>Gazebo</span>
            </div>
            <div className={styles.stackItem}>
              <span className={styles.stackDot} />
              <span>VLA Models</span>
            </div>
          </div>
        </div>

        {/* Right: Robot Assembly Visual */}
        <div className={styles.visualBlock}>
          <div className={styles.assemblyContainer}>
            {/* Isometric robot parts */}
            <svg
              viewBox="0 0 400 400"
              className={styles.robotAssembly}
              aria-label="Robot assembly visualization"
            >
              <defs>
                <linearGradient id="metalGradient" x1="0%" y1="0%" x2="100%" y2="100%">
                  <stop offset="0%" stopColor="#2a2d35" />
                  <stop offset="50%" stopColor="#1a1c21" />
                  <stop offset="100%" stopColor="#0f1115" />
                </linearGradient>
                <linearGradient id="accentGradient" x1="0%" y1="0%" x2="100%" y2="100%">
                  <stop offset="0%" stopColor="#22d3ee" />
                  <stop offset="100%" stopColor="#0891b2" />
                </linearGradient>
                <filter id="glow">
                  <feGaussianBlur stdDeviation="4" result="coloredBlur" />
                  <feMerge>
                    <feMergeNode in="coloredBlur" />
                    <feMergeNode in="SourceGraphic" />
                  </feMerge>
                </filter>
              </defs>

              {/* Connection lines */}
              <g className={styles.connectionLines}>
                <line x1="200" y1="120" x2="200" y2="180" stroke="#22d3ee" strokeWidth="1" opacity="0.3" />
                <line x1="120" y1="200" x2="180" y2="200" stroke="#22d3ee" strokeWidth="1" opacity="0.3" />
                <line x1="220" y1="200" x2="280" y2="200" stroke="#22d3ee" strokeWidth="1" opacity="0.3" />
                <line x1="200" y1="220" x2="200" y2="280" stroke="#22d3ee" strokeWidth="1" opacity="0.3" />
              </g>

              {/* Head/Brain - ROS 2 */}
              <g className={`${styles.part} ${styles.partHead}`}>
                <rect x="160" y="60" width="80" height="60" rx="8" fill="url(#metalGradient)" stroke="#22d3ee" strokeWidth="1" opacity="0.5" />
                <rect x="175" y="75" width="50" height="30" rx="4" fill="#0f1115" />
                <circle cx="190" cy="90" r="6" fill="#22d3ee" filter="url(#glow)" className={styles.eyeLeft} />
                <circle cx="210" cy="90" r="6" fill="#22d3ee" filter="url(#glow)" className={styles.eyeRight} />
                <text x="200" y="50" textAnchor="middle" fill="#8b8b8b" fontSize="10" fontFamily="monospace">ROS 2</text>
              </g>

              {/* Torso - Core */}
              <g className={`${styles.part} ${styles.partTorso}`}>
                <rect x="150" y="140" width="100" height="80" rx="8" fill="url(#metalGradient)" stroke="#22d3ee" strokeWidth="1" opacity="0.5" />
                <circle cx="200" cy="180" r="15" fill="#0f1115" stroke="#22d3ee" strokeWidth="2" />
                <circle cx="200" cy="180" r="8" fill="#22d3ee" filter="url(#glow)" className={styles.corePulse} />
              </g>

              {/* Left Arm - Isaac Sim */}
              <g className={`${styles.part} ${styles.partLeftArm}`}>
                <rect x="70" y="160" width="60" height="40" rx="6" fill="url(#metalGradient)" stroke="#22d3ee" strokeWidth="1" opacity="0.5" />
                <rect x="80" y="210" width="40" height="50" rx="4" fill="url(#metalGradient)" stroke="#22d3ee" strokeWidth="1" opacity="0.3" />
                <text x="100" y="150" textAnchor="middle" fill="#8b8b8b" fontSize="10" fontFamily="monospace">ISAAC</text>
              </g>

              {/* Right Arm - Gazebo */}
              <g className={`${styles.part} ${styles.partRightArm}`}>
                <rect x="270" y="160" width="60" height="40" rx="6" fill="url(#metalGradient)" stroke="#22d3ee" strokeWidth="1" opacity="0.5" />
                <rect x="280" y="210" width="40" height="50" rx="4" fill="url(#metalGradient)" stroke="#22d3ee" strokeWidth="1" opacity="0.3" />
                <text x="300" y="150" textAnchor="middle" fill="#8b8b8b" fontSize="10" fontFamily="monospace">GAZEBO</text>
              </g>

              {/* Legs - VLA */}
              <g className={`${styles.part} ${styles.partLegs}`}>
                <rect x="160" y="240" width="35" height="70" rx="6" fill="url(#metalGradient)" stroke="#22d3ee" strokeWidth="1" opacity="0.5" />
                <rect x="205" y="240" width="35" height="70" rx="6" fill="url(#metalGradient)" stroke="#22d3ee" strokeWidth="1" opacity="0.5" />
                <text x="200" y="330" textAnchor="middle" fill="#8b8b8b" fontSize="10" fontFamily="monospace">VLA MODELS</text>
              </g>

              {/* Data flow particles */}
              <g className={styles.dataFlow}>
                <circle cx="200" cy="130" r="2" fill="#22d3ee" className={styles.particle1} />
                <circle cx="140" cy="180" r="2" fill="#22d3ee" className={styles.particle2} />
                <circle cx="260" cy="180" r="2" fill="#22d3ee" className={styles.particle3} />
                <circle cx="200" cy="230" r="2" fill="#22d3ee" className={styles.particle4} />
              </g>
            </svg>

            {/* Floating labels */}
            <div className={styles.floatingLabel} style={{ top: '10%', right: '10%' }}>
              <span className={styles.labelPulse} />
              <span>Simulation Ready</span>
            </div>
            <div className={styles.floatingLabel} style={{ bottom: '15%', left: '5%' }}>
              <span className={styles.labelPulse} />
              <span>No Hardware Needed</span>
            </div>
          </div>
        </div>
      </div>

      {/* Bottom fade */}
      <div className={styles.bottomFade} />
    </section>
  );
}

export default HeroSection;
