import React, { useState, useEffect, useRef, useCallback } from 'react';
import styles from './styles.module.css';

/**
 * AnimatedRobot - Interactive robot with boot-up sequence and cursor tracking
 *
 * Features:
 * - Boot-up sequence: robot powers on with circuit illumination
 * - Cursor-tracking eyes: eyes follow mouse position
 * - Wave animation and speech bubbles
 * - Circuit line pulse effects
 */
export function AnimatedRobot(): React.ReactElement {
  const [bootPhase, setBootPhase] = useState<'off' | 'booting' | 'ready'>('off');
  const [isWaving, setIsWaving] = useState(false);
  const [eyeState, setEyeState] = useState<'normal' | 'happy' | 'blink'>('normal');
  const [speechBubble, setSpeechBubble] = useState<string | null>(null);
  const [eyeOffset, setEyeOffset] = useState({ x: 0, y: 0 });
  const containerRef = useRef<HTMLDivElement>(null);

  const speeches = [
    "Let's build together!",
    "No hardware needed!",
    "Start learning free!",
    "Ready to learn?",
  ];

  // Boot sequence on mount
  useEffect(() => {
    const bootTimeout = setTimeout(() => {
      setBootPhase('booting');
    }, 300);

    const readyTimeout = setTimeout(() => {
      setBootPhase('ready');
      setSpeechBubble("Ready to learn!");
      setTimeout(() => setSpeechBubble(null), 3000);
    }, 1500);

    return () => {
      clearTimeout(bootTimeout);
      clearTimeout(readyTimeout);
    };
  }, []);

  // Cursor tracking for eyes
  const handleMouseMove = useCallback((e: MouseEvent) => {
    if (!containerRef.current || bootPhase !== 'ready') return;

    const rect = containerRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 3; // Eyes are in upper third

    const deltaX = e.clientX - centerX;
    const deltaY = e.clientY - centerY;

    // Limit eye movement to 5 degrees (roughly 5px offset)
    const maxOffset = 5;
    const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    const normalizedX = distance > 0 ? (deltaX / distance) * Math.min(distance / 50, 1) * maxOffset : 0;
    const normalizedY = distance > 0 ? (deltaY / distance) * Math.min(distance / 50, 1) * maxOffset : 0;

    setEyeOffset({ x: normalizedX, y: normalizedY });
  }, [bootPhase]);

  useEffect(() => {
    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, [handleMouseMove]);

  // Periodic wave animation
  useEffect(() => {
    if (bootPhase !== 'ready') return;

    const waveInterval = setInterval(() => {
      setIsWaving(true);
      setEyeState('happy');
      setSpeechBubble(speeches[Math.floor(Math.random() * speeches.length)]);

      setTimeout(() => {
        setIsWaving(false);
        setEyeState('normal');
      }, 2000);

      setTimeout(() => {
        setSpeechBubble(null);
      }, 3000);
    }, 6000);

    return () => clearInterval(waveInterval);
  }, [bootPhase]);

  // Eye blink
  useEffect(() => {
    if (bootPhase !== 'ready') return;

    const blinkInterval = setInterval(() => {
      if (eyeState === 'normal') {
        setEyeState('blink');
        setTimeout(() => setEyeState('normal'), 150);
      }
    }, 3500);

    return () => clearInterval(blinkInterval);
  }, [eyeState, bootPhase]);

  const robotClass = `${styles.robot} ${styles[bootPhase]}`;

  return (
    <div className={styles.robotContainer} ref={containerRef}>
      {/* Speech bubble */}
      {speechBubble && bootPhase === 'ready' && (
        <div className={styles.speechBubble}>
          {speechBubble}
        </div>
      )}

      {/* Robot SVG */}
      <svg
        viewBox="0 0 200 280"
        className={robotClass}
        aria-label="Friendly humanoid robot mascot"
      >
        <defs>
          {/* Glow filter */}
          <filter id="glow" x="-50%" y="-50%" width="200%" height="200%">
            <feGaussianBlur stdDeviation="3" result="coloredBlur" />
            <feMerge>
              <feMergeNode in="coloredBlur" />
              <feMergeNode in="SourceGraphic" />
            </feMerge>
          </filter>

          {/* Strong glow for boot sequence */}
          <filter id="strongGlow" x="-100%" y="-100%" width="300%" height="300%">
            <feGaussianBlur stdDeviation="6" result="coloredBlur" />
            <feMerge>
              <feMergeNode in="coloredBlur" />
              <feMergeNode in="coloredBlur" />
              <feMergeNode in="SourceGraphic" />
            </feMerge>
          </filter>

          {/* Gradients */}
          <linearGradient id="bodyGradient" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stopColor="#4a5568" />
            <stop offset="100%" stopColor="#2d3748" />
          </linearGradient>
          <linearGradient id="accentGradient" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stopColor="#00f0ff" />
            <stop offset="100%" stopColor="#00b8cc" />
          </linearGradient>
          <linearGradient id="circuitGradient" x1="0%" y1="0%" x2="100%" y2="0%">
            <stop offset="0%" stopColor="transparent" />
            <stop offset="50%" stopColor="#00f0ff" />
            <stop offset="100%" stopColor="transparent" />
          </linearGradient>
        </defs>

        {/* Circuit lines (illuminate during boot) */}
        <g className={styles.circuitLines}>
          {/* Vertical spine */}
          <line x1="100" y1="95" x2="100" y2="180" stroke="#00f0ff" strokeWidth="1" opacity="0.3" />
          {/* Horizontal branches */}
          <line x1="70" y1="120" x2="100" y2="120" stroke="#00f0ff" strokeWidth="1" opacity="0.3" />
          <line x1="100" y1="120" x2="130" y2="120" stroke="#00f0ff" strokeWidth="1" opacity="0.3" />
          <line x1="70" y1="150" x2="100" y2="150" stroke="#00f0ff" strokeWidth="1" opacity="0.3" />
          <line x1="100" y1="150" x2="130" y2="150" stroke="#00f0ff" strokeWidth="1" opacity="0.3" />
        </g>

        {/* Antenna */}
        <g className={styles.antenna}>
          <line x1="100" y1="30" x2="100" y2="10" stroke="#4a5568" strokeWidth="3" />
          <circle cx="100" cy="8" r="6" fill="#00f0ff" filter="url(#strongGlow)" className={styles.antennaBall} />
        </g>

        {/* Head */}
        <g className={styles.head}>
          <rect x="60" y="30" width="80" height="60" rx="12" fill="url(#bodyGradient)" />
          {/* Visor */}
          <rect x="68" y="40" width="64" height="35" rx="8" fill="#1a202c" />

          {/* Eyes with cursor tracking */}
          <g
            className={`${styles.eyes} ${styles[eyeState]}`}
            style={{ transform: `translate(${eyeOffset.x}px, ${eyeOffset.y}px)` }}
          >
            <ellipse
              cx="82"
              cy="55"
              rx="8"
              ry={eyeState === 'blink' ? 1 : eyeState === 'happy' ? 6 : 8}
              fill="#00f0ff"
              filter="url(#glow)"
            />
            <ellipse
              cx="118"
              cy="55"
              rx="8"
              ry={eyeState === 'blink' ? 1 : eyeState === 'happy' ? 6 : 8}
              fill="#00f0ff"
              filter="url(#glow)"
            />
            {eyeState === 'happy' && (
              <>
                <path d="M74 55 Q82 48 90 55" fill="none" stroke="#00f0ff" strokeWidth="3" />
                <path d="M110 55 Q118 48 126 55" fill="none" stroke="#00f0ff" strokeWidth="3" />
              </>
            )}
            {/* Pupil dots for extra life */}
            {eyeState === 'normal' && (
              <>
                <circle cx="84" cy="53" r="2" fill="#fff" opacity="0.8" />
                <circle cx="120" cy="53" r="2" fill="#fff" opacity="0.8" />
              </>
            )}
          </g>

          {/* Mouth */}
          <path
            d={eyeState === 'happy' ? "M88 68 Q100 78 112 68" : "M88 70 L112 70"}
            fill="none"
            stroke="#00f0ff"
            strokeWidth="2"
            strokeLinecap="round"
          />
        </g>

        {/* Neck */}
        <rect x="90" y="90" width="20" height="15" fill="#4a5568" />

        {/* Torso */}
        <g className={styles.torso}>
          <rect x="55" y="105" width="90" height="80" rx="10" fill="url(#bodyGradient)" />
          {/* Chest light (core) */}
          <circle cx="100" cy="135" r="12" fill="#1a202c" />
          <circle
            cx="100"
            cy="135"
            r="8"
            fill="#00f0ff"
            className={styles.chestLight}
            filter="url(#strongGlow)"
          />
          {/* Chest circuit details */}
          <rect x="70" y="155" width="60" height="3" rx="1.5" fill="#00f0ff" opacity="0.4" />
          <rect x="75" y="163" width="50" height="2" rx="1" fill="#00f0ff" opacity="0.25" />
          <rect x="80" y="170" width="40" height="2" rx="1" fill="#00f0ff" opacity="0.15" />
        </g>

        {/* Left Arm */}
        <g className={`${styles.leftArm} ${isWaving ? styles.waving : ''}`}>
          <circle cx="50" cy="115" r="10" fill="url(#bodyGradient)" />
          <rect x="35" y="115" width="18" height="40" rx="8" fill="url(#bodyGradient)" />
          <circle cx="44" cy="155" r="8" fill="#4a5568" />
          <rect x="35" y="155" width="18" height="35" rx="8" fill="url(#bodyGradient)" />
          <circle cx="44" cy="195" r="12" fill="url(#accentGradient)" />
          <rect x="36" y="205" width="4" height="12" rx="2" fill="#00f0ff" />
          <rect x="42" y="207" width="4" height="14" rx="2" fill="#00f0ff" />
          <rect x="48" y="205" width="4" height="12" rx="2" fill="#00f0ff" />
        </g>

        {/* Right Arm (waving arm) */}
        <g className={`${styles.rightArm} ${isWaving ? styles.waving : ''}`}>
          <circle cx="150" cy="115" r="10" fill="url(#bodyGradient)" />
          <rect x="147" y="115" width="18" height="40" rx="8" fill="url(#bodyGradient)" />
          <circle cx="156" cy="155" r="8" fill="#4a5568" />
          <rect x="147" y="155" width="18" height="35" rx="8" fill="url(#bodyGradient)" />
          <circle cx="156" cy="195" r="12" fill="url(#accentGradient)" />
          <rect x="148" y="205" width="4" height="12" rx="2" fill="#00f0ff" />
          <rect x="154" y="207" width="4" height="14" rx="2" fill="#00f0ff" />
          <rect x="160" y="205" width="4" height="12" rx="2" fill="#00f0ff" />
        </g>

        {/* Legs */}
        <g className={styles.legs}>
          <rect x="65" y="185" width="25" height="50" rx="8" fill="url(#bodyGradient)" />
          <rect x="62" y="235" width="30" height="15" rx="4" fill="#4a5568" />
          <rect x="110" y="185" width="25" height="50" rx="8" fill="url(#bodyGradient)" />
          <rect x="108" y="235" width="30" height="15" rx="4" fill="#4a5568" />
        </g>

        {/* Ground shadow */}
        <ellipse cx="100" cy="255" rx="50" ry="8" fill="rgba(0,0,0,0.3)" />
      </svg>

      {/* Floating particles */}
      <div className={styles.particles}>
        <span className={styles.particle} style={{ '--delay': '0s', '--x': '-30px', '--y': '-20px' } as React.CSSProperties} />
        <span className={styles.particle} style={{ '--delay': '0.5s', '--x': '40px', '--y': '-40px' } as React.CSSProperties} />
        <span className={styles.particle} style={{ '--delay': '1s', '--x': '-50px', '--y': '30px' } as React.CSSProperties} />
        <span className={styles.particle} style={{ '--delay': '1.5s', '--x': '60px', '--y': '20px' } as React.CSSProperties} />
      </div>
    </div>
  );
}

export default AnimatedRobot;
