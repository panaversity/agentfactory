import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

interface TerminalLine {
  type: 'command' | 'output' | 'success' | 'code' | 'ai';
  content: string;
  delay: number;
}

const TERMINAL_SEQUENCE: TerminalLine[] = [
  { type: 'command', content: '$ robolearn init --tier cloud', delay: 0 },
  { type: 'success', content: '✓ Environment configured', delay: 800 },
  { type: 'success', content: '✓ MockROS bridge ready', delay: 1200 },
  { type: 'success', content: '✓ RAG context loaded (4 modules)', delay: 1600 },
  { type: 'output', content: '', delay: 2000 },
  { type: 'code', content: '>>> import rclpy', delay: 2400 },
  { type: 'code', content: '>>> node = rclpy.create_node("learner")', delay: 3000 },
  { type: 'code', content: '>>> node.get_logger().info("Hello, Robot!")', delay: 3600 },
  { type: 'output', content: '[INFO] [learner]: Hello, Robot!', delay: 4200 },
  { type: 'output', content: '', delay: 4800 },
  { type: 'ai', content: '🤖 AI: Try publishing a velocity command next!', delay: 5200 },
];

/**
 * TerminalPreview - Animated terminal showing platform capabilities
 * Cycles through: init → code execution → AI suggestion
 */
export function TerminalPreview(): React.ReactElement {
  const [visibleLines, setVisibleLines] = useState<number>(0);
  const [cursorVisible, setCursorVisible] = useState(true);

  useEffect(() => {
    // Typewriter effect - reveal lines progressively
    const timers: NodeJS.Timeout[] = [];

    TERMINAL_SEQUENCE.forEach((line, index) => {
      const timer = setTimeout(() => {
        setVisibleLines(index + 1);
      }, line.delay);
      timers.push(timer);
    });

    // Reset and loop
    const resetTimer = setTimeout(() => {
      setVisibleLines(0);
    }, 8000);
    timers.push(resetTimer);

    // Cursor blink
    const cursorInterval = setInterval(() => {
      setCursorVisible((v) => !v);
    }, 530);

    return () => {
      timers.forEach(clearTimeout);
      clearInterval(cursorInterval);
    };
  }, [visibleLines === 0]); // Reset when visibleLines becomes 0

  return (
    <div className={styles.terminal}>
      {/* Terminal header */}
      <div className={styles.terminalHeader}>
        <div className={styles.terminalDots}>
          <span className={styles.dotRed} />
          <span className={styles.dotYellow} />
          <span className={styles.dotGreen} />
        </div>
        <span className={styles.terminalTitle}>robolearn — zsh</span>
      </div>

      {/* Terminal body */}
      <div className={styles.terminalBody}>
        {TERMINAL_SEQUENCE.slice(0, visibleLines).map((line, index) => (
          <div
            key={index}
            className={`${styles.terminalLine} ${styles[line.type]}`}
          >
            {line.content}
          </div>
        ))}
        {/* Blinking cursor */}
        <span className={`${styles.cursor} ${cursorVisible ? styles.visible : ''}`}>
          █
        </span>
      </div>

      {/* Mini robot visualization */}
      <div className={styles.robotPreview}>
        <div className={styles.robotBody}>
          <div className={styles.robotHead} />
          <div className={styles.robotTorso} />
          <div className={styles.robotArm} />
          <div className={styles.robotArm} />
        </div>
        {visibleLines >= 9 && (
          <div className={styles.robotSpeech}>Hello!</div>
        )}
      </div>
    </div>
  );
}

export default TerminalPreview;
