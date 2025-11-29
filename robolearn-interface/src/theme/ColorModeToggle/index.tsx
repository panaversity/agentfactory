import React, {type ReactNode} from 'react';
import clsx from 'clsx';
import useIsBrowser from '@docusaurus/useIsBrowser';
import {translate} from '@docusaurus/Translate';
import {Sun, Moon} from 'lucide-react';
import type {Props} from '@theme/ColorModeToggle';

import styles from './styles.module.css';

// Simple 2-value toggle: dark ↔ light (single click)
function getNextColorMode(currentMode: string | null): 'dark' | 'light' {
  return currentMode === 'dark' ? 'light' : 'dark';
}

function getColorModeLabel(colorMode: string | null): string {
  return colorMode === 'dark'
    ? translate({
        message: 'dark mode',
        id: 'theme.colorToggle.ariaLabel.mode.dark',
        description: 'The name for the dark color mode',
      })
    : translate({
        message: 'light mode',
        id: 'theme.colorToggle.ariaLabel.mode.light',
        description: 'The name for the light color mode',
      });
}

function getColorModeAriaLabel(colorMode: string | null) {
  return translate(
    {
      message: 'Switch between dark and light mode (currently {mode})',
      id: 'theme.colorToggle.ariaLabel',
      description: 'The ARIA label for the color mode toggle',
    },
    {
      mode: getColorModeLabel(colorMode),
    },
  );
}

function ColorModeToggle({
  className,
  buttonClassName,
  value,
  onChange,
}: Props): ReactNode {
  const isBrowser = useIsBrowser();
  const isDark = value === 'dark' || value === null;

  return (
    <div className={clsx(styles.toggle, className)}>
      <button
        className={clsx(
          'clean-btn',
          styles.toggleButton,
          !isBrowser && styles.toggleButtonDisabled,
          buttonClassName,
        )}
        type="button"
        onClick={() => onChange(getNextColorMode(value))}
        disabled={!isBrowser}
        title={isDark ? 'Switch to light mode' : 'Switch to dark mode'}
        aria-label={getColorModeAriaLabel(value)}
      >
        {isDark ? (
          <Moon className={styles.toggleIcon} aria-hidden />
        ) : (
          <Sun className={styles.toggleIcon} aria-hidden />
        )}
      </button>
    </div>
  );
}

export default React.memo(ColorModeToggle);
