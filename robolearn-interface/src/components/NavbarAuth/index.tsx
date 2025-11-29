import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '@/contexts/AuthContext';
import { getOAuthAuthorizationUrl } from '@/lib/auth-client';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

export function NavbarAuth() {
  const { session, isLoading, signOut } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3001';
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsDropdownOpen(false);
      }
    };
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  // Generate OAuth authorization URL for sign in (async for PKCE)
  const handleSignIn = async () => {
    const authUrl = await getOAuthAuthorizationUrl('signin');
    // Small delay to ensure localStorage write is flushed before navigation
    await new Promise(resolve => setTimeout(resolve, 50));
    window.location.href = authUrl;
  };

  // For sign up, go to auth server sign-up page then OAuth flow
  const handleSignUp = async () => {
    const oauthUrl = await getOAuthAuthorizationUrl('signup');
    const signupUrl = `${authUrl}/auth/sign-up?redirect=${encodeURIComponent(oauthUrl)}`;
    window.location.href = signupUrl;
  };

  // Get user initials for avatar
  const getInitials = (name?: string, email?: string) => {
    if (name) {
      return name.split(' ').map(n => n[0]).join('').toUpperCase().slice(0, 2);
    }
    return email ? email[0].toUpperCase() : '?';
  };

  if (isLoading) {
    return (
      <div className={styles.authContainer}>
        <div className={styles.avatarSkeleton} />
      </div>
    );
  }

  if (session?.user) {
    const displayName = session.user.name || session.user.email.split('@')[0];
    const initials = getInitials(session.user.name, session.user.email);

    return (
      <div className={styles.authContainer} ref={dropdownRef}>
        <button
          className={styles.userButton}
          onClick={() => setIsDropdownOpen(!isDropdownOpen)}
          aria-expanded={isDropdownOpen}
          aria-haspopup="true"
        >
          <div className={styles.avatar}>
            <span className={styles.avatarText}>{initials}</span>
          </div>
          <svg
            className={`${styles.chevron} ${isDropdownOpen ? styles.chevronOpen : ''}`}
            width="12"
            height="12"
            viewBox="0 0 12 12"
            fill="none"
          >
            <path d="M3 4.5L6 7.5L9 4.5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>

        {isDropdownOpen && (
          <div className={styles.dropdown}>
            <div className={styles.dropdownHeader}>
              <div className={styles.dropdownAvatar}>
                <span className={styles.avatarText}>{initials}</span>
              </div>
              <div className={styles.dropdownUserInfo}>
                <span className={styles.dropdownName}>{displayName}</span>
                <span className={styles.dropdownEmail}>{session.user.email}</span>
              </div>
            </div>
            <div className={styles.dropdownDivider} />
            <button onClick={() => signOut()} className={styles.dropdownItem}>
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                <path d="M6 14H3.33333C2.97971 14 2.64057 13.8595 2.39052 13.6095C2.14048 13.3594 2 13.0203 2 12.6667V3.33333C2 2.97971 2.14048 2.64057 2.39052 2.39052C2.64057 2.14048 2.97971 2 3.33333 2H6" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                <path d="M10.6667 11.3333L14 8L10.6667 4.66667" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                <path d="M14 8H6" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
              Sign Out
            </button>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className={styles.authContainer}>
      <button onClick={handleSignIn} className={styles.signInLink}>
        Sign In
      </button>
      <button onClick={handleSignUp} className={styles.getStartedButton}>
        Get Started
      </button>
    </div>
  );
}

export default NavbarAuth;
