import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '@/contexts/AuthContext';
import { getOAuthAuthorizationUrl } from '@/lib/auth-client';
import { getHomeUrl } from '@/lib/url-utils';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

export function NavbarAuth() {
  const { session, isLoading, signOut, refreshUserData } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3001';
  const oauthClientId = (siteConfig.customFields?.oauthClientId as string) || 'robolearn-interface';
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // OAuth config from Docusaurus context
  const oauthConfig = {
    authUrl,
    clientId: oauthClientId,
  };

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
    const authorizationUrl = await getOAuthAuthorizationUrl('signin', oauthConfig);
    // Small delay to ensure localStorage write is flushed before navigation
    await new Promise(resolve => setTimeout(resolve, 50));
    window.location.href = authorizationUrl;
  };

  // For sign up, go to auth server sign-up page then OAuth flow
  // If user is already logged in locally, redirect to docs instead
  const handleSignUp = async () => {
    // If user is already logged in locally, redirect to docs
    if (session?.user) {
      const homeUrl = getHomeUrl();
      window.location.href = `${homeUrl}docs/preface-agent-native`;
      return;
    }

    // If not logged in locally, use signup flow
    // The sign-up page now properly handles users already logged in on auth server
    const oauthUrl = await getOAuthAuthorizationUrl('signup', oauthConfig);
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

  // Handle Edit Profile - open auth server profile page
  const handleEditProfile = () => {
    const currentUrl = typeof window !== 'undefined' ? window.location.href : '';
    const profileUrl = `${authUrl}/account/profile?redirect=${encodeURIComponent(currentUrl)}`;

    // Store a flag so we know to refresh data when user returns
    localStorage.setItem('robolearn_refresh_on_return', 'true');

    window.location.href = profileUrl;
  };

  // Refresh data if user just returned from profile editing
  useEffect(() => {
    const shouldRefresh = localStorage.getItem('robolearn_refresh_on_return');
    if (shouldRefresh === 'true' && session?.user) {
      localStorage.removeItem('robolearn_refresh_on_return');
      refreshUserData();
    }
  }, [session]);

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
            
            {/* Profile data section */}
            {(session.user.softwareBackground || session.user.hardwareTier) && (
              <>
                <div className={styles.dropdownDivider} />
                <div className={styles.profileInfo}>
                  {session.user.softwareBackground && (
                    <div className={styles.profileItem}>
                      <span className={styles.profileLabel}>Software:</span>
                      <span className={styles.profileValue}>
                        {session.user.softwareBackground.charAt(0).toUpperCase() + session.user.softwareBackground.slice(1)}
                      </span>
                    </div>
                  )}
                  {session.user.hardwareTier && (
                    <div className={styles.profileItem}>
                        {session.user.hardwareTier === 'tier1' ? 'Windows PC' :
                         session.user.hardwareTier === 'tier2' ? 'Mac' :
                         session.user.hardwareTier === 'tier3' ? 'Linux' :
                         session.user.hardwareTier === 'tier4' ? 'Chromebook/Web' :
                         session.user.hardwareTier}
                      </span>
                    </div>
                  )}
                </div>
              </>
            )}
            
            <div className={styles.dropdownDivider} />
            <button onClick={handleEditProfile} className={styles.dropdownItem}>
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                <path d="M11.3333 2.00004C11.5084 1.82494 11.716 1.68605 11.9441 1.59129C12.1722 1.49653 12.4165 1.44775 12.6633 1.44775C12.9101 1.44775 13.1544 1.49653 13.3825 1.59129C13.6106 1.68605 13.8183 1.82494 13.9933 2.00004C14.1684 2.17513 14.3073 2.38283 14.4021 2.61091C14.4968 2.83899 14.5456 3.08333 14.5456 3.33004C14.5456 3.57675 14.4968 3.82109 14.4021 4.04917C14.3073 4.27725 14.1684 4.48495 13.9933 4.66004L5.33333 13.32L2 14L2.68 10.6667L11.3333 2.00004Z" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
              Edit Profile
            </button>
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
