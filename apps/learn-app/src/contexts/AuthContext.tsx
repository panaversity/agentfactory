import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import { refreshAccessToken } from '../lib/auth-client';
import { verifyIDToken, extractUserFromToken } from '../lib/jwt-verifier';
import { getHomeUrl } from '../lib/url-utils';

interface User {
  id: string;
  email: string;
  name?: string;
  role?: string;
  softwareBackground?: string | null;
  hardwareTier?: string | null;
}

interface Session {
  user: User;
  accessToken?: string;
}

interface AuthContextType {
  session: Session | null;
  isLoading: boolean;
  signOut: (global?: boolean) => void;
  refreshUserData: () => Promise<boolean>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children?: ReactNode;
  authUrl?: string;
  oauthClientId?: string;
}

// Default authUrl uses empty string - callers should provide via Docusaurus config
// In development, this will be set by Root.tsx via siteConfig.customFields.authUrl
export function AuthProvider({ children, authUrl, oauthClientId }: AuthProviderProps) {
  // Require authUrl to be provided - no hardcoded fallback
  if (!authUrl) {
    console.error('AuthProvider: authUrl is required. Configure it in docusaurus.config.ts customFields.');
  }
  const effectiveAuthUrl = authUrl || '';
  const effectiveClientId = oauthClientId || 'ai-native-public-client';
  const [session, setSession] = useState<Session | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Fetch user info with a given access token
  const fetchUserInfo = async (accessToken: string): Promise<User | null> => {
    try {
      const response = await fetch(`${effectiveAuthUrl}/api/auth/oauth2/userinfo`, {
        headers: { 'Authorization': `Bearer ${accessToken}` },
      });

      if (response.ok) {
        const userInfo = await response.json();
        return {
          id: userInfo.sub,
          email: userInfo.email,
          name: userInfo.name,
          role: userInfo.role,
          softwareBackground: userInfo.software_background,
          hardwareTier: userInfo.hardware_tier,
        };
      }
    } catch (error) {
      console.error('Failed to fetch user info:', error);
    }
    return null;
  };

  useEffect(() => {
    // Check session on mount - Use JWKS verification first (client-side, no server call)
    const checkSession = async () => {
      try {
        const idToken = localStorage.getItem('ainative_id_token');
        const accessToken = localStorage.getItem('ainative_access_token');

        // Strategy 1: Verify ID token using JWKS (client-side, reduces server load)
        if (idToken && effectiveAuthUrl) {
          try {
            const payload = await verifyIDToken(idToken, effectiveAuthUrl, effectiveClientId);
            if (payload) {
              // Token is valid, extract user info from token (no server call needed!)
              const user = extractUserFromToken(payload);
              setSession({ user, accessToken: accessToken || undefined });
              setIsLoading(false);
              return;
            }
          } catch (error) {
            console.log('ID token verification failed, falling back to userinfo endpoint:', error);
            // Fall through to userinfo endpoint fallback
          }
        }

        // Strategy 2: Fallback to userinfo endpoint (if ID token missing/invalid)
        if (accessToken) {
          // Try to fetch user info with current token
          let user = await fetchUserInfo(accessToken);

          // If token expired (401), try to refresh
          if (!user) {
            console.log('Access token expired, attempting refresh...');
            const newToken = await refreshAccessToken();

            if (newToken) {
              // After refresh, try to verify new ID token if available
              const newIdToken = localStorage.getItem('ainative_id_token');
              if (newIdToken && effectiveAuthUrl) {
                try {
                  const payload = await verifyIDToken(newIdToken, effectiveAuthUrl, effectiveClientId);
                  if (payload) {
                    const verifiedUser = extractUserFromToken(payload);
                    setSession({ user: verifiedUser, accessToken: newToken });
                    setIsLoading(false);
                    return;
                  }
                } catch (error) {
                  // Fall through to userinfo
                }
              }

              // Fallback to userinfo endpoint
              user = await fetchUserInfo(newToken);
            }
          }

          if (user) {
            setSession({ user, accessToken });
          } else {
            // Both token and refresh failed, clear everything
            localStorage.removeItem('ainative_access_token');
            localStorage.removeItem('ainative_refresh_token');
            localStorage.removeItem('ainative_id_token');
            setSession(null);
          }
        } else {
          // No token means not logged in
          setSession(null);
        }
      } catch (error) {
        console.error('Failed to check session:', error);
        setSession(null);
      } finally {
        setIsLoading(false);
      }
    };

    checkSession();
  }, [effectiveAuthUrl]);

  const handleSignOut = (global: boolean = false) => {
    // Clear OAuth tokens from localStorage
    localStorage.removeItem('ainative_access_token');
    localStorage.removeItem('ainative_refresh_token');
    localStorage.removeItem('ainative_id_token');

    // Clear session state
    setSession(null);

    // Get home URL - auto-detects base path from current URL
    const homeUrl = getHomeUrl();

    if (global) {
      // Global logout: redirect to auth server to end session there too
      // This logs user out from all apps using this auth server
      const redirectTo = typeof window !== 'undefined' ? `${window.location.origin}${homeUrl}` : homeUrl;
      window.location.href = `${effectiveAuthUrl}/api/auth/sign-out?redirectTo=${encodeURIComponent(redirectTo)}`;
    } else {
      // Local logout: just redirect to home
      // User stays logged in at auth server (SSO pattern)
      window.location.href = homeUrl;
    }
  };

  /**
   * Refresh user data from the auth server
   * Call this after profile updates to get the latest user information
   * @returns true if refresh succeeded, false otherwise
   */
  const handleRefreshUserData = async (): Promise<boolean> => {
    try {
      const accessToken = localStorage.getItem('ainative_access_token');

      if (!accessToken) {
        console.warn('No access token available for refresh');
        return false;
      }

      // Fetch fresh user data from /oauth2/userinfo endpoint
      const user = await fetchUserInfo(accessToken);

      if (user) {
        // Update session with fresh user data
        setSession({ user, accessToken });
        console.log('User data refreshed successfully');
        return true;
      } else {
        console.warn('Failed to refresh user data');
        return false;
      }
    } catch (error) {
      console.error('Error refreshing user data:', error);
      return false;
    }
  };

  return (
    <AuthContext.Provider value={{
      session,
      isLoading,
      signOut: handleSignOut,
      refreshUserData: handleRefreshUserData
    }}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}
