import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import { refreshAccessToken } from '../lib/auth-client';

interface User {
  id: string;
  email: string;
  name?: string;
  role?: string;
  softwareBackground?: string | null;
}

interface Session {
  user: User;
  accessToken?: string;
}

interface AuthContextType {
  session: Session | null;
  isLoading: boolean;
  signOut: (global?: boolean) => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
  authUrl?: string;
}

export function AuthProvider({ children, authUrl = 'http://localhost:3001' }: AuthProviderProps) {
  const [session, setSession] = useState<Session | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Fetch user info with a given access token
  const fetchUserInfo = async (accessToken: string): Promise<User | null> => {
    try {
      const response = await fetch(`${authUrl}/api/auth/oauth2/userinfo`, {
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
        };
      }
    } catch (error) {
      console.error('Failed to fetch user info:', error);
    }
    return null;
  };

  useEffect(() => {
    // Check session on mount - OAuth tokens only with automatic refresh
    const checkSession = async () => {
      try {
        let accessToken = localStorage.getItem('robolearn_access_token');

        if (accessToken) {
          // Try to fetch user info with current token
          let user = await fetchUserInfo(accessToken);

          // If token expired (401), try to refresh
          if (!user) {
            console.log('Access token expired, attempting refresh...');
            const newToken = await refreshAccessToken();

            if (newToken) {
              accessToken = newToken;
              user = await fetchUserInfo(accessToken);
            }
          }

          if (user) {
            setSession({ user, accessToken });
          } else {
            // Both token and refresh failed, clear everything
            localStorage.removeItem('robolearn_access_token');
            localStorage.removeItem('robolearn_refresh_token');
            localStorage.removeItem('robolearn_id_token');
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
  }, [authUrl]);

  const handleSignOut = (global: boolean = false) => {
    // Clear OAuth tokens from localStorage
    localStorage.removeItem('robolearn_access_token');
    localStorage.removeItem('robolearn_refresh_token');
    localStorage.removeItem('robolearn_id_token');

    // Clear session state
    setSession(null);

    if (global) {
      // Global logout: redirect to auth server to end session there too
      // This logs user out from all apps using this auth server
      window.location.href = `${authUrl}/api/auth/sign-out?redirectTo=${encodeURIComponent(window.location.origin)}`;
    } else {
      // Local logout: just redirect to home
      // User stays logged in at auth server (SSO pattern)
      window.location.href = '/';
    }
  };

  return (
    <AuthContext.Provider value={{ session, isLoading, signOut: handleSignOut }}>
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
