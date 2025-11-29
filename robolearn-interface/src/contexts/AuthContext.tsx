import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  name?: string;
  role?: string;
}

interface Session {
  user: User;
  accessToken?: string;
}

interface AuthContextType {
  session: Session | null;
  isLoading: boolean;
  signOut: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
  authUrl?: string;
}

export function AuthProvider({ children, authUrl = 'http://localhost:3001' }: AuthProviderProps) {
  const [session, setSession] = useState<Session | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Check session on mount - OAuth tokens only (no cookie fallback)
    const checkSession = async () => {
      try {
        // Check if we have an access token from OAuth flow
        const accessToken = localStorage.getItem('robolearn_access_token');

        if (accessToken) {
          // Validate token by fetching user info from OIDC userinfo endpoint
          const userInfoResponse = await fetch(`${authUrl}/api/auth/oauth2/userinfo`, {
            headers: {
              'Authorization': `Bearer ${accessToken}`,
            },
          });

          if (userInfoResponse.ok) {
            const userInfo = await userInfoResponse.json();
            setSession({
              user: {
                id: userInfo.sub,
                email: userInfo.email,
                name: userInfo.name,
              },
              accessToken,
            });
          } else {
            // Token invalid, clear it
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

  const handleSignOut = () => {
    // Clear OAuth tokens from localStorage
    localStorage.removeItem('robolearn_access_token');
    localStorage.removeItem('robolearn_refresh_token');
    localStorage.removeItem('robolearn_id_token');

    // Clear session state
    setSession(null);

    // Redirect to home - standard OAuth: client just clears its own tokens
    // The auth server session is separate (user can still be logged in there for other apps)
    window.location.href = '/';
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
