'use client';

import { Suspense, useEffect, useState } from 'react';
import { useSearchParams, useRouter } from 'next/navigation';
import { Card, CardHeader, CardTitle, CardDescription, CardContent } from '@repo/ui';
import { ConsentForm } from './consent-form';
import { AlertCircle } from 'lucide-react';
import { authClient } from '@repo/auth-config/client';

function ConsentContent() {
  const searchParams = useSearchParams();
  const router = useRouter();
  const [hasSession, setHasSession] = useState<boolean | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Better-auth OIDC flow provides consent_code
  const consentCode = searchParams.get('consent_code');

  // Params from URL (better-auth passes these along with consent_code)
  const clientId = searchParams.get('client_id');
  const redirectUri = searchParams.get('redirect_uri');
  const scope = searchParams.get('scope');
  const state = searchParams.get('state');

  // Check for session
  useEffect(() => {
    const checkSession = async () => {
      try {
        const session = await authClient.getSession();
        setHasSession(!!session.data);
      } catch (error) {
        console.error('Session check failed:', error);
        setHasSession(false);
      }
      setIsLoading(false);
    };

    checkSession();
  }, []);

  // Redirect to login if no session
  useEffect(() => {
    if (hasSession === false) {
      // Preserve OAuth params when redirecting to login
      const loginParams = new URLSearchParams();
      if (clientId) loginParams.set('client_id', clientId);
      if (redirectUri) loginParams.set('redirect_uri', redirectUri);
      if (scope) loginParams.set('scope', scope);
      if (state) loginParams.set('state', state);

      router.push(`/login?${loginParams.toString()}`);
    }
  }, [hasSession, clientId, redirectUri, scope, state, router]);

  // Validate request - accept either:
  // - consent_code flow (better-auth OIDC): consent_code + client_id + scope
  // - Legacy flow: client_id + redirect_uri + scope
  const isValidRequest = (consentCode && clientId && scope) ||
                         (clientId && redirectUri && scope);

  // Show loading state
  if (isLoading || hasSession === null) {
    return (
      <Card>
        <CardContent className="py-8">
          <div className="flex justify-center">
            <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
          </div>
        </CardContent>
      </Card>
    );
  }

  // Show error if invalid request
  if (!isValidRequest) {
    return (
      <Card>
        <CardHeader>
          <div className="flex justify-center mb-4">
            <div className="rounded-full bg-red-100 dark:bg-red-900 p-3">
              <AlertCircle className="h-8 w-8 text-red-600 dark:text-red-400" />
            </div>
          </div>
          <CardTitle className="text-center">Invalid Request</CardTitle>
          <CardDescription className="text-center text-red-600 dark:text-red-400">
            Missing required consent parameters
          </CardDescription>
        </CardHeader>
      </Card>
    );
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle>Grant Permission</CardTitle>
        <CardDescription>
          An application is requesting access to your account
        </CardDescription>
      </CardHeader>
      <CardContent>
        <ConsentForm
          consentCode={consentCode}
          clientId={clientId!}
          redirectUri={redirectUri}
          scope={scope!}
          state={state}
        />
      </CardContent>
    </Card>
  );
}

export default function ConsentPage() {
  return (
    <Suspense fallback={
      <Card>
        <CardContent className="py-8">
          <div className="flex justify-center">
            <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
          </div>
        </CardContent>
      </Card>
    }>
      <ConsentContent />
    </Suspense>
  );
}
