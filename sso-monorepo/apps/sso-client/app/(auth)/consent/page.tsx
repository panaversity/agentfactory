'use client';

import { useEffect, useState } from 'react';
import { useSearchParams, useRouter } from 'next/navigation';
import Link from 'next/link';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter, Button } from '@repo/ui';
import { ConsentForm } from './consent-form';
import { AlertCircle } from 'lucide-react';

export default function ConsentPage() {
  const searchParams = useSearchParams();
  const router = useRouter();
  const [hasSession, setHasSession] = useState<boolean | null>(null);
  
  const clientId = searchParams.get('client_id');
  const redirectUri = searchParams.get('redirect_uri');
  const scope = searchParams.get('scope');
  const state = searchParams.get('state');
  const responseType = searchParams.get('response_type');

  // Check for session
  useEffect(() => {
    // Simple session check - in production, call authClient.getSession()
    const checkSession = async () => {
      // For now, assume no session and redirect to login
      // TODO: Implement actual session check
      setHasSession(false);
    };
    
    checkSession();
  }, []);

  // Redirect to login if no session
  useEffect(() => {
    if (hasSession === false) {
      const oidcParams = new URLSearchParams();
      if (clientId) oidcParams.set('client_id', clientId);
      if (redirectUri) oidcParams.set('redirect_uri', redirectUri);
      if (responseType) oidcParams.set('response_type', responseType);
      if (scope) oidcParams.set('scope', scope);
      if (state) oidcParams.set('state', state);
      
      router.push(`/login?${oidcParams.toString()}`);
    }
  }, [hasSession, clientId, redirectUri, responseType, scope, state, router]);

  // Validate parameters
  const isValidRequest = clientId && redirectUri && scope;

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

  if (hasSession === null) {
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
          clientId={clientId!}
          redirectUri={redirectUri!}
          scope={scope!}
          state={state}
        />
      </CardContent>
    </Card>
  );
}
