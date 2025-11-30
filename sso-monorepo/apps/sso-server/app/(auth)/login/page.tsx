'use client';

import { Suspense } from 'react';
import { useSearchParams } from 'next/navigation';
import Link from 'next/link';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter, Button } from '@repo/ui';
import { SignInForm } from '../signin/signin-form';
import { AlertCircle } from 'lucide-react';

function LoginContent() {
  const searchParams = useSearchParams();

  const clientId = searchParams.get('client_id');
  const redirectUri = searchParams.get('redirect_uri');
  const responseType = searchParams.get('response_type');
  const scope = searchParams.get('scope');
  const state = searchParams.get('state');

  // Validate OIDC parameters
  const isValidOIDCRequest = clientId && redirectUri && responseType && scope;

  if (!isValidOIDCRequest) {
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
            Missing required OIDC parameters
          </CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <p className="text-sm text-center text-gray-600 dark:text-gray-400">
            This page requires valid OIDC authentication parameters. Please initiate the login from your application.
          </p>
        </CardContent>
        <CardFooter>
          <Link href="/signin" className="w-full">
            <Button variant="outline" className="w-full">
              Go to Sign In
            </Button>
          </Link>
        </CardFooter>
      </Card>
    );
  }

  // Construct callback URL with OIDC parameters
  const oidcParams = new URLSearchParams({
    client_id: clientId,
    redirect_uri: redirectUri,
    response_type: responseType,
    scope: scope,
    ...(state && { state }),
  });

  const callbackUrl = `/consent?${oidcParams.toString()}`;

  return (
    <Card>
      <CardHeader>
        <CardTitle>Sign In to Continue</CardTitle>
        <CardDescription>
          An application is requesting access to your account
        </CardDescription>
      </CardHeader>
      <CardContent>
        {clientId && (
          <div className="mb-4 p-3 bg-blue-50 dark:bg-blue-900/20 rounded-md">
            <p className="text-sm text-blue-800 dark:text-blue-200">
              <strong>Application:</strong> {clientId}
            </p>
            {scope && (
              <p className="text-sm text-blue-800 dark:text-blue-200 mt-1">
                <strong>Requested permissions:</strong> {scope.split(' ').join(', ')}
              </p>
            )}
          </div>
        )}
        <SignInForm />
      </CardContent>
      <CardFooter>
        <p className="text-sm text-gray-600 dark:text-gray-400 text-center w-full">
          Don't have an account?{' '}
          <Link href={`/signup?callbackUrl=${encodeURIComponent(callbackUrl)}`} className="text-blue-600 hover:underline dark:text-blue-400">
            Sign up
          </Link>
        </p>
      </CardFooter>
    </Card>
  );
}

export default function LoginPage() {
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
      <LoginContent />
    </Suspense>
  );
}
