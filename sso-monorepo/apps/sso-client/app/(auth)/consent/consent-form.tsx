'use client';

import { useState } from 'react';
import { Button, Alert } from '@repo/ui';
import { Shield, Mail, User, CheckCircle, XCircle } from 'lucide-react';

const API_URL = process.env.NEXT_PUBLIC_SSO_SERVER_URL || 'http://localhost:3000';

interface ConsentFormProps {
  clientId: string;
  redirectUri: string;
  scope: string;
  state?: string | null;
}

const SCOPE_DESCRIPTIONS: Record<string, { icon: any; title: string; description: string }> = {
  openid: {
    icon: Shield,
    title: 'OpenID Connect',
    description: 'Authenticate your identity',
  },
  profile: {
    icon: User,
    title: 'Profile Information',
    description: 'Access your name and profile picture',
  },
  email: {
    icon: Mail,
    title: 'Email Address',
    description: 'Access your email address',
  },
};

export function ConsentForm({ clientId, redirectUri, scope, state }: ConsentFormProps) {
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string>('');

  const scopes = scope.split(' ').filter(Boolean);

  const handleConsent = async (approved: boolean) => {
    setIsSubmitting(true);
    setError('');

    try {
      const response = await fetch(`${API_URL}/api/auth/consent`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',
        body: JSON.stringify({
          client_id: clientId,
          redirect_uri: redirectUri,
          scope,
          state,
          approved,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        setError(data.error?.message || data.message || 'Failed to process consent');
        setIsSubmitting(false);
        return;
      }

      // Redirect back to application
      if (data.redirectUrl) {
        window.location.href = data.redirectUrl;
      } else if (redirectUri) {
        // Construct redirect URL with error or code
        const redirectUrl = new URL(redirectUri);
        if (approved && data.code) {
          redirectUrl.searchParams.set('code', data.code);
        } else {
          redirectUrl.searchParams.set('error', 'access_denied');
        }
        if (state) {
          redirectUrl.searchParams.set('state', state);
        }
        window.location.href = redirectUrl.toString();
      }
    } catch (err) {
      setError('Network error occurred. Please try again.');
      setIsSubmitting(false);
    }
  };

  return (
    <div className="space-y-6">
      {error && (
        <Alert variant="destructive">
          <XCircle className="h-4 w-4" />
          <div className="ml-2">
            <p className="text-sm">{error}</p>
          </div>
        </Alert>
      )}

      <div className="space-y-4">
        <div className="p-4 bg-gray-50 dark:bg-gray-800 rounded-lg">
          <h3 className="font-semibold text-sm mb-2">Application Details</h3>
          <p className="text-sm text-gray-600 dark:text-gray-400">
            <strong>Client ID:</strong> {clientId}
          </p>
          <p className="text-sm text-gray-600 dark:text-gray-400 mt-1">
            <strong>Redirect URI:</strong> {redirectUri}
          </p>
        </div>

        <div>
          <h3 className="font-semibold text-sm mb-3">This application will be able to:</h3>
          <div className="space-y-3">
            {scopes.map((scopeName) => {
              const scopeInfo = SCOPE_DESCRIPTIONS[scopeName] || {
                icon: Shield,
                title: scopeName,
                description: `Access your ${scopeName}`,
              };
              const Icon = scopeInfo.icon;

              return (
                <div key={scopeName} className="flex items-start space-x-3 p-3 bg-white dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-md">
                  <div className="flex-shrink-0">
                    <Icon className="h-5 w-5 text-blue-600 dark:text-blue-400" />
                  </div>
                  <div>
                    <p className="font-medium text-sm">{scopeInfo.title}</p>
                    <p className="text-xs text-gray-500 dark:text-gray-400">
                      {scopeInfo.description}
                    </p>
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      </div>

      <div className="flex gap-3 pt-4">
        <Button
          variant="outline"
          onClick={() => handleConsent(false)}
          disabled={isSubmitting}
          className="flex-1"
        >
          Deny
        </Button>
        <Button
          onClick={() => handleConsent(true)}
          disabled={isSubmitting}
          className="flex-1"
        >
          {isSubmitting ? (
            <>
              <div className="mr-2 h-4 w-4 animate-spin rounded-full border-2 border-current border-t-transparent" />
              Processing...
            </>
          ) : (
            <>
              <CheckCircle className="mr-2 h-4 w-4" />
              Allow
            </>
          )}
        </Button>
      </div>

      <p className="text-xs text-center text-gray-500 dark:text-gray-400">
        By allowing access, you agree to share the requested information with this application.
      </p>
    </div>
  );
}
