'use client';

import { useEffect, useState } from 'react';
import { useSearchParams, useRouter } from 'next/navigation';
import Link from 'next/link';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter, Button } from '@repo/ui';
import { CheckCircle, XCircle, Loader2 } from 'lucide-react';
import { verifyEmailAction } from '@/app/server/auth';
import { ERROR_MESSAGES } from '@/lib/constants';

export default function VerifyEmailTokenPage() {
  const router = useRouter();
  const searchParams = useSearchParams();
  const token = searchParams.get('token');
  
  const [status, setStatus] = useState<'loading' | 'success' | 'error'>('loading');
  const [errorMessage, setErrorMessage] = useState('');
  const [countdown, setCountdown] = useState(3);

  useEffect(() => {
    async function verifyEmail() {
      if (!token) {
        setStatus('error');
        setErrorMessage('Verification token is missing. Please use the link from your email.');
        return;
      }

      try {
        const result = await verifyEmailAction({
          token,
          callbackURL: '/signin',
        });

        if (result.error) {
          setStatus('error');
          
          // Handle specific error cases
          if (result.error.message?.toLowerCase().includes('expired')) {
            setErrorMessage(ERROR_MESSAGES.TOKEN_EXPIRED);
          } else if (result.error.message?.toLowerCase().includes('invalid')) {
            setErrorMessage(ERROR_MESSAGES.INVALID_TOKEN);
          } else {
            setErrorMessage(result.error.message || 'Failed to verify email. Please try again.');
          }
          return;
        }

        // Success
        setStatus('success');
        
        // Start countdown and redirect
        const timer = setInterval(() => {
          setCountdown(prev => {
            if (prev <= 1) {
              clearInterval(timer);
              router.push('/signin');
              return 0;
            }
            return prev - 1;
          });
        }, 1000);

        return () => clearInterval(timer);
      } catch (error) {
        setStatus('error');
        setErrorMessage('An unexpected error occurred. Please try again.');
      }
    }

    verifyEmail();
  }, [token, router]);

  if (status === 'loading') {
    return (
      <Card>
        <CardHeader>
          <div className="flex justify-center mb-4">
            <Loader2 className="h-8 w-8 text-blue-600 dark:text-blue-400 animate-spin" />
          </div>
          <CardTitle className="text-center">Verifying Your Email</CardTitle>
          <CardDescription className="text-center">
            Please wait while we verify your email address...
          </CardDescription>
        </CardHeader>
      </Card>
    );
  }

  if (status === 'error') {
    return (
      <Card>
        <CardHeader>
          <div className="flex justify-center mb-4">
            <div className="rounded-full bg-red-100 dark:bg-red-900 p-3">
              <XCircle className="h-8 w-8 text-red-600 dark:text-red-400" />
            </div>
          </div>
          <CardTitle className="text-center">Verification Failed</CardTitle>
          <CardDescription className="text-center text-red-600 dark:text-red-400">
            {errorMessage}
          </CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <p className="text-sm text-center text-gray-600 dark:text-gray-400">
            {errorMessage.includes('expired') 
              ? 'Please sign up again or request a new verification email.'
              : 'Please check the verification link in your email or try signing up again.'}
          </p>
        </CardContent>
        <CardFooter className="flex flex-col space-y-2">
          <Link href="/signup" className="w-full">
            <Button className="w-full">
              Back to Sign Up
            </Button>
          </Link>
          <Link href="/signin" className="w-full">
            <Button variant="outline" className="w-full">
              Go to Sign In
            </Button>
          </Link>
        </CardFooter>
      </Card>
    );
  }

  // Success state
  return (
    <Card>
      <CardHeader>
        <div className="flex justify-center mb-4">
          <div className="rounded-full bg-green-100 dark:bg-green-900 p-3">
            <CheckCircle className="h-8 w-8 text-green-600 dark:text-green-400" />
          </div>
        </div>
        <CardTitle className="text-center">Email Verified Successfully!</CardTitle>
        <CardDescription className="text-center">
          Your email has been verified. You can now sign in to your account.
        </CardDescription>
      </CardHeader>
      <CardContent className="space-y-4">
        <p className="text-sm text-center text-gray-600 dark:text-gray-400">
          Redirecting to sign in page in {countdown} second{countdown !== 1 ? 's' : ''}...
        </p>
      </CardContent>
      <CardFooter>
        <Link href="/signin" className="w-full">
          <Button className="w-full">
            Continue to Sign In
          </Button>
        </Link>
      </CardFooter>
    </Card>
  );
}
