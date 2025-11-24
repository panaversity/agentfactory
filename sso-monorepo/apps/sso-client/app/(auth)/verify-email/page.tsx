'use client';

import { useSearchParams } from 'next/navigation';
import Link from 'next/link';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter, Button } from '@repo/ui';
import { Mail, CheckCircle } from 'lucide-react';

export default function VerifyEmailPage() {
  const searchParams = useSearchParams();
  const email = searchParams.get('email');

  return (
    <Card>
      <CardHeader>
        <div className="flex justify-center mb-4">
          <div className="rounded-full bg-green-100 dark:bg-green-900 p-3">
            <CheckCircle className="h-8 w-8 text-green-600 dark:text-green-400" />
          </div>
        </div>
        <CardTitle className="text-center">Account Created Successfully!</CardTitle>
        <CardDescription className="text-center">
          Please verify your email address to complete registration
        </CardDescription>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="flex justify-center">
          <div className="rounded-full bg-blue-100 dark:bg-blue-900 p-4">
            <Mail className="h-12 w-12 text-blue-600 dark:text-blue-400" />
          </div>
        </div>
        <div className="text-center space-y-2">
          <p className="text-sm text-gray-700 dark:text-gray-300">
            We've sent a verification email to:
          </p>
          {email && (
            <p className="text-sm font-semibold text-gray-900 dark:text-white">
              {email}
            </p>
          )}
          <p className="text-sm text-gray-600 dark:text-gray-400 mt-4">
            Please check your inbox and click the verification link to activate your account.
          </p>
          <p className="text-xs text-gray-500 dark:text-gray-500 mt-2">
            Don't forget to check your spam folder if you don't see the email.
          </p>
        </div>
      </CardContent>
      <CardFooter className="flex flex-col space-y-2">
        <Link href="/signin" className="w-full">
          <Button variant="outline" className="w-full">
            Go to Sign In
          </Button>
        </Link>
        <p className="text-xs text-center text-gray-500 dark:text-gray-500">
          You'll be able to sign in after verifying your email
        </p>
      </CardFooter>
    </Card>
  );
}
