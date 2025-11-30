'use client';

import { Suspense } from 'react';
import Link from 'next/link';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter } from '@repo/ui';
import { SignInForm } from './signin-form';
import { Loader2 } from 'lucide-react';

export default function SignInPage() {
  return (
    <Card>
      <CardHeader>
        <CardTitle>Sign In</CardTitle>
        <CardDescription>
          Enter your email and password to access your account
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Suspense fallback={
          <div className="flex justify-center py-8">
            <Loader2 className="h-8 w-8 animate-spin text-blue-600" />
          </div>
        }>
          <SignInForm />
        </Suspense>
      </CardContent>
      <CardFooter>
        <p className="text-sm text-gray-600 dark:text-gray-400 text-center w-full">
          Don't have an account?{' '}
          <Link href="/signup" className="text-blue-600 hover:underline dark:text-blue-400">
            Sign up
          </Link>
        </p>
      </CardFooter>
    </Card>
  );
}
