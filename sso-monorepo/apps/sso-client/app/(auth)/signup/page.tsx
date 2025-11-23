'use client';

import Link from 'next/link';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter } from '@repo/ui';
import { SignUpForm } from './signup-form';
import { useAuthRedirect } from '@/lib/hooks/use-auth-redirect';

export default function SignUpPage() {
  // Redirect logged-in users to dashboard
  useAuthRedirect();

  return (
    <Card>
      <CardHeader>
        <CardTitle>Create an account</CardTitle>
        <CardDescription>
          Enter your information to create your account
        </CardDescription>
      </CardHeader>
      <CardContent>
        <SignUpForm />
      </CardContent>
      <CardFooter>
        <p className="text-sm text-gray-600 dark:text-gray-400 text-center w-full">
          Already have an account?{' '}
          <Link href="/signin" className="text-blue-600 hover:underline dark:text-blue-400">
            Sign in
          </Link>
        </p>
      </CardFooter>
    </Card>
  );
}
