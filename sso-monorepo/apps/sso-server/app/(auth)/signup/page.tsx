'use client';

import { Suspense } from 'react';
import Link from 'next/link';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter } from '@repo/ui';
import { SignUpForm } from './signup-form';
import { Loader2 } from 'lucide-react';

export default function SignUpPage() {
  return (
    <Card>
      <CardHeader>
        <CardTitle>Create an account</CardTitle>
        <CardDescription>
          Enter your information to create your account
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Suspense fallback={
          <div className="flex justify-center py-8">
            <Loader2 className="h-8 w-8 animate-spin text-blue-600" />
          </div>
        }>
          <SignUpForm />
        </Suspense>
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
