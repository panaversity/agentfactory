'use client';

import { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { useRouter, useSearchParams } from 'next/navigation';
import Link from 'next/link';
import { authClient } from '@repo/auth-config/client';
import {
  Form,
  FormControl,
  FormField,
  FormItem,
  FormLabel,
  FormMessage,
  Button,
  Input
} from '@repo/ui';
import { signInSchema, type SignInFormData } from '@/lib/schemas/auth';
import { FormError } from '@/components/auth/form-error';
import { SocialLoginButtons } from '@/components/auth/social-login-buttons';
import { PasswordInput } from '@/components/auth/password-input';
import { normalizeAuthError } from '@/lib/utils/auth-errors';
import { getRedirectUrl } from '@/lib/utils/redirect';
import { Loader2 } from 'lucide-react';

export function SignInForm() {
  const router = useRouter();
  const searchParams = useSearchParams();
  const [formError, setFormError] = useState<string>('');

  const form = useForm<SignInFormData>({
    resolver: zodResolver(signInSchema),
    defaultValues: {
      email: '',
      password: '',
    },
  });

  const { isSubmitting } = form.formState;

  async function onSubmit(data: SignInFormData) {
    try {
      setFormError('');

      // Use better-auth client for sign in (maintains OAuth flow)
      const result = await authClient.signIn.email({
        email: data.email,
        password: data.password,
        callbackURL: getRedirectUrl(searchParams),
      });

      if (result.error) {
        // Normalize error message
        const normalizedError = normalizeAuthError(result.error);
        setFormError(normalizedError.message);
        return;
      }

      // Success - better-auth OIDC plugin will handle OAuth redirection
      // For standard sign-in, redirect to the callback URL
      if (result.data) {
        const redirectUrl = getRedirectUrl(searchParams);
        router.push(redirectUrl);
        router.refresh(); // Force refresh to update session
      }
    } catch (error) {
      const normalizedError = normalizeAuthError(error);
      setFormError(normalizedError.message);
    }
  }

  return (
    <Form {...form}>
      <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">
        {formError && <FormError message={formError} />}

        <FormField
          control={form.control}
          name="email"
          render={({ field }) => (
            <FormItem>
              <FormLabel>Email</FormLabel>
              <FormControl>
                <Input
                  type="email"
                  placeholder="john@example.com"
                  autoComplete="email"
                  disabled={isSubmitting}
                  autoFocus
                  {...field}
                />
              </FormControl>
              <FormMessage />
            </FormItem>
          )}
        />

        <FormField
          control={form.control}
          name="password"
          render={({ field }) => (
            <FormItem>
              <FormLabel>Password</FormLabel>
              <FormControl>
                <PasswordInput
                  placeholder="••••••••"
                  autoComplete="current-password"
                  disabled={isSubmitting}
                  {...field}
                />
              </FormControl>
              <FormMessage />
            </FormItem>
          )}
        />

        <div className="flex items-center justify-end">
          <Link 
            href="/forgot-password" 
            className="text-sm text-blue-600 hover:underline dark:text-blue-400"
          >
            Forgot password?
          </Link>
        </div>

        <Button
          type="submit"
          className="w-full"
          disabled={isSubmitting}
        >
          {isSubmitting ? (
            <>
              <Loader2 className="mr-2 h-4 w-4 animate-spin" />
              Signing in...
            </>
          ) : (
            'Sign in'
          )}
        </Button>

        <SocialLoginButtons callbackUrl={getRedirectUrl(searchParams)} />
      </form>
    </Form>
  );
}
