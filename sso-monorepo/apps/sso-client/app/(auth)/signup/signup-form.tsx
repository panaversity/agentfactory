'use client';

import { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { useRouter, useSearchParams } from 'next/navigation';
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
import { signUpSchema, type SignUpFormData } from '@/lib/schemas/auth';
import { FormError } from '@/components/auth/form-error';
import { withTimeout, handleAuthError } from '@/lib/utils/api';
import { getRedirectUrl } from '@/lib/utils/redirect';
import { ERROR_MESSAGES } from '@/lib/constants';
import { Loader2 } from 'lucide-react';

export function SignUpForm() {
  const router = useRouter();
  const searchParams = useSearchParams();
  const [formError, setFormError] = useState<string>('');

  const form = useForm<SignUpFormData>({
    resolver: zodResolver(signUpSchema),
    defaultValues: {
      email: '',
      password: '',
      name: '',
    },
  });

  const { isSubmitting } = form.formState;

  async function onSubmit(data: SignUpFormData) {
    try {
      setFormError('');

      // Call auth API with timeout
      const result = await withTimeout(
        authClient.signUp.email({
          email: data.email,
          password: data.password,
          name: data.name,
          callbackURL: getRedirectUrl(searchParams),
        }),
        30000
      );

      if (result.error) {
        // Handle backend errors
        const error = result.error;
        
        // Check for email already exists
        if (error.message?.toLowerCase().includes('email') && 
            error.message?.toLowerCase().includes('exist')) {
          setFormError(ERROR_MESSAGES.EMAIL_ALREADY_EXISTS);
          form.setError('email', {
            message: ERROR_MESSAGES.EMAIL_ALREADY_EXISTS,
          });
          return;
        }

        // Check for weak password
        if (error.message?.toLowerCase().includes('password')) {
          setFormError(ERROR_MESSAGES.WEAK_PASSWORD);
          form.setError('password', {
            message: ERROR_MESSAGES.WEAK_PASSWORD,
          });
          return;
        }

        // Generic error
        setFormError(error.message || ERROR_MESSAGES.UNKNOWN);
        return;
      }

      // Success - redirect
      if (result.data) {
        const redirectUrl = getRedirectUrl(searchParams);
        router.push(redirectUrl);
      }
    } catch (error) {
      const apiError = handleAuthError(error);
      setFormError(apiError.message);
    }
  }

  return (
    <Form {...form}>
      <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">
        {formError && <FormError message={formError} />}

        <FormField
          control={form.control}
          name="name"
          render={({ field }) => (
            <FormItem>
              <FormLabel>Name</FormLabel>
              <FormControl>
                <Input
                  placeholder="John Doe"
                  autoComplete="name"
                  disabled={isSubmitting}
                  {...field}
                />
              </FormControl>
              <FormMessage />
            </FormItem>
          )}
        />

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
                <Input
                  type="password"
                  placeholder="••••••••"
                  autoComplete="new-password"
                  disabled={isSubmitting}
                  {...field}
                />
              </FormControl>
              <FormMessage />
            </FormItem>
          )}
        />

        <Button
          type="submit"
          className="w-full"
          disabled={isSubmitting}
        >
          {isSubmitting ? (
            <>
              <Loader2 className="mr-2 h-4 w-4 animate-spin" />
              Creating account...
            </>
          ) : (
            'Create account'
          )}
        </Button>
      </form>
    </Form>
  );
}
