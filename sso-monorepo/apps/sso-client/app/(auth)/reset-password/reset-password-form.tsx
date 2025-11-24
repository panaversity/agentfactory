'use client';

import { useState, useEffect } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { useRouter, useSearchParams } from 'next/navigation';
import { resetPasswordAction } from '@/app/server/auth';
import { 
  Form, 
  FormControl, 
  FormField, 
  FormItem, 
  FormLabel, 
  FormMessage,
  Button,
  Input,
  Alert
} from '@repo/ui';
import { resetPasswordSchema, type ResetPasswordFormData } from '@/lib/schemas/auth';
import { FormError } from '@/components/auth/form-error';
import { withTimeout, handleAuthError } from '@/lib/utils/api';
import { ERROR_MESSAGES } from '@/lib/constants';
import { Loader2, XCircle, CheckCircle } from 'lucide-react';
import Link from 'next/link';

export function ResetPasswordForm() {
  const router = useRouter();
  const searchParams = useSearchParams();
  const token = searchParams.get('token');
  
  const [formError, setFormError] = useState<string>('');
  const [tokenError, setTokenError] = useState<string>('');
  const [successMessage, setSuccessMessage] = useState<string>('');
  const [countdown, setCountdown] = useState(3);

  const form = useForm<ResetPasswordFormData>({
    resolver: zodResolver(resetPasswordSchema),
    defaultValues: {
      token: token || '',
      newPassword: '',
      confirmPassword: '',
    },
  });

  const { isSubmitting } = form.formState;

  // Check if token exists
  useEffect(() => {
    if (!token) {
      setTokenError('Reset token is missing. Please use the link from your email.');
    }
  }, [token]);

  // Handle countdown and redirect after success
  useEffect(() => {
    if (successMessage) {
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
    }
  }, [successMessage, router]);

  async function onSubmit(data: ResetPasswordFormData) {
    try {
      setFormError('');

      // Call server action with timeout
      const result = await withTimeout(
        resetPasswordAction({
          newPassword: data.newPassword,
          token: data.token,
        }),
        30000
      );

      if (result.error) {
        // Handle backend errors
        const error = result.error;

        // Check for token expiration
        if (error.message?.toLowerCase().includes('expired')) {
          setFormError(ERROR_MESSAGES.TOKEN_EXPIRED);
          return;
        }

        // Check for invalid token
        if (error.message?.toLowerCase().includes('invalid')) {
          setFormError(ERROR_MESSAGES.INVALID_TOKEN);
          return;
        }

        // Generic error
        setFormError(error.message || ERROR_MESSAGES.UNKNOWN);
        return;
      }

      // Success
      if (result.data) {
        setSuccessMessage('Your password has been reset successfully!');
      }
    } catch (error) {
      const apiError = handleAuthError(error);
      setFormError(apiError.message);
    }
  }

  // Show token error if no token
  if (tokenError) {
    return (
      <div className="space-y-4">
        <Alert className="bg-red-50 dark:bg-red-900 border-red-200 dark:border-red-800">
          <XCircle className="h-4 w-4 text-red-600 dark:text-red-400" />
          <div className="ml-2">
            <p className="text-sm text-red-800 dark:text-red-200">
              {tokenError}
            </p>
          </div>
        </Alert>
        <div className="flex flex-col space-y-2">
          <Link href="/forgot-password">
            <Button className="w-full">
              Request New Reset Link
            </Button>
          </Link>
          <Link href="/signin">
            <Button variant="outline" className="w-full">
              Go to Sign In
            </Button>
          </Link>
        </div>
      </div>
    );
  }

  // Show success message
  if (successMessage) {
    return (
      <div className="space-y-4">
        <Alert className="bg-green-50 dark:bg-green-900 border-green-200 dark:border-green-800">
          <CheckCircle className="h-4 w-4 text-green-600 dark:text-green-400" />
          <div className="ml-2">
            <p className="text-sm text-green-800 dark:text-green-200">
              {successMessage}
            </p>
            <p className="text-xs text-green-700 dark:text-green-300 mt-1">
              Redirecting to sign in page in {countdown} second{countdown !== 1 ? 's' : ''}...
            </p>
          </div>
        </Alert>
        <Link href="/signin">
          <Button className="w-full">
            Continue to Sign In
          </Button>
        </Link>
      </div>
    );
  }

  return (
    <Form {...form}>
      <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">
        {formError && <FormError message={formError} />}

        <FormField
          control={form.control}
          name="newPassword"
          render={({ field }) => (
            <FormItem>
              <FormLabel>New Password</FormLabel>
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
              <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
                Must be at least 8 characters with uppercase, lowercase, number, and special character
              </p>
            </FormItem>
          )}
        />

        <FormField
          control={form.control}
          name="confirmPassword"
          render={({ field }) => (
            <FormItem>
              <FormLabel>Confirm Password</FormLabel>
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
              Resetting password...
            </>
          ) : (
            'Reset password'
          )}
        </Button>
      </form>
    </Form>
  );
}
