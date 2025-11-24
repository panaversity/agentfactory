'use client';

import { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { forgetPasswordAction } from '@/app/server/auth';
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
import { forgotPasswordSchema, type ForgotPasswordFormData } from '@/lib/schemas/auth';
import { FormError } from '@/components/auth/form-error';
import { withTimeout, handleAuthError } from '@/lib/utils/api';
import { ERROR_MESSAGES } from '@/lib/constants';
import { Loader2, CheckCircle } from 'lucide-react';

export function ForgotPasswordForm() {
  const [formError, setFormError] = useState<string>('');
  const [successMessage, setSuccessMessage] = useState<string>('');

  const form = useForm<ForgotPasswordFormData>({
    resolver: zodResolver(forgotPasswordSchema),
    defaultValues: {
      email: '',
    },
  });

  const { isSubmitting } = form.formState;

  async function onSubmit(data: ForgotPasswordFormData) {
    try {
      setFormError('');
      setSuccessMessage('');

      // Call server action with timeout
      const result = await withTimeout(
        forgetPasswordAction({
          email: data.email,
          redirectTo: `${window.location.origin}/reset-password`,
        }),
        30000
      );

      if (result.error) {
        // Handle backend errors
        setFormError(result.error.message || ERROR_MESSAGES.UNKNOWN);
        return;
      }

      // Success - show generic message (prevent user enumeration)
      if (result.data) {
        setSuccessMessage(
          'If an account exists with this email, you will receive password reset instructions shortly. Please check your inbox and spam folder.'
        );
        form.reset();
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
        
        {successMessage && (
          <Alert className="bg-green-50 dark:bg-green-900 border-green-200 dark:border-green-800">
            <CheckCircle className="h-4 w-4 text-green-600 dark:text-green-400" />
            <div className="ml-2">
              <p className="text-sm text-green-800 dark:text-green-200">
                {successMessage}
              </p>
            </div>
          </Alert>
        )}

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

        <Button
          type="submit"
          className="w-full"
          disabled={isSubmitting}
        >
          {isSubmitting ? (
            <>
              <Loader2 className="mr-2 h-4 w-4 animate-spin" />
              Sending reset link...
            </>
          ) : (
            'Send reset link'
          )}
        </Button>
      </form>
    </Form>
  );
}
