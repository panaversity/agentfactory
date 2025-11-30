import { z } from 'zod';

/**
 * Sign Up Form Schema
 * Validates email, password complexity, and name
 */
export const signUpSchema = z.object({
  email: z.string()
    .min(1, 'Email is required')
    .email('Invalid email format')
    .max(255, 'Email is too long'),
  password: z.string()
    .min(8, 'Password must be at least 8 characters')
    .regex(/[A-Z]/, 'Must contain an uppercase letter')
    .regex(/[a-z]/, 'Must contain a lowercase letter')
    .regex(/[0-9]/, 'Must contain a number')
    .regex(/[^A-Za-z0-9]/, 'Must contain a special character'),
  name: z.string()
    .min(1, 'Name is required')
    .max(100, 'Name is too long'),
});

export type SignUpFormData = z.infer<typeof signUpSchema>;

/**
 * Sign In Form Schema
 * Validates email and password (no complexity check on login)
 */
export const signInSchema = z.object({
  email: z.string()
    .min(1, 'Email is required')
    .email('Invalid email format'),
  password: z.string()
    .min(1, 'Password is required'),
});

export type SignInFormData = z.infer<typeof signInSchema>;

/**
 * Forgot Password Form Schema
 * Validates email only
 */
export const forgotPasswordSchema = z.object({
  email: z.string()
    .min(1, 'Email is required')
    .email('Invalid email format'),
});

export type ForgotPasswordFormData = z.infer<typeof forgotPasswordSchema>;

/**
 * Reset Password Form Schema
 * Validates token, new password, and password confirmation
 */
export const resetPasswordSchema = z.object({
  token: z.string().min(1, 'Reset token is required'),
  newPassword: z.string()
    .min(8, 'Password must be at least 8 characters')
    .regex(/[A-Z]/, 'Must contain an uppercase letter')
    .regex(/[a-z]/, 'Must contain a lowercase letter')
    .regex(/[0-9]/, 'Must contain a number')
    .regex(/[^A-Za-z0-9]/, 'Must contain a special character'),
  confirmPassword: z.string(),
}).refine((data) => data.newPassword === data.confirmPassword, {
  message: 'Passwords do not match',
  path: ['confirmPassword'],
});

export type ResetPasswordFormData = z.infer<typeof resetPasswordSchema>;
