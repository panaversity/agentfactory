import { Resend } from 'resend';
import nodemailer from 'nodemailer';

// Email provider type
type EmailProvider = 'resend' | 'smtp' | 'none';

// Determine which email provider to use based on environment variables
const getEmailProvider = (): EmailProvider => {
  if (process.env.RESEND_API_KEY) {
    return 'resend';
  }
  if (process.env.SMTP_HOST && process.env.SMTP_USER && process.env.SMTP_PASS) {
    return 'smtp';
  }
  return 'none';
};

const EMAIL_PROVIDER = getEmailProvider();

// Initialize Resend (only if API key is available)
const resend = EMAIL_PROVIDER === 'resend' ? new Resend(process.env.RESEND_API_KEY) : null;

// Initialize SMTP transporter (only if SMTP credentials are available)
const smtpTransporter = EMAIL_PROVIDER === 'smtp'
  ? nodemailer.createTransport({
      host: process.env.SMTP_HOST!,
      port: parseInt(process.env.SMTP_PORT || '587'),
      secure: process.env.SMTP_SECURE === 'true', // true for 465, false for other ports
      auth: {
        user: process.env.SMTP_USER!,
        pass: process.env.SMTP_PASS!,
      },
    })
  : null;

// Email configuration
const EMAIL_FROM = process.env.EMAIL_FROM || 'noreply@example.com';
const APP_NAME = process.env.APP_NAME || 'SSO Platform';

export interface SendEmailOptions {
  to: string;
  subject: string;
  html?: string;
  text?: string;
}

/**
 * Send an email using the configured provider (Resend or SMTP)
 */
export async function sendEmail({ to, subject, html, text }: SendEmailOptions) {
  // Log email provider being used
  if (EMAIL_PROVIDER === 'none') {
    console.warn('⚠️ No email provider configured. Email not sent:', { to, subject });
    console.warn('Configure either RESEND_API_KEY or SMTP credentials (SMTP_HOST, SMTP_USER, SMTP_PASS)');
    return { message: 'Email provider not configured' };
  }

  try {
    if (EMAIL_PROVIDER === 'resend') {
      // Use Resend - ensure we have valid content
      const emailContent = html || text || '';
      const { data, error } = await resend!.emails.send({
        from: EMAIL_FROM,
        to,
        subject,
        html: emailContent,
      });

      if (error) {
        console.error('Failed to send email via Resend:', error);
        throw new Error(`Email sending failed: ${error.message}`);
      }

      console.log('✅ Email sent successfully via Resend:', data);
      return data;

    } else if (EMAIL_PROVIDER === 'smtp') {
      // Use SMTP
      const info = await smtpTransporter!.sendMail({
        from: EMAIL_FROM,
        to,
        subject,
        html: html || text,
        text,
      });

      console.log('✅ Email sent successfully via SMTP:', info.messageId);
      return { id: info.messageId };
    }
  } catch (error) {
    console.error(`Email error (${EMAIL_PROVIDER}):`, error);
    throw error;
  }
}

/**
 * Send email verification email
 */
export async function sendVerificationEmail({
  to,
  name,
  verificationUrl,
}: {
  to: string;
  name?: string;
  verificationUrl: string;
}) {
  const subject = `Verify your email for ${APP_NAME}`;

  const html = `
    <!DOCTYPE html>
    <html>
      <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Verify Your Email</title>
      </head>
      <body style="font-family: Arial, sans-serif; line-height: 1.6; color: #333; max-width: 600px; margin: 0 auto; padding: 20px;">
        <div style="background: linear-gradient(to right, #4F46E5, #7C3AED); padding: 30px; text-align: center; border-radius: 10px 10px 0 0;">
          <h1 style="color: white; margin: 0; font-size: 28px;">${APP_NAME}</h1>
        </div>

        <div style="background: #f9fafb; padding: 30px; border-radius: 0 0 10px 10px;">
          <h2 style="color: #1f2937; margin-top: 0;">Welcome${name ? `, ${name}` : ''}!</h2>

          <p style="font-size: 16px; color: #4b5563;">
            Thank you for signing up. Please verify your email address to complete your registration and access all features.
          </p>

          <div style="text-align: center; margin: 35px 0;">
            <a href="${verificationUrl}"
               style="background: #4F46E5; color: white; padding: 14px 30px; text-decoration: none; border-radius: 6px; font-weight: bold; display: inline-block; font-size: 16px;">
              Verify Email Address
            </a>
          </div>

          <p style="font-size: 14px; color: #6b7280; margin-top: 30px;">
            Or copy and paste this link into your browser:
          </p>
          <p style="font-size: 12px; color: #9ca3af; word-break: break-all; background: white; padding: 10px; border-radius: 4px; border: 1px solid #e5e7eb;">
            ${verificationUrl}
          </p>

          <hr style="border: none; border-top: 1px solid #e5e7eb; margin: 30px 0;">

          <p style="font-size: 12px; color: #9ca3af; margin: 0;">
            This verification link will expire in 24 hours. If you didn't create an account, you can safely ignore this email.
          </p>
        </div>
      </body>
    </html>
  `;

  const text = `
Welcome to ${APP_NAME}${name ? `, ${name}` : ''}!

Please verify your email address by clicking the link below:

${verificationUrl}

This link will expire in 24 hours.

If you didn't create an account, you can safely ignore this email.
  `.trim();

  return sendEmail({ to, subject, html, text });
}

/**
 * Send password reset email
 */
export async function sendPasswordResetEmail({
  to,
  name,
  resetUrl,
}: {
  to: string;
  name?: string;
  resetUrl: string;
}) {
  const subject = `Reset your password for ${APP_NAME}`;

  const html = `
    <!DOCTYPE html>
    <html>
      <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Reset Your Password</title>
      </head>
      <body style="font-family: Arial, sans-serif; line-height: 1.6; color: #333; max-width: 600px; margin: 0 auto; padding: 20px;">
        <div style="background: linear-gradient(to right, #DC2626, #EA580C); padding: 30px; text-align: center; border-radius: 10px 10px 0 0;">
          <h1 style="color: white; margin: 0; font-size: 28px;">${APP_NAME}</h1>
        </div>

        <div style="background: #f9fafb; padding: 30px; border-radius: 0 0 10px 10px;">
          <h2 style="color: #1f2937; margin-top: 0;">Password Reset Request</h2>

          <p style="font-size: 16px; color: #4b5563;">
            Hi${name ? ` ${name}` : ''},
          </p>

          <p style="font-size: 16px; color: #4b5563;">
            We received a request to reset your password. Click the button below to create a new password:
          </p>

          <div style="text-align: center; margin: 35px 0;">
            <a href="${resetUrl}"
               style="background: #DC2626; color: white; padding: 14px 30px; text-decoration: none; border-radius: 6px; font-weight: bold; display: inline-block; font-size: 16px;">
              Reset Password
            </a>
          </div>

          <p style="font-size: 14px; color: #6b7280; margin-top: 30px;">
            Or copy and paste this link into your browser:
          </p>
          <p style="font-size: 12px; color: #9ca3af; word-break: break-all; background: white; padding: 10px; border-radius: 4px; border: 1px solid #e5e7eb;">
            ${resetUrl}
          </p>

          <hr style="border: none; border-top: 1px solid #e5e7eb; margin: 30px 0;">

          <p style="font-size: 12px; color: #9ca3af; margin: 0;">
            This password reset link will expire in 1 hour. If you didn't request a password reset, you can safely ignore this email. Your password will remain unchanged.
          </p>
        </div>
      </body>
    </html>
  `;

  const text = `
Password Reset Request for ${APP_NAME}

Hi${name ? ` ${name}` : ''},

We received a request to reset your password. Click the link below to create a new password:

${resetUrl}

This link will expire in 1 hour.

If you didn't request a password reset, you can safely ignore this email. Your password will remain unchanged.
  `.trim();

  return sendEmail({ to, subject, html, text });
}

/**
 * Get current email provider info (for debugging)
 */
export function getEmailProviderInfo() {
  return {
    provider: EMAIL_PROVIDER,
    from: EMAIL_FROM,
    configured: EMAIL_PROVIDER !== 'none',
  };
}
