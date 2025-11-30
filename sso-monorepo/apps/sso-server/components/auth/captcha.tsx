'use client';

import HCaptcha from '@hcaptcha/react-hcaptcha';
import { useRef } from 'react';

interface CaptchaProps {
  onVerify: (token: string) => void;
  onExpire?: () => void;
  onError?: (error: string) => void;
}

export function Captcha({ onVerify, onExpire, onError }: CaptchaProps) {
  const captchaRef = useRef<HCaptcha>(null);
  const siteKey = process.env.NEXT_PUBLIC_HCAPTCHA_SITE_KEY;

  if (!siteKey) {
    console.error('NEXT_PUBLIC_HCAPTCHA_SITE_KEY is not configured');
    return null;
  }

  return (
    <div className="flex justify-center my-4">
      <HCaptcha
        ref={captchaRef}
        sitekey={siteKey}
        onVerify={onVerify}
        onExpire={() => {
          onExpire?.();
        }}
        onError={(err) => {
          console.error('hCaptcha error:', err);
          onError?.(err);
        }}
      />
    </div>
  );
}
