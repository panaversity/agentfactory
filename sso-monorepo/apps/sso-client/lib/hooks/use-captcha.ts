'use client';

import { useState, useCallback } from 'react';

export interface CaptchaState {
  required: boolean;
  token: string | null;
  verified: boolean;
}

export function useCaptcha(initialRequired: boolean = false) {
  const [captcha, setCaptcha] = useState<CaptchaState>({
    required: initialRequired,
    token: null,
    verified: false,
  });

  const showCaptcha = useCallback(() => {
    setCaptcha(prev => ({ ...prev, required: true }));
  }, []);

  const hideCaptcha = useCallback(() => {
    setCaptcha({ required: false, token: null, verified: false });
  }, []);

  const setToken = useCallback((token: string | null) => {
    setCaptcha(prev => ({ ...prev, token, verified: !!token }));
  }, []);

  const reset = useCallback(() => {
    setCaptcha({ required: false, token: null, verified: false });
  }, []);

  return {
    captcha,
    showCaptcha,
    hideCaptcha,
    setToken,
    reset,
  };
}
