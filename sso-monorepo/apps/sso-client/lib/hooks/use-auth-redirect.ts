'use client';

import { useEffect } from 'react';
import { useRouter, useSearchParams } from 'next/navigation';
import { authClient } from '@repo/auth-config/client';
import { getRedirectUrl } from '@/lib/utils/redirect';

/**
 * useAuthRedirect Hook
 * Checks if user is already authenticated and redirects them away from auth pages
 */
export function useAuthRedirect() {
  const router = useRouter();
  const searchParams = useSearchParams();

  useEffect(() => {
    async function checkSession() {
      try {
        const { data } = await authClient.getSession();
        if (data?.session) {
          // User is already logged in, redirect them
          const redirectUrl = getRedirectUrl(searchParams);
          router.replace(redirectUrl);
        }
      } catch {
        // Ignore errors - user is not logged in
      }
    }
    
    checkSession();
  }, [router, searchParams]);
}
