/**
 * Get Redirect URL from Search Params
 * For SSO server: Returns the callbackUrl from query parameters or defaults to root
 * Note: Better-auth OIDC provider automatically handles OAuth flow continuation
 *
 * @param searchParams - URLSearchParams from Next.js
 * @returns The redirect URL (callbackUrl or /)
 */
export function getRedirectUrl(searchParams: URLSearchParams | Record<string, string | string[]>): string {
  // Handle URLSearchParams
  if (searchParams instanceof URLSearchParams) {
    const callbackUrl = searchParams.get('callbackUrl');
    return callbackUrl || '/';
  }

  // Handle Next.js params object
  const callbackUrl = searchParams.callbackUrl;
  if (typeof callbackUrl === 'string') {
    return callbackUrl;
  }
  if (Array.isArray(callbackUrl) && callbackUrl.length > 0) {
    return callbackUrl[0];
  }

  // Default to root - better-auth will handle OAuth continuation automatically
  return '/';
}
