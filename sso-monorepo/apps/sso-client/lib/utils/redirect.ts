/**
 * Get Redirect URL from Search Params
 * Returns the callbackUrl from query parameters or defaults to /dashboard
 * 
 * @param searchParams - URLSearchParams from Next.js
 * @returns The redirect URL (callbackUrl or /dashboard)
 */
export function getRedirectUrl(searchParams: URLSearchParams | Record<string, string | string[]>): string {
  // Handle URLSearchParams
  if (searchParams instanceof URLSearchParams) {
    const callbackUrl = searchParams.get('callbackUrl');
    return callbackUrl || '/dashboard';
  }
  
  // Handle Next.js params object
  const callbackUrl = searchParams.callbackUrl;
  if (typeof callbackUrl === 'string') {
    return callbackUrl;
  }
  if (Array.isArray(callbackUrl) && callbackUrl.length > 0) {
    return callbackUrl[0];
  }
  
  return '/dashboard';
}
