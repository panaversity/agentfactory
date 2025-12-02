import { NextResponse } from 'next/server';
import type { NextRequest } from 'next/server';

/**
 * CORS Middleware
 * 
 * Handles CORS for API routes dynamically based on request origin.
 * Only allows origins specified in ALLOWED_ORIGINS environment variable.
 * 
 * This is necessary because Next.js config headers() are static and can't
 * dynamically check the request origin.
 */
export function middleware(request: NextRequest) {
  // Only apply to API routes
  if (!request.nextUrl.pathname.startsWith('/api')) {
    return NextResponse.next();
  }

  // Get allowed origins from environment variable
  const allowedOrigins = process.env.ALLOWED_ORIGINS?.split(',').map(origin => origin.trim()) || [
    'http://localhost:3000',
  ];

  // Get the origin from the request
  const origin = request.headers.get('origin');

  // Check if origin is allowed
  const isAllowedOrigin = origin && allowedOrigins.includes(origin);

  // Create response
  const response = NextResponse.next();

  // Set CORS headers
  if (isAllowedOrigin) {
    response.headers.set('Access-Control-Allow-Origin', origin);
    response.headers.set('Access-Control-Allow-Credentials', 'true');
    response.headers.set('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
    response.headers.set(
      'Access-Control-Allow-Headers',
      'Content-Type, Authorization, X-Requested-With'
    );
  }

  // Handle preflight OPTIONS request
  if (request.method === 'OPTIONS') {
    return new NextResponse(null, {
      status: 200,
      headers: response.headers,
    });
  }

  return response;
}

// Apply middleware to all API routes
export const config = {
  matcher: '/api/:path*',
};


