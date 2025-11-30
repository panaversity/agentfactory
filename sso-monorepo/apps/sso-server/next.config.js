const path = require('path');

/** @type {import('next').NextConfig} */
const nextConfig = {
  // Set turbopack root to monorepo root to avoid lockfile warning
  turbopack: {
    root: path.resolve(__dirname, '../../'),
  },
  async rewrites() {
    return [
      {
        source: '/.well-known/openid-configuration',
        destination: '/api/auth/oauth2/.well-known/openid-configuration',
      },
      {
        source: '/.well-known/jwks.json',
        destination: '/api/auth/jwks',
      },
    ];
  },
  transpilePackages: ['@repo/auth-config', '@repo/database'],
  env: {
    // Pass through required env vars
    DATABASE_URL: process.env.DATABASE_URL,
    BETTER_AUTH_SECRET: process.env.BETTER_AUTH_SECRET,
    BETTER_AUTH_URL: process.env.BETTER_AUTH_URL,
  },
};

module.exports = nextConfig;
