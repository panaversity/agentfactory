# RoboLearn Auth Server

Standalone authentication server for the RoboLearn platform using Better Auth with Next.js.

## Features

- Email/password authentication
- User profile with software background level (beginner/intermediate/advanced)
- Session management (7-day duration)
- Rate limiting (5 attempts/minute/IP)
- CORS support for robolearn-interface

## Tech Stack

- **Framework**: Next.js 15 (App Router)
- **Auth**: Better Auth
- **Database**: Neon Postgres (serverless)
- **ORM**: Drizzle

## Getting Started

### Prerequisites

- Node.js 20+
- Neon Postgres database

### Installation

```bash
cd auth-server
npm install
```

### Environment Variables

Copy `.env.example` to `.env.local` and fill in:

```env
# Database (Neon Postgres)
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# Better Auth
BETTER_AUTH_SECRET=your-secret-key-minimum-32-characters-long
BETTER_AUTH_URL=http://localhost:3001

# CORS - Allowed origins (comma-separated)
ALLOWED_ORIGINS=http://localhost:3000,https://your-production-domain.com

# Public URL for client-side redirects
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3001
NEXT_PUBLIC_BOOK_URL=http://localhost:3000
```

### Database Setup

Push the schema to your database:

```bash
npm run db:push
```

### Development

```bash
npm run dev
```

Server runs on http://localhost:3001

## API Endpoints

### Authentication (Better Auth)

- `POST /api/auth/sign-up` - Register new user
- `POST /api/auth/sign-in/email` - Sign in with email/password
- `POST /api/auth/sign-out` - Sign out
- `GET /api/auth/session` - Get current session

### Profile

- `GET /api/profile` - Get current user's profile (authenticated)
- `POST /api/profile` - Create user profile after signup (authenticated)
- `PUT /api/profile` - Update software background level (authenticated)

## Integration with robolearn-interface

Add to your robolearn-interface:

```typescript
// lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "http://localhost:3001", // or production URL
});
```

## Deployment

### Vercel

1. Connect repository to Vercel
2. Set environment variables
3. Deploy

### Environment Variables for Production

- `DATABASE_URL` - Neon Postgres connection string
- `BETTER_AUTH_SECRET` - Random 32+ character secret
- `BETTER_AUTH_URL` - Production auth server URL
- `ALLOWED_ORIGINS` - Production origins (comma-separated)
