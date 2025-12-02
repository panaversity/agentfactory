/**
 * Redis client for rate limiting and session storage (optional)
 *
 * Uses Upstash Redis for serverless-friendly HTTP-based connection.
 * Falls back to memory storage if REDIS_URL is not configured.
 *
 * Environment Variables:
 * - REDIS_URL: Full Redis connection URL (Upstash format)
 *   Example: https://your-redis.upstash.io
 * - REDIS_TOKEN: Upstash Redis REST token
 */

import { Redis } from "@upstash/redis";

/**
 * Initialize Redis client if REDIS_URL and REDIS_TOKEN are provided
 */
export const redis = (() => {
  const redisUrl = process.env.REDIS_URL;
  const redisToken = process.env.REDIS_TOKEN;

  // Return null if Redis is not configured
  if (!redisUrl || !redisToken) {
    console.log("[Redis] Not configured - using memory storage for rate limiting");
    return null;
  }

  try {
    const client = new Redis({
      url: redisUrl,
      token: redisToken,
    });

    console.log("[Redis] Connected successfully");
    return client;
  } catch (error) {
    console.error("[Redis] Failed to initialize:", error);
    return null;
  }
})();

/**
 * Secondary storage adapter for Better Auth
 * Implements the storage interface required by Better Auth's secondaryStorage option
 */
export const redisStorage = redis
  ? {
      get: async (key: string) => {
        try {
          const value = await redis.get<string>(key);
          return value;
        } catch (error) {
          console.error("[Redis] Get error:", error);
          return null;
        }
      },
      set: async (key: string, value: string, ttl?: number) => {
        try {
          if (ttl) {
            await redis.set(key, value, { ex: ttl });
          } else {
            await redis.set(key, value);
          }
        } catch (error) {
          console.error("[Redis] Set error:", error);
        }
      },
      delete: async (key: string) => {
        try {
          await redis.del(key);
        } catch (error) {
          console.error("[Redis] Delete error:", error);
        }
      },
    }
  : undefined;
