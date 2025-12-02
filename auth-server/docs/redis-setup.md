# Redis Rate Limiting Setup

This guide explains how to configure Redis for distributed rate limiting when deploying multiple auth-server instances.

## When Do You Need Redis?

| Deployment Type | Redis Required? | Why |
|----------------|-----------------|-----|
| **Single Instance** | ❌ No | Memory storage works fine for single-server deployments |
| **Multiple Instances** | ✅ Yes | Prevents rate limit bypass across different server instances |
| **Cloud Run / Kubernetes** | ✅ Yes | Auto-scaling creates multiple instances |
| **Load Balanced** | ✅ Yes | Requests distributed across multiple servers |

## Quick Setup (5 Minutes)

### 1. Create Upstash Redis Database

**Why Upstash?**
- Serverless Redis (HTTP-based, no connection pooling issues)
- Free tier: 10,000 commands/day (sufficient for most use cases)
- Global edge replication for low latency (<10ms)
- Perfect for serverless deployments (Cloud Run, Vercel, etc.)

**Steps:**
1. Go to [upstash.com](https://upstash.com/) and sign up
2. Create a new Redis database
   - Name: `robolearn-auth-ratelimit`
   - Type: **Global** (recommended for low latency worldwide)
   - Enable **Eviction** (automatically delete old rate limit data)
3. Copy credentials from the **REST API** tab:
   - `UPSTASH_REDIS_REST_URL` → Use as `REDIS_URL`
   - `UPSTASH_REDIS_REST_TOKEN` → Use as `REDIS_TOKEN`

### 2. Configure Environment Variables

Add to your `.env.local`:

```bash
# Redis Rate Limiting (Upstash)
REDIS_URL=https://worthy-condor-12345.upstash.io
REDIS_TOKEN=AYasASQgxxx...your-token-here
```

### 3. Install Dependencies

```bash
cd auth-server
pnpm install
```

The `@upstash/redis` package is already in `package.json`.

### 4. Verify Configuration

Start the server and check logs:

```bash
pnpm dev
```

You should see:
```
[Redis] Connected successfully
```

If Redis is not configured, you'll see:
```
[Redis] Not configured - using memory storage for rate limiting
```

## How It Works

### Without Redis (Memory Storage)

```
User → Instance 1 (memory: 5 attempts)
User → Instance 2 (memory: 0 attempts)  ← Rate limit bypassed!
```

Each instance has its own memory, so users can bypass rate limits by hitting different instances.

### With Redis (Distributed Storage)

```
User → Instance 1 → Redis (5 attempts)
User → Instance 2 → Redis (5 attempts)  ← Rate limit enforced!
```

All instances share the same Redis storage, so rate limits work correctly across the cluster.

## Rate Limit Configuration

The auth server automatically uses Redis when configured:

```typescript
// auth-server/src/lib/auth.ts
rateLimit: {
  storage: redis ? "secondary-storage" : "memory",

  customRules: {
    "/sign-in/email": { window: 60, max: 10 },    // 10 login attempts/min
    "/sign-up/email": { window: 60, max: 5 },     // 5 signups/min
    "/oauth2/token": { window: 60, max: 100 },    // 100 token exchanges/min
  }
}
```

## Testing Rate Limiting

### Test 1: Verify Redis Connection

```bash
# Check server logs for Redis connection
pnpm dev

# Expected output:
# [Redis] Connected successfully
```

### Test 2: Test Rate Limits

```bash
# Attempt 11 logins (should be rate limited after 10)
for i in {1..11}; do
  curl -X POST http://localhost:3001/api/auth/sign-in/email \
    -H "Content-Type: application/json" \
    -d '{"email":"test@example.com","password":"wrong"}' \
    && echo " - Attempt $i"
done

# Expected: First 10 succeed, 11th returns 429 Too Many Requests
```

### Test 3: Verify Cross-Instance Enforcement

If you have multiple instances running (e.g., via Docker Compose):

```bash
# Hit Instance 1 five times
for i in {1..5}; do
  curl http://instance1:3001/api/auth/sign-in/email -d '...'
done

# Hit Instance 2 six times (should be rate limited after 5 more)
for i in {1..6}; do
  curl http://instance2:3001/api/auth/sign-in/email -d '...'
done

# Expected: Rate limit enforced at 10 total attempts across both instances
```

## Alternative Redis Providers

| Provider | Free Tier | Latency | Best For |
|----------|-----------|---------|----------|
| **Upstash** | 10k req/day | <10ms (global) | Production (recommended) |
| **Redis Cloud** | 30MB storage | ~20ms | Testing |
| **Render** | 25MB + 100 conn | ~15ms | Small projects |

### Using Redis Cloud

```bash
# Redis Cloud provides a standard Redis URL
REDIS_URL=redis://default:password@redis-12345.c123.us-east-1-1.ec2.cloud.redislabs.com:12345
REDIS_TOKEN=your-password
```

**Note**: Upstash is recommended because it's HTTP-based (no TCP connection pooling), which works better with serverless deployments.

## Performance Impact

### Request Overhead
- **Without Redis**: ~0.1ms (in-memory lookup)
- **With Redis**: ~2-5ms (Upstash global edge)

### Upstash Free Tier Capacity

10,000 commands/day = **~6 requests/minute sustained load**

Typical auth server usage:
- 1 rate limit check per request
- 100 active users × 10 requests/hour = ~17 req/min
- **Sufficient for ~350 daily active users** on free tier

For larger scale:
- **Upstash Pay-as-you-go**: $0.20 per 100k commands (~$6/month for 1M users)
- **Enterprise tier**: Unlimited commands, ~$50/month

## Troubleshooting

### Redis Connection Fails

**Error**: `[Redis] Failed to initialize: Error: connect ECONNREFUSED`

**Solution**: Check that `REDIS_URL` and `REDIS_TOKEN` are correct. For Upstash, use the **REST API** credentials, not the Redis connection string.

### Rate Limits Not Enforced

**Symptom**: Users can exceed rate limits

**Checklist**:
1. ✅ Verify Redis is connected: Check logs for `[Redis] Connected successfully`
2. ✅ Check `storage` config: Should be `"secondary-storage"` when Redis is enabled
3. ✅ Verify Upstash dashboard shows commands being executed
4. ✅ Test with curl to confirm rate limit returns 429 after threshold

### High Redis Usage

**Symptom**: Hitting Upstash free tier limits quickly

**Solutions**:
1. **Increase TTL**: Rate limit windows are short (60s default), so data expires quickly
2. **Disable rate limits on internal endpoints**: e.g., `/get-session` already has `false` in config
3. **Upgrade to paid tier**: Pay-as-you-go is very affordable ($0.20 per 100k commands)

## Production Checklist

Before deploying to production with Redis:

- [ ] Redis credentials stored in secure environment variables (not committed to git)
- [ ] Redis URL uses HTTPS (Upstash REST API)
- [ ] Verified rate limits work correctly with curl tests
- [ ] Monitored Upstash dashboard to confirm commands are being executed
- [ ] Set up alerts for Redis connection failures (optional)
- [ ] Documented Redis credentials in team password manager
- [ ] Tested failover: Server falls back to memory storage if Redis is unavailable

## Rollback

If you need to disable Redis:

1. Remove `REDIS_URL` and `REDIS_TOKEN` from environment variables
2. Restart the server
3. Rate limiting will automatically fall back to memory storage

**Note**: This only works for single-instance deployments. Multi-instance deployments MUST use Redis.

## Summary

✅ **Setup Time**: 5 minutes
✅ **Cost**: Free (Upstash free tier)
✅ **Performance Impact**: ~2-5ms per request
✅ **Scaling**: Supports unlimited instances
✅ **Required For**: Cloud Run, Kubernetes, any multi-instance deployment

For most production deployments, Redis is **highly recommended** to ensure rate limiting works correctly across all instances.
