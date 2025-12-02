-- Seed the trusted public client for robolearn-interface
-- This client uses PKCE (no secret) and JWKS signing

INSERT INTO oauth_application (
  id,
  client_id,
  client_secret,
  name,
  redirect_urls,
  type,
  disabled,
  metadata,
  created_at,
  updated_at
) VALUES (
  'robolearn-public-client-id',
  'robolearn-public-client',
  NULL, -- No secret for public client (PKCE only)
  'RoboLearn Public Client',
  'http://localhost:3000/auth/callback', -- Add production URLs as comma-separated if needed
  'public',
  false,
  '{"token_endpoint_auth_method":"none","grant_types":["authorization_code","refresh_token"]}',
  NOW(),
  NOW()
)
ON CONFLICT (client_id) DO UPDATE SET
  name = EXCLUDED.name,
  redirect_urls = EXCLUDED.redirect_urls,
  type = EXCLUDED.type,
  disabled = EXCLUDED.disabled,
  metadata = EXCLUDED.metadata,
  updated_at = NOW();


