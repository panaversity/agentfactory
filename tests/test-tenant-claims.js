/**
 * Tenant Claims Test
 * Tests that tenant_id and organization claims are included in userinfo
 *
 * Prerequisites:
 * 1. Auth server running: pnpm dev
 * 2. Test user exists: admin@robolearn.io
 * 3. Test organization seeded: pnpm seed:org
 *
 * Run: pnpm test-tenant
 */

const crypto = require('crypto');

const AUTH_URL = 'http://localhost:3001';
const TEST_EMAIL = 'admin@robolearn.io';
const TEST_PASSWORD = "admin@robolearn.io";

function generateCodeVerifier() {
  return crypto.randomBytes(32).toString('base64url');
}

function generateCodeChallenge(verifier) {
  return crypto.createHash('sha256').update(verifier).digest().toString('base64url');
}

async function testTenantClaims() {
  console.log('Testing tenant_id claims in userinfo...\n');

  // Sign in
  const signInResponse = await fetch(AUTH_URL + '/api/auth/sign-in/email', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email: TEST_EMAIL, password: TEST_PASSWORD }),
  });

  if (!signInResponse.ok) {
    console.error('Sign-in failed:', await signInResponse.text());
    process.exit(1);
  }

  const cookies = signInResponse.headers.get('set-cookie');

  // PKCE
  const codeVerifier = generateCodeVerifier();
  const codeChallenge = generateCodeChallenge(codeVerifier);

  // Auth
  const redirectUri = encodeURIComponent('http://localhost:3000/auth/callback');
  const authUrl = AUTH_URL + '/api/auth/oauth2/authorize?' +
    'client_id=robolearn-public-client&' +
    'redirect_uri=' + redirectUri + '&' +
    'response_type=code&scope=openid%20profile%20email&state=test&' +
    'code_challenge=' + codeChallenge + '&code_challenge_method=S256';

  const authResponse = await fetch(authUrl, {
    headers: { Cookie: cookies },
    redirect: 'manual',
  });
  const body = await authResponse.json();
  const code = body.url.match(/code=([^&]+)/)[1];

  // Token exchange
  const tokenResponse = await fetch(AUTH_URL + '/api/auth/oauth2/token', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: new URLSearchParams({
      grant_type: 'authorization_code',
      code,
      redirect_uri: 'http://localhost:3000/auth/callback',
      client_id: 'robolearn-public-client',
      code_verifier: codeVerifier,
    }),
  });
  const tokens = await tokenResponse.json();

  // Userinfo
  const userinfoResponse = await fetch(AUTH_URL + '/api/auth/oauth2/userinfo', {
    headers: { Authorization: 'Bearer ' + tokens.access_token },
  });
  const userinfo = await userinfoResponse.json();

  console.log('Userinfo response:');
  console.log(JSON.stringify(userinfo, null, 2));

  console.log('\n--- Tenant/Organization Claims ---');
  console.log('tenant_id:', userinfo.tenant_id);
  console.log('organization_ids:', userinfo.organization_ids);
  console.log('org_role:', userinfo.org_role);
  console.log('role:', userinfo.role);
  console.log('software_background:', userinfo.software_background);
  console.log('hardware_tier:', userinfo.hardware_tier);

  console.log('\n--- Additional Profile Claims (003-user-profile-fields) ---');
  console.log('gender:', userinfo.gender);
  console.log('father_name:', userinfo.father_name);
  console.log('city:', userinfo.city);
  console.log('country:', userinfo.country);
  console.log('phone_number:', userinfo.phone_number);

  // Verify tenant claims exist
  const hasTenantClaims =
    userinfo.hasOwnProperty('tenant_id') &&
    userinfo.hasOwnProperty('organization_ids') &&
    userinfo.hasOwnProperty('org_role');

  if (!hasTenantClaims) {
    console.error('\nFAIL: Missing tenant claims in userinfo');
    process.exit(1);
  }

  // Verify additional profile claims are exposed (003-user-profile-fields)
  // These claims should exist in the response (even if null for users who haven't set them)
  const hasProfileClaims =
    userinfo.hasOwnProperty('gender') &&
    userinfo.hasOwnProperty('father_name') &&
    userinfo.hasOwnProperty('city') &&
    userinfo.hasOwnProperty('country') &&
    userinfo.hasOwnProperty('phone_number');

  if (!hasProfileClaims) {
    console.error('\nFAIL: Missing additional profile claims in userinfo');
    console.error('Expected: gender, father_name, city, country, phone_number');
    process.exit(1);
  }

  console.log('\nPASS: All tenant claims present');
  console.log('PASS: All additional profile claims present');
}

testTenantClaims().catch((err) => {
  console.error('Test error:', err);
  process.exit(1);
});
