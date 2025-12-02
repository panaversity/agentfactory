/**
 * Test Suite Auditor
 * Runs each test file and reports results
 */

const { spawn } = require('child_process');
const fs = require('fs');

const API_TESTS = [
  { file: 'test-oauth-flows.js', cmd: 'node', currentlyIn: 'test-all' },
  { file: 'test-tenant-claims.js', cmd: 'node', currentlyIn: 'test-all' },
  { file: 'test-edge-cases.js', cmd: 'node', currentlyIn: 'test-all' },
  { file: 'test-tenant-edge-cases.js', cmd: 'node', currentlyIn: 'test-all' },
  { file: 'test-confidential-client.js', cmd: 'node', currentlyIn: 'test-all' },
  { file: 'test-client-edit.js', cmd: 'node', currentlyIn: 'test-all' },
  { file: 'test-default-organization.js', cmd: 'node', currentlyIn: 'test-all' },
  { file: 'complete-oauth-flow.ts', cmd: 'npx tsx', currentlyIn: null },
  { file: 'oauth-validation.test.ts', cmd: 'npx tsx', currentlyIn: null },
  { file: 'test-oauth-api.mjs', cmd: 'node', currentlyIn: null },
  { file: 'test-pkce-oauth.js', cmd: 'node', currentlyIn: null },
];

const PLAYWRIGHT_TESTS = [
  { file: 'test-complete-sso.js', cmd: 'node', currentlyIn: null },
  { file: 'test-full-oauth.js', cmd: 'node', currentlyIn: null },
  { file: 'test-oauth-flow.js', cmd: 'node', currentlyIn: null },
  { file: 'test-pkce-playwright.mjs', cmd: 'node', currentlyIn: null },
  { file: 'test-visual-flow.js', cmd: 'node', currentlyIn: null },
  { file: 'e2e-auth-test.spec.ts', cmd: 'npx tsx', currentlyIn: null },
];

const TIMEOUT_MS = 15000; // 15 seconds per test

async function runTest(test) {
  return new Promise((resolve) => {
    const startTime = Date.now();
    const cmdParts = test.cmd.split(' ');
    const cmd = cmdParts[0];
    const args = [...cmdParts.slice(1), `tests/${test.file}`];

    let output = '';
    let timedOut = false;

    const proc = spawn(cmd, args, { cwd: __dirname + '/..' });

    const timeout = setTimeout(() => {
      timedOut = true;
      proc.kill();
    }, TIMEOUT_MS);

    proc.stdout.on('data', (data) => {
      output += data.toString();
    });

    proc.stderr.on('data', (data) => {
      output += data.toString();
    });

    proc.on('close', (code) => {
      clearTimeout(timeout);
      const duration = Date.now() - startTime;

      let status;
      if (timedOut) {
        status = '⏱️ TIMEOUT';
      } else if (code === 0) {
        status = '✅ PASS';
      } else if (code === null) {
        status = '⚠️ KILLED';
      } else {
        status = `❌ FAIL (${code})`;
      }

      resolve({
        ...test,
        status,
        exitCode: code,
        duration,
        output: output.slice(0, 500), // First 500 chars
        timedOut,
      });
    });

    proc.on('error', (err) => {
      clearTimeout(timeout);
      resolve({
        ...test,
        status: '❌ ERROR',
        error: err.message,
        duration: Date.now() - startTime,
      });
    });
  });
}

async function main() {
  console.log('\n╔════════════════════════════════════════════════════════════╗');
  console.log('║          TEST SUITE AUDIT - Running All Tests             ║');
  console.log('╚════════════════════════════════════════════════════════════╝\n');

  console.log('⚠️  Note: Playwright tests may fail if browser not available\n');

  const results = {
    api: [],
    playwright: [],
  };

  // Test API tests
  console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
  console.log('  API Tests (No Browser Required)');
  console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

  for (const test of API_TESTS) {
    process.stdout.write(`Testing ${test.file}... `);
    const result = await runTest(test);
    results.api.push(result);
    console.log(`${result.status} (${result.duration}ms)`);
  }

  // Test Playwright tests
  console.log('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
  console.log('  Playwright Tests (Browser + Book Interface Required)');
  console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

  for (const test of PLAYWRIGHT_TESTS) {
    process.stdout.write(`Testing ${test.file}... `);
    const result = await runTest(test);
    results.playwright.push(result);
    console.log(`${result.status} (${result.duration}ms)`);
  }

  // Summary
  console.log('\n╔════════════════════════════════════════════════════════════╗');
  console.log('║                      AUDIT SUMMARY                         ║');
  console.log('╚════════════════════════════════════════════════════════════╝\n');

  const allResults = [...results.api, ...results.playwright];
  const passed = allResults.filter(r => r.status === '✅ PASS').length;
  const failed = allResults.filter(r => r.status.startsWith('❌')).length;
  const timeout = allResults.filter(r => r.status === '⏱️ TIMEOUT').length;
  const killed = allResults.filter(r => r.status === '⚠️ KILLED').length;

  console.log(`Total Tests:   ${allResults.length}`);
  console.log(`✅ Passed:     ${passed}`);
  console.log(`❌ Failed:     ${failed}`);
  console.log(`⏱️  Timeout:    ${timeout}`);
  console.log(`⚠️  Killed:     ${killed}`);

  // Detailed failures
  const failures = allResults.filter(r => !r.status.startsWith('✅'));
  if (failures.length > 0) {
    console.log('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
    console.log('  Failed/Timeout Tests');
    console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

    failures.forEach((r, i) => {
      console.log(`${i + 1}. ${r.file}`);
      console.log(`   Status: ${r.status}`);
      if (r.error) console.log(`   Error: ${r.error}`);
      if (r.output) console.log(`   Output: ${r.output.substring(0, 200)}...`);
      console.log('');
    });
  }

  // Write JSON results
  fs.writeFileSync(
    'tests/audit-results.json',
    JSON.stringify(results, null, 2)
  );

  console.log('\n📄 Detailed results saved to: tests/audit-results.json\n');

  process.exit(failed > 0 ? 1 : 0);
}

main().catch(console.error);
