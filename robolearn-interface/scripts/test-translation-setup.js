/**
 * Quick Test Script: Verify Translation Setup
 * 
 * Run: node scripts/test-translation-setup.js
 * 
 * Checks:
 * - API key is set
 * - Dependencies installed
 * - Plugin files exist
 * - Config is correct
 */

const fs = require('fs');
const path = require('path');

console.log('🔍 Testing Translation Setup...\n');

let allChecksPassed = true;

// Check 1: API Key
console.log('1. Checking GEMINI_API_KEY...');
require('dotenv').config();
const apiKey = process.env.GEMINI_API_KEY;
if (apiKey) {
  console.log('   ✅ API key found');
} else {
  console.log('   ❌ API key missing - Set GEMINI_API_KEY in .env file');
  allChecksPassed = false;
}

// Check 2: Dependencies
console.log('\n2. Checking dependencies...');
const packageJson = JSON.parse(fs.readFileSync('package.json', 'utf8'));
const deps = { ...packageJson.dependencies, ...packageJson.devDependencies };

const requiredDeps = ['@google/generative-ai', 'gray-matter'];
requiredDeps.forEach(dep => {
  if (deps[dep]) {
    console.log(`   ✅ ${dep} installed`);
  } else {
    console.log(`   ❌ ${dep} missing - Run: npm install ${dep}`);
    allChecksPassed = false;
  }
});

// Check 3: Plugin files
console.log('\n3. Checking plugin files...');
const pluginFiles = [
  'plugins/docusaurus-plugin-auto-translate/index.js',
  'plugins/docusaurus-plugin-auto-translate/lib/cache.js',
  'plugins/docusaurus-plugin-auto-translate/lib/file-processor.js',
  'plugins/docusaurus-plugin-auto-translate/lib/i18n-structure.js',
  'plugins/docusaurus-plugin-auto-translate/lib/translator.js',
];

pluginFiles.forEach(file => {
  if (fs.existsSync(file)) {
    console.log(`   ✅ ${file}`);
  } else {
    console.log(`   ❌ ${file} missing`);
    allChecksPassed = false;
  }
});

// Check 4: Config
console.log('\n4. Checking docusaurus.config.ts...');
const configPath = 'docusaurus.config.ts';
if (fs.existsSync(configPath)) {
  const config = fs.readFileSync(configPath, 'utf8');
  
  if (config.includes('docusaurus-plugin-auto-translate')) {
    console.log('   ✅ Plugin configured');
  } else {
    console.log('   ❌ Plugin not found in config');
    allChecksPassed = false;
  }
  
  if (config.includes('locales: ["en", "ur"]')) {
    console.log('   ✅ Urdu locale configured');
  } else {
    console.log('   ❌ Urdu locale not configured');
    allChecksPassed = false;
  }
} else {
  console.log('   ❌ docusaurus.config.ts not found');
  allChecksPassed = false;
}

// Check 5: RTL CSS
console.log('\n5. Checking RTL support...');
if (fs.existsSync('src/css/rtl.css')) {
  console.log('   ✅ rtl.css exists');
} else {
  console.log('   ❌ rtl.css missing');
  allChecksPassed = false;
}

// Summary
console.log('\n' + '='.repeat(50));
if (allChecksPassed) {
  console.log('✅ All checks passed! Ready to test translation.');
  console.log('\nNext steps:');
  console.log('  1. Run: npm run build');
  console.log('  2. Check: i18n/ur/ folder for translations');
  console.log('  3. Run: npm start');
  console.log('  4. Test: Click locale dropdown in navbar');
} else {
  console.log('❌ Some checks failed. Please fix the issues above.');
  console.log('\nQuick fixes:');
  console.log('  - Set GEMINI_API_KEY in .env file');
  console.log('  - Run: npm install');
  console.log('  - Verify plugin is in docusaurus.config.ts');
}
console.log('='.repeat(50));

process.exit(allChecksPassed ? 0 : 1);

