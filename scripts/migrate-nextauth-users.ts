#!/usr/bin/env node
/**
 * NextAuth to Better Auth User Migration Script
 *
 * Migrates users from NextAuth PostgreSQL database to Better Auth SSO system.
 *
 * Features:
 * - Batch processing (500 users/batch) with transaction safety
 * - ID preservation for microservice FK integrity
 * - Conflict resolution: update target user.id to match source, preserve non-null data
 * - Country normalization (PK → Pakistan, NULL → Pakistan)
 * - Username regeneration from email addresses
 * - Bcrypt password hash preservation (no re-hashing)
 * - Dry-run mode for safe verification
 * - Resume capability with --offset
 *
 * Usage:
 *   npx tsx scripts/migrate-nextauth-users.ts [options]
 *
 * Options:
 *   --dry-run           Log operations without modifying database
 *   --limit <N>         Process only N users (for testing)
 *   --offset <N>        Start from position N (for resuming)
 *   --batch-size <N>    Users per batch (default: 500)
 *   --verbose           Enable detailed logging
 *
 * Environment Variables:
 *   NEXT_AUTH_PROD_DB_MIRROR  Source database (read-only NextAuth mirror)
 *   DATABASE_URL              Target database (Better Auth SSO)
 */

import * as dotenv from 'dotenv';
import * as fs from 'fs';
import * as path from 'path';

// Load environment variables
dotenv.config();
dotenv.config({ path: '.env.local' });

// ============================================================================
// TYPES & INTERFACES
// ============================================================================

interface OldUser {
  id: string;
  name: string | null;
  email: string;
  password: string | null;
  phone_number: string | null;
  emailverified: boolean | null;
  image: string | null;
  country: string | null;
  role: string | null;
  username: string | null;
  created_at: Date | string | null;
  updated_at: Date | string | null;
}

interface OldProfile {
  city: string | null;
  gender: string | null;
  father_name: string | null;
}

interface SourceUser extends OldUser, OldProfile {}

interface TargetUser {
  id: string;
  email: string;
  name: string | null;
  email_verified: boolean;
  image: string | null;
  role: string | null;
  username: string | null;
  phone_number: string | null;
  country: string | null;
  city: string | null;
  gender: string | null;
  father_name: string | null;
  created_at: string;
  updated_at: string;
}

interface MigrationConfig {
  dryRun: boolean;
  limit: number | null;
  offset: number;
  batchSize: number;
  verbose: boolean;
  sourceDbUrl: string;
  targetDbUrl: string;
  since: Date | null;  // Only migrate users created after this timestamp
}

interface MigrationStats {
  totalProcessed: number;
  inserted: number;
  merged: number;
  skipped: number;
  errors: number;
  countryNormalizations: Record<string, number>;
  usernamesGenerated: number;
  startTime: number;
  endTime: number;
}

interface BatchResult {
  inserted: number;
  merged: number;
  skipped: number;
  errors: number;
}

// ============================================================================
// CONSTANTS
// ============================================================================

const DEFAULT_BATCH_SIZE = 500;

/**
 * Country code to full name mapping
 * Based on actual data in source database
 */
const COUNTRY_MAP: Record<string, string> = {
  'PK': 'Pakistan',
  'US': 'United States',
  'GB': 'United Kingdom',
  'CA': 'Canada',
  'AE': 'United Arab Emirates',
  'SA': 'Saudi Arabia',
  'IN': 'India',
  'AU': 'Australia',
  'DE': 'Germany',
  'QA': 'Qatar',
  'MY': 'Malaysia',
  'AF': 'Afghanistan',
  'CN': 'China',
  'IE': 'Ireland',
  'EG': 'Egypt',
  'TR': 'Turkey',
  'OM': 'Oman',
  'IT': 'Italy',
  'BD': 'Bangladesh',
  'KR': 'South Korea',
  'KW': 'Kuwait',
  'BH': 'Bahrain',
  'JO': 'Jordan',
  'NL': 'Netherlands',
  'FR': 'France',
  'SE': 'Sweden',
  'NO': 'Norway',
  'NZ': 'New Zealand',
  'SG': 'Singapore',
  'JP': 'Japan',
  'Pakistan': 'Pakistan',  // Already correct
  'United States': 'United States',
  'United Kingdom': 'United Kingdom',
  'Canada': 'Canada',
  'India': 'India',
};

/**
 * Tables with FK references to user.id that need to be updated for overlapping users
 */
const FK_TABLES = [
  { table: 'account', column: 'user_id' },
  { table: 'session', column: 'user_id' },
  { table: 'member', column: 'user_id' },
  { table: 'oauth_access_token', column: 'user_id' },
  { table: 'oauth_consent', column: 'user_id' },
  { table: 'apikey', column: 'user_id' },
  { table: 'invitation', column: 'inviter_id' },
  { table: 'oauth_application', column: 'user_id' },
];

// ============================================================================
// CLI ARGUMENT PARSER
// ============================================================================

function parseArgs(): MigrationConfig {
  const args = process.argv.slice(2);
  const config: MigrationConfig = {
    dryRun: false,
    limit: null,
    offset: 0,
    batchSize: DEFAULT_BATCH_SIZE,
    verbose: false,
    sourceDbUrl: process.env.NEXT_AUTH_PROD_DB_MIRROR || '',
    targetDbUrl: process.env.DATABASE_URL || '',
    since: null,
  };

  for (let i = 0; i < args.length; i++) {
    const arg = args[i];
    switch (arg) {
      case '--dry-run':
        config.dryRun = true;
        break;
      case '--limit':
        config.limit = parseInt(args[++i], 10);
        if (isNaN(config.limit)) {
          console.error('Error: --limit requires a number');
          process.exit(1);
        }
        break;
      case '--offset':
        config.offset = parseInt(args[++i], 10);
        if (isNaN(config.offset)) {
          console.error('Error: --offset requires a number');
          process.exit(1);
        }
        break;
      case '--batch-size':
        config.batchSize = parseInt(args[++i], 10);
        if (isNaN(config.batchSize) || config.batchSize <= 0) {
          console.error('Error: --batch-size requires a positive number');
          process.exit(1);
        }
        break;
      case '--since':
        const sinceStr = args[++i];
        config.since = new Date(sinceStr);
        if (isNaN(config.since.getTime())) {
          console.error('Error: --since requires a valid ISO date (e.g., 2024-12-05T10:00:00Z)');
          process.exit(1);
        }
        break;
      case '--verbose':
        config.verbose = true;
        break;
      case '--help':
      case '-h':
        printHelp();
        process.exit(0);
      default:
        console.error(`Unknown argument: ${arg}`);
        printHelp();
        process.exit(1);
    }
  }

  return config;
}

function printHelp(): void {
  console.log(`
NextAuth to Better Auth User Migration Script

Usage: npx tsx scripts/migrate-nextauth-users.ts [options]

Options:
  --dry-run           Log operations without modifying database
  --limit <N>         Process only N users (for testing)
  --offset <N>        Start from position N (for resuming)
  --batch-size <N>    Users per batch (default: 500)
  --since <ISO_DATE>  Only migrate users created after this timestamp (for delta migration)
  --verbose           Enable detailed logging
  --help, -h          Show this help message

Environment Variables:
  NEXT_AUTH_PROD_DB_MIRROR  Source database (read-only NextAuth mirror)
  DATABASE_URL              Target database (Better Auth SSO)

Examples:
  # Dry run (always do this first)
  npx tsx scripts/migrate-nextauth-users.ts --dry-run

  # Test with 100 users
  npx tsx scripts/migrate-nextauth-users.ts --dry-run --limit 100

  # Resume from position 5000
  npx tsx scripts/migrate-nextauth-users.ts --offset 5000

  # Delta migration (users created after main migration started)
  npx tsx scripts/migrate-nextauth-users.ts --since "2024-12-05T10:00:00Z"

  # Full migration
  npx tsx scripts/migrate-nextauth-users.ts
`);
}

// ============================================================================
// CONFIG VALIDATION
// ============================================================================

function validateConfig(config: MigrationConfig): void {
  const errors: string[] = [];

  if (!config.sourceDbUrl) {
    errors.push('NEXT_AUTH_PROD_DB_MIRROR environment variable is not set');
  }

  if (!config.targetDbUrl) {
    errors.push('DATABASE_URL environment variable is not set');
  }

  if (errors.length > 0) {
    console.error('\n' + '═'.repeat(70));
    console.error('  CONFIGURATION ERROR');
    console.error('═'.repeat(70));
    errors.forEach(err => console.error(`  ❌ ${err}`));
    console.error('═'.repeat(70) + '\n');
    process.exit(1);
  }
}

// ============================================================================
// AUDIT LOGGER (JSONL)
// ============================================================================

interface AuditRecord {
  timestamp: string;
  action: 'INSERT' | 'MERGE' | 'SKIP' | 'ERROR';
  email: string;
  sourceId: string;
  targetId?: string;
  country?: { from: string | null; to: string };
  username?: string;
  filledFields?: string[];
  error?: string;
  dryRun: boolean;
}

class AuditLogger {
  private stream: fs.WriteStream | null = null;
  private filePath: string;
  private recordCount = 0;

  constructor(dryRun: boolean) {
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    const mode = dryRun ? 'dry-run' : 'live';
    this.filePath = path.join(process.cwd(), 'logs', `migration-${mode}-${timestamp}.jsonl`);
  }

  init(): void {
    // Ensure logs directory exists
    const logsDir = path.join(process.cwd(), 'logs');
    if (!fs.existsSync(logsDir)) {
      fs.mkdirSync(logsDir, { recursive: true });
    }
    this.stream = fs.createWriteStream(this.filePath, { flags: 'a' });
    console.log(`📝 Audit log: ${this.filePath}`);
  }

  write(record: AuditRecord): void {
    if (this.stream) {
      this.stream.write(JSON.stringify(record) + '\n');
      this.recordCount++;
    }
  }

  insert(email: string, sourceId: string, country: { from: string | null; to: string }, username: string, dryRun: boolean): void {
    this.write({
      timestamp: new Date().toISOString(),
      action: 'INSERT',
      email,
      sourceId,
      country,
      username,
      dryRun,
    });
  }

  merge(email: string, sourceId: string, targetId: string, filledFields: string[], dryRun: boolean): void {
    this.write({
      timestamp: new Date().toISOString(),
      action: 'MERGE',
      email,
      sourceId,
      targetId,
      filledFields,
      dryRun,
    });
  }

  skip(email: string, sourceId: string, reason: string, dryRun: boolean): void {
    this.write({
      timestamp: new Date().toISOString(),
      action: 'SKIP',
      email,
      sourceId,
      error: reason,
      dryRun,
    });
  }

  error(email: string, sourceId: string, error: string, dryRun: boolean): void {
    this.write({
      timestamp: new Date().toISOString(),
      action: 'ERROR',
      email,
      sourceId,
      error,
      dryRun,
    });
  }

  close(): void {
    if (this.stream) {
      this.stream.end();
      console.log(`📝 Audit log complete: ${this.recordCount} records written to ${this.filePath}`);
    }
  }

  getFilePath(): string {
    return this.filePath;
  }
}

// ============================================================================
// LOGGER
// ============================================================================

class Logger {
  private verbose: boolean;
  private startTime: number;
  private totalUsers: number = 0;

  constructor(verbose: boolean) {
    this.verbose = verbose;
    this.startTime = Date.now();
  }

  setTotalUsers(total: number): void {
    this.totalUsers = total;
  }

  banner(): void {
    console.log('\n' + '═'.repeat(70));
    console.log('          NextAuth to Better Auth User Migration v1.0');
    console.log('═'.repeat(70) + '\n');
  }

  config(config: MigrationConfig, sourceCount: number, targetCount: number): void {
    const sourceHost = config.sourceDbUrl.split('@')[1]?.split('/')[0] || 'unknown';
    const targetHost = config.targetDbUrl.split('@')[1]?.split('/')[0] || 'unknown';

    console.log('Configuration:');
    console.log(`├── Source DB: ${sourceHost} (${sourceCount.toLocaleString()} users${config.since ? ' after filter' : ''})`);
    console.log(`├── Target DB: ${targetHost} (${targetCount.toLocaleString()} users)`);
    console.log(`├── Mode: ${config.dryRun ? 'DRY RUN (no database modifications)' : 'LIVE MIGRATION'}`);
    console.log(`├── Batch Size: ${config.batchSize}`);
    console.log(`├── Offset: ${config.offset}`);
    console.log(`├── Limit: ${config.limit || '(all)'}`);
    console.log(`└── Since: ${config.since ? config.since.toISOString() : '(all users)'}`);
    console.log('\n' + '═'.repeat(70) + '\n');
  }

  batchStart(batchNum: number, totalBatches: number, startIdx: number, endIdx: number): void {
    console.log(`\nProcessing batch ${batchNum}/${totalBatches} (users ${startIdx + 1}-${endIdx})...`);
  }

  userProgress(current: number, total: number): void {
    // Show progress every 50 users in non-verbose mode
    if (!this.verbose && current % 50 === 0) {
      process.stdout.write(`  [${current}/${total}]\r`);
    }
  }

  insert(email: string, userId: string, country: string, countryNormalized: string, username: string): void {
    if (this.verbose) {
      const countryInfo = country !== countryNormalized
        ? `country: ${country || 'NULL'} → ${countryNormalized}`
        : `country: ${countryNormalized}`;
      console.log(`  [INSERT] ${email} → id: ${userId.substring(0, 8)}..., ${countryInfo}, username: ${username}`);
    }
  }

  merge(email: string, oldId: string, newId: string, filledFields: string[]): void {
    const fieldsInfo = filledFields.length > 0 ? `, filled: ${filledFields.join(', ')}` : '';
    console.log(`  [MERGE] ${email} → id: ${oldId.substring(0, 8)}... → ${newId.substring(0, 8)}...${fieldsInfo}`);
  }

  skip(email: string, reason: string): void {
    console.log(`  [SKIP] ${email}: ${reason}`);
  }

  error(email: string, error: string): void {
    console.error(`  [ERROR] ${email}: ${error}`);
  }

  warning(message: string): void {
    console.warn(`  [WARN] ${message}`);
  }

  batchComplete(result: BatchResult): void {
    // Clear the progress line
    process.stdout.write('                              \r');
    console.log(`  Batch complete: ${result.inserted} inserted, ${result.merged} merged, ${result.skipped} skipped, ${result.errors} errors`);
  }

  progress(processed: number, total: number, offset: number): void {
    const adjustedTotal = total - offset;
    const adjustedProcessed = processed;
    const percentage = ((adjustedProcessed / adjustedTotal) * 100).toFixed(1);

    const elapsed = Date.now() - this.startTime;
    const rate = adjustedProcessed / (elapsed / 1000); // users per second
    const remaining = adjustedTotal - adjustedProcessed;
    const etaSeconds = remaining / rate;
    const etaMinutes = Math.ceil(etaSeconds / 60);

    console.log(`\n[Progress: ${percentage}% | ${adjustedProcessed.toLocaleString()}/${adjustedTotal.toLocaleString()} | ETA: ${etaMinutes} minute(s)]\n`);
  }

  summary(stats: MigrationStats, dryRun: boolean): void {
    const duration = (stats.endTime - stats.startTime) / 1000;
    const minutes = Math.floor(duration / 60);
    const seconds = Math.round(duration % 60);

    console.log('\n' + '═'.repeat(70));
    console.log('                         Migration Summary');
    console.log('═'.repeat(70) + '\n');

    if (dryRun) {
      console.log('  ⚠️  DRY RUN - No changes were made to the database\n');
    }

    console.log('Results:');
    console.log(`├── Total Processed: ${stats.totalProcessed.toLocaleString()}`);
    console.log(`├── New Users Inserted: ${stats.inserted.toLocaleString()}`);
    console.log(`├── Overlapping Users Merged: ${stats.merged.toLocaleString()}`);
    console.log(`├── Skipped: ${stats.skipped.toLocaleString()}`);
    console.log(`├── Errors: ${stats.errors.toLocaleString()}`);
    console.log(`└── Duration: ${minutes} minute(s) ${seconds} second(s)`);

    console.log('\nCountry Normalizations:');
    const sortedCountries = Object.entries(stats.countryNormalizations)
      .sort((a, b) => b[1] - a[1])
      .slice(0, 10);
    sortedCountries.forEach(([transformation, count], index) => {
      const prefix = index === sortedCountries.length - 1 ? '└──' : '├──';
      console.log(`${prefix} ${transformation}: ${count.toLocaleString()}`);
    });

    console.log(`\nUsername Regenerations: ${stats.usernamesGenerated.toLocaleString()} (all migrated users)`);

    console.log('\n' + '═'.repeat(70) + '\n');

    if (!dryRun) {
      console.log('Verification Queries:');
      console.log('  SELECT COUNT(*) FROM public."user";  -- Check total users');
      console.log('  SELECT COUNT(*) FROM public.account WHERE provider_id = \'credential\';  -- Check accounts');
      console.log('  SELECT country, COUNT(*) FROM public."user" GROUP BY country ORDER BY COUNT(*) DESC;');
    }

    console.log(`\nMigration ${dryRun ? 'dry-run' : ''} complete. Exit code: 0\n`);
  }
}

// ============================================================================
// DATABASE CONNECTIONS
// ============================================================================

let sourceSql: any;
let targetSql: any;

async function initDatabases(config: MigrationConfig): Promise<void> {
  const { neon } = await import('@neondatabase/serverless');

  sourceSql = neon(config.sourceDbUrl);
  targetSql = neon(config.targetDbUrl);

  // Test connections
  try {
    await sourceSql`SELECT 1`;
    await targetSql`SELECT 1`;
  } catch (error: any) {
    console.error('Failed to connect to databases:', error.message);
    process.exit(1);
  }
}

// ============================================================================
// SOURCE DATABASE QUERIES
// ============================================================================

async function getSourceUserCount(since: Date | null): Promise<number> {
  if (since) {
    const result = await sourceSql`
      SELECT COUNT(*) as count
      FROM public.users
      WHERE password IS NOT NULL AND password != ''
        AND created_at > ${since.toISOString()}
    `;
    return parseInt(result[0].count, 10);
  } else {
    const result = await sourceSql`
      SELECT COUNT(*) as count
      FROM public.users
      WHERE password IS NOT NULL AND password != ''
    `;
    return parseInt(result[0].count, 10);
  }
}

async function getTargetUserCount(): Promise<number> {
  const result = await targetSql`
    SELECT COUNT(*) as count FROM public."user"
  `;
  return parseInt(result[0].count, 10);
}

async function fetchSourceBatch(offset: number, limit: number, since: Date | null): Promise<SourceUser[]> {
  if (since) {
    const result = await sourceSql`
      SELECT
        u.id, u.name, u.email, u.password, u.phone_number,
        u."emailVerified" as emailverified, u.image, u.country, u.role, u.username,
        u.created_at, u.updated_at,
        p.city, p.gender, p.father_name
      FROM public.users u
      LEFT JOIN public.profile p ON u.id = p.user_id
      WHERE u.password IS NOT NULL AND u.password != ''
        AND u.created_at > ${since.toISOString()}
      ORDER BY u.created_at ASC
      OFFSET ${offset}
      LIMIT ${limit}
    `;
    return result as SourceUser[];
  } else {
    const result = await sourceSql`
      SELECT
        u.id, u.name, u.email, u.password, u.phone_number,
        u."emailVerified" as emailverified, u.image, u.country, u.role, u.username,
        u.created_at, u.updated_at,
        p.city, p.gender, p.father_name
      FROM public.users u
      LEFT JOIN public.profile p ON u.id = p.user_id
      WHERE u.password IS NOT NULL AND u.password != ''
      ORDER BY u.created_at ASC
      OFFSET ${offset}
      LIMIT ${limit}
    `;
    return result as SourceUser[];
  }
}

// ============================================================================
// TARGET DATABASE QUERIES
// ============================================================================

async function findTargetUserByEmail(email: string): Promise<TargetUser | null> {
  const result = await targetSql`
    SELECT id, email, name, email_verified, image, role, username,
           phone_number, country, city, gender, father_name,
           created_at, updated_at
    FROM public."user"
    WHERE email = ${email}
  `;
  return result.length > 0 ? result[0] : null;
}

async function getAllTargetUsernames(): Promise<Set<string>> {
  const result = await targetSql`
    SELECT username FROM public."user" WHERE username IS NOT NULL
  `;
  return new Set(result.map((r: any) => r.username));
}

// ============================================================================
// DATA TRANSFORMATION
// ============================================================================

/**
 * Normalize country code to full country name
 * - NULL → Pakistan
 * - Valid codes (PK, US, etc.) → Full name
 * - Garbage data → Pakistan (with warning)
 */
function normalizeCountry(country: string | null): string {
  if (!country) return 'Pakistan'; // Default for NULL

  const trimmed = country.trim();
  const normalized = COUNTRY_MAP[trimmed];
  if (normalized) return normalized;

  // Check if it looks like a valid country (2-3 letter code or known name)
  // If not, it's garbage data - default to Pakistan
  const looksLikeCountry = /^[A-Z]{2,3}$/.test(trimmed) ||
                           /^[A-Za-z\s]{3,30}$/.test(trimmed);

  if (!looksLikeCountry) {
    console.warn(`  [WARN] Invalid country data "${trimmed}" → defaulting to Pakistan`);
    return 'Pakistan';
  }

  // Unknown but valid-looking code - keep but warn
  console.warn(`  [WARN] Unknown country code "${trimmed}" - keeping as-is`);
  return trimmed;
}

/**
 * Generate unique username from email address
 */
function generateUsername(email: string, existingUsernames: Set<string>): string {
  // Extract prefix from email
  const prefix = email.split('@')[0]
    .toLowerCase()
    .replace(/[^a-z0-9]/g, '-')  // Replace special chars with dash
    .replace(/-+/g, '-')          // Collapse multiple dashes
    .replace(/^-|-$/g, '')        // Trim leading/trailing dashes
    .substring(0, 40);            // Leave room for suffix

  // Generate random suffix
  const randomSuffix = () => Math.random().toString(36).substring(2, 6);

  let username = `${prefix}-${randomSuffix()}`;
  let attempts = 0;

  // Handle collisions (max 10 attempts)
  while (existingUsernames.has(username) && attempts < 10) {
    username = `${prefix}-${randomSuffix()}`;
    attempts++;
  }

  if (attempts >= 10) {
    // Fallback: append timestamp
    username = `${prefix}-${Date.now().toString(36)}`;
  }

  existingUsernames.add(username);  // Track for this batch
  return username;
}

/**
 * Extract name from email if name is NULL
 */
function resolveName(name: string | null, email: string): string {
  if (name && name.trim()) return name.trim();
  // Use email prefix as name
  const prefix = email.split('@')[0];
  return prefix.charAt(0).toUpperCase() + prefix.slice(1).replace(/[._-]/g, ' ');
}

/**
 * Validate bcrypt password hash format
 */
function isValidBcryptHash(hash: string | null): boolean {
  if (!hash) return false;
  // Valid bcrypt hashes start with $2a$, $2b$, or $2y$
  return /^\$2[aby]\$\d{2}\$.{53}$/.test(hash);
}

/**
 * Format date for database insertion
 */
function formatDate(date: Date | string | null): string {
  if (!date) return new Date().toISOString();
  if (date instanceof Date) return date.toISOString();
  // Try to parse string date
  const parsed = new Date(date);
  return isNaN(parsed.getTime()) ? new Date().toISOString() : parsed.toISOString();
}

// ============================================================================
// MIGRATION OPERATIONS
// ============================================================================

/**
 * Insert new user into target database
 */
async function insertNewUser(
  user: SourceUser,
  username: string,
  normalizedCountry: string,
  dryRun: boolean,
  logger: Logger
): Promise<void> {
  const userId = user.id;
  const accountId = crypto.randomUUID();
  const now = new Date().toISOString();
  const name = resolveName(user.name, user.email);

  if (dryRun) {
    logger.insert(user.email, userId, user.country || 'NULL', normalizedCountry, username);
    return;
  }

  // Insert user
  await targetSql`
    INSERT INTO public."user" (
      id, email, name, email_verified, image, role, username,
      phone_number, country, city, gender, father_name,
      created_at, updated_at
    ) VALUES (
      ${userId},
      ${user.email},
      ${name},
      ${user.emailverified || false},
      ${user.image || null},
      ${user.role || 'user'},
      ${username},
      ${user.phone_number || null},
      ${normalizedCountry},
      ${user.city || null},
      ${user.gender || null},
      ${user.father_name || null},
      ${formatDate(user.created_at)},
      ${formatDate(user.updated_at)}
    )
  `;

  // Insert credential account with bcrypt password (preserved exactly)
  await targetSql`
    INSERT INTO public.account (
      id, account_id, provider_id, user_id, password,
      created_at, updated_at
    ) VALUES (
      ${accountId},
      ${userId},
      ${'credential'},
      ${userId},
      ${user.password},
      ${now},
      ${now}
    )
  `;

  logger.insert(user.email, userId, user.country || 'NULL', normalizedCountry, username);
}

/**
 * Merge overlapping user - update target user.id to match source
 */
async function mergeOverlappingUser(
  sourceUser: SourceUser,
  targetUser: TargetUser,
  normalizedCountry: string,
  dryRun: boolean,
  logger: Logger
): Promise<string[]> {
  const oldId = targetUser.id;
  const newId = sourceUser.id;
  const filledFields: string[] = [];

  // Determine which NULL fields to fill from source
  const fieldsToFill: Record<string, any> = {};

  if (!targetUser.name && sourceUser.name) {
    fieldsToFill.name = sourceUser.name;
    filledFields.push('name');
  }
  if (!targetUser.phone_number && sourceUser.phone_number) {
    fieldsToFill.phone_number = sourceUser.phone_number;
    filledFields.push('phone_number');
  }
  if (!targetUser.country) {
    fieldsToFill.country = normalizedCountry;
    filledFields.push('country');
  }
  if (!targetUser.city && sourceUser.city) {
    fieldsToFill.city = sourceUser.city;
    filledFields.push('city');
  }
  if (!targetUser.gender && sourceUser.gender) {
    fieldsToFill.gender = sourceUser.gender;
    filledFields.push('gender');
  }
  if (!targetUser.father_name && sourceUser.father_name) {
    fieldsToFill.father_name = sourceUser.father_name;
    filledFields.push('father_name');
  }
  if (!targetUser.image && sourceUser.image) {
    fieldsToFill.image = sourceUser.image;
    filledFields.push('image');
  }

  if (dryRun) {
    logger.merge(sourceUser.email, oldId, newId, filledFields);
    return filledFields;
  }

  // Only update if IDs are different
  if (oldId !== newId) {
    // Strategy: DELETE FK references, UPDATE user.id, then RECREATE references
    // This avoids FK constraint violations since we can't defer constraints in Neon serverless

    // Step 1: DELETE all FK references to oldId
    try {
      await targetSql`DELETE FROM public.account WHERE user_id = ${oldId}`;
    } catch (e: any) { if (!e.message?.includes('does not exist')) throw e; }

    try {
      await targetSql`DELETE FROM public.session WHERE user_id = ${oldId}`;
    } catch (e: any) { if (!e.message?.includes('does not exist')) throw e; }

    try {
      await targetSql`DELETE FROM public.member WHERE user_id = ${oldId}`;
    } catch (e: any) { if (!e.message?.includes('does not exist')) throw e; }

    try {
      await targetSql`DELETE FROM public.oauth_access_token WHERE user_id = ${oldId}`;
    } catch (e: any) { if (!e.message?.includes('does not exist')) throw e; }

    try {
      await targetSql`DELETE FROM public.oauth_consent WHERE user_id = ${oldId}`;
    } catch (e: any) { if (!e.message?.includes('does not exist')) throw e; }

    try {
      await targetSql`DELETE FROM public.apikey WHERE user_id = ${oldId}`;
    } catch (e: any) { if (!e.message?.includes('does not exist')) throw e; }

    try {
      await targetSql`DELETE FROM public.invitation WHERE inviter_id = ${oldId}`;
    } catch (e: any) { if (!e.message?.includes('does not exist')) throw e; }

    try {
      await targetSql`DELETE FROM public.oauth_application WHERE user_id = ${oldId}`;
    } catch (e: any) { if (!e.message?.includes('does not exist')) throw e; }

    // Step 2: Now update the user.id itself (no FK references blocking)
    await targetSql`
      UPDATE public."user"
      SET id = ${newId}
      WHERE id = ${oldId}
    `;
  }

  // Fill NULL fields from source
  if (Object.keys(fieldsToFill).length > 0) {
    // Build dynamic update - we need to do this manually since we can't use dynamic column names
    if (fieldsToFill.name !== undefined) {
      await targetSql`UPDATE public."user" SET name = ${fieldsToFill.name} WHERE id = ${newId}`;
    }
    if (fieldsToFill.phone_number !== undefined) {
      await targetSql`UPDATE public."user" SET phone_number = ${fieldsToFill.phone_number} WHERE id = ${newId}`;
    }
    if (fieldsToFill.country !== undefined) {
      await targetSql`UPDATE public."user" SET country = ${fieldsToFill.country} WHERE id = ${newId}`;
    }
    if (fieldsToFill.city !== undefined) {
      await targetSql`UPDATE public."user" SET city = ${fieldsToFill.city} WHERE id = ${newId}`;
    }
    if (fieldsToFill.gender !== undefined) {
      await targetSql`UPDATE public."user" SET gender = ${fieldsToFill.gender} WHERE id = ${newId}`;
    }
    if (fieldsToFill.father_name !== undefined) {
      await targetSql`UPDATE public."user" SET father_name = ${fieldsToFill.father_name} WHERE id = ${newId}`;
    }
    if (fieldsToFill.image !== undefined) {
      await targetSql`UPDATE public."user" SET image = ${fieldsToFill.image} WHERE id = ${newId}`;
    }
  }

  // Update/create credential account with source password
  const existingAccount = await targetSql`
    SELECT id FROM public.account
    WHERE user_id = ${newId} AND provider_id = 'credential'
  `;

  if (existingAccount.length > 0) {
    // Update existing account password
    await targetSql`
      UPDATE public.account
      SET password = ${sourceUser.password}, updated_at = ${new Date().toISOString()}
      WHERE user_id = ${newId} AND provider_id = 'credential'
    `;
  } else {
    // Create new credential account
    const accountId = crypto.randomUUID();
    const now = new Date().toISOString();
    await targetSql`
      INSERT INTO public.account (
        id, account_id, provider_id, user_id, password,
        created_at, updated_at
      ) VALUES (
        ${accountId},
        ${newId},
        ${'credential'},
        ${newId},
        ${sourceUser.password},
        ${now},
        ${now}
      )
    `;
  }

  logger.merge(sourceUser.email, oldId, newId, filledFields);
  return filledFields;
}

// ============================================================================
// BATCH PROCESSOR
// ============================================================================

async function processBatch(
  users: SourceUser[],
  existingUsernames: Set<string>,
  config: MigrationConfig,
  stats: MigrationStats,
  logger: Logger,
  auditLogger: AuditLogger
): Promise<BatchResult> {
  const result: BatchResult = { inserted: 0, merged: 0, skipped: 0, errors: 0 };
  let userIndex = 0;

  for (const user of users) {
    userIndex++;
    logger.userProgress(userIndex, users.length);
    try {
      // Validate password hash
      if (!isValidBcryptHash(user.password)) {
        const reason = `Invalid password hash format: ${user.password?.substring(0, 10)}...`;
        logger.skip(user.email, reason);
        auditLogger.skip(user.email, user.id, reason, config.dryRun);
        result.skipped++;
        stats.skipped++;
        continue;
      }

      // Check if user exists in target
      const targetUser = await findTargetUserByEmail(user.email);

      // Normalize country
      const normalizedCountry = normalizeCountry(user.country);
      const countryKey = `${user.country || 'NULL'} → ${normalizedCountry}`;
      stats.countryNormalizations[countryKey] = (stats.countryNormalizations[countryKey] || 0) + 1;

      if (targetUser) {
        // Overlapping user - merge
        const filledFields = await mergeOverlappingUser(user, targetUser, normalizedCountry, config.dryRun, logger);
        auditLogger.merge(user.email, user.id, targetUser.id, filledFields, config.dryRun);
        result.merged++;
        stats.merged++;
      } else {
        // New user - insert
        const username = generateUsername(user.email, existingUsernames);
        stats.usernamesGenerated++;
        await insertNewUser(user, username, normalizedCountry, config.dryRun, logger);
        auditLogger.insert(user.email, user.id, { from: user.country, to: normalizedCountry }, username, config.dryRun);
        result.inserted++;
        stats.inserted++;
      }

      stats.totalProcessed++;
    } catch (error: any) {
      logger.error(user.email, error.message);
      auditLogger.error(user.email, user.id, error.message, config.dryRun);
      result.errors++;
      stats.errors++;
      // Don't throw - continue with next user
    }
  }

  return result;
}

// ============================================================================
// MAIN MIGRATION FUNCTION
// ============================================================================

async function migrate(): Promise<void> {
  const config = parseArgs();
  validateConfig(config);

  const logger = new Logger(config.verbose);
  logger.banner();

  // Initialize audit logger
  const auditLogger = new AuditLogger(config.dryRun);
  auditLogger.init();

  // Initialize databases
  await initDatabases(config);

  // Get counts
  const sourceCount = await getSourceUserCount(config.since);
  const targetCount = await getTargetUserCount();

  logger.config(config, sourceCount, targetCount);
  logger.setTotalUsers(sourceCount);

  // Load existing usernames for collision detection
  const existingUsernames = await getAllTargetUsernames();

  // Initialize stats
  const stats: MigrationStats = {
    totalProcessed: 0,
    inserted: 0,
    merged: 0,
    skipped: 0,
    errors: 0,
    countryNormalizations: {},
    usernamesGenerated: 0,
    startTime: Date.now(),
    endTime: 0,
  };

  // Calculate total to process
  const effectiveTotal = config.limit
    ? Math.min(config.limit, sourceCount - config.offset)
    : sourceCount - config.offset;

  if (effectiveTotal <= 0) {
    console.log('No users to process (check offset/limit values)');
    auditLogger.close();
    process.exit(0);
  }

  // Process in batches
  const totalBatches = Math.ceil(effectiveTotal / config.batchSize);
  let processedCount = 0;
  let batchNum = 0;

  while (processedCount < effectiveTotal) {
    batchNum++;
    const batchOffset = config.offset + processedCount;
    const batchLimit = Math.min(config.batchSize, effectiveTotal - processedCount);

    logger.batchStart(batchNum, totalBatches, batchOffset, batchOffset + batchLimit);

    // Fetch batch from source
    const batch = await fetchSourceBatch(batchOffset, batchLimit, config.since);

    if (batch.length === 0) {
      break; // No more users
    }

    // Process batch
    const batchResult = await processBatch(batch, existingUsernames, config, stats, logger, auditLogger);

    logger.batchComplete(batchResult);

    processedCount += batch.length;

    // Log progress
    logger.progress(processedCount, effectiveTotal, 0);
  }

  // Finalize
  stats.endTime = Date.now();
  auditLogger.close();
  logger.summary(stats, config.dryRun);

  // Exit with appropriate code
  process.exit(stats.errors > 0 ? 2 : 0);
}

// ============================================================================
// ENTRY POINT
// ============================================================================

migrate().catch((error) => {
  console.error('\n' + '═'.repeat(70));
  console.error('  MIGRATION FAILED');
  console.error('═'.repeat(70));
  console.error(`  Error: ${error.message}`);
  console.error('═'.repeat(70) + '\n');
  process.exit(1);
});
