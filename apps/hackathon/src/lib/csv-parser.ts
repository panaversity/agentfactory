/**
 * Simple CSV parser for external form sync
 * Handles Google Forms CSV export format
 */

export interface CSVRow {
  [key: string]: string;
}

export interface ParseResult {
  headers: string[];
  rows: CSVRow[];
  errors: string[];
}

/**
 * Parse CSV string into rows with headers as keys
 */
export function parseCSV(csvContent: string): ParseResult {
  const errors: string[] = [];
  const lines = csvContent.trim().split("\n");

  if (lines.length === 0) {
    return { headers: [], rows: [], errors: ["Empty CSV file"] };
  }

  // Parse headers (first line)
  const headers = parseCSVLine(lines[0]);

  if (headers.length === 0) {
    return { headers: [], rows: [], errors: ["No headers found"] };
  }

  // Parse data rows
  const rows: CSVRow[] = [];
  for (let i = 1; i < lines.length; i++) {
    const line = lines[i].trim();
    if (!line) continue; // Skip empty lines

    const values = parseCSVLine(line);

    if (values.length !== headers.length) {
      errors.push(
        `Row ${i + 1}: Expected ${headers.length} columns, got ${values.length}`
      );
      continue;
    }

    const row: CSVRow = {};
    headers.forEach((header, index) => {
      row[header] = values[index] || "";
    });
    rows.push(row);
  }

  return { headers, rows, errors };
}

/**
 * Parse a single CSV line, handling quoted fields
 */
function parseCSVLine(line: string): string[] {
  const values: string[] = [];
  let current = "";
  let inQuotes = false;

  for (let i = 0; i < line.length; i++) {
    const char = line[i];
    const nextChar = line[i + 1];

    if (inQuotes) {
      if (char === '"' && nextChar === '"') {
        // Escaped quote
        current += '"';
        i++; // Skip next quote
      } else if (char === '"') {
        // End of quoted field
        inQuotes = false;
      } else {
        current += char;
      }
    } else {
      if (char === '"') {
        // Start of quoted field
        inQuotes = true;
      } else if (char === ",") {
        // Field separator
        values.push(current.trim());
        current = "";
      } else {
        current += char;
      }
    }
  }

  // Don't forget the last field
  values.push(current.trim());

  return values;
}

/**
 * Find the email column in CSV headers
 * Google Forms typically uses "Email Address" or similar
 */
export function findEmailColumn(headers: string[]): string | null {
  const emailPatterns = [
    /^email$/i,
    /^email address$/i,
    /^e-mail$/i,
    /^your email$/i,
    /email/i,
  ];

  for (const pattern of emailPatterns) {
    const match = headers.find((h) => pattern.test(h));
    if (match) return match;
  }

  return null;
}

/**
 * Find common field columns in CSV headers
 */
export function findCommonColumns(headers: string[]): {
  email: string | null;
  name: string | null;
  projectName: string | null;
  repositoryUrl: string | null;
  demoUrl: string | null;
  youtubeUrl: string | null;
} {
  const find = (patterns: RegExp[]): string | null => {
    for (const pattern of patterns) {
      const match = headers.find((h) => pattern.test(h));
      if (match) return match;
    }
    return null;
  };

  return {
    email: find([/^email$/i, /^email address$/i, /email/i]),
    name: find([/^full name$/i, /^your name$/i, /^name$/i]),
    projectName: find([/project name/i, /project title/i]),
    repositoryUrl: find([/github.*url/i, /repo.*url/i, /repository/i, /github/i]),
    demoUrl: find([/demo.*url/i, /live.*url/i, /deployed/i, /vercel/i, /pages/i]),
    youtubeUrl: find([/youtube/i, /video/i, /demo.*video/i]),
  };
}

/**
 * Map CSV row to submission data
 */
export interface ColumnMapping {
  email: string;
  projectName?: string;
  repositoryUrl?: string;
  demoUrl?: string;
  // All other columns go to formData
}

export function mapRowToSubmission(
  row: CSVRow,
  mapping: ColumnMapping,
  allHeaders: string[]
): {
  email: string;
  projectName: string;
  repositoryUrl: string;
  demoUrl?: string;
  formData: Record<string, string>;
} {
  const email = row[mapping.email] || "";
  const projectName = mapping.projectName
    ? row[mapping.projectName] || "Untitled Project"
    : "Untitled Project";
  const repositoryUrl = mapping.repositoryUrl
    ? row[mapping.repositoryUrl] || ""
    : "";
  const demoUrl = mapping.demoUrl ? row[mapping.demoUrl] : undefined;

  // All non-mapped columns go to formData
  const mappedColumns = new Set([
    mapping.email,
    mapping.projectName,
    mapping.repositoryUrl,
    mapping.demoUrl,
  ].filter(Boolean));

  const formData: Record<string, string> = {};
  for (const header of allHeaders) {
    if (!mappedColumns.has(header) && row[header]) {
      formData[header] = row[header];
    }
  }

  return {
    email,
    projectName,
    repositoryUrl,
    demoUrl,
    formData,
  };
}
