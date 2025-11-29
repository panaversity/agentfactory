#!/usr/bin/env node
import { promises as fs } from "node:fs";
import path from "node:path";

async function* walk(dir) {
  for (const entry of await fs.readdir(dir, { withFileTypes: true })) {
    const fullPath = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      yield* walk(fullPath);
    } else if (entry.isFile() && entry.name.endsWith(".html")) {
      yield fullPath;
    }
  }
}

async function main() {
  const buildDir = path.resolve(process.cwd(), "build");
  try {
    await fs.access(buildDir);
  } catch {
    console.error(`[check-jsonld-ssr] Build directory not found: ${buildDir}`);
    process.exit(2);
  }

  const results = [];
  for await (const file of walk(buildDir)) {
    const html = await fs.readFile(file, "utf8");
    // Match script tag with id="jsonld-article" and type="application/ld+json" in any order
    const hasArticle =
      /<script[^>]*\bid=["']jsonld-article["'][^>]*>/.test(html) &&
      /<script[^>]*\btype=["']application\/ld\+json["'][^>]*>/.test(html);
    const dupCount = (html.match(/id=["']jsonld-article["']/g) || []).length;
    results.push({ file, hasArticle, dupCount });
  }

  // Summarize docs pages only (contains '/docs/' in path)
  const docs = results.filter((r) =>
    r.file.includes(`${path.sep}docs${path.sep}`)
  );
  const ok = docs.filter((r) => r.hasArticle && r.dupCount === 1).length;
  const total = docs.length;

  console.log(`Docs with SSR Article JSON-LD (exactly one): ${ok}/${total}`);
  for (const r of docs.slice(0, 20)) {
    console.log(
      `${r.hasArticle && r.dupCount === 1 ? "OK " : "NO "} | dups=${
        r.dupCount
      } | ${path.relative(buildDir, r.file)}`
    );
  }
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
