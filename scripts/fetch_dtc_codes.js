#!/usr/bin/env node
/**
 * Fetch manufacturer-specific OBD-II DTC codes from dot.report and save to
 * dtc_codes.json.
 *
 * dot.report has a three-level hierarchy:
 *   /dtc/           → prefix groups (P10, P11 … B00, C00 …)
 *   /dtc/P10        → individual code links (P1000 … P10FF)
 *   /dtc/P1000      → per-manufacturer descriptions
 *
 * Cloudflare auto-resolves after the first page load. Subsequent requests in
 * the same browser session are fast. Scraping is done with N concurrent pages.
 * Results are saved incrementally — Ctrl+C gives a partial but valid JSON file.
 *
 * Usage:
 *   cd scripts && npm install
 *   node fetch_dtc_codes.js
 *   node fetch_dtc_codes.js --headed
 *   node fetch_dtc_codes.js --concurrency 5
 *   node fetch_dtc_codes.js --ranges P0 P1 P2 P3        # restrict to specific ranges
 *   node fetch_dtc_codes.js --output /path/to/dtc_codes.json
 */

const { addExtra } = require("puppeteer-extra");
const rebrowser    = require("rebrowser-puppeteer");
const Stealth      = require("puppeteer-extra-plugin-stealth");
const fs           = require("fs");
const path         = require("path");

const pptr = addExtra(rebrowser);
pptr.use(Stealth());

// ── CLI ───────────────────────────────────────────────────────────────────────

const argv = process.argv.slice(2);
const flag = (name, def) => {
  const i = argv.indexOf(name);
  return i !== -1 ? argv[i + 1] : def;
};
const has = name => argv.includes(name);

const headed      = has("--headed");
const outputDir   = path.resolve(flag("--output", path.join(__dirname, "..", "dtc_codes")));
const concurrency = parseInt(flag("--concurrency", "10"), 10);
const rangesIdx   = argv.indexOf("--ranges");
const wantRanges  = rangesIdx !== -1
  ? new Set(argv.slice(rangesIdx + 1).filter(a => !a.startsWith("--")))
  : new Set(["P0", "P1", "P2", "P3", "B", "C", "U"]);   // default: all code types

// ── Make normalisation ────────────────────────────────────────────────────────

const MAKE_ALIASES = {
  "ford":               "ford",
  "lincoln":            "lincoln",
  "mercury":            "mercury",
  "gm":                 "chevrolet",
  "general motors":     "chevrolet",
  "chevrolet":          "chevrolet",
  "chevy":              "chevrolet",
  "buick":              "buick",
  "cadillac":           "cadillac",
  "gmc":                "gmc",
  "oldsmobile":         "oldsmobile",
  "pontiac":            "pontiac",
  "saturn":             "saturn",
  "toyota":             "toyota",
  "lexus":              "lexus",
  "honda":              "honda",
  "acura":              "acura",
  "nissan":             "nissan",
  "infiniti":           "infiniti",
  "mazda":              "mazda",
  "subaru":             "subaru",
  "mitsubishi":         "mitsubishi",
  "hyundai":            "hyundai",
  "kia":                "kia",
  "chrysler":           "chrysler",
  "dodge":              "dodge",
  "jeep":               "jeep",
  "ram":                "ram",
  "volvo":              "volvo",
  "bmw":                "bmw",
  "mini":               "mini",
  "mercedes":           "mercedes-benz",
  "mercedes benz":      "mercedes-benz",
  "mercedes-benz":      "mercedes-benz",
  "volkswagen":         "volkswagen",
  "vw":                 "volkswagen",
  "audi":               "audi",
  "porsche":            "porsche",
  "land rover":         "land rover",
  "jaguar":             "jaguar",
  "saab":               "saab",
  "isuzu":              "isuzu",
  "suzuki":             "suzuki",
  "freightliner":       "freightliner",
  "mack":               "mack",
  "peterbilt":          "peterbilt",
  // European
  "opel":               "opel",
  "vauxhall":           "opel",
  "holden":             "holden",
  "fiat":               "fiat",
  "alfa romeo":         "alfa romeo",
  "alfa":               "alfa romeo",
  "lancia":             "lancia",
  "skoda":              "skoda",
  "škoda":              "skoda",
  "seat":               "seat",
  "lamborghini":        "lamborghini",
  "lotus":              "lotus",
  "maserati":           "maserati",
  "peugeot":            "peugeot",
  "renault":            "renault",
  "dacia":              "renault",
  "citroen":            "citroen",
  "citroën":            "citroen",
  "ds":                 "citroen",
};

function normalizeMake(raw) {
  return MAKE_ALIASES[raw.toLowerCase().trim()] ?? null;
}

// ── Description parsing ───────────────────────────────────────────────────────

/**
 * Parse the list of "CODE: description [Make]" lines from a code page.
 * Returns { makeKey: description }.  Uses "_generic" for unattributed entries.
 */
function parseDescriptionLines(lines) {
  const result = {};

  for (const line of lines) {
    // Strip leading "P1000: " code prefix
    const text = line.replace(/^[A-Z][0-9A-F]{4,5}:\s*/i, "").trim();
    if (!text) continue;

    // Format: "Description text [Make1/Make2]"
    const bracketMatch = text.match(/^(.*?)\s*\[([^\]]+)\]\s*$/);
    if (bracketMatch) {
      const desc  = bracketMatch[1].trim();
      const makes = bracketMatch[2].split(/[/,]/).map(m => m.trim());
      for (const raw of makes) {
        const key = normalizeMake(raw);
        if (key && desc) result[key] = desc;
      }
      continue;
    }

    // Format: "Make: Description"  (only if Make is a known alias)
    const colonIdx = text.indexOf(":");
    if (colonIdx > 0 && colonIdx < 30) {
      const potentialMake = text.slice(0, colonIdx).trim();
      const desc          = text.slice(colonIdx + 1).trim();
      const key           = normalizeMake(potentialMake);
      if (key && desc) {
        result[key] = desc;
        continue;
      }
    }

    // Unattributed — generic fallback (first one wins)
    if (!result["_generic"]) result["_generic"] = text;
  }

  return result;
}

// ── Browser helpers ───────────────────────────────────────────────────────────

async function waitCF(page, ms = 20000) {
  const deadline = Date.now() + ms;
  while (Date.now() < deadline) {
    const t = await page.title().catch(() => "");
    if (!/just a moment|checking your browser|enable javascript|cloudflare/i.test(t)) return true;
    await sleep(800);
  }
  return false;
}

async function navigate(page, url) {
  try {
    await page.goto(url, { waitUntil: "domcontentloaded", timeout: 30000 });
  } catch {
    // timeout ok — content may still be there
  }
  return waitCF(page, 15000);
}

async function getPrefixLinks(page) {
  return page.evaluate(() =>
    [...document.querySelectorAll("a[href*='/dtc/']")]
      .map(a => a.href)
      .filter(h => /\/dtc\/[A-Z][0-9A-F]{2}$/i.test(h))
  );
}

async function getCodeLinks(page) {
  return page.evaluate(() =>
    [...document.querySelectorAll("a[href*='/dtc/']")]
      .map(a => a.href)
      .filter(h => /\/dtc\/[A-Z][0-9A-F]{4,5}$/i.test(h))
  );
}

async function scrapeCodePage(page, url) {
  const ok = await navigate(page, url);
  if (!ok) return {};

  return page.evaluate(() => {
    const lines = [];
    for (const el of document.querySelectorAll("li, p, td")) {
      const t = el.innerText?.trim();
      if (t && /^[A-Z][0-9A-F]{4,5}:/i.test(t)) lines.push(t);
    }
    return lines;
  }).then(parseDescriptionLines).catch(() => ({}));
}

// ── ETA tracker ──────────────────────────────────────────────────────────────

class ETA {
  constructor(windowSize = 150) {
    this.times = [];
    this.window = windowSize;
  }
  tick() {
    this.times.push(Date.now());
    if (this.times.length > this.window) this.times.shift();
  }
  // Returns estimated ms remaining, or null if not enough data yet.
  estimate(remaining) {
    if (this.times.length < 2) return null;
    const elapsed = this.times[this.times.length - 1] - this.times[0];
    const rate = (this.times.length - 1) / elapsed; // completions/ms
    return remaining / rate;
  }
}

function fmtEta(ms) {
  if (ms == null) return "--:--";
  const s = Math.round(ms / 1000);
  if (s < 60) return `${s}s`;
  const m = Math.floor(s / 60), r = s % 60;
  if (m < 60) return `${m}m ${r.toString().padStart(2, "0")}s`;
  const h = Math.floor(m / 60), rm = m % 60;
  return `${h}h ${rm.toString().padStart(2, "0")}m`;
}

// ── Concurrency pool ──────────────────────────────────────────────────────────

async function pool(tasks, limit, worker) {
  const results = new Array(tasks.length);
  let idx = 0;

  async function run(workerIdx) {
    while (idx < tasks.length) {
      const i = idx++;
      results[i] = await worker(tasks[i], i, workerIdx);
    }
  }

  await Promise.all(Array.from({ length: limit }, (_, workerIdx) => run(workerIdx)));
  return results;
}

// ── Incremental save ──────────────────────────────────────────────────────────

let db = {};
const dirtyMakes = new Set();
const knownCodes = new Set();   // O(1) skip check

function merge(parsed) {
  for (const [make, codes] of Object.entries(parsed)) {
    db[make] ??= {};
    Object.assign(db[make], codes);
    dirtyMakes.add(make);
    for (const code of Object.keys(codes)) knownCodes.add(code);
  }
}

// Debounced save — flushes at most every 500 ms; call flush() to force immediately.
let _saveTimer = null;
function save() {
  if (_saveTimer) return;
  _saveTimer = setTimeout(flush, 500);
}
function flush() {
  clearTimeout(_saveTimer);
  _saveTimer = null;
  if (dirtyMakes.size === 0) return;
  fs.mkdirSync(outputDir, { recursive: true });
  for (const make of dirtyMakes) {
    fs.writeFileSync(path.join(outputDir, `${make}.json`), JSON.stringify(db[make], null, 2));
  }
  dirtyMakes.clear();
}

function hasCode(code) {
  return knownCodes.has(code);
}

// ── Main ──────────────────────────────────────────────────────────────────────

const sleep = ms => new Promise(r => setTimeout(r, ms));

async function main() {
  // Load existing per-make files for incremental updates
  try {
    if (fs.existsSync(outputDir)) {
      for (const file of fs.readdirSync(outputDir).filter(f => f.endsWith(".json"))) {
        const make = file.slice(0, -5);
        db[make] = JSON.parse(fs.readFileSync(path.join(outputDir, file), "utf8"));
      }
      const total = Object.values(db).reduce((s, m) => s + Object.keys(m).length, 0);
      // Populate knownCodes for O(1) skip checks
      for (const codes of Object.values(db))
        for (const code of Object.keys(codes)) knownCodes.add(code);
      if (total > 0) console.log(`Loaded ${Object.keys(db).length} make files (${total} codes)\n`);
    }
  } catch { /* fresh start */ }

  // Save on exit (including Ctrl+C)
  process.on("SIGINT", () => { flush(); console.log(`\nInterrupted — saved partial results to ${outputDir}`); process.exit(0); });

  const browser = await pptr.launch({
    headless: !headed,
    args: ["--no-sandbox", "--disable-setuid-sandbox", "--disable-blink-features=AutomationControlled"],
  });

  try {
    // ── Step 1: fetch index, warm up CF session ──────────────────────────────
    console.log("Fetching code index from dot.report/dtc/ ...");
    const indexPage = await browser.newPage();
    await indexPage.setViewport({ width: 1366, height: 768 });

    const ok = await navigate(indexPage, "https://dot.report/dtc/");
    if (!ok) {
      console.error("Cloudflare challenge did not resolve. Try --headed to solve manually.");
      process.exit(1);
    }

    const allPrefixes = await getPrefixLinks(indexPage);
    const prefixes = allPrefixes.filter(url => {
      const prefix = url.match(/\/dtc\/([A-Z][0-9A-F]{2})$/i)?.[1]?.toUpperCase();
      return prefix && [...wantRanges].some(r => prefix.startsWith(r));
    });

    console.log(`Found ${prefixes.length} prefix groups matching ranges [${[...wantRanges].join(", ")}]\n`);

    // ── Create worker pages (shared across step 2 and 3) ────────────────────
    const pages = await Promise.all(
      Array.from({ length: concurrency }, async () => {
        const p = await browser.newPage();
        await p.setViewport({ width: 1366, height: 768 });
        return p;
      })
    );

    // ── Step 2: collect all individual code URLs (parallel) ──────────────────
    const codeUrlSet = new Set();
    let prefixDone = 0;
    const etaPrefix = new ETA();
    await pool(prefixes, concurrency, async (url, i, workerIdx) => {
      const page = pages[workerIdx];
      const prefix = url.match(/\/([A-Z][0-9A-F]+)$/i)[1];
      await navigate(page, url);
      const links = await getCodeLinks(page);
      links.forEach(l => codeUrlSet.add(l));
      prefixDone++;
      etaPrefix.tick();
      const eta = fmtEta(etaPrefix.estimate(prefixes.length - prefixDone));
      process.stdout.write(`\r  ${prefixDone}/${prefixes.length} prefix groups fetched (${codeUrlSet.size} codes so far) — ETA ${eta}   `);
    });
    console.log();

    const codeUrls = [...codeUrlSet];
    console.log(`\nTotal unique codes: ${codeUrls.length}`);

    // ── Step 3: scrape each code page with N concurrent workers ─────────────
    const skipped = codeUrls.filter(url => {
      const code = url.match(/\/dtc\/([A-Z0-9]+)$/i)?.[1]?.toUpperCase();
      return code && hasCode(code);
    }).length;
    if (skipped > 0) console.log(`Skipping ${skipped} already-scraped codes`);
    let done = 0;
    const etaCode = new ETA();

    await pool(codeUrls, concurrency, async (url, i, workerIdx) => {
      const page = pages[workerIdx];
      const code = url.match(/\/dtc\/([A-Z0-9]+)$/i)?.[1]?.toUpperCase() ?? "?";
      if (hasCode(code)) { done++; etaCode.tick(); return; }
      const flat = await scrapeCodePage(page, url);
      // flat = { make: "description" } — wrap into { make: { CODE: "description" } }
      const parsed = {};
      for (const [make, desc] of Object.entries(flat)) {
        parsed[make] = { [code]: desc };
      }
      merge(parsed);
      save();
      done++;
      etaCode.tick();
      if (done % 25 === 0 || done === codeUrls.length) {
        const makes = Object.keys(db).filter(k => k !== "_generic").length;
        const total = Object.values(db).reduce((s, m) => s + Object.keys(m).length, 0);
        const eta = fmtEta(etaCode.estimate(codeUrls.length - done));
        process.stdout.write(`\r  ${done}/${codeUrls.length} codes scraped — ${total} descriptions across ${makes} makes — ETA ${eta}   `);
      }
    });

    console.log();

  } finally {
    await browser.close();
    flush();
  }

  const makes = Object.keys(db).filter(k => k !== "_generic").sort();
  const total = Object.values(db).reduce((s, m) => s + Object.keys(m).length, 0);
  console.log(`\nWrote ${outputDir}/`);
  console.log(`${total} descriptions across ${makes.length} makes: ${makes.join(", ")}`);
}

// need to pass parseDescriptionLines into page.evaluate scope
function parseDescriptionLines(lines) {
  const ALIASES = {"ford":"ford","lincoln":"lincoln","mercury":"mercury","gm":"chevrolet","general motors":"chevrolet","chevrolet":"chevrolet","chevy":"chevrolet","buick":"buick","cadillac":"cadillac","gmc":"gmc","oldsmobile":"oldsmobile","pontiac":"pontiac","saturn":"saturn","toyota":"toyota","lexus":"lexus","honda":"honda","acura":"acura","nissan":"nissan","infiniti":"infiniti","mazda":"mazda","subaru":"subaru","mitsubishi":"mitsubishi","hyundai":"hyundai","kia":"kia","chrysler":"chrysler","dodge":"dodge","jeep":"jeep","ram":"ram","volvo":"volvo","bmw":"bmw","mini":"mini","mercedes":"mercedes-benz","mercedes benz":"mercedes-benz","mercedes-benz":"mercedes-benz","volkswagen":"volkswagen","vw":"volkswagen","audi":"audi","porsche":"porsche","land rover":"land rover","jaguar":"jaguar","saab":"saab","isuzu":"isuzu","suzuki":"suzuki","freightliner":"freightliner","mack":"mack","peterbilt":"peterbilt","opel":"opel","vauxhall":"opel","holden":"holden","fiat":"fiat","alfa romeo":"alfa romeo","alfa":"alfa romeo","lancia":"lancia","skoda":"skoda","škoda":"skoda","seat":"seat","lamborghini":"lamborghini","lotus":"lotus","maserati":"maserati","peugeot":"peugeot","renault":"renault","dacia":"renault","citroen":"citroen","citroën":"citroen","ds":"citroen"};
  const norm = r => ALIASES[r.toLowerCase().trim()] ?? null;
  const result = {};
  for (const line of lines) {
    const text = line.replace(/^[A-Z][0-9A-F]{4,5}:\s*/i, "").trim();
    if (!text) continue;
    const bm = text.match(/^(.*?)\s*\[([^\]]+)\]\s*$/);
    if (bm) {
      const desc = bm[1].trim();
      for (const raw of bm[2].split(/[/,]/)) { const k = norm(raw); if (k && desc) result[k] = desc; }
      continue;
    }
    const ci = text.indexOf(":");
    if (ci > 0 && ci < 30) { const k = norm(text.slice(0, ci)); if (k) { result[k] = text.slice(ci + 1).trim(); continue; } }
    if (!result["_generic"]) result["_generic"] = text;
  }
  return result;
}

main().catch(e => { console.error(e); process.exit(1); });
