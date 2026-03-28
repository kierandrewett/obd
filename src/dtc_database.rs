use std::collections::HashMap;

/// Manufacturer-specific DTC database loaded from dtc_codes.json.
///
/// JSON format:
/// ```json
/// {
///   "ford":    { "P1000": "OBD Systems Readiness Test Not Complete", ... },
///   "toyota":  { "P1100": "MAF Sensor Circuit Intermittent", ... },
///   ...
/// }
/// ```
/// Keys are lowercase make names matching the output of `vin_decoder` (e.g. "ford",
/// "mercedes-benz", "land rover").
#[derive(Debug, Default)]
pub struct DtcDatabase {
    by_make: HashMap<String, HashMap<String, String>>,
}

impl DtcDatabase {
    /// Load from a path — either a directory of per-make JSON files or a single
    /// combined JSON file.  Returns an empty database on any error.
    pub fn load(path: &str) -> Self {
        let p = std::path::Path::new(path);
        if p.is_dir() {
            Self::load_dir(p)
        } else {
            Self::load_file(p)
        }
    }

    fn load_file(p: &std::path::Path) -> Self {
        let content = match std::fs::read_to_string(p) {
            Ok(s) => s,
            Err(_) => return Self::default(),
        };
        let raw: HashMap<String, HashMap<String, String>> = match serde_json::from_str(&content) {
            Ok(v) => v,
            Err(e) => {
                tracing::warn!(path = %p.display(), error = %e, "Failed to parse DTC file");
                return Self::default();
            }
        };
        Self { by_make: Self::normalise(raw) }
    }

    fn load_dir(dir: &std::path::Path) -> Self {
        let entries = match std::fs::read_dir(dir) {
            Ok(e) => e,
            Err(_) => return Self::default(),
        };
        let mut by_make = HashMap::new();
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) != Some("json") {
                continue;
            }
            let make = match path.file_stem().and_then(|s| s.to_str()) {
                Some(s) => s.to_lowercase(),
                None => continue,
            };
            let content = match std::fs::read_to_string(&path) {
                Ok(s) => s,
                Err(_) => continue,
            };
            let codes: HashMap<String, String> = match serde_json::from_str(&content) {
                Ok(v) => v,
                Err(e) => {
                    tracing::warn!(path = %path.display(), error = %e, "Failed to parse DTC file");
                    continue;
                }
            };
            by_make.insert(make, codes.into_iter().map(|(k, v)| (k.to_uppercase(), v)).collect());
        }
        Self { by_make }
    }

    fn normalise(raw: HashMap<String, HashMap<String, String>>) -> HashMap<String, HashMap<String, String>> {
        raw.into_iter()
            .map(|(make, codes)| {
                let codes = codes.into_iter().map(|(k, v)| (k.to_uppercase(), v)).collect();
                (make.to_lowercase(), codes)
            })
            .collect()
    }

    /// Look up a description for a DTC code.
    /// `make` is the raw make string from the VIN decoder (e.g. "Ford", "Mercedes-Benz").
    /// Priority: manufacturer-specific → alias group → generic fallback.
    /// Returns `(description, alias_source)` where `alias_source` is `Some("chevrolet")` when
    /// the match came from an alias group rather than a direct match.
    pub fn lookup_with_source<'a>(&'a self, make: &str, code: &str) -> Option<(&'a str, Option<&'static str>)> {
        // Strip regional qualifier: "Toyota (Australia)" → "toyota"
        let make_lc = make.split('(').next().unwrap_or(make).trim().to_lowercase();
        let code_uc = code.to_uppercase();

        // Direct match — no source label
        if let Some(desc) = self.by_make.get(&make_lc).and_then(|m| m.get(&code_uc)) {
            return Some((desc.as_str(), None));
        }

        // Corporate family alias groups — ordered by most likely to match first.
        // Each entry is a slice of canonical make keys to try in sequence.
        let aliases: &[&'static str] = match make_lc.as_str() {
            // ── GM ───────────────────────────────────────────────────────────
            "buick" | "gmc" | "cadillac" | "pontiac" | "holden"
                => &["chevrolet", "oldsmobile", "saturn"],
            "oldsmobile"
                => &["chevrolet", "saturn"],
            "saturn"
                => &["chevrolet", "oldsmobile"],
            // Opel/Vauxhall: GM until 2017, then PSA/Stellantis
            "opel" | "vauxhall"
                => &["chevrolet", "oldsmobile", "saturn", "citroen", "peugeot"],
            // Saab: GM-owned 1990–2011
            "saab"
                => &["chevrolet", "oldsmobile", "saturn"],
            // Isuzu: long-running GM partnership, shared platforms
            "isuzu"
                => &["chevrolet", "oldsmobile"],

            // ── Ford ─────────────────────────────────────────────────────────
            "lincoln" | "mercury"
                => &["ford"],
            // Jaguar/Land Rover: Ford-owned 1989–2008
            "jaguar" | "land rover"
                => &["ford", "lincoln"],
            // Mazda: Ford held 33% stake until 2008, shared many platforms
            "mazda"
                => &["ford"],
            // Volvo Cars: Ford-owned 1999–2010
            "volvo"
                => &["ford", "lincoln"],

            // ── Toyota group ─────────────────────────────────────────────────
            "lexus" | "scion" | "nummi" | "daihatsu"
                => &["toyota"],
            // Subaru: Toyota holds ~20% stake, shares platforms (BRZ/GR86)
            "subaru"
                => &["toyota", "lexus"],
            // Suzuki: Toyota cross-shareholding, shared kei/compact platforms
            "suzuki"
                => &["toyota"],

            // ── Honda ────────────────────────────────────────────────────────
            "acura" | "gac honda"
                => &["honda"],

            // ── Renault-Nissan-Mitsubishi alliance ───────────────────────────
            "infiniti"
                => &["nissan", "mitsubishi"],
            "mitsubishi"
                => &["nissan"],
            "renault" | "dacia" | "renault samsung" | "alpine"
                => &["nissan", "mitsubishi"],

            // ── Stellantis US (Chrysler/FCA side) ────────────────────────────
            "dodge" | "ram"
                => &["chrysler", "jeep"],
            "jeep"
                => &["chrysler", "dodge"],
            "chrysler"
                => &["dodge", "jeep"],

            // ── Stellantis EU / FCA cross-over ───────────────────────────────
            // Fiat-Chrysler merged 2014 — try both sides
            "fiat"
                => &["alfa romeo", "chrysler", "dodge", "citroen", "peugeot"],
            "alfa romeo" | "lancia" | "maserati" | "abarth"
                => &["fiat", "citroen", "peugeot"],

            // ── Stellantis PSA side ──────────────────────────────────────────
            "peugeot" | "ds automobiles" | "ds"
                => &["citroen"],
            "citroen"
                => &["peugeot"],

            // ── VW Group ─────────────────────────────────────────────────────
            "volkswagen"
                => &["audi", "porsche"],
            "audi" | "audi sport"
                => &["volkswagen", "porsche"],
            "porsche"
                => &["audi", "volkswagen"],
            "lamborghini"
                => &["audi", "volkswagen", "porsche"],
            "bentley" | "seat" | "cupra" | "skoda"
            | "faw-volkswagen" | "shanghai volkswagen"
                => &["volkswagen", "audi", "porsche"],

            // ── BMW Group ────────────────────────────────────────────────────
            "mini"
                => &["bmw"],
            "bmw m" | "bmw i" | "rolls-royce" | "alpina"
                => &["bmw", "mini"],

            // ── Hyundai-Kia Group ────────────────────────────────────────────
            "kia" | "genesis"
                => &["hyundai"],
            "hyundai"
                => &["kia"],

            // ── Mercedes-Benz Group ──────────────────────────────────────────
            // smart: Mercedes joint venture until 2020 full acquisition
            "smart"
                => &["mercedes-benz"],

            _ => &[],
        };

        for &alias in aliases {
            if let Some(desc) = self.by_make.get(alias).and_then(|m| m.get(&code_uc)) {
                return Some((desc.as_str(), Some(alias)));
            }
        }

        // Generic fallback — no source label
        self.by_make
            .get("_generic")
            .and_then(|m| m.get(&code_uc))
            .map(|s| (s.as_str(), None))
    }

    /// Convenience wrapper — returns the description only (no source attribution).
    #[allow(dead_code)]
    pub fn lookup(&self, make: &str, code: &str) -> Option<&str> {
        self.lookup_with_source(make, code).map(|(desc, _)| desc)
    }

    /// Search every manufacturer's table for a code when no primary match was found.
    /// Returns `(description, make_name)` from the first match found, excluding
    /// `exclude_make` and the `_generic` bucket (already tried by `lookup`).
    pub fn lookup_any(&self, exclude_make: &str, code: &str) -> Option<(&str, &str)> {
        let exclude = exclude_make.split('(').next().unwrap_or(exclude_make).trim().to_lowercase();
        let code_uc = code.to_uppercase();
        for (make, codes) in &self.by_make {
            if make == "_generic" || *make == exclude {
                continue;
            }
            if let Some(desc) = codes.get(&code_uc) {
                return Some((desc.as_str(), make.as_str()));
            }
        }
        None
    }

    pub fn is_loaded(&self) -> bool {
        !self.by_make.is_empty()
    }
}

/// Return the corporate-family display name for a canonical alias make.
/// Used in the UI to explain why a description came from a related manufacturer.
pub fn family_label(canonical: &str) -> &'static str {
    match canonical {
        "chevrolet"  => "GM",
        "toyota"     => "Toyota Group",
        "honda"      => "Honda Group",
        "nissan"     => "Renault-Nissan-Mitsubishi",
        "ford"       => "Ford Group",
        "chrysler"   => "Stellantis",
        "fiat"       => "Stellantis",
        "volkswagen" => "VW Group",
        "audi"       => "VW Group",
        "bmw"        => "BMW Group",
        "renault"    => "Renault Group",
        "citroen"    => "Stellantis",
        _            => "related manufacturer",
    }
}

impl DtcDatabase {

    pub fn make_count(&self) -> usize {
        self.by_make.len()
    }

    pub fn code_count(&self) -> usize {
        self.by_make.values().map(|m| m.len()).sum()
    }
}

/// A compile-time-embedded copy of the DTC database, available on all platforms
/// but primarily used by WASM builds (which cannot access the filesystem at runtime).
#[cfg(target_arch = "wasm32")]
pub static EMBEDDED_DB: std::sync::LazyLock<DtcDatabase> =
    std::sync::LazyLock::new(DtcDatabase::load_embedded);

impl DtcDatabase {
    /// Build a `DtcDatabase` from JSON files compiled into the binary via `include_str!`.
    #[cfg(target_arch = "wasm32")]
    pub fn load_embedded() -> Self {
        let files: &[(&str, &str)] = &[
            ("_generic",       include_str!("../dtc_codes/_generic.json")),
            ("acura",          include_str!("../dtc_codes/acura.json")),
            ("alfa romeo",     include_str!("../dtc_codes/alfa romeo.json")),
            ("audi",           include_str!("../dtc_codes/audi.json")),
            ("bmw",            include_str!("../dtc_codes/bmw.json")),
            ("chevrolet",      include_str!("../dtc_codes/chevrolet.json")),
            ("chrysler",       include_str!("../dtc_codes/chrysler.json")),
            ("citroen",        include_str!("../dtc_codes/citroen.json")),
            ("dodge",          include_str!("../dtc_codes/dodge.json")),
            ("fiat",           include_str!("../dtc_codes/fiat.json")),
            ("ford",           include_str!("../dtc_codes/ford.json")),
            ("honda",          include_str!("../dtc_codes/honda.json")),
            ("hyundai",        include_str!("../dtc_codes/hyundai.json")),
            ("isuzu",          include_str!("../dtc_codes/isuzu.json")),
            ("jaguar",         include_str!("../dtc_codes/jaguar.json")),
            ("jeep",           include_str!("../dtc_codes/jeep.json")),
            ("kia",            include_str!("../dtc_codes/kia.json")),
            ("lamborghini",    include_str!("../dtc_codes/lamborghini.json")),
            ("land rover",     include_str!("../dtc_codes/land rover.json")),
            ("lexus",          include_str!("../dtc_codes/lexus.json")),
            ("lincoln",        include_str!("../dtc_codes/lincoln.json")),
            ("mazda",          include_str!("../dtc_codes/mazda.json")),
            ("mercedes-benz",  include_str!("../dtc_codes/mercedes-benz.json")),
            ("mercury",        include_str!("../dtc_codes/mercury.json")),
            ("mini",           include_str!("../dtc_codes/mini.json")),
            ("mitsubishi",     include_str!("../dtc_codes/mitsubishi.json")),
            ("nissan",         include_str!("../dtc_codes/nissan.json")),
            ("oldsmobile",     include_str!("../dtc_codes/oldsmobile.json")),
            ("opel",           include_str!("../dtc_codes/opel.json")),
            ("peugeot",        include_str!("../dtc_codes/peugeot.json")),
            ("porsche",        include_str!("../dtc_codes/porsche.json")),
            ("saab",           include_str!("../dtc_codes/saab.json")),
            ("saturn",         include_str!("../dtc_codes/saturn.json")),
            ("subaru",         include_str!("../dtc_codes/subaru.json")),
            ("suzuki",         include_str!("../dtc_codes/suzuki.json")),
            ("toyota",         include_str!("../dtc_codes/toyota.json")),
            ("volkswagen",     include_str!("../dtc_codes/volkswagen.json")),
            ("volvo",          include_str!("../dtc_codes/volvo.json")),
        ];
        let mut by_make = std::collections::HashMap::new();
        for (make, content) in files {
            let codes: std::collections::HashMap<String, String> =
                match serde_json::from_str(content) {
                    Ok(v) => v,
                    Err(_) => continue,
                };
            by_make.insert(
                make.to_string(),
                codes.into_iter().map(|(k, v)| (k.to_uppercase(), v)).collect(),
            );
        }
        Self { by_make }
    }
}

/// Find the DTC database — prefers a `dtc_codes/` directory, falls back to a
/// single `dtc_codes.json` file.  Checks next to the executable first, then CWD.
pub fn find_database_path() -> Option<String> {
    let candidates: &[(&str, bool)] = &[
        ("dtc_codes", true),       // directory (new format)
        ("dtc_codes.json", false), // single file (legacy)
    ];

    // Check next to the executable
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            for (name, is_dir) in candidates {
                let p = dir.join(name);
                if *is_dir && p.is_dir() || !*is_dir && p.is_file() {
                    return Some(p.to_string_lossy().into_owned());
                }
            }
        }
    }

    // Check current working directory
    for (name, is_dir) in candidates {
        let p = std::path::Path::new(name);
        if *is_dir && p.is_dir() || !*is_dir && p.is_file() {
            return Some(p.to_string_lossy().into_owned());
        }
    }

    None
}
