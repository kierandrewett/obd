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
    pub fn lookup(&self, make: &str, code: &str) -> Option<&str> {
        // Strip regional qualifier: "Toyota (Australia)" → "toyota"
        let make_lc = make.split('(').next().unwrap_or(make).trim().to_lowercase();
        let code_uc = code.to_uppercase();

        // Direct match
        if let Some(desc) = self.by_make.get(&make_lc).and_then(|m| m.get(&code_uc)) {
            return Some(desc.as_str());
        }

        // Common alias groups (brands that share code tables)
        let alias = match make_lc.as_str() {
            // GM family
            "chevrolet" | "buick" | "gmc" | "cadillac" | "oldsmobile" | "pontiac" | "saturn"
            | "holden" | "opel" | "vauxhall" => Some("chevrolet"),
            // Toyota family
            "lexus" | "scion" | "nummi" => Some("toyota"),
            // Honda family
            "acura" | "gac honda" => Some("honda"),
            // Nissan family
            "infiniti" => Some("nissan"),
            // Ford family
            "lincoln" | "mercury" => Some("ford"),
            // Stellantis / Chrysler family
            "dodge" | "jeep" | "ram" => Some("chrysler"),
            // Fiat / Stellantis European
            "lancia" | "maserati" | "alfa romeo" => Some("fiat"),
            // VW Group
            "bentley" | "lamborghini" | "seat" | "skoda"
            | "faw-volkswagen" | "shanghai volkswagen" => Some("volkswagen"),
            // BMW Group
            "bmw m" | "bmw i" => Some("bmw"),
            // Audi / VW Sport
            "audi sport" => Some("audi"),
            // Renault Group
            "dacia" | "renault samsung" => Some("renault"),
            // PSA / Stellantis
            "ds automobiles" | "ds" => Some("citroen"),
            _ => None,
        };

        if let Some(a) = alias {
            if let Some(desc) = self.by_make.get(a).and_then(|m| m.get(&code_uc)) {
                return Some(desc.as_str());
            }
        }

        // Generic fallback (codes with no make attribution on the source page)
        self.by_make
            .get("_generic")
            .and_then(|m| m.get(&code_uc))
            .map(|s| s.as_str())
    }

    pub fn is_loaded(&self) -> bool {
        !self.by_make.is_empty()
    }

    pub fn make_count(&self) -> usize {
        self.by_make.len()
    }

    pub fn code_count(&self) -> usize {
        self.by_make.values().map(|m| m.len()).sum()
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
