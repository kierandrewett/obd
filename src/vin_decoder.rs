use std::collections::HashMap;
use std::sync::LazyLock;

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct VinInfo {
    pub vin: String,
    pub make: String,
    pub country: String,
    pub year: Option<String>,
    pub wmi: String,
}

/// Decode a VIN string into make/country/year info
pub fn decode(vin: &str) -> VinInfo {
    let vin = vin.trim().to_uppercase();
    let wmi = if vin.len() >= 3 { &vin[..3] } else { &vin };

    let (make, country) = lookup_wmi(wmi);

    let year = if vin.len() >= 10 {
        decode_year(vin.as_bytes()[9] as char)
    } else {
        None
    };

    VinInfo {
        vin: vin.to_string(),
        make,
        country,
        year,
        wmi: wmi.to_string(),
    }
}

fn decode_year(c: char) -> Option<String> {
    let year = match c {
        'A' => 2010, 'B' => 2011, 'C' => 2012, 'D' => 2013, 'E' => 2014,
        'F' => 2015, 'G' => 2016, 'H' => 2017, 'J' => 2018, 'K' => 2019,
        'L' => 2020, 'M' => 2021, 'N' => 2022, 'P' => 2023, 'R' => 2024,
        'S' => 2025, 'T' => 2026, 'V' => 2027, 'W' => 2028, 'X' => 2029,
        'Y' => 2030,
        '1' => 2001, '2' => 2002, '3' => 2003, '4' => 2004, '5' => 2005,
        '6' => 2006, '7' => 2007, '8' => 2008, '9' => 2009,
        _ => return None,
    };
    Some(format!("{year}"))
}

fn lookup_wmi(wmi: &str) -> (String, String) {
    // Try exact 3-char match first
    if let Some(&(make, country)) = WMI_MAP.get(wmi) {
        return (make.to_string(), country.to_string());
    }

    // Try 2-char prefix for manufacturers that use ranges
    if wmi.len() >= 2 {
        if let Some(&(make, country)) = WMI_2CHAR.get(&wmi[..2]) {
            return (make.to_string(), country.to_string());
        }
    }

    // Decode country from first char
    let country = match wmi.as_bytes().first() {
        Some(b'1' | b'4' | b'5') => "United States",
        Some(b'2') => "Canada",
        Some(b'3') => "Mexico",
        Some(b'6' | b'7') => "Australia/New Zealand",
        Some(b'8' | b'9') => "South America",
        Some(b'J') => "Japan",
        Some(b'K') => "South Korea",
        Some(b'L') => "China",
        Some(b'S') => "United Kingdom",
        Some(b'V') => "France/Spain",
        Some(b'W') => "Germany",
        Some(b'Y') => "Sweden/Finland",
        Some(b'Z') => "Italy",
        _ => "Unknown",
    };

    ("Unknown".to_string(), country.to_string())
}

static WMI_MAP: LazyLock<HashMap<&'static str, (&'static str, &'static str)>> = LazyLock::new(|| {
    let mut m = HashMap::new();

    // ── Germany (W) ─────────────────────────────────────────────────────
    m.insert("W0L", ("Opel", "Germany"));
    m.insert("W0V", ("Opel", "Germany"));
    m.insert("W0S", ("Opel Special Vehicles", "Germany"));
    m.insert("WBA", ("BMW", "Germany"));
    m.insert("WBS", ("BMW M", "Germany"));
    m.insert("WBY", ("BMW i", "Germany"));
    m.insert("WDB", ("Mercedes-Benz", "Germany"));
    m.insert("WDC", ("Mercedes-Benz (SUV/Crossover)", "Germany"));
    m.insert("WDD", ("Mercedes-Benz (Sedan)", "Germany"));
    m.insert("WDF", ("Mercedes-Benz (Van)", "Germany"));
    m.insert("WMW", ("MINI", "Germany"));
    m.insert("WUA", ("Audi Sport", "Germany"));
    m.insert("WVW", ("Volkswagen", "Germany"));
    m.insert("WVG", ("Volkswagen (SUV)", "Germany"));
    m.insert("WV1", ("Volkswagen Commercial", "Germany"));
    m.insert("WV2", ("Volkswagen Commercial", "Germany"));
    m.insert("WAU", ("Audi", "Germany"));
    m.insert("WA1", ("Audi (SUV)", "Germany"));
    m.insert("WP0", ("Porsche", "Germany"));
    m.insert("WP1", ("Porsche (SUV)", "Germany"));
    m.insert("WF0", ("Ford Germany", "Germany"));
    m.insert("WJR", ("Irmscher (Opel tuner)", "Germany"));
    m.insert("W1K", ("Mercedes-Benz", "Germany"));
    m.insert("W1N", ("Mercedes-Benz", "Germany"));
    m.insert("W1V", ("Mercedes-Benz", "Germany"));

    // ── Japan (J) ───────────────────────────────────────────────────────
    m.insert("JHM", ("Honda", "Japan"));
    m.insert("JHL", ("Honda (Light truck)", "Japan"));
    m.insert("JHG", ("Honda", "Japan"));
    m.insert("JN1", ("Nissan", "Japan"));
    m.insert("JN3", ("Nissan (SUV)", "Japan"));
    m.insert("JN8", ("Nissan (SUV)", "Japan"));
    m.insert("JTE", ("Toyota", "Japan"));
    m.insert("JTD", ("Toyota", "Japan"));
    m.insert("JTH", ("Lexus", "Japan"));
    m.insert("JTJ", ("Lexus (SUV)", "Japan"));
    m.insert("JTK", ("Toyota (Scion)", "Japan"));
    m.insert("JTL", ("Toyota", "Japan"));
    m.insert("JTM", ("Toyota (Truck/SUV)", "Japan"));
    m.insert("JMZ", ("Mazda", "Japan"));
    m.insert("JM1", ("Mazda", "Japan"));
    m.insert("JM3", ("Mazda (SUV)", "Japan"));
    m.insert("JF1", ("Subaru", "Japan"));
    m.insert("JF2", ("Subaru (SUV)", "Japan"));
    m.insert("JS1", ("Suzuki", "Japan"));
    m.insert("JS2", ("Suzuki", "Japan"));
    m.insert("JA3", ("Mitsubishi", "Japan"));
    m.insert("JA4", ("Mitsubishi (SUV)", "Japan"));
    m.insert("JMB", ("Mitsubishi", "Japan"));
    m.insert("JMY", ("Mitsubishi", "Japan"));
    m.insert("JDA", ("Daihatsu", "Japan"));

    // ── South Korea (K) ─────────────────────────────────────────────────
    m.insert("KMH", ("Hyundai", "South Korea"));
    m.insert("KNA", ("Kia", "South Korea"));
    m.insert("KNB", ("Kia", "South Korea"));
    m.insert("KNC", ("Kia", "South Korea"));
    m.insert("KND", ("Kia (SUV)", "South Korea"));
    m.insert("KNM", ("Renault Samsung", "South Korea"));
    m.insert("KPA", ("SsangYong", "South Korea"));
    m.insert("KPT", ("SsangYong", "South Korea"));
    m.insert("5NP", ("Hyundai (US)", "South Korea"));

    // ── United Kingdom (S) ──────────────────────────────────────────────
    m.insert("SAJ", ("Jaguar", "United Kingdom"));
    m.insert("SAL", ("Land Rover", "United Kingdom"));
    m.insert("SAR", ("Land Rover", "United Kingdom"));
    m.insert("SCA", ("Rolls-Royce", "United Kingdom"));
    m.insert("SCB", ("Bentley", "United Kingdom"));
    m.insert("SCC", ("Lotus", "United Kingdom"));
    m.insert("SCE", ("DeLorean", "United Kingdom"));
    m.insert("SCF", ("Aston Martin", "United Kingdom"));
    m.insert("SDB", ("Peugeot (UK)", "United Kingdom"));
    m.insert("SFD", ("Alexander Dennis", "United Kingdom"));
    m.insert("SHH", ("Honda (UK)", "United Kingdom"));
    m.insert("SMT", ("Triumph", "United Kingdom"));

    // ── France (VF) ─────────────────────────────────────────────────────
    m.insert("VF1", ("Renault", "France"));
    m.insert("VF2", ("Renault", "France"));
    m.insert("VF3", ("Peugeot", "France"));
    m.insert("VF6", ("Renault (Truck)", "France"));
    m.insert("VF7", ("Citro\u{00eb}n", "France"));
    m.insert("VF8", ("Citro\u{00eb}n", "France"));
    m.insert("VF9", ("Bugatti", "France"));
    m.insert("VNK", ("Toyota (France)", "France"));
    m.insert("VR1", ("DS Automobiles", "France"));

    // ── Italy (Z) ───────────────────────────────────────────────────────
    m.insert("ZAM", ("Maserati", "Italy"));
    m.insert("ZAP", ("Piaggio", "Italy"));
    m.insert("ZAR", ("Alfa Romeo", "Italy"));
    m.insert("ZCG", ("Cagiva", "Italy"));
    m.insert("ZDM", ("Ducati", "Italy"));
    m.insert("ZFA", ("Fiat", "Italy"));
    m.insert("ZFF", ("Ferrari", "Italy"));
    m.insert("ZHW", ("Lamborghini", "Italy"));
    m.insert("ZLA", ("Lancia", "Italy"));

    // ── Sweden (Y) ──────────────────────────────────────────────────────
    m.insert("YV1", ("Volvo", "Sweden"));
    m.insert("YV4", ("Volvo (SUV)", "Sweden"));
    m.insert("YS2", ("Scania", "Sweden"));
    m.insert("YS3", ("Saab", "Sweden"));
    m.insert("YK1", ("Saab", "Sweden"));
    m.insert("YTN", ("Saab (NEVS)", "Sweden"));
    m.insert("YSM", ("Polestar", "Sweden"));

    // ── Spain (VS) ──────────────────────────────────────────────────────
    m.insert("VSS", ("SEAT", "Spain"));
    m.insert("VS6", ("Ford Spain", "Spain"));
    m.insert("VS7", ("Citro\u{00eb}n Spain", "Spain"));
    m.insert("VS9", ("Carrocerias Ayats", "Spain"));
    m.insert("VWV", ("Volkswagen Spain", "Spain"));

    // ── Czech Republic ──────────────────────────────────────────────────
    m.insert("TMB", ("Skoda", "Czech Republic"));
    m.insert("TMP", ("Skoda", "Czech Republic"));
    m.insert("TMT", ("Tatra", "Czech Republic"));

    // ── USA (1, 4, 5) ──────────────────────────────────────────────────
    m.insert("1C3", ("Chrysler", "United States"));
    m.insert("1C4", ("Chrysler (SUV)", "United States"));
    m.insert("1C6", ("Chrysler (Truck)", "United States"));
    m.insert("1FA", ("Ford", "United States"));
    m.insert("1FB", ("Ford (Bus)", "United States"));
    m.insert("1FC", ("Ford (Stripped Chassis)", "United States"));
    m.insert("1FD", ("Ford (Truck)", "United States"));
    m.insert("1FM", ("Ford (SUV)", "United States"));
    m.insert("1FT", ("Ford (Truck)", "United States"));
    m.insert("1FU", ("Freightliner", "United States"));
    m.insert("1FV", ("Freightliner", "United States"));
    m.insert("1G1", ("Chevrolet", "United States"));
    m.insert("1G2", ("Pontiac", "United States"));
    m.insert("1G3", ("Oldsmobile", "United States"));
    m.insert("1G4", ("Buick", "United States"));
    m.insert("1G6", ("Cadillac", "United States"));
    m.insert("1G8", ("Saturn", "United States"));
    m.insert("1GC", ("Chevrolet (Truck)", "United States"));
    m.insert("1GM", ("Pontiac (Truck)", "United States"));
    m.insert("1GT", ("GMC (Truck)", "United States"));
    m.insert("1GY", ("Cadillac (SUV)", "United States"));
    m.insert("1HD", ("Harley-Davidson", "United States"));
    m.insert("1HG", ("Honda (US)", "United States"));
    m.insert("1J4", ("Jeep", "United States"));
    m.insert("1J8", ("Jeep", "United States"));
    m.insert("1LN", ("Lincoln", "United States"));
    m.insert("1ME", ("Mercury", "United States"));
    m.insert("1N4", ("Nissan (US)", "United States"));
    m.insert("1N6", ("Nissan (Truck US)", "United States"));
    m.insert("1NX", ("NUMMI (Toyota US)", "United States"));
    m.insert("1VW", ("Volkswagen (US)", "United States"));
    m.insert("1YV", ("Mazda (US)", "United States"));
    m.insert("1ZV", ("Ford (Mazda platform US)", "United States"));
    m.insert("2C3", ("Chrysler (Canada)", "Canada"));
    m.insert("2FA", ("Ford (Canada)", "Canada"));
    m.insert("2G1", ("Chevrolet (Canada)", "Canada"));
    m.insert("2HG", ("Honda (Canada)", "Canada"));
    m.insert("2HK", ("Honda (Canada)", "Canada"));
    m.insert("2HM", ("Hyundai (Canada)", "Canada"));
    m.insert("2T1", ("Toyota (Canada)", "Canada"));
    m.insert("3C4", ("Chrysler (Mexico)", "Mexico"));
    m.insert("3FA", ("Ford (Mexico)", "Mexico"));
    m.insert("3G1", ("Chevrolet (Mexico)", "Mexico"));
    m.insert("3HG", ("Honda (Mexico)", "Mexico"));
    m.insert("3N1", ("Nissan (Mexico)", "Mexico"));
    m.insert("3VW", ("Volkswagen (Mexico)", "Mexico"));
    m.insert("4F2", ("Mazda (US)", "United States"));
    m.insert("4S3", ("Subaru (US)", "United States"));
    m.insert("4S4", ("Subaru (US)", "United States"));
    m.insert("4T1", ("Toyota (US)", "United States"));
    m.insert("4T3", ("Toyota (US)", "United States"));
    m.insert("4T4", ("Toyota (US)", "United States"));
    m.insert("4US", ("BMW (US)", "United States"));
    m.insert("5FN", ("Honda (US)", "United States"));
    m.insert("5FP", ("Honda (US)", "United States"));
    m.insert("5J6", ("Honda (US)", "United States"));
    m.insert("5N1", ("Nissan (US)", "United States"));
    m.insert("5NP", ("Hyundai (US)", "United States"));
    m.insert("5TD", ("Toyota (US)", "United States"));
    m.insert("5UX", ("BMW (US)", "United States"));
    m.insert("5YJ", ("Tesla", "United States"));
    m.insert("7SA", ("Tesla", "United States"));

    // ── China (L) ───────────────────────────────────────────────────────
    m.insert("LFV", ("FAW-Volkswagen", "China"));
    m.insert("LSV", ("Shanghai Volkswagen", "China"));
    m.insert("LBV", ("BMW Brilliance", "China"));
    m.insert("LHG", ("GAC Honda", "China"));
    m.insert("LVS", ("Ford Changan", "China"));
    m.insert("LRW", ("Tesla (China)", "China"));

    // ── India ───────────────────────────────────────────────────────────
    m.insert("MA1", ("Mahindra", "India"));
    m.insert("MA3", ("Suzuki India", "India"));
    m.insert("MAJ", ("Ford India", "India"));
    m.insert("MAK", ("Honda India", "India"));
    m.insert("MAL", ("Hyundai India", "India"));
    m.insert("MAT", ("Tata", "India"));
    m.insert("MBH", ("Suzuki India", "India"));

    // ── Turkey ──────────────────────────────────────────────────────────
    m.insert("NMT", ("Toyota Turkey", "Turkey"));
    m.insert("NM0", ("Ford Turkey", "Turkey"));
    m.insert("NM4", ("Tofas/Fiat Turkey", "Turkey"));

    // ── Australia ───────────────────────────────────────────────────────
    m.insert("6G1", ("Holden", "Australia"));
    m.insert("6G2", ("Pontiac (Australia)", "Australia"));
    m.insert("6T1", ("Toyota (Australia)", "Australia"));

    m
});

static WMI_2CHAR: LazyLock<HashMap<&'static str, (&'static str, &'static str)>> = LazyLock::new(|| {
    let mut m = HashMap::new();
    m.insert("WA", ("Audi", "Germany"));
    m.insert("WB", ("BMW", "Germany"));
    m.insert("WD", ("Mercedes-Benz", "Germany"));
    m.insert("WF", ("Ford Germany", "Germany"));
    m.insert("WP", ("Porsche", "Germany"));
    m.insert("WV", ("Volkswagen", "Germany"));
    m.insert("W0", ("Opel", "Germany"));
    m.insert("W1", ("Mercedes-Benz", "Germany"));
    m.insert("JH", ("Honda", "Japan"));
    m.insert("JN", ("Nissan", "Japan"));
    m.insert("JT", ("Toyota", "Japan"));
    m.insert("JM", ("Mazda", "Japan"));
    m.insert("JF", ("Subaru", "Japan"));
    m.insert("JS", ("Suzuki", "Japan"));
    m.insert("JA", ("Mitsubishi", "Japan"));
    m.insert("KM", ("Hyundai", "South Korea"));
    m.insert("KN", ("Kia", "South Korea"));
    m.insert("SA", ("Jaguar/Land Rover", "United Kingdom"));
    m.insert("SC", ("Rolls-Royce/Bentley/Lotus", "United Kingdom"));
    m.insert("VF", ("Renault/Peugeot/Citro\u{00eb}n", "France"));
    m.insert("YV", ("Volvo", "Sweden"));
    m.insert("ZA", ("Alfa Romeo/Maserati", "Italy"));
    m.insert("ZF", ("Ferrari/Fiat", "Italy"));
    m.insert("TM", ("Skoda", "Czech Republic"));
    m
});

/// Short summary string like "Opel (Germany) 2018"
pub fn summary(vin: &str) -> String {
    let info = decode(vin);
    let mut parts = Vec::new();

    if info.make != "Unknown" {
        parts.push(info.make);
    }

    if !info.country.is_empty() && info.country != "Unknown" {
        parts.push(format!("({})", info.country));
    }

    if let Some(year) = info.year {
        parts.push(year);
    }

    if parts.is_empty() {
        format!("VIN: {}", info.vin)
    } else {
        parts.join(" ")
    }
}