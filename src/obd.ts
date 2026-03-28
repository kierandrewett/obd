import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";
import * as readline from "readline";

const PORT = process.env.OBD_PORT || "/dev/ttyUSB0";
const BAUD = 38400;

// ── OBD-II PID definitions ──────────────────────────────────────────────────
interface Pid {
  name: string;
  cmd: string;
  parse: (bytes: number[]) => string;
}

const PIDS: Pid[] = [
  {
    name: "Engine RPM",
    cmd: "010C",
    parse: ([a, b]) => `${((a * 256 + b) / 4).toFixed(0)} RPM`,
  },
  {
    name: "Vehicle Speed",
    cmd: "010D",
    parse: ([a]) => `${a} km/h`,
  },
  {
    name: "Engine Coolant Temp",
    cmd: "0105",
    parse: ([a]) => `${a - 40} °C`,
  },
  {
    name: "Throttle Position",
    cmd: "0111",
    parse: ([a]) => `${((a / 255) * 100).toFixed(1)} %`,
  },
  {
    name: "Engine Load",
    cmd: "0104",
    parse: ([a]) => `${((a / 255) * 100).toFixed(1)} %`,
  },
  {
    name: "Intake Air Temp",
    cmd: "010F",
    parse: ([a]) => `${a - 40} °C`,
  },
  {
    name: "MAF Air Flow Rate",
    cmd: "0110",
    parse: ([a, b]) => `${((a * 256 + b) / 100).toFixed(2)} g/s`,
  },
  {
    name: "Fuel Tank Level",
    cmd: "012F",
    parse: ([a]) => `${((a / 255) * 100).toFixed(1)} %`,
  },
  {
    name: "Short-term Fuel Trim (Bank 1)",
    cmd: "0106",
    parse: ([a]) => `${(((a - 128) * 100) / 128).toFixed(1)} %`,
  },
  {
    name: "Long-term Fuel Trim (Bank 1)",
    cmd: "0107",
    parse: ([a]) => `${(((a - 128) * 100) / 128).toFixed(1)} %`,
  },
  {
    name: "Control Module Voltage",
    cmd: "0142",
    parse: ([a, b]) => `${((a * 256 + b) / 1000).toFixed(2)} V`,
  },
  {
    name: "Engine Oil Temp",
    cmd: "015C",
    parse: ([a]) => `${a - 40} °C`,
  },
];

// ── ELM327 communication ─────────────────────────────────────────────────────
class ELM327 {
  private port: SerialPort;
  private parser: ReadlineParser;
  private responseBuffer: string[] = [];
  private responseResolve: ((lines: string[]) => void) | null = null;
  private promptPattern = />\s*$/;

  constructor() {
    this.port = new SerialPort({ path: PORT, baudRate: BAUD, autoOpen: false });
    this.parser = this.port.pipe(new ReadlineParser({ delimiter: "\r" }));
  }

  open(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.port.open((err) => {
        if (err) return reject(err);
        this.parser.on("data", (line: string) => this.onData(line));
        resolve();
      });
    });
  }

  private onData(raw: string) {
    const line = raw.replace(/[\r\n]/g, "").trim();
    if (!line) return;

    if (this.promptPattern.test(line) || line === ">") {
      if (this.responseResolve) {
        const resolve = this.responseResolve;
        this.responseResolve = null;
        const buf = [...this.responseBuffer];
        this.responseBuffer = [];
        resolve(buf);
      }
    } else {
      this.responseBuffer.push(line);
    }
  }

  send(cmd: string, timeoutMs = 3000): Promise<string[]> {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.responseResolve = null;
        this.responseBuffer = [];
        reject(new Error(`Timeout waiting for response to: ${cmd}`));
      }, timeoutMs);

      this.responseResolve = (lines) => {
        clearTimeout(timer);
        resolve(lines);
      };

      this.port.write(cmd + "\r");
    });
  }

  async init() {
    await this.send("ATZ", 5000); // reset
    await delay(500);
    await this.send("ATE0"); // echo off
    await this.send("ATL0"); // linefeeds off
    await this.send("ATS0"); // spaces off
    await this.send("ATSP0"); // auto protocol
    await this.send("ATH0"); // headers off
  }

  close(): Promise<void> {
    return new Promise((resolve) => this.port.close(() => resolve()));
  }
}

function delay(ms: number) {
  return new Promise((r) => setTimeout(r, ms));
}

// ── Response parsing ─────────────────────────────────────────────────────────
function parseObdResponse(pid: Pid, lines: string[]): string | null {
  for (const line of lines) {
    // strip any echoed command, look for data line
    if (
      line.startsWith("NO DATA") ||
      line.startsWith("ERROR") ||
      line.startsWith("?") ||
      line.startsWith("UNABLE")
    ) {
      return null;
    }
    // Expected format without headers/spaces: e.g. "410C1AF8"
    const modeResponse = pid.cmd.substring(0, 2).replace("0", "4"); // 01 -> 41
    const pidHex = pid.cmd.substring(2, 4);
    const prefix = modeResponse + pidHex;

    if (line.toUpperCase().includes(prefix)) {
      const dataStr = line
        .toUpperCase()
        .substring(line.toUpperCase().indexOf(prefix) + prefix.length);
      const bytes: number[] = [];
      for (let i = 0; i < dataStr.length; i += 2) {
        const byte = parseInt(dataStr.substring(i, i + 2), 16);
        if (!isNaN(byte)) bytes.push(byte);
      }
      if (bytes.length > 0) {
        try {
          return pid.parse(bytes);
        } catch {
          return null;
        }
      }
    }
  }
  return null;
}

// ── DTC decoding ─────────────────────────────────────────────────────────────
function decodeDtc(byte1: number, byte2: number): string {
  const prefix = ["P", "C", "B", "U"][(byte1 >> 6) & 0x03];
  const d1 = (byte1 >> 4) & 0x03;
  const d2 = byte1 & 0x0f;
  const d3 = (byte2 >> 4) & 0x0f;
  const d4 = byte2 & 0x0f;
  return `${prefix}${d1}${d2.toString(16).toUpperCase()}${d3.toString(16).toUpperCase()}${d4.toString(16).toUpperCase()}`;
}

function parseDtcResponse(lines: string[]): string[] {
  const codes: string[] = [];
  for (const line of lines) {
    if (
      line === "NODATA" ||
      line === "OK" ||
      line.startsWith("43") === false ||
      line.length < 4
    )
      continue;
    // format: 43 NN [b1 b2] [b1 b2] ...
    const cleaned = line.replace(/\s/g, "");
    if (!cleaned.startsWith("43")) continue;
    const numCodes = parseInt(cleaned.substring(2, 4), 16);
    for (let i = 0; i < numCodes; i++) {
      const offset = 4 + i * 4;
      if (offset + 4 > cleaned.length) break;
      const b1 = parseInt(cleaned.substring(offset, offset + 2), 16);
      const b2 = parseInt(cleaned.substring(offset + 2, offset + 4), 16);
      if (b1 === 0 && b2 === 0) continue;
      codes.push(decodeDtc(b1, b2));
    }
  }
  return codes;
}

// ── Display helpers ───────────────────────────────────────────────────────────
function clearScreen() {
  process.stdout.write("\x1b[2J\x1b[H");
}

function bold(s: string) {
  return `\x1b[1m${s}\x1b[0m`;
}
function green(s: string) {
  return `\x1b[32m${s}\x1b[0m`;
}
function red(s: string) {
  return `\x1b[31m${s}\x1b[0m`;
}
function yellow(s: string) {
  return `\x1b[33m${s}\x1b[0m`;
}
function cyan(s: string) {
  return `\x1b[36m${s}\x1b[0m`;
}

// ── Main application ──────────────────────────────────────────────────────────
async function readLivePids(elm: ELM327) {
  clearScreen();
  console.log(bold(cyan("── Live Sensor Data ─────────────────────────────────")));
  console.log(yellow("  Press Ctrl+C to return to menu\n"));

  process.on("SIGINT", () => {
    console.log("\nReturning to menu...");
    process.exit(0);
  });

  while (true) {
    const results: { name: string; value: string }[] = [];

    for (const pid of PIDS) {
      try {
        const lines = await elm.send(pid.cmd, 2000);
        const value = parseObdResponse(pid, lines);
        if (value !== null) {
          results.push({ name: pid.name, value });
        }
      } catch {
        // skip unsupported PIDs
      }
    }

    process.stdout.write("\x1b[H\x1b[2J");
    console.log(bold(cyan("── Live Sensor Data ─────────────────────────────────")));
    console.log(yellow("  Press Ctrl+C to exit\n"));

    for (const { name, value } of results) {
      const padded = name.padEnd(32);
      console.log(`  ${green(padded)} ${bold(value)}`);
    }

    if (results.length === 0) {
      console.log(red("  No data received. Is the engine running?"));
    }

    await delay(1000);
  }
}

async function readDtcs(elm: ELM327) {
  console.log("\n" + cyan("Reading stored trouble codes (Mode 03)..."));
  const lines = await elm.send("03", 5000);
  const codes = parseDtcResponse(lines);

  if (codes.length === 0) {
    console.log(green("  No trouble codes found. ✓"));
  } else {
    console.log(red(`  Found ${codes.length} trouble code(s):`));
    for (const code of codes) {
      console.log(red(`    • ${code}`));
    }
    console.log(
      yellow(
        "\n  Tip: Search each code at https://www.obd-codes.com/ for descriptions."
      )
    );
  }
}

async function readPendingDtcs(elm: ELM327) {
  console.log("\n" + cyan("Reading pending trouble codes (Mode 07)..."));
  const lines = await elm.send("07", 5000);
  // Mode 07 responds with 47 prefix
  const pending = lines
    .filter((l) => l.startsWith("47"))
    .flatMap((line) => {
      const cleaned = line.replace(/\s/g, "");
      const codes: string[] = [];
      for (let i = 2; i + 4 <= cleaned.length; i += 4) {
        const b1 = parseInt(cleaned.substring(i, i + 2), 16);
        const b2 = parseInt(cleaned.substring(i + 2, i + 4), 16);
        if (b1 !== 0 || b2 !== 0) codes.push(decodeDtc(b1, b2));
      }
      return codes;
    });

  if (pending.length === 0) {
    console.log(green("  No pending codes found. ✓"));
  } else {
    console.log(yellow(`  Found ${pending.length} pending code(s):`));
    for (const code of pending) {
      console.log(yellow(`    • ${code}`));
    }
  }
}

async function clearDtcs(elm: ELM327, rl: readline.Interface) {
  return new Promise<void>((resolve) => {
    rl.question(
      red(
        "\n  ⚠  This will clear all stored DTCs and reset the MIL (Check Engine Light).\n  Are you sure? (yes/no): "
      ),
      async (answer) => {
        if (answer.trim().toLowerCase() === "yes") {
          await elm.send("04", 5000);
          console.log(green("  DTCs cleared."));
        } else {
          console.log("  Cancelled.");
        }
        resolve();
      }
    );
  });
}

async function readVin(elm: ELM327) {
  console.log("\n" + cyan("Requesting VIN (Mode 09, PID 02)..."));
  const lines = await elm.send("0902", 5000);
  const data = lines
    .filter((l) => l.match(/^49/i))
    .join("")
    .replace(/^4902\d{2}/i, "")
    .replace(/\s/g, "");

  let vin = "";
  for (let i = 0; i < data.length; i += 2) {
    const code = parseInt(data.substring(i, i + 2), 16);
    if (code >= 32 && code < 127) vin += String.fromCharCode(code);
  }

  if (vin.trim()) {
    console.log(green(`  VIN: ${bold(vin.trim())}`));
  } else {
    console.log(yellow("  VIN not available from this vehicle."));
  }
}

async function readSupportedPids(elm: ELM327) {
  console.log("\n" + cyan("Checking supported PIDs (Mode 01, PID 00/20/40/60)..."));
  const pidChecks = ["0100", "0120", "0140", "0160"];

  for (const cmd of pidChecks) {
    try {
      const lines = await elm.send(cmd, 3000);
      for (const line of lines) {
        if (line.startsWith("41")) {
          const hex = line.substring(4).replace(/\s/g, "");
          const bits = parseInt(hex, 16).toString(2).padStart(32, "0");
          const base = parseInt(cmd.substring(2), 16);
          const supported: string[] = [];
          for (let i = 0; i < 32; i++) {
            if (bits[i] === "1") {
              supported.push(
                `PID ${(base + i + 1).toString(16).toUpperCase().padStart(2, "0")}`
              );
            }
          }
          console.log(
            green(`  PIDs ${base + 1}-${base + 32}: `) + supported.join(", ")
          );
        }
      }
    } catch {
      // not all ranges supported
    }
  }
}

async function showMenu(elm: ELM327) {
  const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout,
  });

  const ask = (q: string) =>
    new Promise<string>((resolve) => rl.question(q, resolve));

  while (true) {
    console.log("\n" + bold(cyan("══════════════════════════════════════════════")));
    console.log(bold(cyan("  OBD-II Diagnostic Tool")));
    console.log(bold(cyan("══════════════════════════════════════════════")));
    console.log("  1. Live sensor data (real-time)");
    console.log("  2. Read stored trouble codes (DTCs)");
    console.log("  3. Read pending trouble codes");
    console.log("  4. Read VIN");
    console.log("  5. Check supported PIDs");
    console.log(red("  6. Clear DTCs / Reset Check Engine Light"));
    console.log("  0. Exit\n");

    const choice = await ask("  Select option: ");

    switch (choice.trim()) {
      case "1":
        await readLivePids(elm);
        break;
      case "2":
        await readDtcs(elm);
        break;
      case "3":
        await readPendingDtcs(elm);
        break;
      case "4":
        await readVin(elm);
        break;
      case "5":
        await readSupportedPids(elm);
        break;
      case "6":
        await clearDtcs(elm, rl);
        break;
      case "0":
        rl.close();
        return;
      default:
        console.log(yellow("  Unknown option."));
    }
  }
}

// ── Entry point ───────────────────────────────────────────────────────────────
async function main() {
  console.log(bold(cyan(`\nConnecting to OBD adapter on ${PORT}...`)));

  const elm = new ELM327();

  try {
    await elm.open();
    console.log(green("  Port opened."));
    console.log(cyan("  Initializing ELM327..."));
    await elm.init();
    console.log(green("  ELM327 ready.\n"));
  } catch (err: any) {
    console.error(red(`\nFailed to connect: ${err.message}`));
    console.error(yellow(`  Check that ${PORT} is correct and you have read/write access.`));
    console.error(yellow(`  You may need: sudo chmod a+rw ${PORT}  or  sudo usermod -aG dialout $USER`));
    process.exit(1);
  }

  try {
    await showMenu(elm);
  } finally {
    await elm.close();
    console.log("\nDisconnected.");
  }
}

main().catch((err) => {
  console.error(red("Fatal error: " + err.message));
  process.exit(1);
});
