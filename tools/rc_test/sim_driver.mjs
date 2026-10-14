// Headless driver for the dashboard's flight simulator (tools/hc_emulator/web/sim.js).
//
//   node sim_driver.mjs path/to/sim.js < scenario.json > samples.txt
//
// The scenario (JSON on stdin) is a list of pilot segments: power schedule,
// armed or not, on the ground or flying, optional baro dropouts. The physics,
// air mass, baro/IIR model, BMS power ripple, SOC drain and part temperatures
// are all sim.js itself (imported unchanged); this file only plays the pilot
// and the controller's view of the sensors, and computes the still-air and
// thermal ground truth. Everything is seeded, so a scenario replays
// identically.
//
// Output, one record per line:
//   S,t_ms,alt,power,armed,soc,motor,esc_mos,esc_cap,esc_mcu,battery,bms_mos,bms_bal,volts,seg,ground,true_alt
//       every UI tick (30 Hz, the dashboard's SIM_DT). alt is the controller's
//       baro altitude relative to where it was armed, "nan" during a dropout.
//   T,t_ms,seg,true_alt,level,best16,band_lo16,band_hi16,best_all,
//     warn_kw,crit_kw,warn_kw_m,crit_kw_m,warn_part,crit_part,horizon_s
//       ground truth once per simulated second (see truthAt / thermalTruth).
//   M,{json}
//       segment start/end times, lift-off and landing times.

import { readFileSync } from "node:fs";

const simPath = process.argv[2];
if (!simPath) {
  process.stderr.write("usage: node sim_driver.mjs path/to/sim.js < scenario.json\n");
  process.exit(2);
}
const spec = JSON.parse(readFileSync(0, "utf8"));
// sim.js is an ES module with no imports. Loading it from a data: URL works
// on any Node >= 18 without a package.json "type" next to it.
const simSrc = readFileSync(simPath, "utf8");
const sim = await import("data:text/javascript;base64," + Buffer.from(simSrc).toString("base64"));

const SIM_DT = 1 / 30;  // the dashboard's controller UI task rate (app.js)
const PART_ORDER = ["motor", "escMos", "escCap", "escMcu", "battery", "bmsMos", "bmsBalance"];

// --- configure the simulator -------------------------------------------------
const fs = new sim.FlightSim(spec.seed ?? 1);
fs.scenario = "manual";
Object.assign(fs.setup, spec.setup || {});
Object.assign(fs.env, spec.env || {});
// A cooler- or hotter-running setup: scale sim.js's fitted heating gains.
for (const [part, scale] of Object.entries(spec.thermalScale || {})) {
  sim.THERMAL_PARTS[part].k *= scale;
}
const thresholds = spec.thresholds;      // part -> [warnC, critC] (monitor_config.h)
const marginC = spec.marginC ?? 3.0;     // the firmware's thresholdMarginC
const horizonMaxS = spec.horizonMaxS ?? 300;
const socReservePct = spec.socReservePct ?? 15;
const packKwh = spec.packKwh ?? 4.8;     // sim.js drains SOC as a 4.8 kWh pack
const maxBarKw = spec.maxBarKw ?? 16;    // SP140 peak; the bar ends here

// --- still-air truth ---------------------------------------------------------
function levelPower(alt) {
  let lo = 0.2, hi = fs.setup.maxPowerKw;
  if (sim.stillAirClimb(fs.setup, hi, alt) < 0) return NaN;
  for (let i = 0; i < 50; i++) {
    const mid = (lo + hi) / 2;
    if (sim.stillAirClimb(fs.setup, mid, alt) < 0) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

// Best climb per Wh (max of w/P) and the band within `fraction` of it, on a
// 0.02 kW grid, both inside the bar (<= 16 kW) and up to the sim's motor max.
function curveTruth(alt, fraction = 0.95) {
  const L = levelPower(alt);
  const pts = [];
  for (let p = 0.5; p <= fs.setup.maxPowerKw + 1e-9; p += 0.02) {
    pts.push([p, sim.stillAirClimb(fs.setup, p, alt) / p]);
  }
  const bandOf = (list) => {
    let k = 0;
    for (let i = 1; i < list.length; i++) if (list[i][1] > list[k][1]) k = i;
    const best = list[k];
    let a = k, b = k;
    while (a > 0 && list[a - 1][1] >= fraction * best[1]) a--;
    while (b < list.length - 1 && list[b + 1][1] >= fraction * best[1]) b++;
    return { best: best[0], lo: list[a][0], hi: list[b][0] };
  };
  const capped = bandOf(pts.filter(([p]) => p <= maxBarKw + 1e-9));
  const all = bandOf(pts);
  return { level: L, best16: capped.best, lo16: capped.lo, hi16: capped.hi, bestAll: all.best };
}
const curveCache = new Map();
function curveAt(alt) {
  const key = Math.round(Math.max(0, alt) / 10);  // 10 m buckets
  if (!curveCache.has(key)) curveCache.set(key, curveTruth(key * 10));
  return curveCache.get(key);
}

// --- thermal truth -----------------------------------------------------------
// The power that, held from now, takes some part to its threshold within the
// climb horizon (5 min, or until the pack reaches the SOC reserve at that
// power), using sim.js's own part models: T(H) = Tss + (T0 - Tss) e^(-H/tau),
// Tss = air + baseOverAir + k P^2. The temperature moves monotonically toward
// Tss, so the hottest point of the hold is max(T0, T(H)): a part already over
// its threshold limits the bar at 0 kW, as in the firmware. maxBarKw means
// "no limit inside the bar".
function horizonFor(p, soc) {
  if (p <= 0.1) return horizonMaxS;
  const usable = (soc - socReservePct) / 100 * packKwh;
  if (usable <= 0) return 30;
  return Math.min(Math.max(usable * 3600 / p, 30), horizonMaxS);
}
function holdTemp(part, t0, p, h) {
  const m = sim.THERMAL_PARTS[part];
  const tss = fs.env.airTempC + m.baseOverAir + m.k * p * p;
  return Math.max(t0, tss + (t0 - tss) * Math.exp(-h / m.tau));
}
function hottest(temps, p, soc, which, margin) {
  const h = horizonFor(p, soc);
  let worst = -Infinity, who = -1;
  PART_ORDER.forEach((part, i) => {
    const excess = holdTemp(part, temps[part], p, h) - (thresholds[part][which] - margin);
    if (excess > worst) { worst = excess; who = i; }
  });
  return { excess: worst, part: who, horizon: h };
}
function thermalLimit(temps, soc, which, margin) {
  const step = 0.05;
  if (hottest(temps, 0, soc, which, margin).excess >= 0) {
    const z = hottest(temps, 0, soc, which, margin);
    return { kw: 0, part: z.part, horizon: z.horizon };
  }
  for (let p = step; p <= maxBarKw + 1e-9; p += step) {
    if (hottest(temps, p, soc, which, margin).excess >= 0) {
      let lo = p - step, hi = p;
      for (let i = 0; i < 20; i++) {
        const mid = (lo + hi) / 2;
        if (hottest(temps, mid, soc, which, margin).excess >= 0) hi = mid; else lo = mid;
      }
      const at = hottest(temps, hi, soc, which, margin);
      return { kw: lo, part: at.part, horizon: at.horizon };
    }
  }
  return { kw: maxBarKw, part: -1, horizon: horizonFor(maxBarKw, soc) };
}

// --- pilot -------------------------------------------------------------------
const SWEEP_STEPS = [4, 6, 8, 10, 12, 14, 16, 18, 20];  // sim.js sweepPower()
function sweepPower(tau) {
  const hold = 40;
  const cycle = SWEEP_STEPS.length * hold + 30;
  const i = Math.floor((tau % cycle) / hold);
  return i < SWEEP_STEPS.length ? SWEEP_STEPS[i] : 0;  // glide at the end
}
function cyclePower(list, tau) {
  const total = list.reduce((a, [d]) => a + d, 0);
  let k = tau % total;
  for (const [d, p] of list) {
    if (k < d) return p;
    k -= d;
  }
  return list[list.length - 1][1];
}

const segments = spec.segments.map((s) => ({ armed: true, ground: false, gaps: [], ...s }));
const meta = { segments: [], liftoffMs: [], landingMs: [], armMs: [], disarmMs: [] };

// Random short baro dropouts: their own RNG, so adding gaps never changes the
// flight itself (same seed -> same trajectory with or without gaps).
const gapRand = sim.mulberry32(spec.gapSeed ?? 12345);
const rg = spec.randomGaps || null;
let randomGapUntil = -1;

const out = [];
const f = (v, n) => (Number.isFinite(v) ? v.toFixed(n) : "nan");
let tMs = 1000;
let msAcc = 0;
let step = 0;
let groundRef = null;
let wasArmed = false;
let airborne = false;
let landedFor = 0;

for (let si = 0; si < segments.length; si++) {
  const seg = segments[si];
  fs.armed = seg.armed;
  let tau = 0;
  let levelKw = NaN;
  const segStart = tMs;
  const dur = seg.until ? (seg.maxDur ?? 1800) : seg.dur;
  if (typeof seg.power === "object" && seg.power !== null && "level" in seg.power) {
    levelKw = levelPower(fs.alt) + seg.power.level;  // the pilot finds level and holds it
  }
  while (tau < dur - 1e-9) {
    tau += SIM_DT;
    let cmd;
    if (typeof seg.power === "number") cmd = seg.power;
    else if (seg.power === "sweep") cmd = sweepPower(tau);
    else if (seg.power && seg.power.cycle) cmd = cyclePower(seg.power.cycle, tau);
    else cmd = levelKw;
    fs.throttleKw = cmd;

    const r = fs.step(SIM_DT);
    if (seg.ground) {  // standing / running on the ground: no climb yet
      fs.alt = 0;
      fs.wMotor = 0;
      fs.climb = 0;
    }
    msAcc += SIM_DT * 1000;
    const dt = Math.round(msAcc);
    msAcc -= dt;
    tMs += dt;

    // Arming zeroes the controller's altitude (setGroundAltitude at arm).
    if (seg.armed && !wasArmed) {
      groundRef = r.alt;
      meta.armMs.push(tMs);
    }
    if (!seg.armed && wasArmed) meta.disarmMs.push(tMs);
    wasArmed = seg.armed;
    if (groundRef === null) groundRef = r.alt;
    const trueAlt = seg.ground ? 0 : fs.alt;

    if (!airborne && !seg.ground && trueAlt > 1.0) {
      airborne = true;
      meta.liftoffMs.push(tMs);
    }
    if (airborne && trueAlt <= 0.01) {
      landedFor += SIM_DT;
      if (landedFor >= 2) {
        airborne = false;
        meta.landingMs.push(tMs);
      }
    } else {
      landedFor = 0;
    }

    // Baro dropouts: segment-relative fixed gaps plus random short ones.
    let gap = seg.gaps.some(([a, d]) => tau >= a && tau < a + d);
    if (rg) {
      const tS = tMs / 1000;
      if (tS >= randomGapUntil && gapRand() < SIM_DT / rg.meanEveryS) {
        randomGapUntil = tS + rg.minS + (rg.maxS - rg.minS) * gapRand();
      }
      if (tS < randomGapUntil) gap = true;
    }
    const alt = gap ? NaN : r.alt - groundRef;
    const tp = r.temps;
    out.push(`S,${tMs},${f(alt, 3)},${f(r.power, 3)},${seg.armed ? 1 : 0},${f(r.soc, 3)},` +
      PART_ORDER.map((p) => f(tp[p], 2)).join(",") +
      `,${f(r.volts, 2)},${si},${seg.ground ? 1 : 0},${f(trueAlt, 2)}`);

    if (step % 30 === 0) {
      const c = curveAt(trueAlt);
      const w = thermalLimit(tp, r.soc, 0, 0);
      const k = thermalLimit(tp, r.soc, 1, 0);
      const wm = thermalLimit(tp, r.soc, 0, marginC);
      const km = thermalLimit(tp, r.soc, 1, marginC);
      out.push(`T,${tMs},${si},${f(trueAlt, 2)},${f(c.level, 4)},${f(c.best16, 3)},` +
        `${f(c.lo16, 3)},${f(c.hi16, 3)},${f(c.bestAll, 3)},${f(w.kw, 3)},${f(k.kw, 3)},` +
        `${f(wm.kw, 3)},${f(km.kw, 3)},${w.part},${k.part},${f(w.horizon, 1)}`);
    }
    step++;
    if (seg.until === "landed" && meta.landingMs.length && meta.landingMs.at(-1) > segStart) break;
  }
  meta.segments.push({ name: seg.name, startMs: segStart, endMs: tMs, ground: seg.ground,
    armed: seg.armed, levelKw: Number.isFinite(levelKw) ? levelKw : null });
}
meta.setup = fs.setup;
meta.env = fs.env;
meta.thermalParts = sim.THERMAL_PARTS;
out.push("M," + JSON.stringify(meta));
process.stdout.write(out.join("\n") + "\n");
