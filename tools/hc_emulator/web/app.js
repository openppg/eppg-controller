// Dashboard for the hand-controller emulator. Feeds the firmware (running in
// hc_emulator via server.py) with simulated or recorded telemetry, shows the
// real screen, and charts the firmware's climb-efficiency estimate against the
// simulator's ground truth.

import { FlightSim, SCENARIOS, stillAirClimb, trueCurve } from "./sim.js";

const $ = (id) => document.getElementById(id);
const SIM_DT = 1 / 30;     // controller UI task rate (s)
const HISTORY_S = 1800;    // chart history kept (s)
const RECORD_EVERY = 0.25; // truth sampling for charts (s)

const COLORS = {
  accent: "#45b5ff",
  est: "#ff9f43",
  truth: "#e7e9ee",
  good: "#3ddc97",
  violet: "#b48cff",
  pink: "#ff50be",
  muted: "#8a93a5",
  grid: "#232833",
  air: "#6fb7ff",
};

const state = {
  source: "sim",
  running: true,
  speed: 5,
  span: 300,
  bar: 1,
  theme: 1,
  metric: 1,
  perf: 1,
  bandFraction: 0.95,
  sim: new FlightSim(1),
  simAcc: 0,
  msAcc: 0,
  lastRecord: -1e9,
  truth: null,
  truthAt: -1e9,
  replay: null,
  est: null,
  hist: [],
  estHist: [],
  firstLevelT: NaN,
  firstBandT: NaN,
  // A page reload restarts the flight, so the firmware estimator starts over too
  pending: ["reset", "init theme=1 metric=1 perf=1", "bar v=1"],
};

// ---------------------------------------------------------------- emulator IO
async function emu(lines) {
  const res = await fetch("/api/cmd", { method: "POST", body: lines.join("\n") });
  const body = await res.json();
  if (!res.ok) throw new Error(body.error || res.statusText);
  return body;
}

const screenCtx = $("screen").getContext("2d");
const screenImg = screenCtx.createImageData(160, 128);
function drawFrame(b64) {
  const bin = atob(b64);
  const d = screenImg.data;
  for (let i = 0, n = bin.length / 2; i < n; i++) {
    const v = bin.charCodeAt(2 * i) | (bin.charCodeAt(2 * i + 1) << 8);
    d[4 * i] = (((v >> 11) & 31) * 527 + 23) >> 6;
    d[4 * i + 1] = (((v >> 5) & 63) * 259 + 33) >> 6;
    d[4 * i + 2] = ((v & 31) * 527 + 23) >> 6;
    d[4 * i + 3] = 255;
  }
  screenCtx.putImageData(screenImg, 0, 0);
}

function sampleLine(dtMs, s) {
  // strtod() in the emulator reads "nan": missing temps show as "-" on screen
  const f = (v, n) => (Number.isFinite(v) ? v.toFixed(n) : "nan");
  return `s dt=${dtMs} alt=${f(s.alt, 3)} p=${f(s.power, 3)} soc=${f(s.soc, 1)} ` +
    `v=${f(s.volts, 1)} armed=${s.armed ? 1 : 0} cruise=${s.cruise ? 1 : 0} ` +
    `bt=${f(s.bt, 1)} et=${f(s.et, 1)} mt=${f(s.mt, 1)} ` +
    (Number.isFinite(s.ec) ? `ec=${f(s.ec, 1)} ` : "") +
    (Number.isFinite(s.em) ? `em=${f(s.em, 1)} ` : "") +
    (Number.isFinite(s.bm) ? `bm=${f(s.bm, 1)} ` : "") +
    (Number.isFinite(s.bb) ? `bb=${f(s.bb, 1)} ` : "") +
    `bms=${s.bms === false ? 0 : 1} esc=${s.esc === false ? 0 : 1}`;
}

// ------------------------------------------------------------------ simulator
function simLines(simSeconds) {
  const sim = state.sim;
  const lines = [];
  state.simAcc += simSeconds;
  while (state.simAcc >= SIM_DT) {
    state.simAcc -= SIM_DT;
    const r = sim.step(SIM_DT);
    state.msAcc += SIM_DT * 1000;
    const dt = Math.round(state.msAcc);
    state.msAcc -= dt;
    lines.push(sampleLine(dt, {
      ...r,
      bt: r.temps.battery,
      et: r.temps.escMos,
      ec: r.temps.escCap,
      em: r.temps.escMcu,
      mt: r.temps.motor,
      bm: r.temps.bmsMos,
      bb: r.temps.bmsBalance,
    }));
    if (r.t - state.lastRecord >= RECORD_EVERY) {
      state.lastRecord = r.t;
      const still = r.power > 0.5 ? stillAirClimb(sim.setup, r.power, r.trueAlt) : NaN;
      state.hist.push({
        t: r.t,
        power: r.power,
        cmd: r.command,
        climbTrue: r.trueClimb,
        climbStill: r.stillClimb,
        yieldTrue: r.power >= 1.5 ? (still / r.power) * 3.6 : NaN,
        airW: r.airW,
      });
    }
  }
  return lines;
}

// ----------------------------------------------------------------- log replay
function parseLog(text) {
  const lines = text.split(/\r?\n/);
  const h = lines.findIndex((l) => l.startsWith("Timestamp,"));
  if (h < 0) throw new Error('no "Timestamp," header row - is this an OpenPPG app CSV?');
  const meta = {};
  for (let i = 0; i < h; i++) {
    const c = lines[i].indexOf(",");
    if (c > 0) meta[lines[i].slice(0, c)] = lines[i].slice(c + 1);
  }
  const cols = lines[h].split(",");
  const ix = (n) => cols.indexOf(n);
  const I = {
    t: ix("Timestamp"),
    alt: ix("Controller_Altitude(m)"),
    vario: ix("Controller_Vario(m/s)"),
    st: ix("Controller_DeviceState"),
    bmsSt: ix("BMS_Status"),
    bp: ix("BMS_Power(kW)"),
    soc: ix("BMS_SOC(%)"),
    pv: ix("BMS_PackVoltage(V)"),
    escSt: ix("ESC_Status"),
    ev: ix("ESC_Voltage(V)"),
    ei: ix("ESC_DCCurrent(A)"),
    mos: ix("ESC_MOSTemp(C)"),
    cap: ix("ESC_CAPTemp(C)"),
    mot: ix("ESC_MotorTemp(C)"),
    mcu: ix("ESC_MCUTemp(C)"),
    bmsMos: ix("BMS_MOSFETTemp(C)"),
    bmsBal: ix("BMS_BalanceTemp(C)"),
    cell: [1, 2, 3, 4].map((k) => ix(`BMS_CellTemp${k}(C)`)),
  };
  if (I.alt < 0) throw new Error("log has no Controller_Altitude(m) column");
  const num = (f, i) => (i < 0 ? NaN : parseFloat(f[i]));
  const rows = [];
  let t0 = null;
  let lastT = -1;
  for (let i = h + 1; i < lines.length; i++) {
    const f = lines[i].split(",");
    if (f.length < 5) continue;
    const ms = Date.parse(f[I.t]);
    const alt = num(f, I.alt);
    if (!Number.isFinite(ms) || !Number.isFinite(alt)) continue;
    if (t0 === null) t0 = ms;
    const t = (ms - t0) / 1000;
    if (t <= lastT) continue;
    lastT = t;
    const bms = num(f, I.bmsSt) > 0;
    const esc = num(f, I.escSt) > 0;
    const bp = num(f, I.bp);
    const escPower = num(f, I.ev) * num(f, I.ei) / 1000;
    const cellTemps = I.cell.map((k) => num(f, k)).filter(Number.isFinite);
    rows.push({
      t,
      alt,
      vario: num(f, I.vario),
      state: num(f, I.st) || 0,
      power: bms && Number.isFinite(bp) ? bp : Number.isFinite(escPower) ? escPower : 0,
      soc: num(f, I.soc),
      volts: bms ? num(f, I.pv) : num(f, I.ev),
      bt: cellTemps.length ? Math.max(...cellTemps) : NaN,
      et: num(f, I.mos),
      ec: num(f, I.cap),
      em: num(f, I.mcu),
      mt: num(f, I.mot),
      bm: num(f, I.bmsMos),
      bb: num(f, I.bmsBal),
      bms,
      esc,
    });
  }
  if (!rows.length) throw new Error("no usable rows");
  return { meta, rows, duration: rows[rows.length - 1].t };
}

function describeLog(meta, duration, name) {
  const bits = [
    meta["Flight Log Name"] || name,
    [meta["Wing Manufacturer"], meta["Wing Model"], meta["Wing Size"]].filter(Boolean).join(" "),
    meta["Propeller Blades"] ? `${meta["Propeller Blades"]}-blade ${meta["Propeller Manufacturer"] || ""} ${meta["Propeller Size (cm)"] || ""} cm prop` : "",
    meta["Equipment Weight (kg)"] ? `equipment ${meta["Equipment Weight (kg)"]} kg` : "",
    `${Math.floor(duration / 60)}m ${Math.round(duration % 60)}s`,
  ];
  return bits.filter((b) => b && b.trim()).join(" · ");
}

function loadLog(text, name) {
  const log = parseLog(text);
  state.replay = { ...log, name, idx: 0, t: 0, sentMs: 0 };
  $("logMeta").textContent = describeLog(log.meta, log.duration, name);
  clearHistory();
  state.pending.push("reset");
  state.running = true;
  updatePlayButton();
}

function seekReplay(frac) {
  const rp = state.replay;
  if (!rp) return;
  rp.t = frac * rp.duration;
  rp.idx = 0;
  rp.sentMs = Math.round(rp.t * 1000);
  clearHistory();
  state.pending.push("reset");
}

// Logs come at whatever rate the app recorded (raw ~50 Hz BLE, 300 ms from the
// app's database, 1 Hz summaries). The firmware sees the barometer at its ~30 Hz
// UI rate, so replay steps a 30 Hz clock through the log and interpolates
// altitude and power between rows. A gap of more than 3 s between rows is a
// logging dropout: nothing is sent across it, exactly like a stalled sensor.
function replayLines(simSeconds) {
  const rp = state.replay;
  if (!rp) return [];
  const rows = rp.rows;
  const target = Math.min(rp.t + simSeconds, rp.duration);
  const lines = [];
  while (rp.t + SIM_DT <= target) {
    rp.t += SIM_DT;
    while (rp.idx < rows.length && rows[rp.idx].t <= rp.t) rp.idx++;
    const a = rows[rp.idx - 1];
    const b = rows[rp.idx];
    if (!a) continue;
    let row = a;
    if (b) {
      const span = b.t - a.t;
      if (span > 3) continue;
      const f = (rp.t - a.t) / span;
      row = { ...a, alt: a.alt + (b.alt - a.alt) * f, power: a.power + (b.power - a.power) * f };
    }
    const ms = Math.round(rp.t * 1000);
    const dt = Math.max(1, ms - rp.sentMs);
    rp.sentMs = ms;
    lines.push(sampleLine(dt, { ...row, armed: row.state > 0, cruise: row.state === 2 }));
    if (rp.t - state.lastRecord >= RECORD_EVERY || rp.t < state.lastRecord) {
      state.lastRecord = rp.t;
      state.hist.push({ t: rp.t, power: row.power, logVario: row.vario });
    }
  }
  if (rp.t + SIM_DT > rp.duration) {
    state.running = false;
    updatePlayButton();
  }
  return lines;
}

// ------------------------------------------------------------ estimator state
const now = () => (state.source === "sim" ? state.sim.t : state.replay ? state.replay.t : 0);
const toWh = (y) => (Number.isFinite(y) ? y * 3.6 : NaN);

function onEstimator(st) {
  state.est = st;
  const t = now();
  let truth = null;
  let tYieldAtP = NaN;
  if (state.source === "sim") {
    if (!state.truth || t - state.truthAt > 5 || t < state.truthAt) {
      state.truth = trueCurve(state.sim.setup, state.sim.alt, state.bandFraction);
      state.truthAt = t;
    }
    truth = state.truth;
    if (Number.isFinite(st.power) && st.power >= 1.5) {
      tYieldAtP = stillAirClimb(state.sim.setup, st.power, state.sim.alt) / st.power;
    }
  }
  state.estHist.push({
    t,
    phase: st.phase,
    climb: st.climb,
    power: st.power,
    yield: toWh(st.yield),
    bestYield: toWh(st.bestYield),
    best: st.curveValid ? st.best : NaN,
    bandLo: st.curveValid ? st.bandLo : NaN,
    bandHi: st.curveValid ? st.bandHi : NaN,
    tBest: truth ? truth.bestPower : NaN,
    tLo: truth ? truth.bandLo : NaN,
    tHi: truth ? truth.bandHi : NaN,
    tBestYield: truth ? toWh(truth.bestYield) : NaN,
    tYieldAtP: toWh(tYieldAtP),
    level: st.levelValid ? st.level : NaN,
    tLevel: truth ? truth.levelPower : NaN,
    warn: Number.isFinite(st.warn) && st.warn < 16 ? st.warn : NaN,
    crit: Number.isFinite(st.crit) && st.crit < 16 ? st.crit : NaN,
    motorT: state.source === "sim" && state.sim.temps ? state.sim.temps.motor : NaN,
  });
  if (st.levelValid && !Number.isFinite(state.firstLevelT)) state.firstLevelT = t;
  if (st.curveValid && !Number.isFinite(state.firstBandT)) state.firstBandT = t;

  trimHistory(t);
  updateReadouts(st, truth, tYieldAtP);
}

function trimHistory(t) {
  const cut = t - HISTORY_S;
  for (const arr of [state.hist, state.estHist]) {
    let k = 0;
    while (k < arr.length && arr[k].t < cut) k++;
    if (k) arr.splice(0, k);
  }
}

function clearHistory() {
  state.hist = [];
  state.estHist = [];
  state.lastRecord = -1e9;
  state.firstLevelT = NaN;
  state.firstBandT = NaN;
  state.truth = null;
}

const fmt = (v, n = 2, unit = "") => (Number.isFinite(v) ? v.toFixed(n) + unit : "–");

function setText(id, text, cls) {
  const el = $(id);
  el.textContent = text;
  el.className = cls || "";
}

function updateReadouts(st, truth, tYieldAtP) {
  const phaseCls = { VALID: "good", SETTLING: "muted", UNSTEADY: "warn", LOW_POWER: "muted", NOT_CLIMBING: "muted" }[st.phase] || "muted";
  setText("rPhase", st.phase, phaseCls);
  const chip = $("chipPhase");
  chip.textContent = st.phase;
  chip.className = `chip ${phaseCls === "muted" ? "" : phaseCls}`;

  setText("rClimb", fmt(st.climb));
  setText("rPower", fmt(st.power, 1));
  setText("rCv", Number.isFinite(st.cv) ? `${(st.cv * 100).toFixed(1)} %` : "–",
    Number.isFinite(st.cv) && st.cv > 0.12 ? "warn" : "");
  setText("rYield", fmt(toWh(st.yield)), st.phase === "VALID" ? "" : "muted");
  setText("rRel", Number.isFinite(st.rel) ? `${(st.rel * 100).toFixed(0)} %` : "–",
    st.rel >= 0.95 ? "good" : "");
  setText("rBest", st.curveValid ? fmt(st.best, 1) : "learning…",
    st.curveValid ? "" : "muted");
  setText("rBand", st.curveValid ? `${fmt(st.bandLo, 1)} – ${fmt(st.bandHi, 1)}` : "–");
  setText("rLearned", fmt(st.learned, 0));

  const sim = state.source === "sim";
  const last = state.hist[state.hist.length - 1];
  setText("tClimb", sim && last ? `${fmt(last.climbStill)} still air` : "–", "muted");
  setText("tPower", sim && last ? fmt(last.power, 1) : "–", "muted");
  setText("tYield", sim ? fmt(toWh(tYieldAtP)) : "–", "muted");
  setText("tRel", sim && truth && Number.isFinite(tYieldAtP)
    ? `${((tYieldAtP / truth.bestYield) * 100).toFixed(0)} %` : "–", "muted");
  setText("tBest", sim && truth ? fmt(truth.bestPower, 1) : "–", "muted");
  setText("tBand", sim && truth ? `${fmt(truth.bandLo, 1)} – ${fmt(truth.bandHi, 1)}` : "–", "muted");

  // How steady and how right the bar markers are
  const t = now();
  const recent = state.estHist.filter((e) => e.t >= t - 60);
  const spread = (key) => {
    const v = recent.map((e) => e[key]).filter(Number.isFinite);
    return v.length > 1 ? Math.max(...v) - Math.min(...v) : NaN;
  };
  const clock = (s) => (Number.isFinite(s)
    ? `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}` : "not yet");
  setText("sLevelAt", clock(state.firstLevelT), Number.isFinite(state.firstLevelT) ? "" : "muted");
  setText("sBandAt", clock(state.firstBandT), Number.isFinite(state.firstBandT) ? "" : "muted");
  setText("sLevelDrift", fmt(spread("level"), 2));
  setText("sBandDrift", fmt(Math.max(spread("bandLo") || 0, spread("bandHi") || 0), 2));
  const levelErr = st.levelValid && truth ? st.level - truth.levelPower : NaN;
  const bestErr = st.curveValid && truth ? st.best - truth.bestPower : NaN;
  const signed = (v, n) => (Number.isFinite(v) ? `${v >= 0 ? "+" : ""}${v.toFixed(n)}` : "–");
  setText("sLevelErr", signed(levelErr, 2), Math.abs(levelErr) > 0.5 ? "warn" : "");
  setText("sBestErr", signed(bestErr, 1), Math.abs(bestErr) > 2 ? "warn" : "");
  setText("rLevel", st.levelValid ? `${fmt(st.level, 2)} ± ${fmt(st.levelMargin, 2)}` : "learning…",
    st.levelValid ? "" : "muted");
  const parts = ["motor", "ESC MOSFET", "ESC capacitor", "ESC MCU", "battery cells", "BMS MOSFET", "BMS balance"];
  const lim = (v) => (Number.isFinite(v) && v < 16 ? `${v.toFixed(1)} kW` : "none");
  setText("rHeat", `${lim(st.warn)} / ${lim(st.crit)}`);
  setText("tHeat", st.warnPart >= 0 ? `${parts[st.warnPart]} limits` : "", "muted");
  setText("tLevel", sim && truth ? fmt(truth.levelPower, 2) : "–", "muted");

  const mm = Math.floor(t / 60);
  const ss = Math.floor(t % 60);
  $("chipTime").textContent = `t ${String(mm).padStart(2, "0")}:${String(ss).padStart(2, "0")} · ${state.speed}×${state.running ? "" : " ⏸"}`;
  if (state.replay && state.source === "replay") {
    $("progressBar").style.width = `${Math.min(100, state.replay.t / state.replay.duration * 100)}%`;
    $("replayPos").textContent = `${fmt(state.replay.t / 60, 1)} / ${fmt(state.replay.duration / 60, 1)} min`;
  }
}

// --------------------------------------------------------------------- charts
function niceTicks(lo, hi, n) {
  const raw = (hi - lo) / n;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw) || raw;
  const ticks = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) ticks.push(v);
  return ticks;
}

function prepCanvas(c) {
  const dpr = window.devicePixelRatio || 1;
  const W = c.clientWidth;
  const H = c.clientHeight;
  if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) {
    c.width = Math.round(W * dpr);
    c.height = Math.round(H * dpr);
  }
  const g = c.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  return { g, W, H };
}

function firstIndex(arr, t) {
  let lo = 0;
  let hi = arr.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (arr[mid].t < t) lo = mid + 1; else hi = mid;
  }
  return lo;
}

function drawLegend(g, W, items) {
  g.font = "11px system-ui";
  let x = W - 8;
  for (let i = items.length - 1; i >= 0; i--) {
    const it = items[i];
    const w = g.measureText(it.label).width;
    x -= w;
    g.fillStyle = COLORS.muted;
    g.fillText(it.label, x, 12);
    x -= 14;
    g.strokeStyle = it.color;
    g.lineWidth = 2;
    g.setLineDash(it.dash || []);
    g.beginPath();
    g.moveTo(x, 8);
    g.lineTo(x + 10, 8);
    g.stroke();
    g.setLineDash([]);
    x -= 10;
  }
}

// series: {label, color, arr, key | lo+hi (band), dash, width, dots}
function drawTimeChart(canvas, title, series, opts = {}) {
  const { g, W, H } = prepCanvas(canvas);
  const L = 44, R = 10, T = 20, B = 18;
  const pw = W - L - R, ph = H - T - B;
  const tEnd = Math.max(now(), state.span);
  const t0 = tEnd - state.span;

  let lo = Infinity, hi = -Infinity;
  const visible = series.map((s) => {
    const arr = s.arr || [];
    const pts = arr.slice(Math.max(0, firstIndex(arr, t0) - 1));
    for (const p of pts) {
      for (const k of s.key ? [s.key] : [s.lo, s.hi]) {
        const v = p[k];
        if (Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
      }
    }
    return pts;
  });
  if (opts.include) for (const v of opts.include) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  if (!Number.isFinite(lo)) { lo = 0; hi = 1; }
  if (opts.clampLo !== undefined) lo = Math.max(lo, opts.clampLo);
  if (opts.clampHi !== undefined) hi = Math.min(hi, opts.clampHi);
  if (hi - lo < (opts.minRange || 0.5)) { const m = (hi + lo) / 2; lo = m - (opts.minRange || 0.5) / 2; hi = m + (opts.minRange || 0.5) / 2; }
  const pad = (hi - lo) * 0.08;
  lo -= pad; hi += pad;
  const X = (t) => L + ((t - t0) / state.span) * pw;
  const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;

  g.font = "11px system-ui";
  g.strokeStyle = COLORS.grid;
  g.lineWidth = 1;
  g.fillStyle = COLORS.muted;
  for (const v of niceTicks(lo, hi, 4)) {
    g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(W - R, Y(v)); g.stroke();
    g.fillText(Math.abs(v) < 1e-9 ? "0" : +v.toFixed(2) + "", 6, Y(v) + 4);
  }
  for (const v of niceTicks(t0, tEnd, 5)) {
    if (v < 0) continue;
    const m = Math.floor(v / 60), s = Math.round(v % 60);
    g.fillText(`${m}:${String(s).padStart(2, "0")}`, X(v) - 12, H - 4);
  }
  g.fillStyle = COLORS.truth;
  g.font = "600 11px system-ui";
  g.fillText(title, L, 12);

  g.save();
  g.beginPath(); g.rect(L, T, pw, ph); g.clip();
  series.forEach((s, i) => {
    const pts = visible[i];
    if (s.lo) {  // band
      g.fillStyle = s.color;
      let open = false;
      const upper = [];
      const flush = () => {
        if (!upper.length) return;
        g.beginPath();
        upper.forEach((p, k) => (k ? g.lineTo(X(p.t), Y(p[s.hi])) : g.moveTo(X(p.t), Y(p[s.hi]))));
        for (let k = upper.length - 1; k >= 0; k--) g.lineTo(X(upper[k].t), Y(upper[k][s.lo]));
        g.closePath(); g.fill();
        upper.length = 0;
      };
      for (const p of pts) {
        if (Number.isFinite(p[s.lo]) && Number.isFinite(p[s.hi])) { upper.push(p); open = true; }
        else if (open) { flush(); open = false; }
      }
      flush();
      return;
    }
    g.strokeStyle = s.color;
    g.fillStyle = s.color;
    g.lineWidth = s.width || 1.5;
    g.setLineDash(s.dash || []);
    g.beginPath();
    let pen = false;
    for (const p of pts) {
      const v = p[s.key];
      if (!Number.isFinite(v)) { pen = false; continue; }
      if (s.dots) { g.fillRect(X(p.t) - 1, Y(v) - 1, 2, 2); continue; }
      if (pen) g.lineTo(X(p.t), Y(v)); else g.moveTo(X(p.t), Y(v));
      pen = true;
    }
    g.stroke();
    g.setLineDash([]);
  });
  g.restore();
  drawLegend(g, W, series.filter((s) => s.label && (s.arr || []).length));
}

function drawCurveChart(canvas, title, yLabel, build) {
  const { g, W, H } = prepCanvas(canvas);
  const L = 44, R = 12, T = 20, B = 26;
  const pw = W - L - R, ph = H - T - B;
  const d = build();
  const pMax = d.pMax;
  let lo = d.yLo, hi = d.yHi;
  const X = (p) => L + (p / pMax) * pw;
  const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;

  g.font = "11px system-ui";
  g.strokeStyle = COLORS.grid;
  g.fillStyle = COLORS.muted;
  for (const v of niceTicks(lo, hi, 4)) {
    g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(W - R, Y(v)); g.stroke();
    g.fillText(+v.toFixed(2) + "", 6, Y(v) + 4);
  }
  for (const p of niceTicks(0, pMax, 6)) {
    g.fillText(`${p}`, X(p) - 4, H - 10);
  }
  g.fillText("power (kW)", W - R - 60, H - 1);
  g.fillStyle = COLORS.truth;
  g.font = "600 11px system-ui";
  g.fillText(title, L, 12);
  g.font = "11px system-ui";
  g.fillStyle = COLORS.muted;
  g.fillText(yLabel, L + g.measureText(title).width + 12, 12);

  g.save();
  g.beginPath(); g.rect(L, T, pw, ph); g.clip();
  for (const band of d.bands || []) {
    g.fillStyle = band.color;
    g.fillRect(X(band.lo), T, X(band.hi) - X(band.lo), ph);
  }
  if (lo < 0 && hi > 0) {
    g.strokeStyle = "#3a4150";
    g.beginPath(); g.moveTo(L, Y(0)); g.lineTo(W - R, Y(0)); g.stroke();
  }
  for (const c of d.curves || []) {
    g.strokeStyle = c.color;
    g.lineWidth = c.width || 1.5;
    g.setLineDash(c.dash || []);
    g.beginPath();
    let pen = false;
    for (const [p, v] of c.pts) {
      if (!Number.isFinite(v)) { pen = false; continue; }
      if (pen) g.lineTo(X(p), Y(v)); else g.moveTo(X(p), Y(v));
      pen = true;
    }
    g.stroke();
    g.setLineDash([]);
  }
  for (const m of d.markers || []) {
    g.strokeStyle = m.color;
    g.lineWidth = 1;
    g.setLineDash(m.dash || []);
    g.beginPath(); g.moveTo(X(m.p), T); g.lineTo(X(m.p), T + ph); g.stroke();
    g.setLineDash([]);
  }
  for (const pt of d.points || []) {
    g.fillStyle = pt.color;
    g.globalAlpha = pt.alpha ?? 1;
    g.beginPath(); g.arc(X(pt.p), Y(pt.v), pt.r, 0, Math.PI * 2); g.fill();
    g.globalAlpha = 1;
  }
  g.restore();
  drawLegend(g, W, (d.legend || []));
}

function fittedClimb(st, p) {
  const [c0, c1, c2] = st.c || [];
  return st.curveValid ? c0 + c1 * p + c2 * p * p : NaN;
}

function drawCharts() {
  const hist = state.hist;
  const est = state.estHist;
  const sim = state.source === "sim";
  drawTimeChart($("cPower"), "Power (kW)", [
    { label: "firmware sweet spot", color: "rgba(61,220,151,.18)", arr: est, lo: "bandLo", hi: "bandHi" },
    { label: sim ? "true sweet spot" : "", color: COLORS.good, arr: sim ? est : [], key: "tLo", dash: [4, 4], width: 1 },
    { color: COLORS.good, arr: sim ? est : [], key: "tHi", dash: [4, 4], width: 1 },
    { label: sim ? "pilot command" : "", color: COLORS.muted, arr: sim ? hist : [], key: "cmd", dash: [2, 3], width: 1 },
    { label: "firmware level", color: COLORS.pink, arr: est, key: "level", width: 2 },
    { label: "hold → warning", color: "#f5b942", arr: est, key: "warn", dash: [3, 3], width: 1.5 },
    { label: "hold → critical", color: "#ff6b6b", arr: est, key: "crit", dash: [3, 3], width: 1.5 },
    { label: "measured", color: COLORS.accent, arr: hist, key: "power" },
  ], { include: [0] });
  drawTimeChart($("cClimb"), "Climb rate (m/s)", [
    { label: sim ? "actual (incl. air)" : "", color: "rgba(111,183,255,.45)", arr: sim ? hist : [], key: "climbTrue", width: 1 },
    { label: sim ? "still-air truth" : "", color: COLORS.truth, arr: sim ? hist : [], key: "climbStill", dash: [5, 4], width: 1 },
    { label: sim ? "" : "controller vario", color: "rgba(111,183,255,.6)", arr: sim ? [] : hist, key: "logVario", width: 1 },
    { label: "firmware 10 s window", color: COLORS.est, arr: est, key: "climb", width: 2 },
  ], { include: [0], clampLo: -6, clampHi: 8 });
  drawTimeChart($("cYield"), "Climb yield (m/Wh)", [
    { label: sim ? "still-air truth" : "", color: COLORS.truth, arr: sim ? hist : [], key: "yieldTrue", dash: [5, 4], width: 1 },
    { label: "firmware best", color: COLORS.good, arr: est, key: "bestYield", width: 1 },
    { label: "firmware (valid only)", color: COLORS.est, arr: est, key: "yield", width: 2 },
  ], { include: [0], clampLo: -1, clampHi: 2 });
  drawTimeChart($("cBest"), "Learned powers (kW)", [
    { label: sim ? "true level" : "", color: COLORS.pink, arr: sim ? est : [], key: "tLevel", dash: [5, 4], width: 1 },
    { label: "firmware level", color: COLORS.pink, arr: est, key: "level", width: 2 },
    { label: "firmware band", color: "rgba(61,220,151,.18)", arr: est, lo: "bandLo", hi: "bandHi" },
    { label: sim ? "true band" : "", color: COLORS.good, arr: sim ? est : [], key: "tLo", dash: [4, 4], width: 1 },
    { color: COLORS.good, arr: sim ? est : [], key: "tHi", dash: [4, 4], width: 1 },
    { label: sim ? "true best" : "", color: COLORS.truth, arr: sim ? est : [], key: "tBest", dash: [5, 4], width: 1 },
    { label: "firmware best", color: COLORS.good, arr: est, key: "best", width: 2 },
  ], { include: [0] });

  const st = state.est || {};
  const truth = sim ? state.truth : null;
  const pMax = sim ? state.sim.setup.maxPowerKw : Math.max(20, Math.ceil((st.dataMax || 0) + 2));
  const grid = [];
  for (let p = 0; p <= pMax; p += 0.25) grid.push(p);
  const binPts = (st.bins || []).map(([p, w, wt]) => ({ p, v: w, r: 1.5 + Math.sqrt(wt) * 0.9, color: COLORS.violet, alpha: 0.55 }));
  const cur = Number.isFinite(st.power) && Number.isFinite(st.climb)
    ? [{ p: st.power, v: st.climb, r: 4, color: COLORS.est }] : [];
  const inData = (p) => Number.isFinite(st.dataMin) && p >= st.dataMin && p <= st.dataMax;

  drawCurveChart($("cCurve"), "Climb vs power", "m/s", () => ({
    pMax, yLo: -2.5, yHi: 6,
    curves: [
      ...(truth ? [{ color: COLORS.truth, dash: [5, 4], width: 1, pts: truth.points.map((q) => [q.p, q.w]) }] : []),
      { color: "rgba(180,140,255,.35)", pts: grid.map((p) => [p, fittedClimb(st, p)]) },
      { color: COLORS.violet, width: 2, pts: grid.map((p) => [p, inData(p) ? fittedClimb(st, p) : NaN]) },
    ],
    points: [...binPts, ...cur],
    markers: [
      ...(st.levelValid ? [{ p: st.level, color: COLORS.pink }] : []),
      ...(truth ? [{ p: truth.levelPower, color: COLORS.pink, dash: [5, 4] }] : []),
    ],
    legend: [
      ...(truth ? [{ label: "still-air truth", color: COLORS.truth, dash: [5, 4] }] : []),
      { label: "firmware fit (bins)", color: COLORS.violet },
      { label: "now", color: COLORS.est },
    ],
  }));

  drawCurveChart($("cYieldCurve"), "Climb yield vs power", "m/Wh", () => ({
    pMax, yLo: 0, yHi: 1.4,
    bands: st.curveValid ? [{ lo: st.bandLo, hi: st.bandHi, color: "rgba(61,220,151,.14)" }] : [],
    curves: [
      ...(truth ? [{ color: COLORS.truth, dash: [5, 4], width: 1, pts: truth.points.map((q) => [q.p, q.y * 3.6]) }] : []),
      { color: "rgba(180,140,255,.35)", pts: grid.map((p) => [p, p > 0.5 ? fittedClimb(st, p) / p * 3.6 : NaN]) },
      { color: COLORS.violet, width: 2, pts: grid.map((p) => [p, inData(p) ? fittedClimb(st, p) / p * 3.6 : NaN]) },
    ],
    markers: [
      ...(st.curveValid ? [{ p: st.best, color: COLORS.good }] : []),
      ...(truth ? [{ p: truth.bestPower, color: COLORS.truth, dash: [5, 4] }] : []),
    ],
    points: st.phase === "VALID" ? [{ p: st.power, v: st.yield * 3.6, r: 4, color: COLORS.est }] : [],
    legend: [
      ...(truth ? [{ label: "truth", color: COLORS.truth, dash: [5, 4] }] : []),
      { label: "firmware fit", color: COLORS.violet },
      { label: "sweet spot", color: COLORS.good },
    ],
  }));
}

// ------------------------------------------------------------------- controls
function seg(id, value, onPick) {
  const el = $(id);
  const paint = (v) => el.querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.v === String(v)));
  paint(value);
  el.addEventListener("click", (e) => {
    const b = e.target.closest("button");
    if (!b) return;
    paint(b.dataset.v);
    onPick(b.dataset.v);
  });
}

function slider(container, spec, get, set) {
  const row = document.createElement("div");
  row.className = "row";
  row.innerHTML = `<label class="name">${spec.label}</label>
    <input type="range" min="${spec.min}" max="${spec.max}" step="${spec.step}">
    <span class="val"></span>`;
  const input = row.querySelector("input");
  const out = row.querySelector(".val");
  input.value = get();
  const show = () => (out.textContent = spec.fmt ? spec.fmt(+input.value) : `${input.value} ${spec.unit || ""}`);
  show();
  input.addEventListener("input", () => { set(+input.value); show(); });
  container.appendChild(row);
}

function updatePlayButton() {
  $("playBtn").textContent = state.running ? "Pause" : "Play";
}

function initScreen() {
  state.pending.push(`init theme=${state.theme} metric=${state.metric} perf=${state.perf}`);
}

function setupControls() {
  seg("barSeg", state.bar, (v) => {
    state.bar = +v;
    state.pending.push(`bar v=${v}`);
  });
  seg("themeSeg", state.theme, (v) => { state.theme = +v; initScreen(); });
  seg("unitSeg", state.metric, (v) => { state.metric = +v; initScreen(); });
  seg("perfSeg", state.perf, (v) => { state.perf = +v; initScreen(); });
  seg("speedSeg", state.speed, (v) => { state.speed = +v; });
  seg("sourceSeg", state.source, (v) => {
    state.source = v;
    $("simControls").classList.toggle("hidden", v !== "sim");
    $("replayControls").classList.toggle("hidden", v !== "replay");
    clearHistory();
    state.pending.push("reset");
    state.running = v === "sim" || !!state.replay;
    updatePlayButton();
  });
  $("playBtn").addEventListener("click", () => { state.running = !state.running; updatePlayButton(); });
  $("spanSel").addEventListener("change", (e) => { state.span = +e.target.value; });

  const sim = state.sim;
  const scen = $("scenarioSel");
  for (const [k, v] of Object.entries(SCENARIOS)) {
    const o = document.createElement("option");
    o.value = k;
    o.textContent = v.label;
    scen.appendChild(o);
  }
  scen.value = sim.scenario;
  const syncThrottle = () => { $("throttle").disabled = sim.scenario !== "manual"; };
  syncThrottle();
  scen.addEventListener("change", () => {
    if (scen.value === "manual") sim.throttleKw = sim.commandKw;
    sim.scenario = scen.value;
    syncThrottle();
  });
  $("throttle").addEventListener("input", (e) => {
    sim.throttleKw = +e.target.value;
    $("throttleVal").textContent = `${(+e.target.value).toFixed(1)} kW`;
  });
  sim.throttleKw = +$("throttle").value;
  $("armChk").addEventListener("change", (e) => { sim.armed = e.target.checked; });
  $("resetBtn").addEventListener("click", () => {
    const scenario = sim.scenario;
    const throttle = sim.throttleKw;
    sim.reset(+$("seedIn").value || 1);
    sim.scenario = scenario;
    sim.throttleKw = throttle;
    sim.armed = $("armChk").checked;
    clearHistory();
    state.pending.push("reset");
    state.running = true;
    updatePlayButton();
  });
  seg("trimSeg", sim.setup.trim, (v) => { sim.setup.trim = +v; state.truth = null; });
  $("brake").addEventListener("input", (e) => {
    sim.setup.brake = +e.target.value;
    $("brakeVal").textContent = `${Math.round(+e.target.value * 100)} %`;
    state.truth = null;
  });

  const env = $("envSliders");
  const envSpecs = [
    ["thermalStrength", "Thermals", 0, 3, 0.1, "m/s"],
    ["thermalRate", "Thermal rate", 0, 4, 0.1, "/min"],
    ["turbulence", "Gusts", 0, 1, 0.05, "m/s"],
    ["handMotion", "Arm movement", 0, 1, 0.05, "m"],
    ["baroNoise", "Baro noise", 0, 0.5, 0.01, "m"],
    ["powerNoise", "Power ripple", 0, 0.05, 0.005, ""],
    ["airTempC", "Air temperature", 0, 45, 1, "C"],
  ];
  for (const [key, label, min, max, step, unit] of envSpecs) {
    slider(env, { label, min, max, step, unit }, () => sim.env[key], (v) => { sim.env[key] = v; });
  }
  const setupSpecs = [
    ["massKg", "All-up mass", 70, 170, 1, "kg"],
    ["sinkMs", "Glide sink", 0.8, 2.2, 0.05, "m/s"],
    ["trimSpeed", "Trim speed", 8, 14, 0.1, "m/s"],
    ["propDiam", "Prop diameter", 1.0, 1.6, 0.01, "m"],
    ["motorLossK", "Motor loss at max", 0, 0.3, 0.01, ""],
    ["maxPowerKw", "Max power", 10, 30, 0.5, "kW"],
    ["climbTau", "Climb response", 0.5, 6, 0.1, "s"],
  ];
  for (const [key, label, min, max, step, unit] of setupSpecs) {
    slider(env, { label, min, max, step, unit }, () => sim.setup[key], (v) => {
      sim.setup[key] = v;
      state.truth = null;
      if (key === "maxPowerKw") $("throttle").max = v;
    });
  }

  const cfg = {
    window: 10, settle: 3, cv: 0.12, minp: 1.5, minclimb: 0.3, minalt: 15, q: 0.016,
    band: 0.95, learn: 1, minbinw: 4,
  };
  const cfgSpecs = [
    ["window", "Window", 4, 30, 1, "s"],
    ["settle", "Settle", 0, 10, 0.5, "s"],
    ["cv", "Max power CV", 0.03, 0.3, 0.01, ""],
    ["minp", "Min power", 0.5, 5, 0.1, "kW"],
    ["minclimb", "Min climb", 0, 1.5, 0.05, "m/s"],
    ["minalt", "Min altitude", 0, 50, 1, "m"],
    ["q", "Curvature q", 0.01, 0.08, 0.002, "/kW"],
    ["band", "Sweet-spot band", 0.85, 0.99, 0.01, "of best"],
    ["learn", "Learn every", 0.5, 5, 0.5, "s"],
    ["minbinw", "Min bin weight", 1, 20, 1, "pts"],
  ];
  for (const [key, label, min, max, step, unit] of cfgSpecs) {
    slider($("cfgSliders"), { label, min, max, step, unit }, () => cfg[key], (v) => { cfg[key] = v; });
  }
  $("cfgApply").addEventListener("click", () => {
    state.bandFraction = cfg.band;
    state.truth = null;
    state.pending.push(`cfg window=${cfg.window * 1000} settle=${cfg.settle * 1000} cv=${cfg.cv} ` +
      `minp=${cfg.minp} minclimb=${cfg.minclimb} minalt=${cfg.minalt} q=${cfg.q} band=${cfg.band} ` +
      `learn=${cfg.learn * 1000} minbinw=${cfg.minbinw}`);
  });

  // Replay sources
  fetch("/api/logs").then((r) => r.json()).then((logs) => {
    for (const l of logs) {
      const o = document.createElement("option");
      o.value = l.name;
      o.textContent = `${l.name} (${(l.size / 1e6).toFixed(1)} MB)`;
      $("logSel").appendChild(o);
    }
  }).catch(() => {});
  $("logSel").addEventListener("change", async (e) => {
    if (!e.target.value) return;
    $("logMeta").textContent = "loading…";
    try {
      const text = await (await fetch(`/api/logs/${encodeURIComponent(e.target.value)}`)).text();
      loadLog(text, e.target.value);
    } catch (err) {
      $("logMeta").textContent = `Could not load: ${err.message}`;
    }
  });
  $("logFile").addEventListener("change", async (e) => {
    const file = e.target.files[0];
    if (!file) return;
    try {
      loadLog(await file.text(), file.name);
    } catch (err) {
      $("logMeta").textContent = `Could not load: ${err.message}`;
    }
  });
  $("progress").addEventListener("click", (e) => {
    const r = e.currentTarget.getBoundingClientRect();
    seekReplay((e.clientX - r.left) / r.width);
  });
}

function setConn(ok, msg) {
  const chip = $("chipConn");
  chip.textContent = ok ? "firmware UI live" : `offline: ${msg}`;
  chip.className = `chip ${ok ? "good" : "bad"}`;
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function loop() {
  let last = performance.now();
  let lastChart = 0;
  for (;;) {
    const t = performance.now();
    const realDt = Math.min(0.25, (t - last) / 1000);
    last = t;
    const lines = state.pending.splice(0);
    if (state.running) {
      const simSeconds = realDt * state.speed;
      lines.push(...(state.source === "sim" ? simLines(simSeconds) : replayLines(simSeconds)));
    }
    lines.push("render");
    try {
      const res = await emu(lines);
      setConn(true);
      if (res.frame) drawFrame(res.frame);
      if (res.state) onEstimator(res.state);
    } catch (err) {
      setConn(false, err.message);
      await sleep(1000);
      continue;
    }
    if (t - lastChart > 100) {
      drawCharts();
      lastChart = t;
    }
    await sleep(Math.max(0, 40 - (performance.now() - t)));
  }
}

setupControls();
updatePlayButton();
loop();
