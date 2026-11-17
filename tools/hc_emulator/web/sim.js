// Paramotor flight simulator used to exercise the climb efficiency estimator.
//
// Physics (quasi-steady, fixed wing trim):
//   - The wing flies at a trim airspeed V with glide sink s, so level-flight
//     drag is D = m g s / V.
//   - Electrical power P becomes shaft power through a motor/ESC efficiency
//     that falls off with load, then thrust through actuator-disk momentum
//     theory (T (V + v_i) = P_shaft * eta_profile), which is what makes
//     thrust - and so climb - flatten out at high power.
//   - Still-air climb w = (T - D) V / (m g); the real climb follows it with a
//     first-order lag (pendulum/pitch settling) plus the air mass motion.
//   - The controller's baro sees altitude + hand movement + sensor noise,
//     through the BMP390 IIR filter the firmware configures (coefficient 15).
//
// Everything is seeded so a scenario replays identically.

export const DEFAULT_SETUP = {
  massKg: 115,          // all-up: pilot + motor + battery + wing
  trimSpeed: 10.5,      // m/s airspeed at neutral trim
  sinkMs: 1.3,          // m/s glide sink at neutral trim
  propDiam: 1.3,        // m
  motorEtaPeak: 0.92,   // motor + ESC
  motorLossK: 0.10,     // efficiency lost at full power (copper losses)
  fixedLossKw: 0.15,    // controller, fans, idle losses
  propEta: 0.85,        // prop profile efficiency (on top of momentum losses)
  maxPowerKw: 22,
  climbTau: 2.5,        // s, climb response lag to a power change
  trim: 0,              // -1 slow, 0 neutral, +1 fast
  brake: 0,             // 0..1
};

export const DEFAULT_ENV = {
  thermalStrength: 0.8,  // m/s typical core lift (0 = still air)
  thermalRate: 1.0,      // thermal/sink encounters per minute
  turbulence: 0.2,       // m/s RMS small-scale vertical gusts
  handMotion: 0.3,       // m RMS controller height change from arm movement
  baroNoise: 0.15,       // m RMS raw baro noise before the IIR filter
  powerNoise: 0.015,     // fractional power ripple
  airTempC: 25,          // outside air temperature
};

// Part temperatures: first-order models fitted to 30 real flights
// (analysis/fleet_thermal.py). T_base is relative to air temperature.
export const THERMAL_PARTS = {
  motor: { k: 1.0, tau: 207, baseOverAir: 21 },
  escMos: { k: 0.55, tau: 179, baseOverAir: 5 },
  escCap: { k: 0.61, tau: 198, baseOverAir: 5 },
  escMcu: { k: 0.75, tau: 286, baseOverAir: -1 },
  battery: { k: 0.35, tau: 5000, baseOverAir: 12 },
  bmsMos: { k: 0.71, tau: 5000, baseOverAir: 8 },
  bmsBalance: { k: 0.65, tau: 5000, baseOverAir: 8 },
};

const G = 9.81;

export function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function gaussian(rand) {
  const u = Math.max(rand(), 1e-12);
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rand());
}

export function airDensity(altM) {
  return 1.225 * Math.pow(Math.max(0.2, 1 - 2.2558e-5 * altM), 4.2559);
}

export function wingState(setup) {
  const trim = setup.trim, brake = setup.brake;
  const speed = setup.trimSpeed * (1 + 0.15 * trim) * (1 - 0.15 * brake);
  const sink = setup.sinkMs *
    (trim > 0 ? 1 + 0.35 * trim : 1 + 0.03 * trim) * (1 + 0.6 * brake * brake);
  return { speed, sink };
}

// Still-air steady climb rate (m/s) at electrical power P (kW).
export function stillAirClimb(setup, powerKw, altM = 0) {
  const { speed: V, sink } = wingState(setup);
  const weight = setup.massKg * G;
  const drag = weight * sink / V;
  const load = Math.min(1, Math.max(0, powerKw / setup.maxPowerKw));
  const etaMotor = Math.max(0.3, setup.motorEtaPeak - setup.motorLossK * load * load);
  const shaftW = Math.max(0, powerKw - setup.fixedLossKw) * etaMotor * 1000 * setup.propEta;
  const twoRhoA = 2 * airDensity(altM) * Math.PI * (setup.propDiam / 2) ** 2;
  const powerFor = (T) => T * (V + (-V / 2 + Math.sqrt(V * V / 4 + T / twoRhoA)));
  let lo = 0, hi = 6000;
  for (let i = 0; i < 40; i++) {
    const mid = (lo + hi) / 2;
    if (powerFor(mid) > shaftW) hi = mid; else lo = mid;
  }
  const thrust = (lo + hi) / 2;
  return (thrust - drag) * V / weight;
}

// Ground-truth yield curve: best power and the band within `fraction` of it.
export function trueCurve(setup, altM, fraction = 0.95) {
  const points = [];
  let best = { p: 0, y: -Infinity };
  for (let p = 0.5; p <= setup.maxPowerKw + 1e-9; p += 0.25) {
    const w = stillAirClimb(setup, p, altM);
    const y = w / p;
    points.push({ p, w, y });
    if (y > best.y) best = { p, y };
  }
  const inBand = points.filter((pt) => pt.y >= fraction * best.y);
  // Level-flight power: where the still-air climb crosses zero
  let levelPower = NaN;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i];
    if (a.w < 0 && b.w >= 0) {
      levelPower = a.p + (b.p - a.p) * (-a.w) / (b.w - a.w);
      break;
    }
  }
  return {
    points,
    levelPower,
    bestPower: best.p,
    bestYield: best.y,
    bandLo: inBand.length ? inBand[0].p : NaN,
    bandHi: inBand.length ? inBand[inBand.length - 1].p : NaN,
  };
}

// Air mass vertical motion: discrete thermals/sink patches + gust noise.
class AirMass {
  constructor(rand) {
    this.rand = rand;
    this.events = [];
    this.gust = 0;
  }

  step(t, dt, env) {
    const perSecond = env.thermalRate / 60;
    if (env.thermalStrength > 0 && this.rand() < perSecond * dt) {
      const lift = this.rand() < 0.55;
      this.events.push({
        start: t,
        dur: 15 + this.rand() * 45,
        amp: env.thermalStrength * (0.4 + 0.8 * this.rand()) * (lift ? 1 : -0.6),
      });
    }
    this.events = this.events.filter((e) => t < e.start + e.dur);
    let w = 0;
    for (const e of this.events) {
      w += e.amp * Math.sin(Math.PI * (t - e.start) / e.dur) ** 2;
    }
    // Ornstein-Uhlenbeck gusts with a 4 s correlation time
    const tau = 4;
    this.gust += (-this.gust / tau) * dt +
      env.turbulence * Math.sqrt(2 * dt / tau) * gaussian(this.rand);
    return w + this.gust;
  }
}

// Throttle "pilots" for hands-off testing.
export const SCENARIOS = {
  manual: { label: "Manual (throttle slider)" },
  sweep: { label: "Power sweep 4 → 20 kW" },
  pilot: { label: "Random pilot" },
  climbs: { label: "Climb / glide cycles" },
};

function sweepPower(t) {
  const warmup = 45;
  if (t < warmup) return 12;
  const steps = [4, 6, 8, 10, 12, 14, 16, 18, 20];
  const hold = 40;
  const cycle = steps.length * hold + 30;
  const k = (t - warmup) % cycle;
  const i = Math.floor(k / hold);
  return i < steps.length ? steps[i] : 0;  // glide at the end of each sweep
}

export class FlightSim {
  constructor(seed = 1) {
    this.setup = { ...DEFAULT_SETUP };
    this.env = { ...DEFAULT_ENV };
    this.reset(seed);
  }

  reset(seed = this.seed) {
    this.seed = seed;
    this.rand = mulberry32(seed);
    this.air = new AirMass(this.rand);
    this.t = 0;
    this.alt = 0;
    this.wMotor = 0;
    this.airW = 0;
    this.climb = 0;
    this.power = 0;
    this.powerMeasured = 0;
    this.lastBmsT = -1;
    this.hand = 0;
    this.baroIir = 0;
    this.throttleKw = 0;       // manual command
    this.commandKw = 0;        // what the "pilot" actually asks for
    this.armed = true;
    this.scenario = "sweep";
    this.pilot = { until: 0, p: 10 };
    this.soc = 95;
    this.temps = null;  // set on the first step from the air temperature
  }

  scenarioPower() {
    switch (this.scenario) {
      case "sweep":
        return sweepPower(this.t);
      case "climbs": {
        const k = this.t % 150;
        return k < 90 ? 13 : 0;
      }
      case "pilot": {
        if (this.t >= this.pilot.until) {
          const glide = this.rand() < 0.15 && this.alt > 150;
          const high = this.alt > 1500;
          this.pilot.p = glide || high ? 0 : 4 + this.rand() * 15;
          this.pilot.until = this.t + (glide ? 15 + this.rand() * 20 : 20 + this.rand() * 40);
        }
        return this.pilot.p;
      }
      default:
        return this.throttleKw;
    }
  }

  // Advance dt seconds; returns the reading the controller would see.
  step(dt) {
    const s = this.setup, e = this.env;
    this.t += dt;
    this.commandKw = this.armed ? Math.min(s.maxPowerKw, this.scenarioPower()) : 0;

    // ESC/motor power follows the command quickly
    this.power += (this.commandKw - this.power) * Math.min(1, dt / 0.3);

    const target = stillAirClimb(s, this.power, this.alt);
    this.wMotor += (target - this.wMotor) * Math.min(1, dt / s.climbTau);
    this.airW = this.air.step(this.t, dt, e);
    this.climb = this.wMotor + this.airW;
    this.alt += this.climb * dt;
    if (this.alt <= 0) {  // on the ground
      this.alt = 0;
      this.climb = Math.max(0, this.climb);
      this.wMotor = Math.max(0, this.wMotor);
    }

    // BMS reports power at 10 Hz with a little ripple
    if (this.t - this.lastBmsT >= 0.1) {
      this.lastBmsT = this.t;
      this.powerMeasured = Math.max(0, this.power * (1 + e.powerNoise * gaussian(this.rand)));
      this.soc = Math.max(0, this.soc - this.power * 0.1 / 3600 / 4.8 * 100);
    }

    // Parts heat with P^2 and cool toward air + offset
    if (!this.temps) {
      this.temps = {};
      for (const k of Object.keys(THERMAL_PARTS)) this.temps[k] = e.airTempC;
    }
    for (const [name, m] of Object.entries(THERMAL_PARTS)) {
      const a = Math.exp(-dt / m.tau);
      const target = e.airTempC + m.baseOverAir + m.k * this.power * this.power;
      this.temps[name] = a * this.temps[name] + (1 - a) * target;
    }

    // Controller baro: altitude + arm movement + noise, through the IIR filter
    const tauHand = 2;
    this.hand += (-this.hand / tauHand) * dt +
      e.handMotion * Math.sqrt(2 * dt / tauHand) * gaussian(this.rand);
    const raw = this.alt + this.hand + e.baroNoise * gaussian(this.rand);
    const alpha = 1 - Math.pow(15 / 16, dt / 0.04);
    if (this.t <= dt) this.baroIir = raw;
    this.baroIir += (raw - this.baroIir) * alpha;

    return {
      t: this.t,
      alt: this.baroIir,
      power: this.powerMeasured,
      armed: this.armed,
      soc: this.soc,
      volts: 100.8 - (100 - this.soc) * 0.2 - this.power * 0.35,
      trueAlt: this.alt,
      trueClimb: this.climb,
      stillClimb: target,
      airW: this.airW,
      command: this.commandKw,
      temps: { ...this.temps },
    };
  }
}
