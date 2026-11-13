import React, { useCallback, useEffect, useRef, useState } from 'react'
import { CanIcon, ResetIcon, WarningIcon } from './icons.jsx'
import { useSimulation } from './useSimulation.js'

const fallbackScreenForState = {
  DISARMED: '/screen/main_idle_light.bmp',
  ARMED: '/screen/main_armed_flying_light.bmp',
  ARMED_CRUISING: '/screen/main_armed_cruising_light.bmp',
}

function formatNumber(value, digits = 1) {
  return Number(value || 0).toLocaleString(undefined, {
    minimumFractionDigits: digits,
    maximumFractionDigits: digits,
  })
}

function Panel({ title, children, className = '' }) {
  return <section className={`panel ${className}`}><h2>{title}</h2>{children}</section>
}

function ThrottleControl({ value, onChange }) {
  const [draft, setDraft] = useState(value)
  const liveCommit = useRef(null)
  useEffect(() => setDraft(value), [value])
  useEffect(() => () => window.clearTimeout(liveCommit.current), [])
  const previewValue = (nextValue) => {
    setDraft(nextValue)
    window.clearTimeout(liveCommit.current)
    liveCommit.current = window.setTimeout(() => onChange(nextValue), 120)
  }
  const commitValue = (nextValue) => {
    window.clearTimeout(liveCommit.current)
    setDraft(nextValue)
    onChange(nextValue)
  }
  const valueAtPointer = (event) => {
    const rect = event.currentTarget.getBoundingClientRect()
    const fraction = Math.max(0, Math.min(1, (rect.bottom - event.clientY) / rect.height))
    return Math.round(fraction * 4095)
  }
  return (
    <div className="throttle-control">
      <div className="control-label"><span>THROTTLE</span><output>{draft} / 4095</output></div>
      <div className="throttle-scale" aria-hidden="true"><span>4095</span><span>2048</span><span>0</span></div>
      <input
        aria-label="Throttle input"
        className="throttle-slider"
        type="range"
        min="0"
        max="4095"
        step="1"
        value={draft}
        onChange={(event) => previewValue(Number(event.currentTarget.value))}
        onPointerDown={(event) => {
          event.currentTarget.setPointerCapture(event.pointerId)
          previewValue(valueAtPointer(event))
        }}
        onPointerMove={(event) => {
          if (event.currentTarget.hasPointerCapture(event.pointerId)) {
            previewValue(valueAtPointer(event))
          }
        }}
        onPointerUp={(event) => {
          const nextValue = valueAtPointer(event)
          event.currentTarget.releasePointerCapture(event.pointerId)
          commitValue(nextValue)
        }}
        onKeyUp={(event) => commitValue(Number(event.currentTarget.value))}
      />
    </div>
  )
}

function ControllerButton({ state, onArm, onCruise, onNotice }) {
  const [pressed, setPressed] = useState(false)
  const [primed, setPrimed] = useState(false)
  const [progress, setProgress] = useState(0)
  const pressStarted = useRef(0)
  const holdTimer = useRef(null)
  const primeTimer = useRef(null)
  const fired = useRef(false)
  const pressedRef = useRef(false)
  const primedRef = useRef(false)
  const stateRef = useRef(state)
  const actionsRef = useRef({ onArm, onCruise, onNotice })
  stateRef.current = state
  actionsRef.current = { onArm, onCruise, onNotice }

  const updatePressed = (value) => {
    pressedRef.current = value
    setPressed(value)
  }
  const updatePrimed = (value) => {
    primedRef.current = value
    setPrimed(value)
  }

  const clearHold = useCallback(() => {
    window.clearInterval(holdTimer.current)
    holdTimer.current = null
    setProgress(0)
  }, [])

  const beginPress = useCallback(() => {
    if (pressedRef.current) return
    updatePressed(true)
    fired.current = false
    pressStarted.current = performance.now()
    const requiresHold = stateRef.current !== 'DISARMED' || primedRef.current
    if (requiresHold) {
      window.clearTimeout(primeTimer.current)
      holdTimer.current = window.setInterval(() => {
        const elapsed = performance.now() - pressStarted.current
        setProgress(Math.min(1, elapsed / 2000))
        if (elapsed >= 2000 && !fired.current) {
          fired.current = true
          clearHold()
          if (stateRef.current === 'DISARMED') {
            updatePrimed(false)
            actionsRef.current.onNotice('Arm sequence complete')
            actionsRef.current.onArm()
          } else {
            actionsRef.current.onNotice(stateRef.current === 'ARMED' ? 'Cruise engaged' : 'Cruise released')
            actionsRef.current.onCruise(stateRef.current === 'ARMED')
          }
        }
      }, 32)
    }
  }, [clearHold])

  const endPress = useCallback(() => {
    if (!pressedRef.current) return
    const elapsed = performance.now() - pressStarted.current
    updatePressed(false)
    clearHold()
    if (!fired.current && stateRef.current === 'DISARMED' &&
        !primedRef.current && elapsed < 500) {
      updatePrimed(true)
      actionsRef.current.onNotice('Arm primed · hold button for 2 seconds')
      window.clearTimeout(primeTimer.current)
      primeTimer.current = window.setTimeout(() => {
        if (!pressedRef.current) {
          updatePrimed(false)
          actionsRef.current.onNotice('Arm sequence timed out')
        }
      }, 1500)
    }
  }, [clearHold])

  useEffect(() => {
    const keyDown = (event) => {
      if (event.code === 'Space' && !event.repeat) {
        event.preventDefault()
        beginPress()
      }
    }
    const keyUp = (event) => {
      if (event.code === 'Space') {
        event.preventDefault()
        endPress()
      }
    }
    window.addEventListener('keydown', keyDown)
    window.addEventListener('keyup', keyUp)
    return () => {
      window.removeEventListener('keydown', keyDown)
      window.removeEventListener('keyup', keyUp)
      window.clearInterval(holdTimer.current)
      window.clearTimeout(primeTimer.current)
    }
  }, [beginPress, endPress])

  return (
    <div className="button-control">
      <div className="control-label"><span>CONTROLLER BUTTON</span><span>{primed ? 'ARM PRIMED' : 'CLICK · THEN HOLD'}</span></div>
      <button
        className={`hardware-button ${pressed ? 'is-pressed' : ''} ${primed ? 'is-primed' : ''}`}
        type="button"
        aria-label="Controller button"
        aria-pressed={pressed}
        onPointerDown={beginPress}
        onPointerUp={endPress}
        onPointerCancel={endPress}
        onContextMenu={(event) => event.preventDefault()}
      >
        <span className="hold-progress" style={{ '--progress': `${progress * 360}deg` }} />
        <span className="button-core"><span className="button-target" /></span>
      </button>
      <p>{state === 'DISARMED' ? 'Quick click, then hold 2s to arm' : 'Hold 2s to toggle cruise'}</p>
    </div>
  )
}

function Device({ snapshot, pressed }) {
  const state = snapshot?.state || 'DISARMED'
  const screenSource = snapshot?.framebuffer?.data_url || fallbackScreenForState[state]
  return (
    <div className={`device-stage ${pressed ? 'button-active' : ''}`}>
      <div className="device">
        <div className="live-screen-bezel">
          <img className="live-screen" src={screenSource} alt={`Live LVGL framebuffer · ${state}`} />
        </div>
        <img className="device-chassis" src="/device-chassis.png" alt="OpenPPG single-button controller enclosure" />
      </div>
      <div className="device-caption"><span>LIVE PRODUCTION LVGL · 160×128</span><strong>{state.replace('_', ' ')}</strong></div>
    </div>
  )
}

function Telemetry({ snapshot }) {
  const rows = [
    ['ESC RPM', formatNumber(snapshot?.esc?.rpm, 0), 'rpm'],
    ['ESC CURRENT', formatNumber(snapshot?.esc?.amps), 'A'],
    ['ESC VOLTAGE', formatNumber(snapshot?.esc?.voltage_v), 'V'],
    ['BMS VOLTAGE', formatNumber(snapshot?.bms?.voltage_v), 'V'],
    ['BMS CURRENT', formatNumber(snapshot?.bms?.current_a), 'A'],
    ['STATE OF CHARGE', formatNumber(snapshot?.bms?.soc_pct, 0), '%'],
    ['ALTITUDE', formatNumber(snapshot?.barometer?.altitude_m), 'm'],
    ['ESC MOS', formatNumber(snapshot?.esc?.mos_temp_c), '°C'],
  ]
  return <dl className="telemetry-list">{rows.map(([label, value, unit]) => <div key={label}><dt>{label}</dt><dd>{value} <small>{unit}</small></dd></div>)}</dl>
}

function NumberField({ label, value, onChange, min, max, step = '0.1', unit }) {
  return <label className="telemetry-field"><span>{label}</span><div><input type="number" value={value} min={min} max={max} step={step} onChange={(event) => onChange(event.target.value)} /><small>{unit}</small></div></label>
}

function TelemetryModal({ snapshot, onClose, onApply }) {
  const [form, setForm] = useState(() => ({
    escRpm: String(Math.round(snapshot?.esc?.rpm || 0)),
    escAmps: (snapshot?.esc?.amps ?? 0).toFixed(1),
    escVoltage: (snapshot?.esc?.voltage_v ?? 100.8).toFixed(1),
    escTemp: (snapshot?.esc?.mos_temp_c ?? 25).toFixed(1),
    escLink: snapshot?.esc?.connected !== false,
    bmsVoltage: (snapshot?.bms?.voltage_v ?? 97.9).toFixed(1),
    bmsCurrent: (snapshot?.bms?.current_a ?? 0).toFixed(1),
    bmsSoc: String(Math.round(snapshot?.bms?.soc_pct ?? 90)),
    bmsTemp: (snapshot?.bms?.temp_c ?? 25).toFixed(1),
    bmsLink: snapshot?.bms?.connected !== false,
  }))
  const update = (name) => (value) => setForm((current) => ({ ...current, [name]: value }))
  useEffect(() => {
    const closeOnEscape = (event) => { if (event.key === 'Escape') onClose() }
    window.addEventListener('keydown', closeOnEscape)
    return () => window.removeEventListener('keydown', closeOnEscape)
  }, [onClose])
  const submit = async (event) => {
    event.preventDefault()
    await onApply([
      { action: 'esc_rpm', value: form.escRpm },
      { action: 'esc_amps', value: form.escAmps },
      { action: 'esc_voltage', value: form.escVoltage },
      { action: 'esc_temp', value: form.escTemp },
      { action: 'esc_link', value: form.escLink ? 'on' : 'off' },
      { action: 'bms_voltage', value: form.bmsVoltage },
      { action: 'bms_current', value: form.bmsCurrent },
      { action: 'soc', value: form.bmsSoc },
      { action: 'bms_temp', value: form.bmsTemp },
      { action: 'bms_link', value: form.bmsLink ? 'on' : 'off' },
    ])
    onClose()
  }
  return <div className="modal-backdrop" role="presentation" onPointerDown={(event) => { if (event.target === event.currentTarget) onClose() }}><section className="telemetry-modal" role="dialog" aria-modal="true" aria-labelledby="telemetry-modal-title"><header><div><span className="eyebrow">WIRE-LEVEL OVERRIDES</span><h2 id="telemetry-modal-title">Inject ESC &amp; BMS Telemetry</h2></div><button type="button" className="modal-close" aria-label="Close telemetry injection" onClick={onClose}>×</button></header><p className="modal-copy">Values are encoded onto the virtual CAN buses, then decoded by the same SINE ESC and ANT BMS libraries used by the controller.</p><form onSubmit={submit}><div className="telemetry-columns"><fieldset><legend><span className={`status-dot ${form.escLink ? '' : 'is-offline'}`} />ESC · DRONECAN</legend><label className="link-toggle"><input type="checkbox" checked={form.escLink} onChange={(event) => update('escLink')(event.target.checked)} />Physical link online</label><NumberField label="RPM" value={form.escRpm} onChange={update('escRpm')} min="0" max="100000" step="1" unit="rpm" /><NumberField label="BUS CURRENT" value={form.escAmps} onChange={update('escAmps')} min="-500" max="500" unit="A" /><NumberField label="BUS VOLTAGE" value={form.escVoltage} onChange={update('escVoltage')} min="0" max="200" unit="V" /><NumberField label="MOS TEMP" value={form.escTemp} onChange={update('escTemp')} min="-40" max="200" unit="°C" /></fieldset><fieldset><legend><span className={`status-dot ${form.bmsLink ? '' : 'is-offline'}`} />BMS · ANT CAN</legend><label className="link-toggle"><input type="checkbox" checked={form.bmsLink} onChange={(event) => update('bmsLink')(event.target.checked)} />Physical link online</label><NumberField label="PACK VOLTAGE" value={form.bmsVoltage} onChange={update('bmsVoltage')} min="0" max="200" unit="V" /><NumberField label="PACK CURRENT" value={form.bmsCurrent} onChange={update('bmsCurrent')} min="-500" max="500" unit="A" /><NumberField label="STATE OF CHARGE" value={form.bmsSoc} onChange={update('bmsSoc')} min="0" max="100" step="1" unit="%" /><NumberField label="PACK TEMP" value={form.bmsTemp} onChange={update('bmsTemp')} min="-40" max="150" unit="°C" /></fieldset></div><footer><button type="button" className="modal-cancel" onClick={onClose}>CANCEL</button><button type="submit" className="modal-apply">APPLY TO CAN WIRE</button></footer></form></section></div>
}

function BusRow({ name, bitrate, bus, connected }) {
  const errors = (bus?.dropped || 0) + (bus?.corrupted || 0)
  return (
    <div className="bus-row">
      <div className="bus-heading"><span className={`status-dot ${connected ? '' : 'is-offline'}`} /><strong>{name} · {bitrate}</strong><span>{connected ? 'ACTIVE' : 'OFFLINE'}</span></div>
      <div className="bus-stats"><span>TX {bus?.tx_frames || 0}</span><span>RX {bus?.rx_frames || 0}</span><span>ERR {errors}</span></div>
    </div>
  )
}

function FaultControls({ target, setTarget, onApply, busOff }) {
  const [drop, setDrop] = useState('0')
  const [corrupt, setCorrupt] = useState('0')
  return (
    <div className="fault-controls">
      <div className="target-switch" aria-label="Fault target">
        {['esc', 'bms'].map((name) => <button key={name} className={target === name ? 'active' : ''} type="button" onClick={() => setTarget(name)}>{name.toUpperCase()}</button>)}
      </div>
      <label><span>DROP EVERY</span><select value={drop} onChange={(event) => { setDrop(event.target.value); onApply('drop', event.target.value) }}><option value="0">NONE</option><option value="5">5TH FRAME</option><option value="2">2ND FRAME</option><option value="1">ALL</option></select></label>
      <label><span>CORRUPT EVERY</span><select value={corrupt} onChange={(event) => { setCorrupt(event.target.value); onApply('corrupt', event.target.value) }}><option value="0">NONE</option><option value="5">5TH FRAME</option><option value="2">2ND FRAME</option><option value="1">ALL</option></select></label>
      <button className={`bus-off-button ${busOff ? 'active' : ''}`} type="button" onClick={() => onApply('bus_off', busOff ? 'off' : 'on')}><WarningIcon />{busOff ? 'RECOVER BUS' : 'TRIGGER BUS OFF'}</button>
    </div>
  )
}

export function App() {
  const { snapshot, loading, error, durationMs, applyEvent, applyEvents, advance, reset } = useSimulation()
  const [throttle, setThrottle] = useState(0)
  const [mode, setMode] = useState('sport')
  const [notice, setNotice] = useState('Controller online')
  const [buttonPulse, setButtonPulse] = useState(false)
  const [faultTarget, setFaultTarget] = useState('esc')
  const [busOff, setBusOff] = useState({ esc: false, bms: false })
  const [telemetryOpen, setTelemetryOpen] = useState(false)

  const state = snapshot?.state || 'DISARMED'
  useEffect(() => {
    if (state === 'DISARMED' || loading || error) return undefined
    const timer = window.setTimeout(() => advance(1000).catch(() => {}), 1000)
    return () => window.clearTimeout(timer)
  }, [advance, durationMs, error, loading, state])
  const commitThrottle = async (value) => {
    setThrottle(value)
    await applyEvent('throttle', value)
    setNotice(`Throttle sampled · ${value} / 4095`)
  }
  const arm = async () => {
    setButtonPulse(true)
    await applyEvent('arm', state === 'DISARMED' ? 'on' : 'off', 700)
    setButtonPulse(false)
  }
  const cruise = async (enabled) => {
    setButtonPulse(true)
    await applyEvent('cruise', enabled ? 'on' : 'off', 700)
    setButtonPulse(false)
  }
  const changeMode = async (nextMode) => {
    setMode(nextMode)
    await applyEvent('mode', nextMode)
    setNotice(`Drive mode · ${nextMode.toUpperCase()}`)
  }
  const applyFault = async (kind, value) => {
    const action = `${faultTarget}_${kind}`
    if (kind === 'bus_off') {
      setBusOff((current) => ({ ...current, [faultTarget]: value === 'on' }))
    }
    await applyEvent(action, value, kind === 'bus_off' && value === 'off' ? 450 : 700)
    setNotice(`${faultTarget.toUpperCase()} ${kind.replace('_', ' ')} · ${value.toUpperCase()}`)
  }
  const doReset = async () => {
    setThrottle(0)
    setMode('sport')
    setBusOff({ esc: false, bms: false })
    setNotice('Simulation reset')
    await reset()
  }
  const injectTelemetry = async (updates) => {
    await applyEvents(updates, 700)
    setNotice('Wire telemetry injected · firmware frame refreshed')
  }

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand-mark"><CanIcon /></div>
        <h1>OpenPPG Hardware Lab</h1>
        <div className="run-state"><span className={`status-dot ${error ? 'is-offline' : ''}`} />{loading ? 'SIMULATING' : error ? 'SIM ERROR' : 'SIM RUNNING'}</div>
        <button className="inject-button" type="button" onClick={() => setTelemetryOpen(true)}><CanIcon />INJECT TELEMETRY</button>
        <button className="reset-button" type="button" onClick={doReset}><ResetIcon />RESET</button>
      </header>

      <div className="workspace">
        <Panel title="Inputs" className="inputs-panel">
          <ThrottleControl value={throttle} onChange={commitThrottle} />
          <div className="mode-control"><div className="control-label"><span>DRIVE MODE</span></div><div className="segmented">{['sport', 'chill'].map((name) => <button className={mode === name ? 'active' : ''} type="button" key={name} onClick={() => changeMode(name)}>{name.toUpperCase()}</button>)}</div></div>
          <ControllerButton state={state} onArm={arm} onCruise={cruise} onNotice={setNotice} />
        </Panel>

        <section className="center-stage">
          <Device snapshot={snapshot} pressed={buttonPulse} />
          <div className="event-strip"><span className="event-time">{(durationMs / 1000).toFixed(3)}s</span><strong>{notice}</strong><span className="live-label">LIVE</span></div>
        </section>

        <aside className="right-rail">
          <Panel title="Live Telemetry"><Telemetry snapshot={snapshot} /></Panel>
          <Panel title="CAN Bus Health">
            <BusRow name="ESC" bitrate="1 Mbps" bus={snapshot?.can?.esc} connected={snapshot?.esc?.connected} />
            <BusRow name="BMS" bitrate="250 kbps" bus={snapshot?.can?.bms} connected={snapshot?.bms?.connected} />
          </Panel>
          <Panel title="Fault Injection"><FaultControls target={faultTarget} setTarget={setFaultTarget} onApply={applyFault} busOff={busOff[faultTarget]} /></Panel>
        </aside>
      </div>

      <footer className="statusbar">
        <div><span>VIRTUAL TIME</span><strong>{(durationMs / 1000).toFixed(3)} s</strong></div>
        <div><span>TX FRAMES</span><strong>{snapshot?.io?.can_tx_frames?.toLocaleString() || '0'}</strong></div>
        <div><span>RX FRAMES</span><strong>{snapshot?.io?.can_rx_frames?.toLocaleString() || '0'}</strong></div>
        <div><span>ERRORS</span><strong>{snapshot ? snapshot.io.can_dropped_frames + snapshot.io.can_corrupted_frames : 0}</strong></div>
        <div className="shortcut"><kbd>SPACE</kbd><strong>PRESS BUTTON</strong></div>
      </footer>
      {error ? <div className="error-banner" role="alert">{error}</div> : null}
      {telemetryOpen ? <TelemetryModal snapshot={snapshot} onClose={() => setTelemetryOpen(false)} onApply={injectTelemetry} /> : null}
    </main>
  )
}
