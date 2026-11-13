import test from 'node:test'
import assert from 'node:assert/strict'
import { spawn } from 'node:child_process'
import { existsSync } from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const root = path.resolve(here, '../..')
const rustBinary = path.join(root, 'rust/target/debug/controller-sim')
const backend = process.env.EPPG_SIM_BACKEND || (existsSync(rustBinary) ? 'rust' : 'native')
const binary = process.env.EPPG_SIMULATOR_BINARY || (backend === 'rust'
  ? rustBinary : path.join(root, 'build-simulator/eppg-sim'))

test('hardware lab executes production controller and renders a live framebuffer', {
  skip: !existsSync(binary) ? 'build the selected simulator binary first' : false,
}, async (t) => {
  const child = spawn(process.execPath, [path.join(here, 'server.mjs')], {
    env: { ...process.env, OPENPPG_UI_PORT: '0', EPPG_SIM_BACKEND: backend, EPPG_SIMULATOR_BINARY: binary },
    stdio: ['ignore', 'pipe', 'pipe'],
  })
  t.after(() => child.kill('SIGTERM'))
  const base = await new Promise((resolve, reject) => {
    let output = ''
    const timer = setTimeout(() => reject(new Error(`server startup timed out: ${output}`)), 5000)
    child.once('exit', (code) => { clearTimeout(timer); reject(new Error(`server exited ${code}: ${output}`)) })
    child.stderr.on('data', (chunk) => { output += chunk })
    child.stdout.on('data', (chunk) => {
      output += chunk
      const match = output.match(/http:\/\/127\.0\.0\.1:\d+/)
      if (match) { clearTimeout(timer); resolve(match[0]) }
    })
  })
  const health = await (await fetch(`${base}/api/health`)).json()
  assert.equal(health.backend, backend)
  assert.equal(health.ready, true)
  const events = [
    { at_ms: 0, action: 'mode', value: 'sport' },
    { at_ms: 600, action: 'arm', value: 'on' },
    { at_ms: 800, action: 'throttle', value: '2000' },
  ]
  const post = (payload) => fetch(`${base}/api/simulate`, {
    method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(payload),
  })
  const response = await post({ events, duration_ms: 1500 })
  assert.equal(response.status, 200)
  const result = await response.json()
  assert.equal(result.state, 'ARMED')
  assert.equal(result.esc.connected, true)
  assert.ok(result.esc.rpm > 1000)
  assert.ok(result.io.can_rx_frames > 0)
  assert.equal(result.framebuffer.width, 160)
  assert.equal(result.framebuffer.height, 128)
  const bmp = Buffer.from(result.framebuffer.data_url.split(',')[1], 'base64')
  assert.equal(bmp.subarray(0, 2).toString(), 'BM')
  assert.equal(bmp.readUInt32LE(18), 160)
  assert.equal(bmp.readUInt32LE(22), 128)
  assert.ok(bmp.subarray(54).some((byte) => byte !== 0))
  const failed = await post({ events: [...events, { at_ms: 1600, action: 'esc_drop_every', value: '1' }], duration_ms: 2400 })
  assert.equal(failed.status, 200)
  const fault = await failed.json()
  assert.equal(fault.esc.connected, false)
  assert.ok(fault.alerts.includes('ESC_CAN_LINK_LOST'))
  const invalid = await post({ events, duration_ms: 3_600_001 })
  assert.equal(invalid.status, 400)
})
