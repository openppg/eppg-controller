import { execFile } from 'node:child_process'
import { createReadStream, existsSync, promises as fs } from 'node:fs'
import http from 'node:http'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { promisify } from 'node:util'

const execFileAsync = promisify(execFile)
const uiRoot = path.dirname(fileURLToPath(import.meta.url))
const projectRoot = path.resolve(uiRoot, '../..')
const rustBinary = path.join(projectRoot, 'rust/target/debug/controller-sim')
const backend = process.env.EPPG_SIM_BACKEND || (existsSync(rustBinary) ? 'rust' : 'native')
if (!['rust', 'native'].includes(backend)) throw new Error('EPPG_SIM_BACKEND must be rust or native')
const simulatorBinary = process.env.EPPG_SIMULATOR_BINARY || (backend === 'rust'
  ? rustBinary : path.join(projectRoot, 'build-simulator/eppg-sim'))
const framebufferBinary = process.env.EPPG_FRAMEBUFFER_BINARY || [
  'build-host-screenshots/framebuffer_renderer',
  'build-host-screenshot/framebuffer_renderer',
  'build-screenshot/framebuffer_renderer',
].map((file) => path.join(projectRoot, file)).find(existsSync) ||
  path.join(projectRoot, 'build-host-screenshots/framebuffer_renderer')
const screenRoot = path.join(projectRoot, 'test/test_screenshots/reference')
const distRoot = path.join(uiRoot, 'dist')
const port = Number(process.env.OPENPPG_UI_PORT || 4174)

const allowedActions = new Set([
  'throttle', 'arm', 'cruise', 'mode', 'altitude', 'esc_link', 'bms_link',
  'barometer_link', 'soc', 'bms_temp', 'esc_temp', 'cpu_temp',
  'esc_rpm', 'esc_amps', 'esc_voltage', 'bms_voltage', 'bms_current',
  'button', 'role', 'maintenance', 'wireless_control',
  'stall_throttle', 'esc_drop_every', 'bms_drop_every',
  'esc_corrupt_every', 'bms_corrupt_every', 'esc_bus_off', 'bms_bus_off',
])

const mimeTypes = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.bmp': 'image/bmp',
  '.json': 'application/json; charset=utf-8',
}

function json(res, status, body) {
  res.writeHead(status, { 'Content-Type': mimeTypes['.json'] })
  res.end(JSON.stringify(body))
}

async function readBody(req) {
  const chunks = []
  let size = 0
  for await (const chunk of req) {
    size += chunk.length
    if (size > 1_000_000) throw new Error('request body is too large')
    chunks.push(chunk)
  }
  return JSON.parse(Buffer.concat(chunks).toString('utf8'))
}

function validateSimulation(input) {
  if (!Array.isArray(input.events)) throw new Error('events must be an array')
  if (!Number.isInteger(input.duration_ms) || input.duration_ms < 0 ||
      input.duration_ms > 3_600_000) {
    throw new Error('duration_ms must be an integer between 0 and 3600000')
  }
  let previous = 0
  const rows = input.events.map((event, index) => {
    if (!Number.isInteger(event.at_ms) || event.at_ms < previous ||
        event.at_ms > input.duration_ms) {
      throw new Error(`event ${index} has invalid time ordering`)
    }
    if (!allowedActions.has(event.action)) {
      throw new Error(`event ${index} has unsupported action`)
    }
    const value = String(event.value)
    if (/[,\r\n]/.test(value)) throw new Error(`event ${index} has invalid value`)
    previous = event.at_ms
    return `${event.at_ms},${event.action},${value}`
  })
  return rows.join('\n') + '\n'
}

async function simulate(req, res) {
  let tempDirectory
  try {
    const input = await readBody(req)
    const scenario = validateSimulation(input)
    await fs.access(simulatorBinary)
    tempDirectory = await fs.mkdtemp(path.join(os.tmpdir(), 'openppg-ui-'))
    const scenarioPath = path.join(tempDirectory, 'scenario.csv')
    await fs.writeFile(scenarioPath, scenario)
    const framebufferPath = path.join(tempDirectory, 'framebuffer.bmp')
    const { stdout } = await execFileAsync(simulatorBinary, [
      '--scenario', scenarioPath,
      '--duration', String(input.duration_ms),
      '--quiet',
      ...(backend === 'rust' ? ['--framebuffer', framebufferPath] : []),
    ], { cwd: projectRoot, maxBuffer: 2_000_000 })
    const snapshot = JSON.parse(stdout.trim())
    if (backend === 'native') {
    await fs.access(framebufferBinary)
    const modeEvent = [...input.events].reverse().find((event) => event.action === 'mode')
    const mode = modeEvent?.value === 'chill' ? 'chill' : 'sport'
    let armedStartMs = 0
    let armedByInput = false
    for (const event of input.events) {
      if (event.action !== 'arm') continue
      if (['1', 'true', 'on'].includes(String(event.value))) {
        armedStartMs = event.at_ms
        armedByInput = true
      } else {
        armedByInput = false
      }
    }
    const armedElapsedMs = snapshot.state !== 'DISARMED' && armedByInput
      ? Math.max(0, snapshot.time_ms - armedStartMs)
      : 0
    await execFileAsync(framebufferBinary, [
      framebufferPath,
      snapshot.state,
      mode,
      String(armedElapsedMs),
      snapshot.esc.connected ? '1' : '0',
      String(snapshot.esc.voltage_v),
      String(snapshot.esc.rpm),
      String(snapshot.esc.amps),
      String(snapshot.esc.mos_temp_c),
      snapshot.bms.connected ? '1' : '0',
      String(snapshot.bms.voltage_v),
      String(snapshot.bms.current_a),
      String(snapshot.bms.soc_pct),
      String(snapshot.bms.temp_c),
      String(snapshot.barometer.altitude_m),
      String(snapshot.barometer.vertical_speed_mps),
    ], { cwd: projectRoot, maxBuffer: 2_000_000 })
    }
    snapshot.backend ||= backend === 'rust' ? 'rust-production-core' : 'native-reference'
    const framebuffer = await fs.readFile(framebufferPath)
    snapshot.framebuffer = {
      width: 160,
      height: 128,
      format: 'bmp',
      data_url: `data:image/bmp;base64,${framebuffer.toString('base64')}`,
    }
    json(res, 200, snapshot)
  } catch (error) {
    const missingBinary = error?.code === 'ENOENT'
    json(res, missingBinary ? 503 : 400, {
      error: missingBinary
        ? 'The selected simulator or renderer is not built. See simulator/README.md for native and Rust build commands.'
        : error.message,
    })
  } finally {
    if (tempDirectory) await fs.rm(tempDirectory, { recursive: true, force: true })
  }
}

function safeAssetPath(root, requestPath, fallback = 'index.html') {
  const relative = decodeURIComponent(requestPath).replace(/^\/+/, '') || fallback
  const resolved = path.resolve(root, relative)
  return resolved.startsWith(`${root}${path.sep}`) || resolved === root
    ? resolved
    : null
}

async function serveFile(res, filePath) {
  try {
    const stat = await fs.stat(filePath)
    if (!stat.isFile()) return false
    const extension = path.extname(filePath).toLowerCase()
    res.writeHead(200, {
      'Content-Type': mimeTypes[extension] || 'application/octet-stream',
      'Content-Length': stat.size,
      'Cache-Control': extension === '.html' ? 'no-cache' : 'public, max-age=3600',
    })
    createReadStream(filePath).pipe(res)
    return true
  } catch {
    return false
  }
}

const server = http.createServer(async (req, res) => {
  const requestUrl = new URL(req.url, `http://${req.headers.host || 'localhost'}`)
  if (req.method === 'GET' && requestUrl.pathname === '/api/health') {
    json(res, 200, { backend, simulator: path.basename(simulatorBinary), ready: existsSync(simulatorBinary) })
    return
  }
  if (req.method === 'POST' && requestUrl.pathname === '/api/simulate') {
    await simulate(req, res)
    return
  }
  if (req.method !== 'GET' && req.method !== 'HEAD') {
    json(res, 405, { error: 'method not allowed' })
    return
  }
  if (requestUrl.pathname.startsWith('/screen/')) {
    const filename = path.basename(requestUrl.pathname)
    if (!/^main_[a-z_]+\.bmp$/.test(filename) ||
        !(await serveFile(res, path.join(screenRoot, filename)))) {
      json(res, 404, { error: 'screen frame not found' })
    }
    return
  }
  const asset = safeAssetPath(distRoot, requestUrl.pathname)
  if (asset && await serveFile(res, asset)) return
  if (await serveFile(res, path.join(distRoot, 'index.html'))) return
  json(res, 503, { error: 'UI is not built. Run npm run build.' })
})

server.listen(port, '127.0.0.1', () => {
  console.log(`OpenPPG Hardware Lab listening on http://127.0.0.1:${server.address().port}`)
})
