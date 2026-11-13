import { spawn } from 'node:child_process'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const uiRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')
const children = [
  spawn(process.execPath, ['server.mjs'], { cwd: uiRoot, stdio: 'inherit' }),
  spawn(process.platform === 'win32' ? 'npx.cmd' : 'npx', ['vite'], {
    cwd: uiRoot,
    stdio: 'inherit',
  }),
]

function shutdown(signal = 'SIGTERM') {
  for (const child of children) child.kill(signal)
}

process.on('SIGINT', () => shutdown('SIGINT'))
process.on('SIGTERM', () => shutdown('SIGTERM'))
for (const child of children) {
  child.on('exit', (code) => {
    if (code && code !== 0) process.exitCode = code
    shutdown()
  })
}
