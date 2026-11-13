export async function runSimulation(events, durationMs) {
  const response = await fetch('/api/simulate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ events, duration_ms: durationMs }),
  })
  const result = await response.json()
  if (!response.ok) throw new Error(result.error || 'Simulation failed')
  return result
}
