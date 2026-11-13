import { useCallback, useEffect, useRef, useState } from 'react'
import { runSimulation } from './simulatorApi.js'

const baselineEvents = [
  { at_ms: 0, action: 'mode', value: 'sport' },
  { at_ms: 0, action: 'throttle', value: '0' },
]

export function useSimulation() {
  const [snapshot, setSnapshot] = useState(null)
  const [events, setEvents] = useState(baselineEvents)
  const [durationMs, setDurationMs] = useState(1000)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')
  const requestId = useRef(0)

  const execute = useCallback(async (nextEvents, nextDuration) => {
    const id = ++requestId.current
    setLoading(true)
    setError('')
    try {
      const result = await runSimulation(nextEvents, nextDuration)
      if (id === requestId.current) setSnapshot(result)
      return result
    } catch (requestError) {
      if (id === requestId.current) setError(requestError.message)
      throw requestError
    } finally {
      if (id === requestId.current) setLoading(false)
    }
  }, [])

  useEffect(() => {
    execute(baselineEvents, 1000).catch(() => {})
  }, [execute])

  const applyEvent = useCallback(async (action, value, settleMs = 500) => {
    const atMs = durationMs + 10
    const nextEvents = [...events, { at_ms: atMs, action, value: String(value) }]
    const nextDuration = atMs + settleMs
    setEvents(nextEvents)
    setDurationMs(nextDuration)
    return execute(nextEvents, nextDuration)
  }, [durationMs, events, execute])

  const applyEvents = useCallback(async (updates, settleMs = 500) => {
    const atMs = durationMs + 10
    const nextEvents = [
      ...events,
      ...updates.map(({ action, value }) => ({
        at_ms: atMs,
        action,
        value: String(value),
      })),
    ]
    const nextDuration = atMs + settleMs
    setEvents(nextEvents)
    setDurationMs(nextDuration)
    return execute(nextEvents, nextDuration)
  }, [durationMs, events, execute])

  const advance = useCallback(async (elapsedMs = 1000) => {
    const nextDuration = durationMs + elapsedMs
    setDurationMs(nextDuration)
    return execute(events, nextDuration)
  }, [durationMs, events, execute])

  const reset = useCallback(() => {
    setEvents(baselineEvents)
    setDurationMs(1000)
    return execute(baselineEvents, 1000)
  }, [execute])

  return { snapshot, loading, error, durationMs, applyEvent, applyEvents, advance, reset }
}
