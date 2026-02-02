# 🚀 WebSocket Quick Reference

## For Developers

### Import the Hook
```typescript
import { useVideoProgress } from '@/hooks/use-video-progress'
```

### Basic Usage
```typescript
const { 
  progress,        // 0-100 (number)
  status,          // 'pending' | 'processing' | 'completed' | 'failed'
  isFinished,      // boolean
  error,           // string | null
  processingTime,  // number | null (seconds)
  isConnected      // boolean
} = useVideoProgress(jobId, enabled)
```

### Example in Component
```typescript
function VideoAnalysis() {
  const [jobId, setJobId] = useState<string | null>(null)
  const [isProcessing, setIsProcessing] = useState(false)
  
  // Hook automatically connects when jobId is set
  const { progress, status, isFinished, isConnected } = useVideoProgress(
    jobId, 
    isProcessing
  )
  
  // Watch for completion
  useEffect(() => {
    if (isFinished && status === 'completed') {
      // Fetch result video
      fetchResult(jobId)
    }
  }, [isFinished, status, jobId])
  
  return (
    <div>
      {isConnected && <Badge>WebSocket Connected</Badge>}
      <ProgressBar value={progress} />
      <p>{progress}% - {status}</p>
    </div>
  )
}
```

---

## WebSocket URL

**Production:**
```
wss://adas-api.aiotlab.edu.vn/ws/video/progress/{job_id}
```

**Development:**
```
ws://localhost:8000/ws/video/progress/{job_id}
```

---

## Message Types

### 1. Connected
```json
{
  "type": "connected",
  "job_id": "abc-123",
  "message": "WebSocket connected for job abc-123"
}
```

### 2. Progress
```json
{
  "type": "progress",
  "job_id": "abc-123",
  "status": "processing",
  "progress_percent": 45,
  "processing_time_seconds": 12,
  "result_path": null,
  "error_message": null
}
```

### 3. Finished
```json
{
  "type": "finished",
  "status": "completed",
  "message": "Job abc-123 completed"
}
```

### 4. Error
```json
{
  "type": "error",
  "message": "Job not found"
}
```

---

## Hook Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `jobId` | `string \| null` | - | Job ID from upload response |
| `enabled` | `boolean` | `true` | Whether to start connection |

---

## Hook Return Values

| Value | Type | Description |
|-------|------|-------------|
| `progress` | `number` | Progress percentage (0-100) |
| `status` | `string` | Current status |
| `isFinished` | `boolean` | Whether processing is complete |
| `error` | `string \| null` | Error message if any |
| `processingTime` | `number \| null` | Processing time in seconds |
| `isConnected` | `boolean` | WebSocket connection status |

---

## Common Patterns

### Pattern 1: Upload and Monitor
```typescript
const handleUpload = async (file: File) => {
  // 1. Upload
  const res = await fetch('/api/video/upload', {
    method: 'POST',
    body: formData
  })
  const { job_id } = await res.json()
  
  // 2. Set job ID (triggers WebSocket)
  setJobId(job_id)
  setIsProcessing(true)
}

// 3. Hook monitors automatically
const { progress, isFinished } = useVideoProgress(jobId, isProcessing)

// 4. Handle completion
useEffect(() => {
  if (isFinished) {
    setIsProcessing(false)
    fetchResult(jobId)
  }
}, [isFinished])
```

### Pattern 2: Error Handling with Fallback
```typescript
const { progress, error } = useVideoProgress(jobId, isProcessing)

useEffect(() => {
  if (error) {
    toast({ title: "WebSocket Error", description: error })
    // Fallback to polling
    pollForResult(jobId)
  }
}, [error])
```

### Pattern 3: Show Connection Status
```typescript
const { isConnected } = useVideoProgress(jobId, isProcessing)

return (
  <div>
    {isConnected ? (
      <Badge variant="success">🟢 Real-time</Badge>
    ) : (
      <Badge variant="warning">🟡 Connecting...</Badge>
    )}
  </div>
)
```

---

## Debugging

### Enable Verbose Logging
Check browser console for these logs:

```
🔌 [WebSocket] Connecting to: wss://...
✅ [WebSocket] Connected
📨 [WebSocket] Message: {...}
✅ [WebSocket] Processing finished: completed
🔌 [WebSocket] Connection closed: 1000
```

### Common Issues

**Issue:** WebSocket not connecting
```typescript
// Check if jobId is set
console.log('Job ID:', jobId)

// Check if enabled
console.log('Is Processing:', isProcessing)

// Check WebSocket URL
import { getWebSocketUrl } from '@/lib/api-config'
console.log('WS URL:', getWebSocketUrl(`ws/video/progress/${jobId}`))
```

**Issue:** Progress not updating
```typescript
// Check if hook is receiving messages
const { progress, status } = useVideoProgress(jobId, isProcessing)

useEffect(() => {
  console.log('Progress changed:', progress, status)
}, [progress, status])
```

**Issue:** Connection keeps dropping
```typescript
// Check reconnection attempts
const { isConnected, error } = useVideoProgress(jobId, isProcessing)

useEffect(() => {
  console.log('Connection status:', isConnected)
  if (error) console.error('Connection error:', error)
}, [isConnected, error])
```

---

## Testing

### Test WebSocket in Browser Console
```javascript
// 1. Connect
const ws = new WebSocket('wss://adas-api.aiotlab.edu.vn/ws/video/progress/test-id')

// 2. Listen for messages
ws.onmessage = (e) => console.log('Message:', JSON.parse(e.data))

// 3. Check connection
ws.onopen = () => console.log('Connected!')
ws.onerror = (e) => console.error('Error:', e)
ws.onclose = () => console.log('Closed')
```

### Test with wscat (CLI)
```bash
# Install
npm install -g wscat

# Connect
wscat -c 'wss://adas-api.aiotlab.edu.vn/ws/video/progress/test-id'

# You should see connection messages
```

---

## Performance Tips

### 1. Disable When Not Needed
```typescript
// Only enable when processing
const { progress } = useVideoProgress(jobId, isProcessing)
```

### 2. Cleanup on Unmount
```typescript
// Hook automatically cleans up, but you can force it:
useEffect(() => {
  return () => {
    setJobId(null)
    setIsProcessing(false)
  }
}, [])
```

### 3. Debounce UI Updates
```typescript
const { progress } = useVideoProgress(jobId, isProcessing)

// Update UI less frequently if needed
const debouncedProgress = useMemo(() => 
  Math.floor(progress / 5) * 5, // Round to nearest 5%
  [progress]
)
```

---

## API Reference

### getWebSocketUrl(endpoint: string): string
Helper function to construct WebSocket URLs.

```typescript
import { getWebSocketUrl } from '@/lib/api-config'

const url = getWebSocketUrl('ws/video/progress/abc-123')
// Returns: wss://adas-api.aiotlab.edu.vn/ws/video/progress/abc-123
```

### API_CONFIG.WS_BASE_URL
Base WebSocket URL from config.

```typescript
import { API_CONFIG } from '@/lib/api-config'

console.log(API_CONFIG.WS_BASE_URL)
// wss://adas-api.aiotlab.edu.vn
```

---

## Environment Variables

```bash
# .env.local
NEXT_PUBLIC_API_URL=https://adas-api.aiotlab.edu.vn

# WebSocket URL is automatically derived:
# https:// → wss://
# http://  → ws://
```

---

## TypeScript Types

```typescript
type ProgressData = {
  type: 'connected' | 'progress' | 'finished' | 'error'
  job_id?: string
  status?: string
  progress_percent?: number
  result_path?: string | null
  error_message?: string | null
  processing_time_seconds?: number
  message?: string
}

type UseVideoProgressReturn = {
  progress: number
  status: string
  isFinished: boolean
  error: string | null
  processingTime: number | null
  isConnected: boolean
}
```

---

## Best Practices

1. ✅ **Always check `isConnected`** before showing real-time badge
2. ✅ **Handle errors gracefully** with fallback to polling
3. ✅ **Disable hook when not processing** to save resources
4. ✅ **Show connection status** to users for transparency
5. ✅ **Log WebSocket events** in development for debugging
6. ❌ **Don't create multiple connections** for same job
7. ❌ **Don't forget to cleanup** on unmount (hook does this automatically)
8. ❌ **Don't hardcode WebSocket URLs** - use helper functions

---

## Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| Not connecting | Check `jobId` is not null, `enabled` is true |
| No progress updates | Check backend is sending messages |
| Connection drops | Check network, firewall, proxy settings |
| High CPU usage | Disable hook when not processing |
| Memory leak | Hook auto-cleans up, but verify unmount |

---

**Last Updated:** 2026-01-11  
**Version:** 1.0.0  
**Author:** Antigravity AI Assistant
