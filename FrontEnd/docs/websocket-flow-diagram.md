# 🔄 WebSocket Flow Diagram

## Upload & Processing Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                         USER UPLOADS VIDEO                       │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Frontend: uploadAndAnalyze()                                    │
│  - Create FormData with video file                               │
│  - POST to /api/video/upload                                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Backend Response                                                │
│  { job_id: "abc-123-xyz" }                                       │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Frontend: setCurrentJobId(job_id)                               │
│  - Triggers useVideoProgress hook                                │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  useVideoProgress Hook                                           │
│  - Constructs WebSocket URL                                      │
│  - wss://adas-api.aiotlab.edu.vn/ws/video/progress/abc-123-xyz  │
│  - Creates WebSocket connection                                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  WebSocket Connection Established                                │
│  Backend sends: { type: "connected", message: "..." }            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Backend Processing Video                                        │
│  Sends periodic updates:                                         │
│  { type: "progress", progress_percent: 10, status: "processing" }│
│  { type: "progress", progress_percent: 25, status: "processing" }│
│  { type: "progress", progress_percent: 50, status: "processing" }│
│  { type: "progress", progress_percent: 75, status: "processing" }│
│  { type: "progress", progress_percent: 95, status: "processing" }│
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Frontend: useEffect watches wsProgress                          │
│  - Updates processingProgress state                              │
│  - Updates processingMsg with time                               │
│  - UI re-renders with new progress                               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Processing Complete                                             │
│  Backend sends: { type: "finished", status: "completed" }        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Frontend: wsIsFinished = true                                   │
│  - Calls fetchProcessedVideo(jobId)                              │
│  - GET /api/video/result/{job_id}                                │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Construct Result Video URL                                      │
│  /api/video/download/{job_id}/{filename}_result.mp4              │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Display Result                                                  │
│  - Show completion dialog                                        │
│  - Load video in player                                          │
│  - User can watch analyzed video                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## WebSocket Message Flow

```
Frontend                          Backend
   │                                 │
   │──── WebSocket Connect ─────────>│
   │                                 │
   │<──── type: "connected" ─────────│
   │                                 │
   │                                 │ (Processing starts)
   │                                 │
   │<──── type: "progress" 10% ──────│
   │                                 │
   │<──── type: "progress" 25% ──────│
   │                                 │
   │<──── type: "progress" 50% ──────│
   │                                 │
   │<──── type: "progress" 75% ──────│
   │                                 │
   │<──── type: "progress" 95% ──────│
   │                                 │
   │<──── type: "finished" ──────────│
   │                                 │
   │──── WebSocket Close ───────────>│
   │                                 │
```

---

## State Management Flow

```
┌──────────────────────────────────────────────────────────────┐
│                    useVideoProgress Hook                      │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  WebSocket Message Received                                  │
│         │                                                     │
│         ▼                                                     │
│  ┌─────────────────┐                                         │
│  │ Parse JSON Data │                                         │
│  └─────────────────┘                                         │
│         │                                                     │
│         ▼                                                     │
│  ┌─────────────────────────────────────┐                     │
│  │ Switch on message type:             │                     │
│  │                                     │                     │
│  │ • connected  → Log confirmation     │                     │
│  │ • progress   → Update state         │                     │
│  │ • finished   → Set isFinished=true  │                     │
│  │ • error      → Set error message    │                     │
│  └─────────────────────────────────────┘                     │
│         │                                                     │
│         ▼                                                     │
│  ┌─────────────────────────────────────┐                     │
│  │ Update React State:                 │                     │
│  │ • setProgress(percent)              │                     │
│  │ • setStatus(status)                 │                     │
│  │ • setProcessingTime(seconds)        │                     │
│  │ • setIsFinished(true/false)         │                     │
│  └─────────────────────────────────────┘                     │
│         │                                                     │
│         ▼                                                     │
│  ┌─────────────────────────────────────┐                     │
│  │ Component Re-renders                │                     │
│  │ • Progress bar updates              │                     │
│  │ • Time display updates              │                     │
│  │ • Status message updates            │                     │
│  └─────────────────────────────────────┘                     │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

---

## Error Handling Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    WebSocket Error Occurs                    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │ What went wrong? │
                    └──────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│ Connection   │    │ Message      │    │ Backend      │
│ Failed       │    │ Error        │    │ Error        │
└──────────────┘    └──────────────┘    └──────────────┘
        │                     │                     │
        ▼                     ▼                     ▼
┌──────────────────────────────────────────────────────────┐
│ Hook sets: error = "error message"                       │
└──────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────┐
│ useEffect in page.tsx detects wsError                    │
└──────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────┐
│ Show toast notification to user                          │
└──────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────┐
│ Fallback to polling: pollForResult(jobId)                │
└──────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────┐
│ Processing continues with 2-second polling               │
└──────────────────────────────────────────────────────────┘
```

---

## Component Hierarchy

```
ADASPage
  │
  ├── useVideoProgress(currentJobId, isProcessing)
  │     │
  │     ├── WebSocket Connection
  │     ├── State: progress, status, isFinished, error
  │     └── Auto cleanup on unmount
  │
  ├── useEffect (watches WebSocket state)
  │     │
  │     ├── Updates processingProgress
  │     ├── Updates processingMsg
  │     ├── Calls fetchProcessedVideo on completion
  │     └── Falls back to polling on error
  │
  └── UI Components
        │
        ├── Upload Section
        │     ├── File input
        │     ├── Sample video button
        │     └── Upload button
        │
        └── Video Display Section
              ├── Processing View
              │     ├── Loader animation
              │     ├── WebSocket status badge
              │     ├── Progress bar
              │     └── Status message
              │
              └── Result View
                    ├── Video player
                    └── Action buttons
```

---

## Data Flow Summary

```
User Action → Upload Video → Get job_id → Set State → Hook Connects
     ↓
WebSocket Messages → Hook Updates State → useEffect Detects Change
     ↓
Update UI State → React Re-renders → User Sees Progress
     ↓
Finished Message → Fetch Result → Display Video
```

---

## Key Benefits Visualized

### Before (Polling):
```
Frontend                    Backend
   │                           │
   │──── GET /result ─────────>│
   │<──── 0% ─────────────────│
   │                           │
   │ (wait 2 seconds)          │
   │                           │
   │──── GET /result ─────────>│
   │<──── 10% ────────────────│
   │                           │
   │ (wait 2 seconds)          │
   │                           │
   │──── GET /result ─────────>│
   │<──── 20% ────────────────│
   │                           │
   ... (many requests)
```
**Result:** 450 requests for 15-minute video

### After (WebSocket):
```
Frontend                    Backend
   │                           │
   │──── WS Connect ──────────>│
   │<──── Connected ───────────│
   │                           │
   │<──── 0% ──────────────────│
   │<──── 10% ─────────────────│
   │<──── 20% ─────────────────│
   │<──── 30% ─────────────────│
   ... (instant updates)
   │<──── 100% ────────────────│
   │<──── Finished ────────────│
   │                           │
```
**Result:** 1 connection, instant updates! 🚀
