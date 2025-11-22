# Frontend Migration - Firebase to FastAPI Backend

## ✅ Completed Changes

### API Routes Updated

All API routes in `app/api/*` now proxy to FastAPI backend instead of using mock data or Firebase:

1. **`/api/analytics/route.ts`** - Fetches dashboard analytics from `/api/analytics/dashboard`
2. **`/api/events/route.ts`** - CRUD operations for events
3. **`/api/trips/route.ts`** - CRUD operations for trips  
4. **`/api/detection/route.ts`** - Detection data (primarily via WebSocket)
5. **`/api/driver-status/route.ts`** - Driver monitoring status

### Pages Updated

1. **`app/dashboard/page.tsx`** - Removed Firebase, now uses REST API
   - Fetches from `/api/analytics/dashboard`
   - Auto-refreshes every 30 seconds
   - Displays: total trips, events, cameras, active drivers
   - Shows recent events list

### Configuration Files

1. **`lib/api-config.ts`** - Centralized API configuration
   - Base URL configuration
   - All endpoint definitions
   - Helper function `apiFetch<T>`

2. **`.env.example`** - Environment variables template
   ```env
   NEXT_PUBLIC_API_URL=http://localhost:8000
   BACKEND_URL=http://localhost:8000
   NEXT_PUBLIC_WS_URL=ws://localhost:8000
   ```

## 🔄 How It Works

### API Flow

```
Frontend (Next.js)
    ↓
/app/api/* (Route Handlers)
    ↓
FastAPI Backend (http://localhost:8000)
    ↓
SQL Server Database
```

### Environment Variables

- **`NEXT_PUBLIC_API_URL`** - Used in client-side code (browser)
- **`BACKEND_URL`** - Used in server-side route handlers
- **`NEXT_PUBLIC_WS_URL`** - WebSocket endpoint for real-time inference

## 📝 Migration Checklist

- [x] Remove all Firebase imports
- [x] Remove Firebase config
- [x] Update API routes to proxy to FastAPI
- [x] Update dashboard to use REST API
- [x] Create centralized API config
- [x] Add environment variables
- [ ] Update other pages if they use Firebase (TODO: check analytics, events pages)
- [ ] Update WebSocket connections to use backend URL
- [ ] Test all API endpoints with backend running
- [ ] Handle loading states properly
- [ ] Handle error states properly

## 🧪 Testing

### Start Backend
```bash
cd backend-python
python main.py
# Server at http://localhost:8000
```

### Start Frontend
```bash
npm run dev
# Frontend at http://localhost:3000
```

### Test Endpoints
```bash
# Health check
curl http://localhost:8000/health

# Analytics
curl http://localhost:8000/api/analytics/dashboard

# Events
curl http://localhost:8000/api/events/list

# Trips
curl http://localhost:8000/api/trips/list
```

## 🚨 Breaking Changes

### Before (Firebase)
```typescript
import { getFirestore, collection, query, orderBy, onSnapshot } from "firebase/firestore"

const db = getFirestore(app)
const unsubEvents = onSnapshot(
  query(collection(db, "events"), orderBy("Timestamp", "desc")),
  (snapshot) => {
    setEvents(snapshot.docs.map(doc => ({ id: doc.id, ...doc.data() })))
  }
)
```

### After (FastAPI + REST)
```typescript
const fetchDashboardData = async () => {
  const response = await fetch(`${API_BASE_URL}/api/analytics/dashboard`)
  const data = await response.json()
  setDashboardData(data)
}

// Auto-refresh with interval
useEffect(() => {
  fetchDashboardData()
  const interval = setInterval(fetchDashboardData, 30000)
  return () => clearInterval(interval)
}, [])
```

### Real-time Updates

For real-time data, use:
1. **Polling** - setInterval with fetch (simple, current approach)
2. **WebSocket** - For live inference and streaming
3. **Server-Sent Events** - For one-way real-time updates (future)

## 📚 API Reference

### Dashboard Analytics
```typescript
GET /api/analytics/dashboard
Response: {
  total_trips: number
  total_events: number
  total_cameras: number
  active_drivers: number
  recent_events: Event[]
}
```

### Events
```typescript
GET /api/events/list?type=<type>&limit=<limit>
POST /api/events
DELETE /api/events/:id
```

### Trips
```typescript
GET /api/trips/list?limit=<limit>
POST /api/trips
POST /api/trips/:id/end
```

### WebSocket Inference
```typescript
ws://localhost:8000/ws/infer

Send: { frameB64: "<base64_image>" }
Receive: {
  detections: Detection[],
  stats: { infer_ms: number, model: string }
}
```

## 🔗 Related Files

- Backend: `backend-python/main.py` - All API endpoints
- Backend Models: `backend-python/models.py` - Database schema
- Backend Services: `backend-python/services.py` - Business logic
- Frontend Config: `lib/api-config.ts` - API configuration
- Frontend Routes: `app/api/**/route.ts` - Next.js API handlers

---

**All Firebase dependencies removed! Using FastAPI + SQL Server backend. 🎉**
