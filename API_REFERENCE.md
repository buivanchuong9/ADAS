# 📡 ADAS API Reference - Tổng Hợp Toàn Bộ Endpoints

**Base URL**: `http://localhost:8000`

---

## 🎯 Table of Contents
- [Authentication](#authentication)
- [Camera Management](#camera-management)
- [Trip Management](#trip-management)
- [Detection & Events](#detection--events)
- [Driver Management](#driver-management)
- [Analytics & Statistics](#analytics--statistics)
- [AI Model Management](#ai-model-management)
- [Data Collection (YOLO Dataset)](#data-collection-yolo-dataset)
- [WebSocket (Real-time)](#websocket-real-time)

---

## 🔐 Authentication

> **Note**: Hiện tại chưa có authentication, tất cả endpoints đều public

---

## 📹 Camera Management

### 1. Get All Cameras
```typescript
GET /api/cameras/list
```

**Response:**
```json
[
  {
    "id": 1,
    "name": "Front Camera",
    "status": "active",
    "rtsp_url": "rtsp://192.168.1.100:554/stream1",
    "location": "front",
    "created_at": "2024-01-01T00:00:00Z"
  }
]
```

**Frontend Usage:**
```typescript
const getCameras = async () => {
  const res = await fetch('/api/cameras/list')
  return await res.json()
}
```

---

### 2. Get Camera by ID
```typescript
GET /api/cameras/{camera_id}
```

**Example:**
```bash
GET /api/cameras/1
```

**Response:**
```json
{
  "id": 1,
  "name": "Front Camera",
  "status": "active",
  "rtsp_url": "rtsp://192.168.1.100:554/stream1"
}
```

**Frontend Usage:**
```typescript
const getCamera = async (id: number) => {
  const res = await fetch(`/api/cameras/${id}`)
  return await res.json()
}
```

---

### 3. Create Camera
```typescript
POST /api/cameras/create
Content-Type: application/json
```

**Request Body:**
```json
{
  "name": "Rear Camera",
  "rtsp_url": "rtsp://192.168.1.101:554/stream1",
  "location": "rear",
  "status": "active"
}
```

**Frontend Usage:**
```typescript
const createCamera = async (data: CameraData) => {
  const res = await fetch('/api/cameras/create', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  })
  return await res.json()
}
```

---

### 4. Update Camera
```typescript
PUT /api/cameras/{camera_id}
Content-Type: application/json
```

**Request Body:**
```json
{
  "name": "Updated Name",
  "status": "inactive"
}
```

**Frontend Usage:**
```typescript
const updateCamera = async (id: number, data: Partial<CameraData>) => {
  const res = await fetch(`/api/cameras/${id}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  })
  return await res.json()
}
```

---

### 5. Delete Camera
```typescript
DELETE /api/cameras/{camera_id}
```

**Frontend Usage:**
```typescript
const deleteCamera = async (id: number) => {
  const res = await fetch(`/api/cameras/${id}`, {
    method: 'DELETE'
  })
  return await res.json()
}
```

---

## 🚗 Trip Management

### 1. Get All Trips
```typescript
GET /api/trips/list?skip=0&limit=100
```

**Query Parameters:**
- `skip`: Số bản ghi bỏ qua (pagination)
- `limit`: Số bản ghi tối đa trả về

**Response:**
```json
[
  {
    "id": 1,
    "driver_id": 5,
    "camera_id": 1,
    "start_time": "2024-01-01T08:00:00Z",
    "end_time": "2024-01-01T09:30:00Z",
    "distance": 25.5,
    "status": "completed"
  }
]
```

**Frontend Usage:**
```typescript
const getTrips = async (skip = 0, limit = 100) => {
  const res = await fetch(`/api/trips/list?skip=${skip}&limit=${limit}`)
  return await res.json()
}
```

---

### 2. Get Trip by ID
```typescript
GET /api/trips/{trip_id}
```

**Frontend Usage:**
```typescript
const getTrip = async (id: number) => {
  const res = await fetch(`/api/trips/${id}`)
  return await res.json()
}
```

---

### 3. Create Trip
```typescript
POST /api/trips/create
Content-Type: application/json
```

**Request Body:**
```json
{
  "driver_id": 5,
  "camera_id": 1,
  "start_time": "2024-01-01T08:00:00Z"
}
```

**Frontend Usage:**
```typescript
const createTrip = async (data: TripData) => {
  const res = await fetch('/api/trips/create', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  })
  return await res.json()
}
```

---

### 4. End Trip
```typescript
PUT /api/trips/{trip_id}/end
```

**Frontend Usage:**
```typescript
const endTrip = async (id: number) => {
  const res = await fetch(`/api/trips/${id}/end`, {
    method: 'PUT'
  })
  return await res.json()
}
```

---

## 🎯 Detection & Events

### 1. Get All Detections
```typescript
GET /api/detections/list?skip=0&limit=100
```

**Response:**
```json
[
  {
    "id": 1,
    "trip_id": 1,
    "object_type": "car",
    "confidence": 0.95,
    "bbox_x": 100,
    "bbox_y": 200,
    "bbox_width": 150,
    "bbox_height": 200,
    "timestamp": "2024-01-01T08:15:30Z"
  }
]
```

**Frontend Usage:**
```typescript
const getDetections = async (skip = 0, limit = 100) => {
  const res = await fetch(`/api/detections/list?skip=${skip}&limit=${limit}`)
  return await res.json()
}
```

---

### 2. Get Detections by Trip
```typescript
GET /api/detections/trip/{trip_id}
```

**Frontend Usage:**
```typescript
const getDetectionsByTrip = async (tripId: number) => {
  const res = await fetch(`/api/detections/trip/${tripId}`)
  return await res.json()
}
```

---

### 3. Create Detection
```typescript
POST /api/detections/create
Content-Type: application/json
```

**Request Body:**
```json
{
  "trip_id": 1,
  "object_type": "pedestrian",
  "confidence": 0.92,
  "bbox_x": 150,
  "bbox_y": 250,
  "bbox_width": 80,
  "bbox_height": 180
}
```

**Frontend Usage:**
```typescript
const createDetection = async (data: DetectionData) => {
  const res = await fetch('/api/detections/create', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  })
  return await res.json()
}
```

---

### 4. Get All Events
```typescript
GET /api/events/list?skip=0&limit=100
```

**Response:**
```json
[
  {
    "id": 1,
    "trip_id": 1,
    "event_type": "collision_warning",
    "severity": "high",
    "description": "Vehicle too close",
    "timestamp": "2024-01-01T08:15:30Z"
  }
]
```

**Frontend Usage:**
```typescript
const getEvents = async (skip = 0, limit = 100) => {
  const res = await fetch(`/api/events/list?skip=${skip}&limit=${limit}`)
  return await res.json()
}
```

---

### 5. Get Events by Trip
```typescript
GET /api/events/trip/{trip_id}
```

**Frontend Usage:**
```typescript
const getEventsByTrip = async (tripId: number) => {
  const res = await fetch(`/api/events/trip/${tripId}`)
  return await res.json()
}
```

---

### 6. Create Event
```typescript
POST /api/events/create
Content-Type: application/json
```

**Request Body:**
```json
{
  "trip_id": 1,
  "event_type": "lane_departure",
  "severity": "medium",
  "description": "Vehicle crossed lane line"
}
```

**Frontend Usage:**
```typescript
const createEvent = async (data: EventData) => {
  const res = await fetch('/api/events/create', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  })
  return await res.json()
}
```

---

## 👤 Driver Management

### 1. Get All Drivers
```typescript
GET /api/drivers/list
```

**Response:**
```json
[
  {
    "id": 1,
    "name": "Nguyen Van A",
    "license_number": "B2-123456",
    "phone": "0901234567",
    "email": "nguyenvana@example.com",
    "status": "active"
  }
]
```

**Frontend Usage:**
```typescript
const getDrivers = async () => {
  const res = await fetch('/api/drivers/list')
  return await res.json()
}
```

---

### 2. Get Driver by ID
```typescript
GET /api/drivers/{driver_id}
```

**Frontend Usage:**
```typescript
const getDriver = async (id: number) => {
  const res = await fetch(`/api/drivers/${id}`)
  return await res.json()
}
```

---

### 3. Create Driver
```typescript
POST /api/drivers/create
Content-Type: application/json
```

**Request Body:**
```json
{
  "name": "Tran Van B",
  "license_number": "B2-654321",
  "phone": "0907654321",
  "email": "tranvanb@example.com"
}
```

**Frontend Usage:**
```typescript
const createDriver = async (data: DriverData) => {
  const res = await fetch('/api/drivers/create', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  })
  return await res.json()
}
```

---

### 4. Update Driver
```typescript
PUT /api/drivers/{driver_id}
```

**Frontend Usage:**
```typescript
const updateDriver = async (id: number, data: Partial<DriverData>) => {
  const res = await fetch(`/api/drivers/${id}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  })
  return await res.json()
}
```

---

### 5. Delete Driver
```typescript
DELETE /api/drivers/{driver_id}
```

**Frontend Usage:**
```typescript
const deleteDriver = async (id: number) => {
  const res = await fetch(`/api/drivers/${id}`, { method: 'DELETE' })
  return await res.json()
}
```

---

## 📊 Analytics & Statistics

### 1. Get Dashboard Summary
```typescript
GET /api/analytics/dashboard
```

**Response:**
```json
{
  "total_trips": 1250,
  "total_detections": 45678,
  "total_events": 234,
  "active_cameras": 8,
  "active_drivers": 25,
  "today_trips": 45,
  "today_events": 12
}
```

**Frontend Usage:**
```typescript
const getDashboard = async () => {
  const res = await fetch('/api/analytics/dashboard')
  return await res.json()
}
```

---

### 2. Get Trip Statistics
```typescript
GET /api/analytics/trips/stats?start_date=2024-01-01&end_date=2024-01-31
```

**Query Parameters:**
- `start_date`: Ngày bắt đầu (YYYY-MM-DD)
- `end_date`: Ngày kết thúc (YYYY-MM-DD)

**Response:**
```json
{
  "total_trips": 120,
  "total_distance": 3500.5,
  "average_distance": 29.17,
  "total_duration": 7200
}
```

**Frontend Usage:**
```typescript
const getTripStats = async (startDate: string, endDate: string) => {
  const res = await fetch(
    `/api/analytics/trips/stats?start_date=${startDate}&end_date=${endDate}`
  )
  return await res.json()
}
```

---

### 3. Get Detection Statistics
```typescript
GET /api/analytics/detections/stats
```

**Response:**
```json
{
  "by_type": {
    "car": 25000,
    "pedestrian": 8500,
    "motorcycle": 6200,
    "bicycle": 3100
  },
  "total": 42800
}
```

**Frontend Usage:**
```typescript
const getDetectionStats = async () => {
  const res = await fetch('/api/analytics/detections/stats')
  return await res.json()
}
```

---

### 4. Get Event Statistics
```typescript
GET /api/analytics/events/stats
```

**Response:**
```json
{
  "by_type": {
    "collision_warning": 45,
    "lane_departure": 89,
    "drowsiness": 23,
    "speed_limit": 67
  },
  "by_severity": {
    "high": 34,
    "medium": 102,
    "low": 88
  }
}
```

**Frontend Usage:**
```typescript
const getEventStats = async () => {
  const res = await fetch('/api/analytics/events/stats')
  return await res.json()
}
```

---

## 🤖 AI Model Management

### 1. Get All Models
```typescript
GET /api/models/list
```

**Response:**
```json
[
  {
    "id": 1,
    "name": "YOLOv8n",
    "version": "1.0.0",
    "type": "object_detection",
    "status": "active",
    "accuracy": 0.89,
    "created_at": "2024-01-01T00:00:00Z"
  }
]
```

**Frontend Usage:**
```typescript
const getModels = async () => {
  const res = await fetch('/api/models/list')
  return await res.json()
}
```

---

### 2. Get Model by ID
```typescript
GET /api/models/{model_id}
```

**Frontend Usage:**
```typescript
const getModel = async (id: number) => {
  const res = await fetch(`/api/models/${id}`)
  return await res.json()
}
```

---

### 3. Upload Model
```typescript
POST /api/models/upload
Content-Type: multipart/form-data
```

**Request Body:**
```typescript
const formData = new FormData()
formData.append('file', modelFile)
formData.append('name', 'YOLOv8m Custom')
formData.append('version', '2.0.0')
formData.append('type', 'object_detection')
```

**Frontend Usage:**
```typescript
const uploadModel = async (file: File, metadata: ModelMetadata) => {
  const formData = new FormData()
  formData.append('file', file)
  formData.append('name', metadata.name)
  formData.append('version', metadata.version)
  formData.append('type', metadata.type)
  
  const res = await fetch('/api/models/upload', {
    method: 'POST',
    body: formData
  })
  return await res.json()
}
```

---

### 4. Activate Model
```typescript
POST /api/models/{model_id}/activate
```

**Frontend Usage:**
```typescript
const activateModel = async (id: number) => {
  const res = await fetch(`/api/models/${id}/activate`, {
    method: 'POST'
  })
  return await res.json()
}
```

---

### 5. Delete Model
```typescript
DELETE /api/models/{model_id}
```

**Frontend Usage:**
```typescript
const deleteModel = async (id: number) => {
  const res = await fetch(`/api/models/${id}`, { method: 'DELETE' })
  return await res.json()
}
```

---

## 📦 Data Collection (YOLO Dataset)

### 1. Create Dataset Item
```typescript
POST /api/dataset
Content-Type: multipart/form-data
```

**Request Body:**
```typescript
const formData = new FormData()
formData.append('file', imageFile)
formData.append('metadata', JSON.stringify({
  labels: ['car', 'pedestrian'],
  boundingBoxes: [
    { x: 100, y: 200, width: 150, height: 200, label: 'car' },
    { x: 300, y: 150, width: 80, height: 180, label: 'pedestrian' }
  ],
  weather: 'sunny',
  roadType: 'urban',
  description: 'City traffic scene'
}))
```

**Frontend Usage:**
```typescript
const createDatasetItem = async (file: File, metadata: DatasetMetadata) => {
  const formData = new FormData()
  formData.append('file', file)
  formData.append('metadata', JSON.stringify(metadata))
  
  const res = await fetch('/api/dataset', {
    method: 'POST',
    body: formData
  })
  return await res.json()
}
```

---

### 2. Get All Dataset Items
```typescript
GET /api/dataset
```

**Response:**
```json
[
  {
    "id": "abc-123",
    "filePath": "raw/abc-123.jpg",
    "labels": ["car", "pedestrian"],
    "boundingBoxes": [...],
    "weather": "sunny",
    "roadType": "urban",
    "timestamp": "2024-01-01T08:00:00Z"
  }
]
```

**Frontend Usage:**
```typescript
const getDatasetItems = async () => {
  const res = await fetch('/api/dataset')
  return await res.json()
}
```

---

### 3. Delete Dataset Item
```typescript
DELETE /api/dataset/{item_id}
```

**Frontend Usage:**
```typescript
const deleteDatasetItem = async (id: string) => {
  const res = await fetch(`/api/dataset/${id}`, { method: 'DELETE' })
  return await res.json()
}
```

---

### 4. Export YOLO Dataset
```typescript
POST /api/dataset/export-yolo
```

**Response:**
```json
{
  "success": true,
  "message": "YOLO dataset exported successfully",
  "dataset_path": "/path/to/dataset",
  "data_yaml": "/path/to/data.yaml",
  "images": 150,
  "labels": 150,
  "ready_for_training": true
}
```

**Frontend Usage:**
```typescript
const exportYOLODataset = async () => {
  const res = await fetch('/api/dataset/export-yolo', { method: 'POST' })
  return await res.json()
}
```

---

### 5. Get Dataset Statistics
```typescript
GET /api/dataset/stats
```

**Response:**
```json
{
  "total_items": 150,
  "total_images": 150,
  "total_labels": 150,
  "class_distribution": {
    "car": 450,
    "pedestrian": 230,
    "motorcycle": 180,
    "bicycle": 95
  }
}
```

**Frontend Usage:**
```typescript
const getDatasetStats = async () => {
  const res = await fetch('/api/dataset/stats')
  return await res.json()
}
```

---

## 🔌 WebSocket (Real-time)

### Connect to Inference WebSocket
```typescript
WS ws://localhost:8000/ws/infer
```

**Frontend Usage:**
```typescript
const ws = new WebSocket('ws://localhost:8000/ws/infer')

ws.onopen = () => {
  console.log('WebSocket connected')
}

ws.onmessage = (event) => {
  const data = JSON.parse(event.data)
  console.log('Detection result:', data)
  // { detections: [...], timestamp: "...", frame_id: 123 }
}

ws.onerror = (error) => {
  console.error('WebSocket error:', error)
}

ws.onclose = () => {
  console.log('WebSocket disconnected')
}

// Send frame for inference
const sendFrame = (imageBase64: string) => {
  ws.send(JSON.stringify({
    action: 'infer',
    image: imageBase64,
    model: 'yolov8n'
  }))
}
```

---

## 🏥 Health Check

### 1. Health Status
```typescript
GET /health
```

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2024-01-01T08:00:00Z",
  "version": "2.0.0"
}
```

**Frontend Usage:**
```typescript
const checkHealth = async () => {
  const res = await fetch('/health')
  return await res.json()
}
```

---

### 2. API Root
```typescript
GET /
```

**Response:**
```json
{
  "message": "ADAS Backend API v2.0",
  "docs": "/docs",
  "health": "/health",
  "endpoints": {
    "cameras": "/api/cameras/list",
    "trips": "/api/trips/list",
    "detections": "/api/detections/list",
    "events": "/api/events/list",
    "drivers": "/api/drivers/list",
    "analytics": "/api/analytics/dashboard",
    "dataset": "/api/dataset",
    "websocket": "ws://localhost:8000/ws/infer"
  }
}
```

---

## 📚 Complete TypeScript SDK

### Create API Client
```typescript
// lib/api-client.ts
const BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000'

class ADASApiClient {
  // Cameras
  async getCameras() {
    const res = await fetch(`${BASE_URL}/api/cameras/list`)
    return res.json()
  }
  
  async getCamera(id: number) {
    const res = await fetch(`${BASE_URL}/api/cameras/${id}`)
    return res.json()
  }
  
  async createCamera(data: CameraData) {
    const res = await fetch(`${BASE_URL}/api/cameras/create`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data)
    })
    return res.json()
  }
  
  // Trips
  async getTrips(skip = 0, limit = 100) {
    const res = await fetch(`${BASE_URL}/api/trips/list?skip=${skip}&limit=${limit}`)
    return res.json()
  }
  
  async createTrip(data: TripData) {
    const res = await fetch(`${BASE_URL}/api/trips/create`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data)
    })
    return res.json()
  }
  
  async endTrip(id: number) {
    const res = await fetch(`${BASE_URL}/api/trips/${id}/end`, {
      method: 'PUT'
    })
    return res.json()
  }
  
  // Detections
  async getDetections(skip = 0, limit = 100) {
    const res = await fetch(`${BASE_URL}/api/detections/list?skip=${skip}&limit=${limit}`)
    return res.json()
  }
  
  async getDetectionsByTrip(tripId: number) {
    const res = await fetch(`${BASE_URL}/api/detections/trip/${tripId}`)
    return res.json()
  }
  
  // Events
  async getEvents(skip = 0, limit = 100) {
    const res = await fetch(`${BASE_URL}/api/events/list?skip=${skip}&limit=${limit}`)
    return res.json()
  }
  
  async createEvent(data: EventData) {
    const res = await fetch(`${BASE_URL}/api/events/create`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data)
    })
    return res.json()
  }
  
  // Drivers
  async getDrivers() {
    const res = await fetch(`${BASE_URL}/api/drivers/list`)
    return res.json()
  }
  
  async createDriver(data: DriverData) {
    const res = await fetch(`${BASE_URL}/api/drivers/create`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data)
    })
    return res.json()
  }
  
  // Analytics
  async getDashboard() {
    const res = await fetch(`${BASE_URL}/api/analytics/dashboard`)
    return res.json()
  }
  
  async getTripStats(startDate: string, endDate: string) {
    const res = await fetch(
      `${BASE_URL}/api/analytics/trips/stats?start_date=${startDate}&end_date=${endDate}`
    )
    return res.json()
  }
  
  // Dataset
  async createDatasetItem(file: File, metadata: DatasetMetadata) {
    const formData = new FormData()
    formData.append('file', file)
    formData.append('metadata', JSON.stringify(metadata))
    
    const res = await fetch(`${BASE_URL}/api/dataset`, {
      method: 'POST',
      body: formData
    })
    return res.json()
  }
  
  async getDatasetItems() {
    const res = await fetch(`${BASE_URL}/api/dataset`)
    return res.json()
  }
  
  async deleteDatasetItem(id: string) {
    const res = await fetch(`${BASE_URL}/api/dataset/${id}`, {
      method: 'DELETE'
    })
    return res.json()
  }
  
  async exportYOLODataset() {
    const res = await fetch(`${BASE_URL}/api/dataset/export-yolo`, {
      method: 'POST'
    })
    return res.json()
  }
  
  // WebSocket
  connectWebSocket(onMessage: (data: any) => void) {
    const ws = new WebSocket(`ws://localhost:8000/ws/infer`)
    
    ws.onopen = () => console.log('WebSocket connected')
    ws.onmessage = (event) => onMessage(JSON.parse(event.data))
    ws.onerror = (error) => console.error('WebSocket error:', error)
    ws.onclose = () => console.log('WebSocket disconnected')
    
    return ws
  }
}

export const apiClient = new ADASApiClient()
```

### Usage in Components
```typescript
// components/CameraList.tsx
import { apiClient } from '@/lib/api-client'
import { useEffect, useState } from 'react'

export default function CameraList() {
  const [cameras, setCameras] = useState([])
  
  useEffect(() => {
    apiClient.getCameras().then(setCameras)
  }, [])
  
  return (
    <div>
      {cameras.map(camera => (
        <div key={camera.id}>{camera.name}</div>
      ))}
    </div>
  )
}
```

---

## 🎯 Quick Reference

### Object Types
```typescript
type ObjectType = 
  | 'car'
  | 'motorcycle'
  | 'pedestrian'
  | 'bicycle'
  | 'traffic_light'
  | 'traffic_sign'
  | 'truck'
  | 'bus'
```

### Event Types
```typescript
type EventType =
  | 'collision_warning'
  | 'lane_departure'
  | 'drowsiness'
  | 'speed_limit'
  | 'pedestrian_crossing'
```

### Severity Levels
```typescript
type Severity = 'low' | 'medium' | 'high'
```

### Weather Conditions
```typescript
type Weather = 'sunny' | 'rainy' | 'foggy' | 'night'
```

### Road Types
```typescript
type RoadType = 'urban' | 'highway' | 'rural'
```

---

## 🔗 External Links

- **API Docs**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc
- **OpenAPI JSON**: http://localhost:8000/openapi.json

---

**📅 Last Updated**: Nov 22, 2025  
**🔖 Version**: 2.0.0  
**📝 Note**: Copy paste code trực tiếp vào FE, đã test sẵn!
