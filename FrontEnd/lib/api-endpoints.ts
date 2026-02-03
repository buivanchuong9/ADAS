// Centralized ADAS API endpoint paths (no domain)
// Base URL is configured separately in api-config.ts

// Common API prefix for non-swagger routes
const API_PREFIX = '/api'

export const API_ENDPOINTS = {
  // =========================
  // System (from Swagger)
  // =========================
  HEALTH: '/health',
  STATUS: '/health', // local "status" maps to ADAS health check

  // =========================
  // Admin / Analytics (from Swagger)
  // =========================
  ADMIN_OVERVIEW: '/admin/overview',
  ADMIN_STATISTICS: '/admin/statistics',
  ADMIN_CHARTS: '/admin/charts',
  ADMIN_VIDEO_TIMELINE: (videoId: string | number) =>
    `/admin/video/${videoId}/timeline`,

  // =========================
  // Vision / Processing (from Swagger)
  // =========================
  VISION_VIDEO: '/vision/video',
  VIDEO_PROCESS: (id: string | number) =>
    `/vision/video/${id}/process`, // assumption if per-id processing exists

  // =========================
  // Alerts (assumption – not in Swagger)
  // =========================
  ALERTS_LATEST: `${API_PREFIX}/alerts/latest`,
  ALERTS_STATS: `${API_PREFIX}/alerts/stats`,

  // =========================
  // Detections (assumption)
  // =========================
  DETECTIONS_SAVE: `${API_PREFIX}/detections/save`,
  DETECTIONS_RECENT: `${API_PREFIX}/detections/recent`,
  DETECTIONS_STATS: '/admin/statistics', // mapped to admin statistics

  // =========================
  // Events (assumption)
  // =========================
  EVENTS: `${API_PREFIX}/events`,
  EVENTS_LIST: `${API_PREFIX}/events/list`,

  // =========================
  // Trips (assumption)
  // =========================
  TRIPS: `${API_PREFIX}/trips`,
  TRIPS_LIST: `${API_PREFIX}/trips/list`,

  // =========================
  // Dataset (assumption)
  // =========================
  DATASET: `${API_PREFIX}/dataset`,

  // =========================
  // Models (assumption)
  // =========================
  MODELS_AVAILABLE: `${API_PREFIX}/models/available`,
  MODELS_DOWNLOAD_ALL: `${API_PREFIX}/models/download-all`,
  MODEL_DOWNLOAD: (id: string) =>
    `${API_PREFIX}/models/download/${id}`,
  MODEL_INFO: (id: string) =>
    `${API_PREFIX}/models/info/${id}`,
  MODEL_DELETE: (id: string) =>
    `${API_PREFIX}/models/delete/${id}`,

  // =========================
  // WebSockets (UI only – not Swagger)
  // =========================
  WS_ADAS_STREAM: '/ws/adas/stream',
  WS_STREAM: '/ws/stream',
  WS_INFERENCE_VIDEO: '/ws/inference/video',
  WS_MODELS_WEBCAM: '/ws/models/webcam',

  // =========================
  // Video (assumption)
  // =========================
  VIDEO_UPLOAD: `${API_PREFIX}/video/upload`,
  VIDEOS_LIST: `${API_PREFIX}/video/list`,
  VIDEO_RESULT: (jobId: string | number) =>
    `${API_PREFIX}/video/result/${jobId}`,
  VIDEO_DETAILS: (id: string | number) =>
    `${API_PREFIX}/video/result/${id}`,
  VIDEO_DOWNLOAD: (jobId: string, filename: string) =>
    `${API_PREFIX}/video/download/${jobId}/${filename}`,
  VIDEO_SAMPLE: (jobId: string, filename: string) =>
    `${API_PREFIX}/video/sample/${jobId}/${filename}`,

  // =========================
  // Driver Monitoring (assumption)
  // =========================
  DRIVER_MONITOR_ANALYZE: `${API_PREFIX}/driver-monitor/analyze`, // POST
  DRIVER_STATUS_SAVE: `${API_PREFIX}/driver-status`,              // POST
  DRIVER_STATUS_CURRENT: `${API_PREFIX}/driver-status`,           // GET
  DRIVER_STATUS_HISTORY: `${API_PREFIX}/driver-status/history`,   // GET
  DRIVER_MONITOR_DOWNLOAD: (jobId: string | number) =>
    `${API_PREFIX}/download/${jobId}`, // GET
} as const

export type ApiEndpointKey = keyof typeof API_ENDPOINTS
