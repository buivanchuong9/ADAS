// Centralized ADAS API endpoint paths (no domain)
// Base URL is configured separately in api-config.ts
export const API_ENDPOINTS = {
  // System (from Swagger)
  HEALTH: '/health',
  STATUS: '/health', // local "status" maps to ADAS health check

  // Admin/analytics (from Swagger)
  ADMIN_OVERVIEW: '/admin/overview',
  ADMIN_STATISTICS: '/admin/statistics',
  ADMIN_CHARTS: '/admin/charts',
  ADMIN_VIDEO_TIMELINE: (videoId: string | number) => `/admin/video/${videoId}/timeline`,

  // Vision / processing (from Swagger)
  VISION_VIDEO: '/vision/video',

  // The following are not in the minimal Swagger list shown, but are kept for current UI flows.
  // BE must provide these routes; otherwise adapt to ADMIN_* as needed.
  ALERTS_LATEST: '/api/alerts/latest', // assumption: backend provides
  ALERTS_STATS: '/api/alerts/stats',   // assumption: backend provides

  DETECTIONS_SAVE: '/api/detections/save',     // assumption: backend provides
  DETECTIONS_RECENT: '/api/detections/recent', // assumption: backend provides
  DETECTIONS_STATS: '/admin/statistics',       // mapped to admin statistics

  EVENTS: '/api/events',            // assumption: backend provides
  EVENTS_LIST: '/api/events/list',  // assumption: backend provides

  TRIPS: '/api/trips',              // assumption: backend provides
  TRIPS_LIST: '/api/trips/list',    // assumption: backend provides

  DATASET: '/api/dataset',          // assumption: backend provides

  MODELS_AVAILABLE: '/api/models/available',      // assumption: backend provides
  MODELS_DOWNLOAD_ALL: '/api/models/download-all',// assumption: backend provides
  MODEL_DOWNLOAD: (id: string) => `/api/models/download/${id}`, // assumption
  MODEL_INFO: (id: string) => `/api/models/info/${id}`,         // assumption
  MODEL_DELETE: (id: string) => `/api/models/delete/${id}`,     // assumption

  // WebSockets (not in Swagger page; kept for UI)
  WS_ADAS_STREAM: '/ws/adas/stream',
  WS_STREAM: '/ws/stream',
  WS_INFERENCE_VIDEO: '/ws/inference/video',
  WS_MODELS_WEBCAM: '/ws/models/webcam',

  // Video processing via vision/video (Swagger)
  VIDEO_UPLOAD: '/vision/video',
  VIDEO_PROCESS: (id: string | number) => `/vision/video/${id}/process`, // assumption if processing per-id exists
} as const

export type ApiEndpointKey = keyof typeof API_ENDPOINTS

