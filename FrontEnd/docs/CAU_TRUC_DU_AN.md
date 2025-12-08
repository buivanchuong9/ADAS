# 📋 CẤU TRÚC DỰ ÁN ADAS - TỔNG QUAN

## 🎯 Tổng Quan Dự Án

**Hệ Thống Hỗ Trợ Lái Xe Nâng Cao (ADAS)** được tổ chức theo kiến trúc enterprise chuẩn, tách biệt rõ ràng Frontend và Backend.

---

## 📁 Cấu Trúc Thư Mục

```
adas-platform/
│
├── 📱 FRONTEND (Next.js 14 + TypeScript)
│   │
│   ├── app/                          # Next.js App Router
│   │   ├── layout.tsx                # Root layout
│   │   ├── page.tsx                  # Trang chủ
│   │   ├── globals.css               # Global styles
│   │   │
│   │   ├── adas/                     # ADAS Detection
│   │   │   └── page.tsx              # Trang phát hiện ADAS
│   │   │
│   │   ├── dashboard/                # Dashboard tổng quan
│   │   │   └── page.tsx
│   │   │
│   │   ├── analytics/                # Phân tích dữ liệu
│   │   │   └── page.tsx
│   │   │
│   │   ├── data-collection/          # Thu thập dataset
│   │   │   └── page.tsx
│   │   │
│   │   ├── driver-monitor/           # Giám sát lái xe
│   │   │   └── page.tsx
│   │   │
│   │   ├── events/                   # Sự kiện
│   │   │   └── page.tsx
│   │   │
│   │   ├── models-webcam/            # Webcam detection
│   │   │   └── page.tsx
│   │   │
│   │   ├── ai-assistant/             # AI Assistant
│   │   │   └── page.tsx
│   │   │
│   │   └── api/                      # API Routes (Next.js)
│   │       ├── ai-chat/              # AI Chat endpoint
│   │       ├── alerts/               # Alerts API
│   │       ├── analytics/            # Analytics API
│   │       ├── dataset/              # Dataset API
│   │       ├── detection/            # Detection API
│   │       ├── driver-status/        # Driver status
│   │       ├── events/               # Events API
│   │       ├── models/               # Models API
│   │       └── trips/                # Trips API
│   │
│   ├── components/                   # React Components
│   │   ├── ui/                       # Shadcn UI components
│   │   │   ├── button.tsx
│   │   │   ├── card.tsx
│   │   │   ├── dialog.tsx
│   │   │   ├── select.tsx
│   │   │   └── ... (30+ components)
│   │   │
│   │   ├── adas/                     # ADAS-specific components
│   │   │   ├── webcam-detector.tsx
│   │   │   ├── detection-overlay.tsx
│   │   │   └── alert-panel.tsx
│   │   │
│   │   ├── sidebar.tsx               # Navigation sidebar
│   │   ├── stat-card.tsx             # Statistics card
│   │   ├── live-chart.tsx            # Real-time charts
│   │   ├── detection-stats.tsx       # Detection statistics
│   │   └── ...
│   │
│   ├── lib/                          # Utilities & Helpers
│   │   ├── api-client.ts             # API client wrapper
│   │   ├── api-config.ts             # API configuration
│   │   ├── translations.ts           # i18n translations
│   │   ├── detection-utils.ts        # Detection helpers
│   │   ├── use-api.ts                # API hooks
│   │   └── utils.ts                  # Common utilities
│   │
│   ├── hooks/                        # Custom React Hooks
│   │   ├── use-mobile.ts
│   │   └── use-toast.ts
│   │
│   ├── styles/                       # Styles
│   │   └── globals.css
│   │
│   └── public/                       # Static assets
│       ├── images/
│       └── icons/
│
├── 🧠 BACKEND (FastAPI + Python 3.11)
│   │
│   ├── backend-python/
│   │   │
│   │   ├── main.py                   # FastAPI entry point
│   │   ├── adas_backend.py           # ADAS core logic (LEGACY)
│   │   ├── database.py               # Database connection
│   │   ├── models.py                 # SQLAlchemy models
│   │   ├── schemas.py                # Pydantic schemas
│   │   ├── config.py                 # Configuration (LEGACY)
│   │   │
│   │   ├── api/                      # API Endpoints
│   │   │   ├── websocket_inference.py    # WebSocket streaming
│   │   │   ├── upload/
│   │   │   │   └── router.py         # File upload
│   │   │   ├── inference/
│   │   │   │   └── router.py         # Inference API
│   │   │   ├── training/
│   │   │   │   └── router.py         # Model training
│   │   │   ├── dataset/
│   │   │   │   └── router.py         # Dataset management
│   │   │   ├── alerts/
│   │   │   │   └── router.py         # Alerts system
│   │   │   ├── detections/
│   │   │   │   └── router.py         # Detection history
│   │   │   ├── models/
│   │   │   │   └── router.py         # Model management
│   │   │   ├── auto_learning/
│   │   │   │   └── router.py         # Auto learning
│   │   │   ├── driver_monitoring/
│   │   │   │   └── router.py         # Driver monitor
│   │   │   └── video_upload.py       # Video upload
│   │   │
│   │   ├── core/                     # Core Infrastructure
│   │   │   ├── config.py             # Settings & configuration
│   │   │   ├── logging_config.py     # Logging setup
│   │   │   ├── responses.py          # API response models
│   │   │   └── exceptions.py         # Exception handlers
│   │   │
│   │   ├── services/                 # Business Logic
│   │   │   ├── enhanced_services.py  # Core services
│   │   │   └── realtime_aggregator.py # Real-time data
│   │   │
│   │   ├── ai_models/                # AI Models
│   │   │   ├── yolo11_detector.py    # YOLOv11 detector ⭐
│   │   │   ├── adas_unified.py       # Unified ADAS model
│   │   │   ├── adas_unified_pro.py   # Pro version
│   │   │   ├── yolo_detector.py      # YOLO wrapper
│   │   │   ├── yolo_trainer.py       # Training logic
│   │   │   │
│   │   │   └── weights/              # Model weights
│   │   │       ├── yolo11n.pt        # YOLOv11 nano
│   │   │       ├── yolo11m.pt        # YOLOv11 medium
│   │   │       └── ...
│   │   │
│   │   ├── adas_core/                # ADAS Core Modules
│   │   │   ├── collision_detector.py # Collision detection
│   │   │   ├── lane_detector.py      # Lane detection
│   │   │   ├── distance_estimator.py # Distance estimation
│   │   │   └── tests/                # Unit tests
│   │   │
│   │   ├── dataset/                  # Training Dataset
│   │   │   ├── raw/                  # Raw images
│   │   │   ├── labels/               # YOLO labels (.txt)
│   │   │   ├── auto_collected/       # Auto-collected data
│   │   │   └── data.yaml             # Dataset config
│   │   │
│   │   ├── logs/                     # Application Logs
│   │   │   ├── alerts/               # Alert logs
│   │   │   └── app.log               # Main log
│   │   │
│   │   ├── scripts/                  # Utility Scripts
│   │   │   ├── seed_db.py            # Seed database
│   │   │   └── migrate_db.py         # Database migration
│   │   │
│   │   ├── tests/                    # Tests
│   │   │   ├── test_api.py
│   │   │   ├── test_models.py
│   │   │   └── test_services.py
│   │   │
│   │   ├── 🐳 Docker Files
│   │   │   ├── Dockerfile.cross-platform     # Multi-stage build
│   │   │   ├── docker-compose.cross-platform.yml
│   │   │   ├── .dockerignore
│   │   │   ├── start-docker-dev.sh           # Quick start (Unix)
│   │   │   ├── docker-start.bat              # Quick start (Windows)
│   │   │   ├── stop-docker.sh
│   │   │   └── stop-docker.bat
│   │   │
│   │   ├── requirements.txt          # Python dependencies
│   │   ├── .env.example              # Environment template
│   │   └── adas.db                   # SQLite database
│   │
│   └── model-worker/                 # Model Worker (Optional)
│       ├── app.py
│       ├── Dockerfile
│       └── requirements.txt
│
├── 🗄️ DATABASE
│   └── database/
│       ├── sql-server-schema.sql     # SQL Server schema
│       └── training-queries.sql      # Training queries
│
├── 🐳 DOCKER (Root level)
│   ├── docker-compose.yml            # Full stack compose
│   ├── docker-compose.dev.yml        # Development compose
│   ├── docker-compose.prod.yml       # Production compose
│   ├── Dockerfile                    # Frontend Dockerfile
│   └── frontend.Dockerfile           # Frontend build
│
├── 📚 DOCUMENTATION
│   ├── HUONG_DAN.md                  # Hướng dẫn tiếng Việt ⭐
│   ├── README.md                     # README chính
│   ├── QUICK_START.md                # Quick start
│   │
│   └── docs/
│       ├── CAU_TRUC_DU_AN.md         # File này
│       └── deprecated/               # Docs cũ
│           ├── DOCKER_GUIDE.md
│           ├── SETUP_FOR_TEAM.md
│           └── ...
│
├── ⚙️ CONFIGURATION
│   ├── package.json                  # Node.js config
│   ├── tsconfig.json                 # TypeScript config
│   ├── next.config.mjs               # Next.js config
│   ├── tailwind.config.js            # Tailwind config
│   ├── postcss.config.mjs            # PostCSS config
│   ├── components.json               # Shadcn config
│   ├── .env.local.example            # Frontend env template
│   └── .gitignore
│
└── 🚀 SCRIPTS
    ├── dev-start.sh                  # Start dev servers
    ├── dev-stop.sh                   # Stop dev servers
    ├── dev-restart.sh                # Restart dev servers
    ├── dev-logs.sh                   # View logs
    ├── prod-start.sh                 # Start production
    ├── prod-stop.sh                  # Stop production
    ├── setup-first-time.sh           # First time setup
    └── quick-start-team.sh           # Team quick start
```

---

## 🔑 Các File Quan Trọng

### Frontend
- **app/layout.tsx** - Root layout với sidebar navigation
- **app/page.tsx** - Trang chủ
- **components/ui/** - Shadcn UI components (reusable)
- **lib/api-client.ts** - API communication layer
- **lib/translations.ts** - Đa ngôn ngữ (Tiếng Việt/English)

### Backend
- **main.py** - FastAPI application entry point
- **api/websocket_inference.py** - WebSocket streaming endpoint
- **ai_models/yolo11_detector.py** - YOLOv11 detection logic
- **core/config.py** - Centralized configuration
- **services/enhanced_services.py** - Business logic layer

### Docker
- **backend-python/Dockerfile.cross-platform** - Multi-stage backend build
- **docker-compose.cross-platform.yml** - Backend dev/prod setup
- **docker-compose.yml** - Full stack orchestration

### Documentation
- **HUONG_DAN.md** - Hướng dẫn chi tiết tiếng Việt ⭐
- **README.md** - Project overview
- **docs/CAU_TRUC_DU_AN.md** - File này

---

## 🎨 Frontend Architecture

```
Next.js App Router
├── Server Components (app/*/page.tsx)
├── Client Components (components/*)
├── API Routes (app/api/*)
└── Utilities (lib/*)
```

**Design System:**
- Tailwind CSS cho styling
- Shadcn UI cho components
- Lucide React cho icons
- Chart.js cho biểu đồ

---

## 🧠 Backend Architecture

```
FastAPI Application
├── API Layer (api/)
│   ├── REST endpoints
│   └── WebSocket endpoints
│
├── Service Layer (services/)
│   ├── Business logic
│   └── Data aggregation
│
├── Core Layer (core/)
│   ├── Configuration
│   ├── Logging
│   └── Error handling
│
└── AI Models (ai_models/)
    ├── YOLOv11 detection
    ├── Training pipeline
    └── Model management
```

---

## 🔄 Data Flow

### Real-time Detection (WebSocket)

```
1. Frontend (Webcam) → Base64 frame
         ↓
2. WebSocket Client → ws://localhost:8080/ws/inference
         ↓
3. Backend receives frame
         ↓
4. YOLOv11 inference
         ↓
5. Detection results → JSON
         ↓
6. WebSocket sends back to client
         ↓
7. Frontend renders bounding boxes
```

### REST API Flow

```
1. Frontend API call (fetch/axios)
         ↓
2. Next.js API route (optional proxy)
         ↓
3. Backend FastAPI endpoint
         ↓
4. Service layer processes
         ↓
5. Database query (if needed)
         ↓
6. Response back to frontend
```

---

## 🚀 NPM Scripts

```json
{
  "dev": "next dev",                    // Chỉ chạy frontend
  "dev:fullstack": "...",               // Chạy cả frontend + backend
  "dev:backend": "python3 ...",         // Chỉ chạy backend
  "build": "next build",                // Build production
  "start": "next start",                // Start production server
  "lint": "eslint ."                    // Lint code
}
```

---

## 🐳 Docker Commands

### Development
```bash
# Backend only
cd backend-python
docker compose -f docker-compose.cross-platform.yml up

# Full stack
docker compose up
```

### Production
```bash
docker compose -f docker-compose.prod.yml up -d
```

---

## 📊 Database Schema

### Main Tables
- **cameras** - Camera configuration
- **drivers** - Driver information
- **trips** - Trip records
- **events** - Detection events
- **detections** - Object detections
- **alerts** - Alert history
- **ai_models** - Model registry

---

## 🔐 Environment Variables

### Frontend (.env.local)
```bash
NEXT_PUBLIC_API_URL=http://localhost:8080
NEXT_PUBLIC_WS_URL=ws://localhost:8080
```

### Backend (.env)
```bash
HOST=0.0.0.0
PORT=8080
DEBUG=true
DATABASE_URL=sqlite:///./adas.db
WEIGHTS_DIR=/app/ai_models/weights
```

---

## 📝 Coding Standards

### Frontend
- TypeScript strict mode
- ESLint + Prettier
- Component naming: PascalCase
- File naming: kebab-case
- Hooks: use-*-*.ts

### Backend
- Python 3.11+ type hints
- Black formatter
- Docstrings for all functions
- File naming: snake_case
- API versioning: /api/v1/*

---

## 🧪 Testing

### Frontend
```bash
npm run test
npm run test:watch
```

### Backend
```bash
pytest backend-python/tests/
pytest --cov=backend-python
```

---

## 📦 Dependencies

### Frontend Main
- next: 16.0.0
- react: 19.2.0
- typescript: ^5
- tailwindcss: ^4.1.9
- lucide-react: ^0.454.0

### Backend Main
- fastapi: 0.104.1
- uvicorn: 0.24.0
- ultralytics: >=8.3.0 (YOLOv11)
- opencv-python: >=4.10.0
- sqlalchemy: >=2.0.36

---

## 🎯 Best Practices

1. **Separation of Concerns**: Frontend & Backend tách biệt hoàn toàn
2. **Type Safety**: TypeScript (FE) + Type Hints (BE)
3. **Docker First**: Development & production đều dùng Docker
4. **API Documentation**: Auto-generated từ FastAPI
5. **Code Reusability**: Components & utilities được reuse
6. **Error Handling**: Centralized error handling
7. **Logging**: Structured logging cho debugging
8. **Security**: Environment variables, CORS, validation

---

## 🔗 Useful Links

- **Frontend:** http://localhost:3000
- **Backend API:** http://localhost:8080
- **API Docs:** http://localhost:8080/docs
- **ReDoc:** http://localhost:8080/redoc

---

**Cấu trúc này được thiết kế theo chuẩn enterprise, dễ mở rộng và bảo trì!** ✨
