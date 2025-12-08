# 🚗 ADAS Frontend

> **Advanced Driver Assistance System** - Next.js Web Application

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Next.js](https://img.shields.io/badge/Next.js-14+-black.svg)](https://nextjs.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5+-blue.svg)](https://www.typescriptlang.org/)

---

## ✨ Tính Năng

- 🎨 **Modern UI/UX** với Tailwind CSS và shadcn/ui
- 📊 **Real-time Dashboard** - WebSocket integration
- 📈 **Analytics & Reporting** 
- 🎥 **Live Detection Viewer**
- 🚨 **Alert Management**
- 📱 **Responsive Design** - Mobile friendly

---

## 🚀 Quick Start

### Prerequisites
- Node.js 18+ 
- pnpm (recommended) or npm
- Backend API running at http://localhost:8000

### Installation & Run

```bash
# 1. Install dependencies
pnpm install

# 2. Configure environment
cp .env.example .env.local
# Edit .env.local and set NEXT_PUBLIC_API_URL

# 3. Run development server
pnpm dev
```

✅ **Frontend:** http://localhost:3000

### Production Build

```bash
pnpm build
pnpm start
```

### Docker

```bash
docker-compose up -d
```

---

## 📁 Project Structure

```
ADAS/
├── app/                    # Next.js pages (App Router)
│   ├── dashboard/         # Main dashboard
│   ├── adas/              # Detection viewer
│   ├── analytics/         # Analytics & reports
│   └── api/               # API routes (proxy to backend)
├── components/            # React components
│   ├── ui/               # shadcn/ui components
│   └── adas/             # ADAS-specific components
├── hooks/                # Custom React hooks
├── lib/                  # Utilities & helpers
│   ├── api-client.ts     # API integration
│   └── utils.ts          # Common utilities
├── public/               # Static assets
└── styles/               # Global styles
```

---

## 🔧 Configuration

### Environment Variables

Create `.env.local`:

```env
# Backend API URL
NEXT_PUBLIC_API_URL=http://localhost:8000
```

### API Integration

The frontend connects to the backend API (located at `../backend-python`).

Make sure the backend is running before starting the frontend.

---

## 🚀 Development

```bash
# Install dependencies
pnpm install

# Run dev server with hot reload
pnpm dev

# Type checking
pnpm type-check

# Linting
pnpm lint

# Build for production
pnpm build
```

---

## 📝 License

Proprietary


Contributions, issues và feature requests đều được chào đón!

1. Fork dự án
2. Tạo branch: `git checkout -b feature/tinh-nang-moi`
3. Commit: `git commit -m 'Thêm tính năng mới'`
4. Push: `git push origin feature/tinh-nang-moi`
5. Tạo Pull Request

---

## 📝 License

MIT License - Xem [LICENSE](LICENSE) để biết chi tiết

---

## 📧 Liên Hệ

- **GitHub:** [@buivanchuong9](https://github.com/buivanchuong9)
- **Repository:** [ADAS](https://github.com/buivanchuong9/ADAS)

---

**Made with ❤️ in Vietnam**
