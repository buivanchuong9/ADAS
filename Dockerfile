# ============================================
# ADAS Frontend - Production Docker Image
# Next.js 16 with Turbopack
# ============================================

# Stage 1: Dependencies
FROM node:20-alpine AS deps

WORKDIR /app

# Copy package files
COPY package.json pnpm-lock.yaml* package-lock.json* yarn.lock* ./

# Install dependencies based on lockfile
RUN \
    if [ -f pnpm-lock.yaml ]; then \
    corepack enable pnpm && pnpm install --frozen-lockfile; \
    elif [ -f package-lock.json ]; then \
    npm ci; \
    elif [ -f yarn.lock ]; then \
    yarn install --frozen-lockfile; \
    else \
    echo "No lockfile found" && exit 1; \
    fi

# Stage 2: Builder
FROM node:20-alpine AS builder

WORKDIR /app

# Copy dependencies from deps stage
COPY --from=deps /app/node_modules ./node_modules

# Copy source code
COPY . .

# Set environment variables for build
ENV NEXT_TELEMETRY_DISABLED=1
ENV NODE_ENV=production

# Build Next.js application
RUN \
    if [ -f pnpm-lock.yaml ]; then \
    corepack enable pnpm && pnpm run build; \
    elif [ -f package-lock.json ]; then \
    npm run build; \
    elif [ -f yarn.lock ]; then \
    yarn build; \
    else \
    npm run build; \
    fi

# Stage 3: Runner (Production)
FROM node:20-alpine AS runner

WORKDIR /app

ENV NODE_ENV=production
ENV NEXT_TELEMETRY_DISABLED=1

# Create non-root user
RUN addgroup --system --gid 1001 nodejs
RUN adduser --system --uid 1001 nextjs

# Copy built application
COPY --from=builder /app/public ./public
COPY --from=builder --chown=nextjs:nodejs /app/.next/standalone ./
COPY --from=builder --chown=nextjs:nodejs /app/.next/static ./.next/static

# Switch to non-root user
USER nextjs

# Expose port
EXPOSE 3000

ENV PORT=3000
ENV HOSTNAME="0.0.0.0"

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD node -e "require('http').get('http://localhost:3000', (r) => {process.exit(r.statusCode === 200 ? 0 : 1)})"

# Start Next.js
CMD ["node", "server.js"]
