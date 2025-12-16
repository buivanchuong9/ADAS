import type React from "react"
import type { Metadata, Viewport } from "next"
import { Geist, Geist_Mono } from "next/font/google"
import { Analytics } from "@vercel/analytics/next"
import "./globals.css"

const _geist = Geist({ subsets: ["latin"] })
const _geistMono = Geist_Mono({ subsets: ["latin"] })

export const metadata: Metadata = {
  title: "ADAS - Hệ Thống Hỗ Trợ Lái Xe",
  description: "Nền tảng giám sát an toàn lái xe thông minh",
  generator: "v0.app",
}

export const viewport: Viewport = {
  width: 'device-width',
  initialScale: 1,
  maximumScale: 5,
}

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode
}>) {
  return (
    <html lang="vi">
      <body className={`font-sans antialiased`}>
        <div className="min-h-screen p-4 md:p-6 lg:p-8">
          {children}
        </div>
        <Analytics />
      </body>
    </html>
  )
}
