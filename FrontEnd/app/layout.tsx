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
    <html lang="vi" className="h-full">
      <body className={`font-sans antialiased h-full overflow-x-hidden`}>
        <div className="min-h-screen h-full w-full">
          {children}
        </div>
        <Analytics />
      </body>
    </html>
  )
}
