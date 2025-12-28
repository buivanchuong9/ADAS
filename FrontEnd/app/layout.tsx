import type React from "react"
import type { Metadata, Viewport } from "next"
import { Orbitron, Rajdhani, Inter } from "next/font/google"
import { Analytics } from "@vercel/analytics/next"
import { AuthProvider } from "@/contexts/auth-context"
import "./globals.css"

const orbitron = Orbitron({
  subsets: ["latin"],
  variable: '--font-orbitron',
  weight: ['400', '500', '600', '700', '800', '900']
})

const rajdhani = Rajdhani({
  subsets: ["latin"],
  variable: '--font-rajdhani',
  weight: ['300', '400', '500', '600', '700']
})

const inter = Inter({
  subsets: ["latin"],
  variable: '--font-inter',
  weight: ['300', '400', '500', '600', '700', '800']
})

export const metadata: Metadata = {
  title: "ADAS - Hệ Thống Hỗ Trợ Lái Xe",
  description: "Nền tảng giám sát an toàn lái xe thông minh với công nghệ AI tiên tiến",
  generator: "v0.app",
  icons: {
    icon: [
      { url: '/favicon.ico', sizes: '32x32', type: 'image/x-icon' },
      { url: '/adas-logo-192.png', sizes: '192x192', type: 'image/png' },
      { url: '/adas-logo-512.png', sizes: '512x512', type: 'image/png' },
    ],
    apple: [
      { url: '/apple-touch-icon.png', sizes: '180x180', type: 'image/png' },
    ],
  },
  openGraph: {
    title: "ADAS - Advanced Driver Assistance Systems",
    description: "Nền tảng giám sát an toàn lái xe thông minh với công nghệ AI tiên tiến",
    images: [
      {
        url: '/adas-logo.jpg',
        width: 1200,
        height: 630,
        alt: 'ADAS Logo',
      },
    ],
    type: 'website',
  },
  twitter: {
    card: 'summary_large_image',
    title: "ADAS - Advanced Driver Assistance Systems",
    description: "Nền tảng giám sát an toàn lái xe thông minh với công nghệ AI tiên tiến",
    images: ['/adas-logo.jpg'],
  },
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
    <html lang="vi" className={`h-full ${orbitron.variable} ${rajdhani.variable} ${inter.variable}`}>
      <body className={`${rajdhani.className} antialiased h-full overflow-x-hidden`}>
        <AuthProvider>
          <div className="min-h-screen h-full w-full">
            {children}
          </div>
        </AuthProvider>
        <Analytics />
      </body>
    </html>
  )
}
