/** @type {import('next').NextConfig} */
const nextConfig = {
  output: 'export', // ⬅️ BẮT BUỘC để tạo static HTML

  distDir: 'view', // ⬅️ folder mày muốn (thay cho out)

  typescript: {
    ignoreBuildErrors: true,
  },

  images: {
    unoptimized: true,
    remotePatterns: [
      {
        protocol: 'https',
        hostname: 'images.unsplash.com',
      },
      {
        protocol: 'https',
        hostname: 'plus.unsplash.com',
      }
    ],
  },
}

export default nextConfig