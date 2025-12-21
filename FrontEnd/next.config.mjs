/** @type {import('next').NextConfig} */
const nextConfig = {
  output: 'export', // ⬅️ BẮT BUỘC để tạo static HTML

  distDir: 'view', // ⬅️ folder mày muốn (thay cho out)

  typescript: {
    ignoreBuildErrors: true,
  },

  images: {
    unoptimized: true,
  },
}

export default nextConfig