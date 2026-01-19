import type { LucideIcon } from "lucide-react";
import { Shield, Zap, Brain, Eye, AlertTriangle, Sparkles } from "lucide-react";

export type IntroSlide = {
  id: number;
  bg: string;
  themeColor: string;
  gradientFrom?: string;
  gradientTo?: string;
  icon: LucideIcon;
  title: string;
  subtitle: string;
  description: string;
  features: string[];
  buttonText: string;
  isFinal?: boolean;
};

export const introSlides: IntroSlide[] = [
    {
      id: 1,
      bg: "/images/intro/slide-2.png",
      themeColor: "#FF3333",
      gradientFrom: "#FF3333",
      gradientTo: "#CC0000",
      icon: AlertTriangle,
      title: "CẢNH BÁO RỦI RO",
      subtitle: "Mỗi giây đều tiềm ẩn rủi ro",
      description: "",
      features: [
      ],
      buttonText: "Tiếp tục",
    },
    {
      id: 2,
      bg: "/images/intro/slide-3.png",
      themeColor: "#00E5FF",
      gradientFrom: "#00E5FF",
      gradientTo: "#00B8D4",
      icon: Eye,
      title: "AI NHÌN THẤY TẤT CẢ",
      subtitle: "360° quan sát không ngừng nghỉ",
      description: "",
      features: [],
      buttonText: "Tiếp tục",
    },
    {
      id: 3,
      bg: "/images/intro/slide-4.png",
      themeColor: "#A855F7",
      gradientFrom: "#A855F7",
      gradientTo: "#7C3AED",
      icon: Brain,
      title: "DỰ ĐOÁN TRƯỚC",
      subtitle: "AI biết nguy hiểm sắp xảy ra",
      description: "",
      features: [],
      buttonText: "Tiếp tục",
    },
    {
      id: 4,
      bg: "/images/intro/slide-5.png",   // ảnh bạn nói
      themeColor: "#FFB800",
      gradientFrom: "#FFB800",
      gradientTo: "#FF8C00",
      icon: Zap,
      title: "CẢNH BÁO TỨC THÌ",
      subtitle: "Phản ứng nhanh hơn con người",
      description: "",
      features: [],
      buttonText: "Tiếp tục",
    },
    {
      id: 5,
      bg: "/images/intro/slide-6.png",
      themeColor: "#00FF88",
      gradientFrom: "#00FF88",
      gradientTo: "#00CC6A",
      icon: Shield,
      title: "AN TOÀN TUYỆT ĐỐI",
      subtitle: "Bảo vệ mọi chuyến đi của bạn",
      description: "",
      features: [],
      buttonText: "Khám phá ngay",
    }
  ];