"use client";

import React, { useMemo, useState } from "react";
import Image from "next/image";
import { AnimatePresence, motion } from "framer-motion";
import type { Variants } from "framer-motion";

import { Sparkles, Play, Gauge, Activity, Shield, AlertTriangle } from "lucide-react";

import { introSlides } from "@/lib/introData";
import styles from "./IntroSlider.module.scss";

type IntroSliderProps = {
  onComplete: () => void;
};

type Phase = "hero" | "onboarding";

const hexToRgb = (hex: string): string => {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result
    ? `${parseInt(result[1], 16)}, ${parseInt(result[2], 16)}, ${parseInt(result[3], 16)}`
    : "0, 229, 255";
};

export default function IntroSlider({ onComplete }: IntroSliderProps) {
  const [phase, setPhase] = useState<Phase>("hero");
  const [currentSlide, setCurrentSlide] = useState<number>(0);

  const heroBgSrc = "https://images.unsplash.com/photo-1492144534655-ae79c964c9d7?q=80&w=2000&auto=format&fit=crop";

  const redStartIndex = useMemo(() => {
    const byId = introSlides.findIndex((s: any) => s?.id === 1);
    if (byId >= 0) return byId;

    const byTitle = introSlides.findIndex((s: any) =>
      String(s?.title || "").toUpperCase().includes("CẢNH BÁO")
    );
    if (byTitle >= 0) return byTitle;

    return 0;
  }, []);

  const startOnboarding = () => {
    setCurrentSlide(redStartIndex);
    setPhase("onboarding");
  };

  const handleSkip = () => onComplete();

  const handleNextOnboarding = () => {
    const isLast = currentSlide >= introSlides.length - 1;
    if (isLast) onComplete();
    else setCurrentSlide((prev) => prev + 1);
  };

  const handleDotClick = (index: number) => setCurrentSlide(index);

  const slideVariants = {
    enter: { opacity: 0, y: 14, filter: "blur(8px)" },
    center: { opacity: 1, y: 0, filter: "blur(0px)" },
    exit: { opacity: 0, y: -10, filter: "blur(8px)" },
  } satisfies Variants;

  // =============== HERO (màu tím) ===============
  const Hero = (
    <div className={styles.root}>
      <div className={styles.bg}>
        <Image src={heroBgSrc} alt="" fill priority className={styles.bgImg} />
        <div className={styles.vignette} />
        <div className={styles.tint} />
      </div>

      <button className={styles.skip} onClick={handleSkip}>
        Bỏ qua
      </button>

      {/* HUD left */}
      <div className={styles.hudLeft}>
        <div className={styles.hudCard}>
          <div className={styles.hudTitle}>
            <span className={styles.dotCyan} />
            PHÁT HIỆN PHƯƠNG TIỆN
          </div>

          <div className={styles.hudRow}>
            <span className={styles.hudLabel}>Khoảng cách:</span>
            <span className={styles.hudValue}>18.5m</span>
          </div>
          <div className={styles.hudRow}>
            <span className={styles.hudLabel}>Tốc độ:</span>
            <span className={styles.hudValue}>65 km/h</span>
          </div>
          <div className={styles.hudRow}>
            <span className={styles.hudLabel}>Rủi ro:</span>
            <span className={styles.hudValueLow}>Thấp</span>
          </div>

          <div className={styles.progressTrack}>
            <div className={styles.progressFill} />
          </div>
        </div>

        <div className={styles.smallStack}>
          <div className={styles.smallCard}>
            <div className={styles.smallIcon}>
              <Gauge size={18} />
            </div>
            <div>
              <div className={styles.bigNumber}>68</div>
              <div className={styles.smallUnit}>KM/H</div>
            </div>
          </div>

          <div className={styles.smallCard}>
            <div className={styles.smallIconPurple}>
              <Activity size={18} />
            </div>
            <div>
              <div className={styles.aiText}>AI</div>
              <div className={styles.aiSub}>ACTIVE</div>
            </div>
          </div>
        </div>
      </div>

      {/* HUD right */}
      <div className={styles.hudRight}>
        <div className={styles.hudCardRight}>
          <div className={styles.hudTitleRight}>
            <Shield size={16} />
            LÀN ĐƯỜNG AN TOÀN
          </div>
          <div className={styles.hudSubRight}>Xe đang chạy giữa làn</div>
        </div>
      </div>

      {/* Center content */}
      <main className={styles.center}>
        <motion.div
          className={styles.pill}
          initial={{ opacity: 0, y: 8 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.45 }}
        >
          <Sparkles size={16} />
          <span>AI DRIVING ASSISTANCE APP</span>
        </motion.div>

        <motion.h1
          className={styles.title}
          initial={{ opacity: 0, y: 14 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.55, delay: 0.05 }}
        >
          <span className={styles.titleLine1}>AI bảo vệ an toàn</span>
          <span className={styles.titleLine2}>cho mọi chuyến đi</span>
        </motion.h1>

        <motion.p
          className={styles.desc}
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.55, delay: 0.12 }}
        >
          Công nghệ ADAS thế hệ mới với AI nhận diện thời gian thực, cảnh báo thông minh.
        </motion.p>

        <motion.div
          className={styles.alertCard}
          initial={{ opacity: 0, y: 14 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.55, delay: 0.18 }}
        >
          <div className={styles.alertLeft}>
            <AlertTriangle size={18} />
          </div>
          <div className={styles.alertBody}>
            <div className={styles.alertTitle}>Cảnh báo khoảng cách</div>
            <div className={styles.alertSub}>Giữ khoảng cách an toàn</div>
          </div>
        </motion.div>

        <motion.button
          className={styles.cta}
          onClick={startOnboarding}
          initial={{ opacity: 0, y: 16, scale: 0.98 }}
          animate={{ opacity: 1, y: 0, scale: 1 }}
          transition={{ duration: 0.55, delay: 0.25 }}
          whileHover={{ scale: 1.02 }}
          whileTap={{ scale: 0.98 }}
        >
          <span className={styles.ctaIcon}>
            <Play size={18} />
          </span>
          Trải nghiệm AI
        </motion.button>

        <div className={styles.metrics}>
          <div className={styles.metric}>
            <span className={styles.dotCyan} />
            AI phát hiện theo thời gian thực
          </div>
          <div className={styles.metric}>
            <span className={styles.dotPurple} />
            Độ chính xác 99.8%
          </div>
          <div className={styles.metric}>
            <span className={styles.dotGreen} />
            Bảo vệ tài xế 24/7
          </div>
        </div>

        <div className={styles.featureList}>
          <div className={styles.featureItem}>Phát hiện va chạm trước 3-5 giây</div>
          <div className={styles.featureItem}>Cảnh báo điểm mù tự động</div>
          <div className={styles.featureItem}>Hỗ trợ giữ làn đường thông minh</div>
        </div>
      </main>
    </div>
  );

  // =============== ONBOARDING (slide) ===============
  const slide: any = introSlides[currentSlide];
  const Icon = slide?.icon;

  const onboardBgSrc: string = slide?.bg;
  const accent = slide?.themeColor;

  const Onboarding = (
    <div
      className={styles.onboardRoot}
      style={
        {
          ["--accent" as any]: accent,
          ["--accent-rgb" as any]: hexToRgb(accent),
        } as React.CSSProperties
      }
    >
      <div className={styles.onboardBg}>
        <AnimatePresence mode="popLayout" initial={false}>
          <motion.div
            key={currentSlide}
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.5, ease: "easeInOut" }}
            style={{ position: "absolute", inset: 0 }}
          >
            <Image src={onboardBgSrc} alt="" fill priority className={styles.onboardBgImg} style={{ objectFit: 'cover' }} />
          </motion.div>
        </AnimatePresence>
        <div className={styles.onboardVignette} />

        {slide?.id === 1 && (
          <div
            aria-hidden
            style={{
              position: "absolute",
              inset: 0,
              zIndex: 2,
              pointerEvents: "none",
              background:
                "radial-gradient(45% 45% at 50% 42%, rgba(var(--accent-rgb), 0.30), transparent 70%)," +
                "radial-gradient(35% 35% at 70% 55%, rgba(var(--accent-rgb), 0.16), transparent 75%)," +
                "linear-gradient(180deg, rgba(0,0,0,0.10), transparent 45%, rgba(0,0,0,0.20))",
              mixBlendMode: "screen",
            }}
          />
        )}
        {slide?.id === 2 && (
          <>
            <div className={`${styles.scanBox} ${styles.scanBox1}`} />
            <div className={`${styles.scanBox} ${styles.scanBox2}`} />
            <div className={`${styles.scanBox} ${styles.scanBox3}`} />
          </>
        )}
        {slide?.id === 3 && (
        <svg
          className={styles.predWave}
          viewBox="0 0 520 240"
          preserveAspectRatio="none"
          aria-hidden
        >
          <path
            d="M0,150 C90,60 200,220 320,140 C400,88 460,120 520,88"
            fill="none"
            stroke="currentColor"
            strokeWidth="4"
            strokeLinecap="round"
          />
        </svg>
        )}
        {slide?.id === 4 && (
          <div className={styles.zapCircle} aria-hidden>
            <Icon size={26} />
          </div>
        )}
        {slide?.id === 5 && (
          <div className={styles.safeRings} aria-hidden>
            <div className={styles.safeRingOuter} />
            <div className={styles.safeRingMid} />
            <div className={styles.safeRingInner} />
          </div>
        )}
      </div>

      <button className={styles.onboardSkip} onClick={handleSkip}>
        Bỏ qua
      </button>

      {/* progress ở góc phải */}
      <div className={styles.onboardTopProgress}>
        {introSlides.map((_: any, idx: number) => (
          <span
            key={idx}
            className={`${styles.onboardSeg} ${idx === currentSlide ? styles.onboardSegActive : ""}`}
          />
        ))}
      </div>

      <AnimatePresence mode="wait">
        <motion.div
          key={currentSlide}
          className={styles.onboardCenter}
          variants={slideVariants}
          initial="enter"
          animate="center"
          exit="exit"
          transition={{ duration: 0.35 }}
        >
          <div className={styles.onboardIconInner}>
            <Icon size={28} />
          </div>

          <div className={styles.onboardText}>
            {slide?.title ? <div className={styles.onboardTitle}>{slide.title}</div> : null}

            <div className={styles.onboardSubtitle}>
              {slide?.subtitle || "Mỗi giây đều tiềm ẩn rủi ro"}
            </div>
          </div>

          <div className={styles.onboardDots}>
            {introSlides.map((_: any, idx: number) => (
              <button
                key={idx}
                className={`${styles.onboardDot} ${idx === currentSlide ? styles.onboardDotActive : ""}`}
                onClick={() => handleDotClick(idx)}
                aria-label={`Go to slide ${idx + 1}`}
              />
            ))}
          </div>

          <motion.button
            className={styles.onboardBtn}
            onClick={handleNextOnboarding}
            whileHover={{ scale: 1.02 }}
            whileTap={{ scale: 0.98 }}
          >
            {slide?.buttonText || "Tiếp tục"} <span className={styles.onboardArrow}>›</span>
          </motion.button>
        </motion.div>
      </AnimatePresence>
    </div>
  );

  return phase === "hero" ? Hero : Onboarding;
}