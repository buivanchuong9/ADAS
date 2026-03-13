"use client";

import { useEffect, useRef, useState, useCallback } from "react";

export function ScrollCarTrack() {
  const [scrollProgress, setScrollProgress] = useState(0); // 0 → 1
  const [scrollDir, setScrollDir] = useState<"down" | "up" | "idle">("idle");
  const [isVisible, setIsVisible] = useState(false);
  
  const lastScrollY = useRef(0);
  const idleTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const hideTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const rafId = useRef<number | null>(null);

  const handleScroll = useCallback((e?: Event) => {
    if (rafId.current) cancelAnimationFrame(rafId.current);
    rafId.current = requestAnimationFrame(() => {
      let scrollTop = 0;
      let scrollHeight = 0;

      if (!e || e.target === document || e.target === window) {
        scrollTop = document.documentElement.scrollTop || document.body.scrollTop;
        scrollHeight = document.documentElement.scrollHeight - document.documentElement.clientHeight;
      } else {
        const target = e.target as HTMLElement;
        // Ignore tiny scrollable areas (like dropdowns or tiny sidebars)
        if (target.scrollHeight - target.clientHeight < 50) return;
        scrollTop = target.scrollTop;
        scrollHeight = target.scrollHeight - target.clientHeight;
      }

      if (scrollHeight <= 0) return;

      const progress = scrollTop / scrollHeight;

      const dir = scrollTop > lastScrollY.current ? "down" : "up";
      lastScrollY.current = scrollTop;
      
      setScrollProgress(progress);
      setScrollDir(dir);
      setIsVisible(true);

      // Reset moving state (lights stop flashing, vibration stops) after 300ms of no scroll
      if (idleTimer.current) clearTimeout(idleTimer.current);
      idleTimer.current = setTimeout(() => setScrollDir("idle"), 300);

      // Auto-hide entire track after 5 seconds of inactivity
      if (hideTimer.current) clearTimeout(hideTimer.current);
      hideTimer.current = setTimeout(() => {
        setIsVisible(false);
      }, 5000); 
    });
  }, []);

  useEffect(() => {
    // Add event listener with capture: true to catch scroll events from ANY inside container
    window.addEventListener("scroll", handleScroll, { passive: true, capture: true });
    // Keep it updated initially and show it for 3s
    handleScroll();
    return () => {
      window.removeEventListener("scroll", handleScroll, { capture: true });
      if (idleTimer.current) clearTimeout(idleTimer.current);
      if (hideTimer.current) clearTimeout(hideTimer.current);
      if (rafId.current) cancelAnimationFrame(rafId.current);
    };
  }, [handleScroll]);

  return (
    <div 
      className={`wow-fade-wrapper ${isVisible ? "wow-fade-in" : "wow-fade-out"}`}
    >
      <TrackSide side="left" progress={scrollProgress} dir={scrollDir} />
      <TrackSide side="right" progress={scrollProgress} dir={scrollDir} />
    </div>
  );
}

function TrackSide({
  side,
  progress,
  dir,
}: {
  side: "left" | "right";
  progress: number;
  dir: "down" | "up" | "idle";
}) {
  const isMoving = dir !== "idle";
  const isReversing = dir === "up";
  const carPct = progress * 100;

  return (
    <div className={`wow-track-root wow-track-${side}`} aria-hidden="true">
      {/* Sleek Line - Fixed on Screen Edge */}
      <div className="wow-rail">
        {/* Glow progress fill */}
        <div className="wow-rail-fill" style={{ height: `${carPct}%` }} />
      </div>

      {/* Car Wrapper */}
      <div
        className={`wow-car-wrapper ${isMoving ? "wow-car-active" : ""}`}
        style={{ top: `${carPct}%` }}
      >
        <div
          className="wow-car-flipper"
          style={{ transform: `scaleY(${isReversing ? -1 : 1})` }}
        >
          <div className={`wow-car-vibes ${isMoving ? "wow-moving" : ""}`}>
            {/* Tech Cyber-Car (Scaled up from 14x28 to 22x44) */}
            <svg
              viewBox="0 0 24 48"
              width="22"
              height="44"
              className="wow-car-svg"
            >
              {/* Soft Drop Shadow under car */}
              <ellipse cx="12" cy="24" rx="10" ry="22" fill="rgba(0,0,0,0.5)" filter="blur(3px)"/>
              
              {/* Core Body - Dark Cyber Aesthetic */}
              <rect x="4" y="4" width="16" height="40" rx="4" fill="#0f172a" />
              <rect x="5" y="5" width="14" height="38" rx="3" fill="#1e293b" />
              
              {/* Glass Canopy (Sleek minimalist window) */}
              <path d="M 6 16 L 18 16 L 15 26 L 9 26 Z" fill="#000" />
              <path d="M 7 34 L 17 34 L 14 40 L 10 40 Z" fill="#000" />
              
              {/* Center Cyber Accent Stripe */}
              <line x1="12" y1="10" x2="12" y2="38" stroke="rgba(255, 122, 26, 0.4)" strokeWidth="0.8" />
              
              {/* Front Headlights (Only glow strong when moving down/forward) */}
              <rect x="5" y="4" width="5" height="1.5" rx="0.5" 
                fill={isMoving && !isReversing ? "#fff" : "#fbbf24"} 
                className={isMoving && !isReversing ? "wow-glow-front" : ""} 
              />
              <rect x="14" y="4" width="5" height="1.5" rx="0.5" 
                fill={isMoving && !isReversing ? "#fff" : "#fbbf24"} 
                className={isMoving && !isReversing ? "wow-glow-front" : ""} 
              />
              
              {/* Rear Taillights (Only glow strong when reversing up) */}
              <rect x="5" y="42.5" width="5" height="1.5" rx="0.5" 
                fill={isReversing && isMoving ? "#ffcccc" : "#dc2626"} 
                className={isReversing && isMoving ? "wow-glow-rear" : ""} 
              />
              <rect x="14" y="42.5" width="5" height="1.5" rx="0.5" 
                fill={isReversing && isMoving ? "#ffcccc" : "#dc2626"} 
                className={isReversing && isMoving ? "wow-glow-rear" : ""} 
              />
              
              {/* Edge Highlights */}
              <line x1="4.5" y1="12" x2="4.5" y2="36" stroke="rgba(255,255,255,0.15)" strokeWidth="0.5" />
              <line x1="19.5" y1="12" x2="19.5" y2="36" stroke="rgba(255,255,255,0.15)" strokeWidth="0.5" />
            </svg>
          </div>
        </div>

        {/* Minimal Progress Value - Only visible when scrolling near edges */}
        <div className={`wow-badge ${isMoving ? "wow-badge-visible" : ""}`}>
          <span className="wow-num">{Math.round(progress * 100)}</span>
          <span className="wow-pct">%</span>
        </div>
      </div>
    </div>
  );
}
