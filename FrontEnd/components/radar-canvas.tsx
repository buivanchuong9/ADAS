"use client";

import { useEffect, useRef } from "react";

export function RadarCanvas() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const frameRef = useRef<number>(0);
  const angleRef = useRef(0);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const W = canvas.width;
    const H = canvas.height;
    const cx = W / 2;
    const cy = H / 2;
    const R = Math.min(W, H) / 2 - 4;

    const dots: { angle: number; r: number; alpha: number }[] = [];
    for (let i = 0; i < 14; i++) {
      dots.push({
        angle: Math.random() * Math.PI * 2,
        r: Math.random() * R * 0.85 + R * 0.1,
        alpha: Math.random() * 0.8 + 0.2,
      });
    }

    function draw() {
      if (!canvas) return;
      const ctx2d = ctx;
      ctx2d.clearRect(0, 0, W, H);

      const rings = [0.3, 0.55, 0.75, 1];
      rings.forEach((factor) => {
        ctx2d.beginPath();
        ctx2d.arc(cx, cy, R * factor, 0, Math.PI * 2);
        ctx2d.strokeStyle = "rgba(249,115,22,0.15)";
        ctx2d.lineWidth = 1;
        ctx2d.stroke();
      });

      ctx2d.strokeStyle = "rgba(249,115,22,0.1)";
      ctx2d.lineWidth = 1;
      for (let a = 0; a < 4; a++) {
        ctx2d.beginPath();
        ctx2d.moveTo(cx, cy);
        const angle = (a * Math.PI) / 2;
        ctx2d.lineTo(cx + Math.cos(angle) * R, cy + Math.sin(angle) * R);
        ctx2d.stroke();
      }

      const sweepAngle = angleRef.current;

      const anyCtx = ctx2d as any;
      const gradient = anyCtx.createConicGradient
        ? anyCtx.createConicGradient(sweepAngle - Math.PI / 2, cx, cy)
        : null;

      if (gradient) {
        gradient.addColorStop(0, "rgba(249,115,22,0)");
        gradient.addColorStop(0.25, "rgba(249,115,22,0.35)");
        gradient.addColorStop(0.251, "rgba(249,115,22,0)");
        gradient.addColorStop(1, "rgba(249,115,22,0)");
        ctx2d.beginPath();
        ctx2d.moveTo(cx, cy);
        ctx2d.arc(cx, cy, R, 0, Math.PI * 2);
        ctx2d.fillStyle = gradient;
        ctx2d.fill();
      } else {
        ctx2d.beginPath();
        ctx2d.moveTo(cx, cy);
        ctx2d.arc(cx, cy, R, sweepAngle - 0.6, sweepAngle);
        ctx2d.closePath();
        const g = ctx2d.createRadialGradient(cx, cy, 0, cx, cy, R);
        g.addColorStop(0, "rgba(249,115,22,0.0)");
        g.addColorStop(0.7, "rgba(249,115,22,0.25)");
        g.addColorStop(1, "rgba(249,115,22,0.08)");
        ctx2d.fillStyle = g;
        ctx2d.fill();
      }

      ctx2d.beginPath();
      ctx2d.moveTo(cx, cy);
      ctx2d.lineTo(cx + Math.cos(sweepAngle) * R, cy + Math.sin(sweepAngle) * R);
      ctx2d.strokeStyle = "rgba(249,115,22,0.9)";
      ctx2d.lineWidth = 1.5;
      ctx2d.stroke();

      dots.forEach((dot) => {
        let diff = (sweepAngle - dot.angle + Math.PI * 4) % (Math.PI * 2);
        if (diff < 0.8) {
          const alpha = (1 - diff / 0.8) * dot.alpha;

          ctx2d.beginPath();
          ctx2d.arc(
            cx + Math.cos(dot.angle) * dot.r,
            cy + Math.sin(dot.angle) * dot.r,
            3,
            0,
            Math.PI * 2,
          );
          ctx2d.fillStyle = `rgba(249,115,22,${alpha})`;
          ctx2d.fill();

          ctx2d.beginPath();
          ctx2d.arc(
            cx + Math.cos(dot.angle) * dot.r,
            cy + Math.sin(dot.angle) * dot.r,
            6,
            0,
            Math.PI * 2,
          );
          ctx2d.fillStyle = `rgba(249,115,22,${alpha * 0.3})`;
          ctx2d.fill();
        }
      });

      ctx2d.beginPath();
      ctx2d.arc(cx, cy, 3, 0, Math.PI * 2);
      ctx2d.fillStyle = "#f97316";
      ctx2d.fill();

      angleRef.current += 0.025;
      frameRef.current = requestAnimationFrame(draw);
    }

    draw();
    return () => cancelAnimationFrame(frameRef.current);
  }, []);

  return (
    <canvas
      ref={canvasRef}
      width={180}
      height={180}
      className="opacity-90"
    />
  );
}

