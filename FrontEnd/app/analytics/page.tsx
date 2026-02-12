"use client";

import Highcharts from "highcharts";
import HighchartsReact from "highcharts-react-official";
import "highcharts/highcharts-3d";
import "highcharts/modules/cylinder";

import { useRef, useEffect, useState } from "react";
import { useLanguage } from "@/contexts/language-context";
import { Sidebar } from "@/components/sidebar";
import { MobileNav } from "@/components/mobile-nav";
import { Card } from "@/components/ui/card";
import { TrendingUp, Clock, Gauge, AlertTriangle } from "lucide-react";

/* ================= API BASE =================
 * Recommended: set NEXT_PUBLIC_API_URL in .env.local to your backend base
 * e.g. NEXT_PUBLIC_API_URL=https://adas-api.aiotlab.edu.vn
 */
const API_BASE = process.env.NEXT_PUBLIC_API_URL || "https://adas-api.aiotlab.edu.vn";

/* ================= TYPES ================= */

type ChartPoint = {
  time?: string;
  trip?: string;
  speed?: number;
  fatigue?: number;
  score?: number;
  color?: string;
};

type CardsType = {
  distance?: number | string;
  drivingTime?: number | string;
  averageSpeed?: number | string;
  safetyScore?: number | string;
};

export default function Analytics() {
  const { t } = useLanguage();

  /* ================= STATE ================= */
  const [cards, setCards] = useState<CardsType>({});
  const [speedData, setSpeedData] = useState<ChartPoint[]>([]);
  const [fatigueData, setFatigueData] = useState<ChartPoint[]>([]);
  const [tripComparisonData, setTripComparisonData] = useState<ChartPoint[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [errorMsg, setErrorMsg] = useState<string | null>(null);

  const speedChartRef = useRef<any>(null);
  const fatigueChartRef = useRef<any>(null);

  /* ================= HELPERS ================= */
  const isJsonResponse = (res: Response) =>
    res.headers?.get("content-type")?.includes("application/json");

  const safeParseJson = async (res: Response) => {
    try {
      if (!res) return null;
      if (isJsonResponse(res)) return await res.json();
      return null;
    } catch (e) {
      console.warn("Failed parse json:", e);
      return null;
    }
  };

  /* ================= FETCH API ================= */
  useEffect(() => {
    let mounted = true;

    async function fetchData() {
      setLoading(true);
      setErrorMsg(null);

      const token = typeof window !== "undefined" ? localStorage.getItem("token") : null;
      const headers: Record<string, string> = { "Content-Type": "application/json" };
      if (token) headers.Authorization = `Bearer ${token}`;

      try {
        // Use allSettled so partial success is handled
        const promises = [
          fetch(`${API_BASE}/api/analytics/summary`, { headers }),
          fetch(`${API_BASE}/api/analytics/speed-over-time`, { headers }),
          fetch(`${API_BASE}/api/analytics/fatigue-over-time`, { headers }),
          fetch(`${API_BASE}/api/analytics/safety-score-comparison`, { headers }),
        ];

        const results = await Promise.allSettled(promises);

        // unwrap responses (could be rejected)
        const responses = results.map((r) =>
          r.status === "fulfilled" ? (r.value as Response) : null
        );

        // SUMMARY
        const summaryRes = responses[0];
        if (summaryRes && summaryRes.ok) {
          const json = await safeParseJson(summaryRes);
          console.log("SUMMARY API:", summaryRes.status, json);
          if (mounted && json) {
            setCards({
              distance: json.total_distance ?? "-",
              drivingTime: json.total_trips ?? "-",
              averageSpeed: json.avg_speed ?? "-",
              safetyScore: json.avg_safety_score ?? "-",
            });
          }
        } else {
          console.warn("SUMMARY missing or error:", summaryRes?.status);
        }

        // SPEED
        const speedRes = responses[1];
        if (speedRes) {
          console.log("SPEED status:", speedRes.status);
          const json = await safeParseJson(speedRes);
          console.log("SPEED API RAW:", json);
          let mappedSpeed: ChartPoint[] = [];
          if (json && Array.isArray(json.labels) && Array.isArray(json.data)) {
            mappedSpeed = json.labels.map((label: string, idx: number) => ({
              time: label,
              speed: Number(json.data?.[idx] ?? 0),
            }));
          } else {
            console.warn("SPEED API format unexpected, will leave empty or set fallback");
          }
          if (mounted) {
            console.table(mappedSpeed);
            setSpeedData(mappedSpeed);
          }
        } else {
          console.warn("SPEED fetch failed");
        }

        // FATIGUE
        const fatigueRes = responses[2];
        if (fatigueRes) {
          console.log("FATIGUE status:", fatigueRes.status);
          const json = await safeParseJson(fatigueRes);
          console.log("FATIGUE API RAW:", json);
          let mappedFatigue: ChartPoint[] = [];
          if (json && Array.isArray(json.labels) && Array.isArray(json.data)) {
            mappedFatigue = json.labels.map((label: string, idx: number) => ({
              time: label,
              fatigue: Number(json.data?.[idx] ?? 0),
            }));
          } else {
            console.warn("FATIGUE API format unexpected, will leave empty or set fallback");
          }
          if (mounted) {
            console.table(mappedFatigue);
            setFatigueData(mappedFatigue);
          }
        } else {
          console.warn("FATIGUE fetch failed");
        }

        // SAFETY
        const safetyRes = responses[3];
        if (safetyRes) {
          console.log("SAFETY status:", safetyRes.status);
          const json = await safeParseJson(safetyRes);
          console.log("SAFETY API RAW:", json);
          let mappedSafety: ChartPoint[] = [];
          if (json && Array.isArray(json.labels) && Array.isArray(json.data)) {
            mappedSafety = json.labels.map((label: string, idx: number) => ({
              trip: label,
              score: Number(json.data?.[idx] ?? 0),
              color: (Array.isArray(json.colors) && json.colors[idx]) ? json.colors[idx] : "#10b981",
            }));
          } else {
            console.warn("SAFETY API format unexpected, will leave empty or set fallback");
          }
          if (mounted) {
            console.table(mappedSafety);
            setTripComparisonData(mappedSafety);
          }
        } else {
          console.warn("SAFETY fetch failed");
        }
      } catch (err) {
        console.error("Analytics API error:", err);
        if (mounted) setErrorMsg("Lỗi khi gọi API. Kiểm tra console.");
      } finally {
        if (mounted) setLoading(false);
      }
    }

    fetchData();

    return () => {
      mounted = false;
    };
  }, []);

  /* ================= NEON EFFECT (guarded) ================= */
  useEffect(() => {
    let hue = 0;
    const interval = setInterval(() => {
      hue = (hue + 2) % 360;
      const color1 = `hsl(${hue},100%,60%)`;
      const color2 = `hsl(${(hue + 60) % 360},100%,60%)`;

      // only update if series exist (avoid runtime error)
      try {
        if (speedChartRef.current?.chart?.series?.length) {
          speedChartRef.current.chart.series[0].update(
            { color: color1, shadow: { color: color1, width: 25 } },
            true
          );
        }
        if (fatigueChartRef.current?.chart?.series?.length) {
          fatigueChartRef.current.chart.series[0].update({ color: color2 }, true);
        }
      } catch (e) {
        // swallow any update errors (safety)
      }
    }, 300); // 300ms to reduce redraw pressure

    return () => clearInterval(interval);
  }, []);

  /* ================= DEBUG CHART DATA ================= */
  useEffect(() => {
    console.log("Speed Chart Data:", speedData);
    console.log("Fatigue Chart Data:", fatigueData);
    console.log("Safety Chart Data:", tripComparisonData);
  }, [speedData, fatigueData, tripComparisonData]);

  /* ================= CHART OPTIONS (3D added) ================= */
  const speedChartOptions = {
    chart: {
      type: "line",
      options3d: {
        enabled: true,
        alpha: 10,
        beta: 15,
        depth: 50,
        viewDistance: 25,
      },
    },
    title: { text: null },
    xAxis: { categories: speedData.map((d) => d.time ?? "") },
    plotOptions: {
      line: {
        // depth isn't a documented option for line but harmless to include
        depth: 25,
      },
    },
    series: [
      {
        name: t("analytics.speed"),
        data: speedData.map((d) => d.speed ?? 0),
      },
    ],
  };

  const fatigueChartOptions = {
    chart: {
      type: "line",
      options3d: {
        enabled: true,
        alpha: 12,
        beta: 10,
        depth: 40,
        viewDistance: 25,
      },
    },
    title: { text: null },
    xAxis: { categories: fatigueData.map((d) => d.time ?? "") },
    plotOptions: {
      line: {
        depth: 20,
      },
    },
    series: [
      {
        name: t("analytics.fatigue"),
        data: fatigueData.map((d) => d.fatigue ?? 0),
      },
    ],
  };

  const safetyChartOptions = {
    chart: {
      type: "column",
      options3d: {
        enabled: true,
        alpha: 15,
        beta: 15,
        depth: 60,
        viewDistance: 25,
      },
    },
    plotOptions: {
      column: {
        depth: 40,
      },
    },
    title: { text: null },
    xAxis: { categories: tripComparisonData.map((d) => d.trip ?? "") },
    series: [
      {
        name: t("analytics.safetyScoreLabel"),
        data: tripComparisonData.map((d) => ({ y: d.score ?? 0, color: d.color ?? "#10b981" })),
      },
    ],
  };

  /* ================= UI ================= */
  return (
    <div className="flex h-screen bg-gradient-to-br from-blue-50 via-purple-50 to-pink-50">
      <MobileNav />
      <Sidebar />
      <main className="flex-1 overflow-auto">
        <div className="p-6">
          {/* TITLE */}
          <div className="mb-8">
            <h1 className="text-3xl font-bold text-black uppercase">{t("analytics.title")}</h1>
            <p className="text-sm mt-1">{t("analytics.subtitle")}</p>
          </div>

          {/* status */}
          {loading && (
            <div className="mb-4 text-sm text-gray-500">Đang tải dữ liệu...</div>
          )}
          {errorMsg && (
            <div className="mb-4 text-sm text-red-600">Lỗi: {errorMsg}</div>
          )}

          {/* CARDS */}
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-8">
            <Card className="p-4">
              <p>{t("analytics.distance")}</p>
              <p className="text-2xl font-bold text-blue-600">{cards.distance ?? "-"}</p>
              <TrendingUp />
            </Card>

            <Card className="p-4">
              <p>{t("analytics.drivingTime")}</p>
              <p className="text-2xl font-bold text-green-600">{cards.drivingTime ?? "-"}</p>
              <Clock />
            </Card>

            <Card className="p-4">
              <p>{t("analytics.averageSpeed")}</p>
              <p className="text-2xl font-bold text-purple-600">{cards.averageSpeed ?? "-"}</p>
              <Gauge />
            </Card>

            <Card className="p-4">
              <p>{t("analytics.safetyScore")}</p>
              <p className="text-2xl font-bold text-orange-600">{cards.safetyScore ?? "-"}</p>
              <AlertTriangle />
            </Card>
          </div>

          {/* CHARTS */}
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
            <Card className="p-6">
              <h3 className="mb-4">{t("analytics.speedOverTime")}</h3>
              <HighchartsReact highcharts={Highcharts} options={speedChartOptions} ref={speedChartRef} />
            </Card>

            <Card className="p-6">
              <h3 className="mb-4">{t("analytics.fatigueOverTime")}</h3>
              <HighchartsReact highcharts={Highcharts} options={fatigueChartOptions} ref={fatigueChartRef} />
            </Card>
          </div>

          {/* SAFETY */}
          <Card className="p-6">
            <h3 className="mb-4">{t("analytics.safetyScoreComparison")}</h3>
            <HighchartsReact highcharts={Highcharts} options={safetyChartOptions} />
          </Card>
        </div>
      </main>
    </div>
  );
}
