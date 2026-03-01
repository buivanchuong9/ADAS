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
import { getApiUrl } from "@/lib/api-config";
import { API_ENDPOINTS } from "@/lib/api-endpoints";

/** GET /api/analytics/summary response */
type SummaryApi = {
  period?: string;
  total_trips?: number;
  total_distance?: number;
  total_distance_km?: number;
  driving_time?: number;
  total_time_sec?: number;
  avg_speed?: number;
  avg_safety_score?: number;
  total_alerts?: number;
  total_critical_alerts?: number;
};

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
  const [period, setPeriod] = useState<"today" | "week" | "month" | "all">("week");
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

  const formatDrivingTime = (seconds: number): string => {
    if (seconds < 60) return `${seconds} giây`;
    const mins = Math.floor(seconds / 60);
    if (mins < 60) return `${mins} phút`;
    const h = Math.floor(mins / 60);
    const m = mins % 60;
    return m ? `${h} giờ ${m} phút` : `${h} giờ`;
  };

  const mapSummaryToCards = (json: SummaryApi | null): CardsType => {
    if (!json) return {};
    const distance = json.total_distance ?? json.total_distance_km;
    const drivingTimeSec = json.driving_time ?? json.total_time_sec;
    const drivingTimeStr =
      drivingTimeSec != null
        ? formatDrivingTime(drivingTimeSec)
        : json.total_trips != null
          ? `${json.total_trips} chuyến`
          : "—";
    const avgSpeed = json.avg_speed;
    const safetyScore = json.avg_safety_score;
    return {
      distance: distance != null ? `${Number(distance).toFixed(1)} km` : "—",
      drivingTime: drivingTimeStr,
      averageSpeed: avgSpeed != null ? `${Number(avgSpeed).toFixed(1)} km/h` : "—",
      safetyScore: safetyScore != null ? String(Math.round(Number(safetyScore))) : "—",
    };
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
        const summaryUrl = getApiUrl(`${API_ENDPOINTS.ANALYTICS_SUMMARY}?period=${period}`);
        const speedUrl = getApiUrl(API_ENDPOINTS.ANALYTICS_SPEED_OVER_TIME);
        const fatigueUrl = getApiUrl(API_ENDPOINTS.ANALYTICS_FATIGUE_OVER_TIME);
        const safetyUrl = getApiUrl(`${API_ENDPOINTS.ANALYTICS_SAFETY_SCORE_COMPARISON}?days=7`);

        const promises = [
          fetch(summaryUrl, { headers }),
          fetch(speedUrl, { headers }),
          fetch(fatigueUrl, { headers }),
          fetch(safetyUrl, { headers }),
        ];

        const results = await Promise.allSettled(promises);
        const responses = results.map((r) =>
          r.status === "fulfilled" ? (r.value as Response) : null
        );

        // 1) KPI cards — GET /api/analytics/summary
        const summaryRes = responses[0];
        if (summaryRes?.ok) {
          const json = (await safeParseJson(summaryRes)) as SummaryApi | null;
          if (mounted) setCards(mapSummaryToCards(json));
        } else {
          if (mounted) setCards(mapSummaryToCards(null));
        }

        // 2) Chart: Tốc độ theo thời gian — GET /api/analytics/speed-over-time
        const speedRes = responses[1];
        if (speedRes?.ok) {
          const json = await safeParseJson(speedRes);
          let mappedSpeed: ChartPoint[] = [];
          if (json && Array.isArray(json.labels) && Array.isArray(json.data)) {
            mappedSpeed = json.labels.map((label: string, idx: number) => ({
              time: label,
              speed: Number(json.data?.[idx] ?? 0),
            }));
          }
          if (mounted) setSpeedData(mappedSpeed);
        } else {
          if (mounted) setSpeedData([]);
        }

        // 3) Chart: Mức mệt mỏi theo thời gian — GET /api/analytics/fatigue-over-time
        const fatigueRes = responses[2];
        if (fatigueRes?.ok) {
          const json = await safeParseJson(fatigueRes);
          let mappedFatigue: ChartPoint[] = [];
          if (json && Array.isArray(json.labels) && Array.isArray(json.data)) {
            mappedFatigue = json.labels.map((label: string, idx: number) => ({
              time: label,
              fatigue: Number(json.data?.[idx] ?? 0),
            }));
          }
          if (mounted) setFatigueData(mappedFatigue);
        } else {
          if (mounted) setFatigueData([]);
        }

        // 4) Chart: So sánh điểm an toàn — GET /api/analytics/safety-score-comparison
        const safetyRes = responses[3];
        if (safetyRes?.ok) {
          const json = await safeParseJson(safetyRes);
          let mappedSafety: ChartPoint[] = [];
          if (json && Array.isArray(json.labels) && Array.isArray(json.data)) {
            mappedSafety = json.labels.map((label: string, idx: number) => ({
              trip: label,
              score: Number(json.data?.[idx] ?? 0),
              color: Array.isArray(json.colors) && json.colors[idx] ? json.colors[idx] : "#10b981",
            }));
          }
          if (mounted) setTripComparisonData(mappedSafety);
        } else {
          if (mounted) setTripComparisonData([]);
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
  }, [period]);

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
  const emptyLabel = t("analytics.noDataChart");
  const speedCategories = speedData.length
    ? speedData.map((d) => d.time ?? "")
    : [emptyLabel, "—"];
  const speedValues = speedData.length
    ? speedData.map((d) => d.speed ?? 0)
    : [0, 0];

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
    xAxis: { categories: speedCategories },
    plotOptions: {
      line: {
        // depth isn't a documented option for line but harmless to include
        depth: 25,
      },
    },
    series: [
      {
        name: t("analytics.speed"),
        data: speedValues,
      },
    ],
  };

  const fatigueCategories = fatigueData.length
    ? fatigueData.map((d) => d.time ?? "")
    : [emptyLabel, "—"];
  const fatigueValues = fatigueData.length
    ? fatigueData.map((d) => d.fatigue ?? 0)
    : [0, 0];

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
    xAxis: { categories: fatigueCategories },
    plotOptions: {
      line: {
        depth: 20,
      },
    },
    series: [
      {
        name: t("analytics.fatigue"),
        data: fatigueValues,
      },
    ],
  };

  const safetyCategories = tripComparisonData.length
    ? tripComparisonData.map((d) => d.trip ?? "")
    : [];
  const safetyValues = tripComparisonData.map((d) => ({ y: d.score ?? 0, color: d.color ?? "#10b981" }));

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
    xAxis: { categories: safetyCategories },
    yAxis: {
      min: 0,
      max: 100,
      title: { text: null },
    },
    series: [
      {
        name: t("analytics.safetyScoreLabel"),
        data: safetyValues,
      },
    ],
  };

  /* ================= UI ================= */
  return (
    <div className="flex h-screen bg-linear-to-br from-blue-50 via-purple-50 to-pink-50">
      <MobileNav />
      <Sidebar />
      <main className="flex-1 overflow-auto">
        <div className="p-6">
          {/* TITLE + PERIOD */}
          <div className="mb-8 flex flex-wrap items-center justify-between gap-4">
            <div>
              <h1 className="text-3xl font-bold text-neon-cyan uppercase">{t("analytics.title")}</h1>
              <p className="text-sm mt-1">{t("analytics.subtitle")}</p>
            </div>
            <select
              value={period}
              onChange={(e) => setPeriod(e.target.value as "today" | "week" | "month" | "all")}
              className="rounded-lg border border-neon-cyan/50 bg-white/80 px-3 py-2 text-sm text-fg-primary focus:border-neon-cyan focus:outline-none"
            >
              <option value="today">{t("analytics.today")}</option>
              <option value="week">{t("analytics.periodWeek")}</option>
              <option value="month">{t("analytics.periodMonth")}</option>
              <option value="all">{t("analytics.periodAll")}</option>
            </select>
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
