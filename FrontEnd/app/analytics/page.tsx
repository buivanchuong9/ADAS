"use client";

import Highcharts from "highcharts";
import HighchartsReact from "highcharts-react-official";

// chỉ import, KHÔNG gọi
import "highcharts/highcharts-3d";
import "highcharts/modules/cylinder";
Highcharts.setOptions({
  plotOptions: {
  series: {
    borderWidth: 0,

    states: {
      hover: {
        brightness: 0.3
      }
    }
  }
}

});

import { useRef, useEffect } from "react";

import { useLanguage } from "@/contexts/language-context";
import { Sidebar } from "@/components/sidebar";
import { MobileNav } from "@/components/mobile-nav";
import { Card } from "@/components/ui/card";
import { TrendingUp, Clock, Gauge, AlertTriangle } from "lucide-react";


export default function Analytics() {
  const { t } = useLanguage();
  const speedData = [
    { time: "0:00", speed: 0 },
    { time: "0:15", speed: 45 },
    { time: "0:30", speed: 60 },
    { time: "0:45", speed: 55 },
    { time: "1:00", speed: 70 },
    { time: "1:15", speed: 65 },
    { time: "1:30", speed: 50 },
    { time: "1:45", speed: 40 },
    { time: "2:00", speed: 0 },
  ];
const speedChartRef = useRef<any>(null);

useEffect(() => {

  let hue = 0;

  const interval = setInterval(() => {

    hue = (hue + 2) % 360; // giảm tốc độ đổi màu

    const color1 = `hsl(${hue}, 100%, 60%)`;
    const color2 = `hsl(${(hue + 60) % 360}, 100%, 60%)`;

    const speedChart = speedChartRef.current?.chart;
    const fatigueChart = fatigueChartRef.current?.chart;
    const safetyChart = safetyChartRef.current?.chart;

    speedChart?.series[0].update({
      color: color1,
      shadow: { color: color1, width: 25 }
    }, false);

    fatigueChart?.series[0].update({
      color: color2
    }, false);

    safetyChart?.series[0].update({
      color: color1,
      shadow: { color: color1, width: 25 }
    }, false);

    // ⭐ redraw 1 lần
    Highcharts.charts.forEach(chart => chart?.redraw());

  }, 180); // ⭐ tăng interval -> mượt hơn

  return () => clearInterval(interval);

}, []);



  const fatigueData = [
    { time: "0:00", fatigue: 10 },
    { time: "0:30", fatigue: 15 },
    { time: "1:00", fatigue: 25 },
    { time: "1:30", fatigue: 35 },
    { time: "2:00", fatigue: 40 },
  ];

  const tripComparisonData = [
    { trip: t("analytics.today"), score: 85 },
    { trip: t("analytics.yesterday"), score: 78 },
    { trip: t("analytics.threeDaysAgo"), score: 82 },
    { trip: t("analytics.oneWeekAgo"), score: 75 },
  ];

  // ================= CHART OPTIONS =================

  const speedChartOptions = {
  chart: {
    type: "column",
    options3d: {
      enabled: true,
      alpha: 15,
      beta: 15,
      depth: 50,
      viewDistance: 25
    }
  },

  title: { text: null },

  xAxis: {
    categories: speedData.map(d => d.time)
  },

  plotOptions: {
    column: {
  depth: 40,
  borderRadius: 6,

  shadow: {
    color: "#00f5ff",
    width: 25
  }
}
  },

  series: [
  {
    name: t("analytics.speed"),

    data: speedData.map(d => ({
      y: d.speed,

      color: {
        linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
        stops: [
          [0, "#00f5ff"],
          [1, "#0066ff"]
        ]
      }
    }))
  }
]
};


  const fatigueChartOptions = {
  chart: {
    type: "pie",
    options3d: {
      enabled: true,
      alpha: 45
    }
  },

  title: { text: null },

  plotOptions: {
    pie: {
  innerSize: 80,
  depth: 45,

  shadow: {
    color: "#ff00ff",
    width: 25
  }
}
  },

  series: [
  {
    name: t("analytics.fatigue"),
    data: fatigueData.map((d, index) => ({
      name: d.time,
      y: d.fatigue,
      color: [
        "#ff00ff",
        "#00f5ff",
        "#00ff9f",
        "#ffd400",
        "#ff4d6d"
      ][index]
    }))
  }
]
};


 const safetyChartOptions = {
  chart: {
    type: "cylinder",
    options3d: {
      enabled: true,
      alpha: 15,
      beta: 15,
      depth: 50,
      viewDistance: 25,
      
    }
  },

  title: { text: null },

  xAxis: {
    categories: tripComparisonData.map(d => d.trip)
  },

  plotOptions: {
    cylinder: {
  depth: 40,
  shadow: {
    color: "#00ff9f",
    width: 25
  }
}
  },

  series: [
  {
    name: t("analytics.safetyScoreLabel"),

    data: tripComparisonData.map(d => ({
      y: d.score,

      color: {
        linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
        stops: [
          [0, "#00ff9f"],
          [1, "#008f5a"]
        ]
      }
    }))
  }
]
};


  

  return (
    <div className="flex h-screen bg-gradient-to-br from-blue-50 via-purple-50 to-pink-50">
      <MobileNav />
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-4 sm:p-6 lg:p-8">
          <div className="mb-6 sm:mb-8">
            <h1 className="text-3xl font-bold text-neon-cyan tracking-wider uppercase">
              {t("analytics.title")}
            </h1>
            <p className="text-sm text-fg-secondary mt-1">
              {t("analytics.subtitle")}
            </p>
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-3 sm:gap-4 mb-6 sm:mb-8">
            <Card className="bg-white border-2 border-blue-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-4">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm text-gray-600 mb-1 font-medium">
                      {t("analytics.distance")}
                    </p>
                    <p className="text-2xl font-bold text-blue-600">42.5 km</p>
                  </div>
                  <TrendingUp className="w-8 h-8 text-blue-500" />
                </div>
              </div>
            </Card>

            <Card className="bg-white border-2 border-green-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-4">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm text-gray-600 mb-1 font-medium">
                      {t("analytics.drivingTime")}
                    </p>
                    <p className="text-2xl font-bold text-green-600">2h 15m</p>
                  </div>
                  <Clock className="w-8 h-8 text-green-500" />
                </div>
              </div>
            </Card>

            <Card className="bg-white border-2 border-purple-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-4">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm text-gray-600 mb-1 font-medium">
                      {t("analytics.averageSpeed")}
                    </p>
                    <p className="text-2xl font-bold text-purple-600">
                      54 km/h
                    </p>
                  </div>
                  <Gauge className="w-8 h-8 text-purple-500" />
                </div>
              </div>
            </Card>

            <Card className="bg-white border-2 border-orange-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-4">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm text-gray-600 mb-1 font-medium">
                      {t("analytics.safetyScore")}
                    </p>
                    <p className="text-2xl font-bold text-orange-600">85/100</p>
                  </div>
                  <AlertTriangle className="w-8 h-8 text-orange-500" />
                </div>
              </div>
            </Card>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
            <Card className="bg-white border-2 border-blue-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4">
                  {t("analytics.speedOverTime")}
                </h3>
                <HighchartsReact
  highcharts={Highcharts}
  options={speedChartOptions}
  ref={speedChartRef}
/>

              </div>
            </Card>

            <Card className="bg-white border-2 border-red-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4">
                  {t("analytics.fatigueOverTime")}
                </h3>
                <HighchartsReact
  highcharts={Highcharts}
  options={fatigueChartOptions}
 ref={speedChartRef}
/>

              </div>
            </Card>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="bg-white border-2 border-indigo-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4">
                  {t("analytics.safetyScoreComparison")}
                </h3>
                <HighchartsReact
  highcharts={Highcharts}
  options={safetyChartOptions}
 ref={speedChartRef}


/>

              </div>
            </Card>

            <Card className="bg-white border-2 border-green-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4">
                  {t("analytics.recommendations")}
                </h3>
                <div className="space-y-3">
                  <div className="p-3 bg-blue-50 rounded-lg border-l-4 border-blue-500">
                    <p className="text-sm font-semibold text-blue-900 mb-1">
                      {t("analytics.increaseSafetyDistance")}
                    </p>
                    <p className="text-xs text-blue-700">
                      {t("analytics.increaseSafetyDistanceDesc")}
                    </p>
                  </div>
                  <div className="p-3 bg-green-50 rounded-lg border-l-4 border-green-500">
                    <p className="text-sm font-semibold text-green-900 mb-1">
                      {t("analytics.restRegularly")}
                    </p>
                    <p className="text-xs text-green-700">
                      {t("analytics.restRegularlyDesc")}
                    </p>
                  </div>
                  <div className="p-3 bg-orange-50 rounded-lg border-l-4 border-orange-500">
                    <p className="text-sm font-semibold text-orange-900 mb-1">
                      {t("analytics.followSpeedLimit")}
                    </p>
                    <p className="text-xs text-orange-700">
                      {t("analytics.followSpeedLimitDesc")}
                    </p>
                  </div>
                </div>
              </div>
            </Card>
          </div>
        </div>
      </main>
    </div>
  );
}
