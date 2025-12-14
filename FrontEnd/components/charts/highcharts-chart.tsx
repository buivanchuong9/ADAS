"use client";

import { useEffect, useRef } from "react";
import Highcharts from "highcharts";
import HighchartsReact from "highcharts-react-official";
import { motion } from "framer-motion";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { cn } from "@/lib/utils";

// Highcharts theme for premium look
const premiumTheme: Highcharts.Options = {
  chart: {
    backgroundColor: "transparent",
    style: {
      fontFamily:
        '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
    },
  },
  colors: [
    "#667eea", // Primary blue
    "#764ba2", // Purple
    "#f093fb", // Pink
    "#4facfe", // Light blue
    "#43e97b", // Green
    "#fa709a", // Rose
    "#fee140", // Yellow
    "#0ba360", // Dark green
  ],
  title: {
    style: {
      color: "#ffffff",
      fontWeight: "600",
      fontSize: "18px",
    },
  },
  subtitle: {
    style: {
      color: "#9ca3af",
    },
  },
  xAxis: {
    gridLineColor: "rgba(255, 255, 255, 0.1)",
    lineColor: "rgba(255, 255, 255, 0.2)",
    minorGridLineColor: "rgba(255, 255, 255, 0.05)",
    tickColor: "rgba(255, 255, 255, 0.2)",
    labels: {
      style: {
        color: "#9ca3af",
      },
    },
    title: {
      style: {
        color: "#ffffff",
      },
    },
  },
  yAxis: {
    gridLineColor: "rgba(255, 255, 255, 0.1)",
    lineColor: "rgba(255, 255, 255, 0.2)",
    minorGridLineColor: "rgba(255, 255, 255, 0.05)",
    tickColor: "rgba(255, 255, 255, 0.2)",
    labels: {
      style: {
        color: "#9ca3af",
      },
    },
    title: {
      style: {
        color: "#ffffff",
      },
    },
  },
  legend: {
    backgroundColor: "transparent",
    itemStyle: {
      color: "#9ca3af",
    },
    itemHoverStyle: {
      color: "#ffffff",
    },
    itemHiddenStyle: {
      color: "#4b5563",
    },
  },
  tooltip: {
    backgroundColor: "rgba(0, 0, 0, 0.8)",
    borderColor: "rgba(255, 255, 255, 0.1)",
    borderRadius: 12,
    style: {
      color: "#ffffff",
    },
    shadow: {
      color: "rgba(0, 0, 0, 0.5)",
    },
  },
  plotOptions: {
    series: {
      dataLabels: {
        color: "#ffffff",
      },
      marker: {
        lineColor: "#ffffff",
      },
    },
    boxplot: {
      fillColor: "#505053",
    },
    candlestick: {
      lineColor: "white",
    },
    errorbar: {
      color: "white",
    },
  },
  credits: {
    enabled: false,
  },
};

// Apply theme
if (typeof Highcharts !== "undefined") {
  Highcharts.setOptions(premiumTheme);
}

interface HighchartsChartProps {
  title?: string;
  description?: string;
  type: "line" | "pie" | "bar" | "area" | "column";
  data: any;
  height?: number;
  className?: string;
  headerClassName?: string;
  options?: Highcharts.Options;
}

export function HighchartsChart({
  title,
  description,
  type,
  data,
  height = 300,
  className,
  headerClassName = "text-white",
  options = {},
}: HighchartsChartProps) {
  const chartRef = useRef<HighchartsReact.RefObject>(null);

  // Default chart configuration based on type
  const getDefaultOptions = (): Highcharts.Options => {
    const baseOptions: Highcharts.Options = {
      chart: {
        type:
          type === "pie"
            ? "pie"
            : type === "bar"
            ? "bar"
            : type === "area"
            ? "area"
            : "line",
        height: height,
        backgroundColor: "transparent",
        borderRadius: 12,
        style: {
          fontFamily:
            '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
        },
      },
      credits: {
        enabled: false,
      },
      ...options,
    };

    if (type === "pie") {
      return {
        ...baseOptions,
        plotOptions: {
          pie: {
            allowPointSelect: true,
            cursor: "pointer",
            dataLabels: {
              enabled: true,
              format: "<b>{point.name}</b>: {point.percentage:.1f} %",
              style: {
                color: "#ffffff",
                textOutline: "none",
              },
            },
            title: {
              style: {
                color: "#ffffff",
              },
            },
            legend: {
              itemStyle: {
                color: "#ffffff",
              },
            },
            showInLegend: true,
            borderWidth: 2,
            borderColor: "rgba(255, 255, 255, 0.1)",
          },
        },
        series: [
          {
            type: "pie",
            name: "Share",
            data: data,
          },
        ],
      } as Highcharts.Options;
    }

    if (type === "line" || type === "area") {
      return {
        ...baseOptions,
        plotOptions: {
          line: {
            marker: {
              enabled: true,
              radius: 4,
              lineWidth: 2,
              lineColor: "#ffffff",
            },
            lineWidth: 3,
            states: {
              hover: {
                lineWidth: 4,
              },
            },
          },
          area: {
            fillOpacity: 0.3,
            marker: {
              enabled: true,
              radius: 4,
            },
            lineWidth: 3,
          },
        },
        series: Array.isArray(data) ? data : [data],
      } as Highcharts.Options;
    }

    return {
      ...baseOptions,
      plotOptions: {
        bar: {
          borderRadius: 4,
          borderWidth: 0,
        },
        column: {
          borderRadius: 4,
          borderWidth: 0,
        },
      },
      series: Array.isArray(data) ? data : [data],
    } as Highcharts.Options;
  };

  const chartOptions = getDefaultOptions();

  return (
    <motion.div
      initial={{ opacity: 0, scale: 0.95 }}
      animate={{ opacity: 1, scale: 1 }}
      transition={{ duration: 0.3 }}
    >
      <Card glass>
        {(title || description) && (
          <CardHeader>
            {title && (
              <CardTitle className={headerClassName}>{title}</CardTitle>
            )}
            {description && (
              <CardDescription className={headerClassName}>
                {description}
              </CardDescription>
            )}
          </CardHeader>
        )}
        <CardContent className={className}>
          <div className="relative">
            <HighchartsReact
              highcharts={Highcharts}
              options={chartOptions}
              ref={chartRef}
            />
          </div>
        </CardContent>
      </Card>
    </motion.div>
  );
}

// Mini line chart component
interface MiniLineChartProps {
  data: number[];
  labels?: string[];
  color?: string;
  height?: number;
  className?: string;
}

export function MiniLineChart({
  data,
  labels,
  color = "#667eea",
  height = 60,
  className,
}: MiniLineChartProps) {
  const chartOptions: Highcharts.Options = {
    chart: {
      type: "line",
      height: height,
      backgroundColor: "transparent",
      spacing: [0, 0, 0, 0],
    },
    title: {
      text: undefined,
    },
    credits: {
      enabled: false,
    },
    legend: {
      enabled: false,
    },
    xAxis: {
      visible: false,
    },
    yAxis: {
      visible: false,
    },
    tooltip: {
      enabled: false,
    },
    plotOptions: {
      line: {
        marker: {
          enabled: false,
        },
        lineWidth: 2,
        color: color,
        enableMouseTracking: false,
      },
      area: {
        fillColor: {
          linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
          stops: [
            [0, color + "40"],
            [1, color + "00"],
          ],
        },
        fillOpacity: 0.3,
      },
    },
    series: [
      {
        type: "area",
        name: "Value",
        data: data,
      },
    ],
  };

  return (
    <div className={className}>
      <HighchartsReact highcharts={Highcharts} options={chartOptions} />
    </div>
  );
}
