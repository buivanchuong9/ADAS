"use client";

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { cn } from "@/lib/utils";
import { HighchartsChart, MiniLineChart } from "@/components/charts/highcharts-chart";

interface LiveChartProps {
  title: string;
  description?: string;
  data: {
    labels: string[];
    datasets: {
      label: string;
      data: number[];
      borderColor?: string;
      backgroundColor?: string;
      fill?: boolean;
    }[];
  };
  type?: "line" | "bar" | "pie";
  height?: number;
  className?: string;
}

export function LiveChart({
  title,
  description,
  data,
  type = "line",
  height = 300,
  className,
}: LiveChartProps) {
  // Convert Chart.js format to Highcharts format
  const convertToHighchartsData = () => {
    if (type === "pie") {
      // For pie charts, use first dataset
      const dataset = data.datasets[0] || { data: [], label: "" };
      return data.labels.map((label, index) => ({
        name: label,
        y: dataset.data[index] || 0,
        color: dataset.backgroundColor || `#667eea`,
      }));
    }

    // For line/bar charts, convert datasets
    return data.datasets.map((dataset, index) => ({
      name: dataset.label,
      data: dataset.data,
      color: dataset.borderColor || dataset.backgroundColor || `#667eea`,
      type: type === "bar" ? "column" : type,
    }));
  };

  const highchartsData = convertToHighchartsData();

  return (
    <HighchartsChart
      title={title}
      description={description}
      type={type}
      data={highchartsData}
      height={height}
      className={className}
    />
  );
}

export { MiniLineChart };
