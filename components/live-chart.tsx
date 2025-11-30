"use client";

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { cn } from "@/lib/utils";
import { useEffect, useRef } from "react";
import { Line, Bar, Pie } from "react-chartjs-2";
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  ArcElement,
  Title,
  Tooltip,
  Legend,
  Filler,
  ChartOptions,
} from "chart.js";

// Register ChartJS components
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  ArcElement,
  Title,
  Tooltip,
  Legend,
  Filler
);

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
  options?: ChartOptions<any>;
}

const defaultLineOptions: ChartOptions<"line"> = {
  responsive: true,
  maintainAspectRatio: false,
  plugins: {
    legend: {
      position: "top" as const,
    },
  },
  scales: {
    y: {
      beginAtZero: true,
    },
  },
};

const defaultBarOptions: ChartOptions<"bar"> = {
  responsive: true,
  maintainAspectRatio: false,
  plugins: {
    legend: {
      position: "top" as const,
    },
  },
};

const defaultPieOptions: ChartOptions<"pie"> = {
  responsive: true,
  maintainAspectRatio: false,
  plugins: {
    legend: {
      position: "right" as const,
    },
  },
};

export function LiveChart({
  title,
  description,
  data,
  type = "line",
  height = 300,
  className,
  options,
}: LiveChartProps) {
  const chartRef = useRef<any>(null);

  useEffect(() => {
    // Cleanup on unmount
    return () => {
      if (chartRef.current) {
        chartRef.current.destroy();
      }
    };
  }, []);

  const chartOptions = options || (
    type === "line" ? defaultLineOptions :
    type === "bar" ? defaultBarOptions :
    defaultPieOptions
  );

  const ChartComponent = 
    type === "line" ? Line :
    type === "bar" ? Bar :
    Pie;

  return (
    <Card className={cn("", className)}>
      <CardHeader>
        <CardTitle>{title}</CardTitle>
        {description && <CardDescription>{description}</CardDescription>}
      </CardHeader>
      <CardContent>
        <div style={{ height: `${height}px` }}>
          <ChartComponent
            ref={chartRef}
            data={data}
            options={chartOptions}
          />
        </div>
      </CardContent>
    </Card>
  );
}

interface MiniChartProps {
  data: number[];
  labels?: string[];
  color?: string;
  height?: number;
  className?: string;
}

export function MiniLineChart({
  data,
  labels,
  color = "rgb(59, 130, 246)",
  height = 60,
  className,
}: MiniChartProps) {
  const chartData = {
    labels: labels || data.map((_, i) => `${i}`),
    datasets: [
      {
        data,
        borderColor: color,
        backgroundColor: `${color}20`,
        fill: true,
        tension: 0.4,
        pointRadius: 0,
      },
    ],
  };

  const options: ChartOptions<"line"> = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: { display: false },
      tooltip: { enabled: false },
    },
    scales: {
      x: { display: false },
      y: { display: false },
    },
    elements: {
      line: { borderWidth: 2 },
    },
  };

  return (
    <div style={{ height: `${height}px` }} className={className}>
      <Line data={chartData} options={options} />
    </div>
  );
}
