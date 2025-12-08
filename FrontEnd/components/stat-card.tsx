"use client";

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { cn } from "@/lib/utils";
import { LucideIcon, TrendingDown, TrendingUp } from "lucide-react";

interface StatCardProps {
  title: string;
  value: string | number;
  description?: string;
  icon?: LucideIcon;
  trend?: {
    value: number;
    label?: string;
  };
  className?: string;
  variant?: "default" | "success" | "warning" | "danger";
}

const variantStyles = {
  default: "border-border",
  success: "border-green-500/50 bg-green-500/5",
  warning: "border-yellow-500/50 bg-yellow-500/5",
  danger: "border-red-500/50 bg-red-500/5",
};

const iconWrapperStyles = {
  default: "bg-primary/10 text-primary",
  success: "bg-green-500/10 text-green-600",
  warning: "bg-yellow-500/10 text-yellow-600",
  danger: "bg-red-500/10 text-red-600",
};

export function StatCard({
  title,
  value,
  description,
  icon: Icon,
  trend,
  className,
  variant = "default",
}: StatCardProps) {
  const isPositiveTrend = trend && trend.value > 0;
  const isNegativeTrend = trend && trend.value < 0;

  return (
    <Card className={cn(variantStyles[variant], className)}>
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-sm font-medium">{title}</CardTitle>
        {Icon && (
          <div className={cn("rounded-full p-2", iconWrapperStyles[variant])}>
            <Icon className="h-4 w-4" />
          </div>
        )}
      </CardHeader>
      <CardContent>
        <div className="text-2xl font-bold">{value}</div>
        <div className="flex items-center gap-2 mt-1">
          {trend && (
            <div
              className={cn(
                "flex items-center gap-1 text-xs font-medium",
                isPositiveTrend && "text-green-600",
                isNegativeTrend && "text-red-600"
              )}
            >
              {isPositiveTrend && <TrendingUp className="h-3 w-3" />}
              {isNegativeTrend && <TrendingDown className="h-3 w-3" />}
              <span>{Math.abs(trend.value)}%</span>
              {trend.label && (
                <span className="text-muted-foreground">{trend.label}</span>
              )}
            </div>
          )}
          {description && !trend && (
            <p className="text-xs text-muted-foreground">{description}</p>
          )}
        </div>
      </CardContent>
    </Card>
  );
}

interface CompactStatProps {
  label: string;
  value: string | number;
  icon?: LucideIcon;
  className?: string;
}

export function CompactStat({
  label,
  value,
  icon: Icon,
  className,
}: CompactStatProps) {
  return (
    <div
      className={cn("flex items-center justify-between rounded-lg", className)}
    >
      <div className="flex items-center gap-2">
        {Icon && <Icon className="h-4 w-4 text-muted-foreground" />}
        <span className="text-sm text-muted-foreground">{label}</span>
      </div>
      <span className="font-semibold">{value}</span>
    </div>
  );
}
