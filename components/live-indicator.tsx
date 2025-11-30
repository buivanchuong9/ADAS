"use client";

import { cn } from "@/lib/utils";
import { Badge } from "@/components/ui/badge";

interface LiveIndicatorProps {
  isLive?: boolean;
  label?: string;
  className?: string;
}

export function LiveIndicator({
  isLive = true,
  label = "LIVE",
  className,
}: LiveIndicatorProps) {
  return (
    <Badge
      variant={isLive ? "default" : "secondary"}
      className={cn(
        "flex items-center gap-1.5 px-2 py-0.5",
        isLive && "bg-red-500 hover:bg-red-600",
        className
      )}
    >
      {isLive && (
        <span className="relative flex h-2 w-2">
          <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-white opacity-75" />
          <span className="relative inline-flex rounded-full h-2 w-2 bg-white" />
        </span>
      )}
      <span className="text-xs font-medium">{label}</span>
    </Badge>
  );
}

interface StatusIndicatorProps {
  status: "online" | "offline" | "warning" | "error";
  label?: string;
  showDot?: boolean;
  className?: string;
}

const statusStyles = {
  online: "bg-green-500 text-white",
  offline: "bg-gray-500 text-white",
  warning: "bg-yellow-500 text-white",
  error: "bg-red-500 text-white",
};

const dotColors = {
  online: "bg-green-500",
  offline: "bg-gray-500",
  warning: "bg-yellow-500",
  error: "bg-red-500",
};

export function StatusIndicator({
  status,
  label,
  showDot = true,
  className,
}: StatusIndicatorProps) {
  return (
    <Badge
      className={cn(
        "flex items-center gap-1.5 px-2 py-0.5",
        statusStyles[status],
        className
      )}
    >
      {showDot && (
        <span className={cn("h-2 w-2 rounded-full", dotColors[status])} />
      )}
      {label && <span className="text-xs font-medium">{label}</span>}
    </Badge>
  );
}
