"use client";

import { cn } from "@/lib/utils";

interface LoadingSpinnerProps {
  size?: "sm" | "md" | "lg" | "xl";
  className?: string;
  text?: string;
}

const sizeClasses = {
  sm: "h-4 w-4 border-2",
  md: "h-8 w-8 border-2",
  lg: "h-12 w-12 border-3",
  xl: "h-16 w-16 border-4",
};

export function LoadingSpinner({
  size = "md",
  className,
  text,
}: LoadingSpinnerProps) {
  return (
    <div className="flex flex-col items-center justify-center gap-3">
      <div
        className={cn(
          "animate-spin rounded-full border-primary border-t-transparent",
          sizeClasses[size],
          className
        )}
      />
      {text && (
        <p className="text-sm text-muted-foreground animate-pulse">{text}</p>
      )}
    </div>
  );
}

interface LoadingOverlayProps {
  text?: string;
}

export function LoadingOverlay({ text = "Loading..." }: LoadingOverlayProps) {
  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-background/80 backdrop-blur-sm">
      <div className="bg-card rounded-lg p-8 shadow-lg">
        <LoadingSpinner size="lg" text={text} />
      </div>
    </div>
  );
}

interface LoadingCardProps {
  className?: string;
  text?: string;
}

export function LoadingCard({ className, text }: LoadingCardProps) {
  return (
    <div
      className={cn(
        "flex items-center justify-center rounded-lg border bg-card p-12",
        className
      )}
    >
      <LoadingSpinner text={text} />
    </div>
  );
}
