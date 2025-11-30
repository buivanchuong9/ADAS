"use client";

import { AlertTriangle, RefreshCw, XCircle } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { cn } from "@/lib/utils";

interface ErrorMessageProps {
  error: Error | string;
  retry?: () => void;
  className?: string;
  variant?: "default" | "destructive";
}

export function ErrorMessage({
  error,
  retry,
  className,
  variant = "destructive",
}: ErrorMessageProps) {
  const errorMessage = typeof error === "string" ? error : error.message;

  return (
    <Alert variant={variant} className={cn("", className)}>
      <AlertTriangle className="h-4 w-4" />
      <AlertTitle>Error</AlertTitle>
      <AlertDescription className="flex flex-col gap-3">
        <p>{errorMessage}</p>
        {retry && (
          <Button onClick={retry} size="sm" variant="outline" className="w-fit">
            <RefreshCw className="mr-2 h-4 w-4" />
            Retry
          </Button>
        )}
      </AlertDescription>
    </Alert>
  );
}

interface ErrorCardProps {
  title?: string;
  message: string;
  retry?: () => void;
  className?: string;
}

export function ErrorCard({
  title = "Something went wrong",
  message,
  retry,
  className,
}: ErrorCardProps) {
  return (
    <div
      className={cn(
        "flex flex-col items-center justify-center gap-4 rounded-lg border border-destructive/50 bg-destructive/10 p-8 text-center",
        className
      )}
    >
      <XCircle className="h-12 w-12 text-destructive" />
      <div>
        <h3 className="font-semibold text-lg">{title}</h3>
        <p className="text-sm text-muted-foreground mt-1">{message}</p>
      </div>
      {retry && (
        <Button onClick={retry} variant="default" size="sm">
          <RefreshCw className="mr-2 h-4 w-4" />
          Try Again
        </Button>
      )}
    </div>
  );
}

interface InlineErrorProps {
  message: string;
  className?: string;
}

export function InlineError({ message, className }: InlineErrorProps) {
  return (
    <div
      className={cn(
        "flex items-center gap-2 text-sm text-destructive",
        className
      )}
    >
      <AlertTriangle className="h-4 w-4" />
      <span>{message}</span>
    </div>
  );
}
