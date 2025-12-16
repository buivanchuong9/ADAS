"use client";

import * as React from "react";
import { motion } from "framer-motion";
import { cn } from "@/lib/utils";

const Card = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement> & {
    hover?: boolean;
    glass?: boolean;
  }
>(({ className, hover = true, glass = false, children, ...props }, ref) => {
  const { onDrag, ...restProps } = props as any;

  return (
    <motion.div
      ref={ref}
      data-slot="card"
      className={cn(
        "relative flex flex-col gap-6 rounded-2xl border py-6 overflow-hidden",
        glass
          ? "bg-white/5 backdrop-blur-xl border-white/10 shadow-xl"
          : "bg-card text-card-foreground border-border/50 shadow-lg",
        hover && "hover:shadow-2xl transition-all duration-300",
        className
      )}
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.3 }}
      whileHover={hover ? { y: -4, scale: 1.01 } : undefined}
      {...restProps}
    >
      {/* Glassmorphism shine effect */}
      {glass && (
        <>
          <div className="absolute inset-0 bg-linear-to-br from-white/10 via-transparent to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-500" />
          <div className="absolute top-0 left-0 w-full h-px bg-linear-to-r from-transparent via-white/20 to-transparent" />
        </>
      )}

      {/* Neumorphism subtle shadow */}
      {!glass && (
        <div className="absolute inset-0 rounded-2xl bg-linear-to-br from-white/5 via-transparent to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-500 pointer-events-none" />
      )}

      <div className="relative z-10">{children}</div>
    </motion.div>
  );
});
Card.displayName = "Card";

const CardHeader = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    data-slot="card-header"
    className={cn(
      "@container/card-header grid auto-rows-min grid-rows-[auto_auto] items-start gap-2 px-6 has-data-[slot=card-action]:grid-cols-[1fr_auto] [.border-b]:pb-6 text-white",
      className
    )}
    {...props}
  />
));
CardHeader.displayName = "CardHeader";

const CardTitle = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    data-slot="card-title"
    className={cn(
      "leading-none font-bold text-lg bg-linear-to-r from-foreground to-foreground/80 bg-clip-text text-transparent text-white",
      className
    )}
    {...props}
  />
));
CardTitle.displayName = "CardTitle";

const CardDescription = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    data-slot="card-description"
    className={cn("text-muted-foreground text-sm leading-relaxed", className)}
    {...props}
  />
));
CardDescription.displayName = "CardDescription";

const CardAction = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    data-slot="card-action"
    className={cn(
      "col-start-2 row-span-2 row-start-1 self-start justify-self-end",
      className
    )}
    {...props}
  />
));
CardAction.displayName = "CardAction";

const CardContent = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    data-slot="card-content"
    className={cn("px-6", className)}
    {...props}
  />
));
CardContent.displayName = "CardContent";

const CardFooter = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, ...props }, ref) => (
  <div
    ref={ref}
    data-slot="card-footer"
    className={cn("flex items-center px-6 [.border-t]:pt-6", className)}
    {...props}
  />
));
CardFooter.displayName = "CardFooter";

export {
  Card,
  CardHeader,
  CardFooter,
  CardTitle,
  CardAction,
  CardDescription,
  CardContent,
};
