"use client"

import * as React from 'react'
import { Slot } from '@radix-ui/react-slot'
import { cva, type VariantProps } from 'class-variance-authority'
import { motion } from 'framer-motion'
import { cn } from '@/lib/utils'
import { designTokens } from '@/lib/design-system'

const buttonVariants = cva(
  "inline-flex items-center justify-center gap-2 whitespace-nowrap rounded-xl text-sm font-semibold transition-all duration-300 disabled:pointer-events-none disabled:opacity-50 [&_svg]:pointer-events-none [&_svg:not([class*='size-'])]:size-4 shrink-0 [&_svg]:shrink-0 outline-none focus-visible:ring-2 focus-visible:ring-primary/50 focus-visible:ring-offset-2 focus-visible:ring-offset-background relative overflow-hidden group",
  {
    variants: {
      variant: {
        default: 'bg-linear-to-r from-primary via-primary/90 to-primary text-primary-foreground shadow-lg shadow-primary/25 hover:shadow-xl hover:shadow-primary/40 hover:scale-[1.02] active:scale-[0.98]',
        destructive:
          'bg-linear-to-r from-destructive via-destructive/90 to-destructive text-white shadow-lg shadow-destructive/25 hover:shadow-xl hover:shadow-destructive/40 hover:scale-[1.02] active:scale-[0.98]',
        outline:
          'border-2 border-border/50 bg-background/50 backdrop-blur-md shadow-sm hover:bg-accent/50 hover:border-primary/50 hover:shadow-md hover:scale-[1.02] active:scale-[0.98]',
        secondary:
          'bg-linear-to-r from-secondary via-secondary/90 to-secondary text-secondary-foreground shadow-lg shadow-secondary/25 hover:shadow-xl hover:shadow-secondary/40 hover:scale-[1.02] active:scale-[0.98]',
        ghost:
          'hover:bg-accent/50 hover:text-accent-foreground backdrop-blur-sm hover:shadow-md hover:scale-[1.02] active:scale-[0.98]',
        link: 'text-primary underline-offset-4 hover:underline',
        glass: 'bg-white/5 backdrop-blur-xl border border-white/10 text-foreground shadow-lg hover:bg-white/10 hover:border-white/20 hover:shadow-xl hover:scale-[1.02] active:scale-[0.98]',
        gradient: 'bg-linear-to-r from-primary via-accent to-primary bg-[length:200%_100%] text-white shadow-lg shadow-primary/30 hover:shadow-xl hover:shadow-primary/50 hover:scale-[1.02] active:scale-[0.98] hover:bg-[position:100%_0] transition-all duration-500',
      },
      size: {
        default: 'h-11 px-6 py-2.5 has-[>svg]:px-5',
        sm: 'h-9 rounded-lg gap-1.5 px-4 has-[>svg]:px-3.5 text-xs',
        lg: 'h-12 rounded-xl px-8 has-[>svg]:px-6 text-base',
        icon: 'size-11 rounded-xl',
        'icon-sm': 'size-9 rounded-lg',
        'icon-lg': 'size-12 rounded-xl',
      },
    },
    defaultVariants: {
      variant: 'default',
      size: 'default',
    },
  },
)

// Ripple effect component
const Ripple = ({ className }: { className?: string }) => (
  <motion.span
    className={cn("absolute inset-0 rounded-xl", className)}
    initial={{ scale: 0, opacity: 0.5 }}
    animate={{ scale: 4, opacity: 0 }}
    transition={{ duration: 0.6 }}
  />
)

function Button({
  className,
  variant,
  size,
  asChild = false,
  children,
  ...props
}: React.ComponentProps<'button'> &
  VariantProps<typeof buttonVariants> & {
    asChild?: boolean
  }) {
  const Comp = asChild ? Slot : motion.button
  const [ripples, setRipples] = React.useState<Array<{ x: number; y: number; id: number }>>([])
  const rippleIdRef = React.useRef(0)
  
  const { onDrag, ...restProps } = props as any

  const handleClick = (e: React.MouseEvent<HTMLButtonElement>) => {
    if (restProps.onClick) {
      restProps.onClick(e)
    }

    // Create ripple effect
    const rect = e.currentTarget.getBoundingClientRect()
    const x = e.clientX - rect.left
    const y = e.clientY - rect.top
    
    const newRipple = { x, y, id: rippleIdRef.current++ }
    setRipples((prev) => [...prev, newRipple])
    
    setTimeout(() => {
      setRipples((prev) => prev.filter((r) => r.id !== newRipple.id))
    }, 600)
  }

  return (
    <Comp
      data-slot="button"
      className={cn(buttonVariants({ variant, size, className }))}
      onClick={handleClick}
      whileHover={{ scale: 1.02 }}
      whileTap={{ scale: 0.98 }}
      transition={{ type: "spring", stiffness: 400, damping: 17 }}
      {...restProps}
    >
      {/* Shine effect on hover */}
      <span className="absolute inset-0 -translate-x-full translate-y-full bg-linear-to-r from-transparent via-white/20 to-transparent opacity-0 group-hover:translate-x-full group-hover:translate-y-0 group-hover:opacity-100 transition-transform duration-1000 ease-in-out" />
      
      {/* Ripple effects */}
      {ripples.map((ripple) => (
        <motion.span
          key={ripple.id}
          className="absolute rounded-full bg-white/30"
          style={{
            left: ripple.x,
            top: ripple.y,
            width: 20,
            height: 20,
            x: -10,
            y: -10,
          }}
          initial={{ scale: 0, opacity: 1 }}
          animate={{ scale: 4, opacity: 0 }}
          transition={{ duration: 0.6 }}
        />
      ))}
      
      <span className="relative z-10">{children}</span>
    </Comp>
  )
}

export { Button, buttonVariants }
