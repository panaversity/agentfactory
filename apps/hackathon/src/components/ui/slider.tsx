"use client";

import * as React from "react";
import { cn } from "@/lib/utils";

interface SliderProps {
  value?: number[];
  defaultValue?: number[];
  min?: number;
  max?: number;
  step?: number;
  onValueChange?: (value: number[]) => void;
  disabled?: boolean;
  className?: string;
}

export function Slider({
  value,
  defaultValue = [0],
  min = 0,
  max = 100,
  step = 1,
  onValueChange,
  disabled = false,
  className,
}: SliderProps) {
  const [internalValue, setInternalValue] = React.useState(
    value ?? defaultValue
  );

  const currentValue = value ?? internalValue;

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newValue = [Number(e.target.value)];
    if (!value) {
      setInternalValue(newValue);
    }
    onValueChange?.(newValue);
  };

  const percentage = ((currentValue[0] - min) / (max - min)) * 100;

  return (
    <div className={cn("relative w-full touch-none select-none", className)}>
      <div className="relative h-2 w-full rounded-full bg-primary/20">
        <div
          className="absolute h-full rounded-full bg-primary"
          style={{ width: `${percentage}%` }}
        />
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={currentValue[0]}
        onChange={handleChange}
        disabled={disabled}
        className="absolute inset-0 w-full h-full opacity-0 cursor-pointer disabled:cursor-not-allowed"
      />
      <div
        className="absolute top-1/2 -translate-y-1/2 h-5 w-5 rounded-full border-2 border-primary bg-background shadow transition-colors focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2 disabled:pointer-events-none disabled:opacity-50"
        style={{ left: `calc(${percentage}% - 10px)` }}
      />
    </div>
  );
}
