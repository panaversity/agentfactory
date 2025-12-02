"use client";

import { useEffect, useState } from "react";

interface ToastProps {
  message: string;
  type?: "success" | "error" | "info";
  duration?: number;
  onClose?: () => void;
}

export function Toast({ message, type = "success", duration = 5000, onClose }: ToastProps) {
  const [isVisible, setIsVisible] = useState(true);
  const [isExiting, setIsExiting] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => {
      setIsExiting(true);
      setTimeout(() => {
        setIsVisible(false);
        onClose?.();
      }, 400);
    }, duration);

    return () => clearTimeout(timer);
  }, [duration, onClose]);

  if (!isVisible) return null;

  const styles = {
    success: {
      container: "bg-gradient-to-br from-emerald-500 via-green-500 to-teal-600",
      glow: "shadow-emerald-500/40",
      particles: "from-emerald-300/60 to-teal-400/60",
      icon: (
        <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={3} d="M5 13l4 4L19 7" />
        </svg>
      ),
    },
    error: {
      container: "bg-gradient-to-br from-rose-500 via-red-500 to-pink-600",
      glow: "shadow-rose-500/40",
      particles: "from-rose-300/60 to-pink-400/60",
      icon: (
        <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={3} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
        </svg>
      ),
    },
    info: {
      container: "bg-gradient-to-br from-blue-500 via-indigo-500 to-violet-600",
      glow: "shadow-blue-500/40",
      particles: "from-blue-300/60 to-violet-400/60",
      icon: (
        <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={3} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
        </svg>
      ),
    },
  };

  const style = styles[type];

  return (
    <div
      className={`fixed top-6 right-6 z-50 transition-all duration-400 ${
        isExiting
          ? "opacity-0 translate-y-[-20px] scale-95"
          : "opacity-100 translate-y-0 scale-100 animate-in slide-in-from-top-4"
      }`}
    >
      {/* Glow effect */}
      <div className={`absolute inset-0 rounded-2xl blur-xl opacity-60 ${style.glow}`} />

      {/* Main toast container */}
      <div className="relative">
        {/* Shimmer overlay for success */}
        {type === "success" && (
          <div className="absolute inset-0 rounded-2xl overflow-hidden pointer-events-none">
            <div className="shimmer-success" />
          </div>
        )}

        {/* Shake animation for error */}
        <div className={type === "error" ? "animate-shake" : ""}>
          <div
            className={`${style.container} text-white pl-6 pr-4 py-5 rounded-2xl shadow-2xl backdrop-blur-sm border border-white/20 flex items-start gap-4 min-w-[360px] max-w-md relative overflow-hidden`}
          >
            {/* Floating particles background */}
            <div className="absolute inset-0 pointer-events-none overflow-hidden">
              <div className={`particle particle-1 bg-gradient-to-br ${style.particles}`} />
              <div className={`particle particle-2 bg-gradient-to-br ${style.particles}`} />
              <div className={`particle particle-3 bg-gradient-to-br ${style.particles}`} />
            </div>

            {/* Icon with pulse animation */}
            <div className={`flex-shrink-0 w-10 h-10 bg-white/25 backdrop-blur-sm rounded-xl flex items-center justify-center relative ${type === "success" ? "animate-pulse-subtle" : ""}`}>
              <div className="absolute inset-0 rounded-xl bg-white/10 animate-ping" style={{ animationDuration: "2s" }} />
              {style.icon}
            </div>

            {/* Content */}
            <div className="flex-1 relative z-10 pt-0.5">
              <p className="text-[15px] font-semibold leading-relaxed tracking-wide drop-shadow-sm">
                {message}
              </p>
            </div>

            {/* Close button */}
            <button
              onClick={() => {
                setIsExiting(true);
                setTimeout(() => {
                  setIsVisible(false);
                  onClose?.();
                }, 400);
              }}
              className="flex-shrink-0 text-white/70 hover:text-white hover:bg-white/10 transition-all duration-200 rounded-lg p-1.5 hover:scale-110"
              aria-label="Close"
            >
              <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>
        </div>
      </div>

      <style jsx>{`
        .shimmer-success {
          position: absolute;
          top: -50%;
          left: -50%;
          width: 200%;
          height: 200%;
          background: linear-gradient(
            90deg,
            transparent,
            rgba(255, 255, 255, 0.3),
            transparent
          );
          animation: shimmer 3s infinite;
        }

        @keyframes shimmer {
          0% {
            transform: translateX(-100%) translateY(-100%) rotate(30deg);
          }
          100% {
            transform: translateX(100%) translateY(100%) rotate(30deg);
          }
        }

        .particle {
          position: absolute;
          width: 6px;
          height: 6px;
          border-radius: 50%;
          opacity: 0;
          animation: float 3s infinite ease-in-out;
        }

        .particle-1 {
          top: 20%;
          left: 15%;
          animation-delay: 0s;
        }

        .particle-2 {
          top: 60%;
          right: 20%;
          animation-delay: 1s;
          width: 8px;
          height: 8px;
        }

        .particle-3 {
          bottom: 25%;
          left: 70%;
          animation-delay: 2s;
        }

        @keyframes float {
          0%, 100% {
            transform: translateY(0) scale(0);
            opacity: 0;
          }
          50% {
            transform: translateY(-20px) scale(1);
            opacity: 0.6;
          }
        }

        .animate-shake {
          animation: shake 0.5s;
        }

        @keyframes shake {
          0%, 100% { transform: translateX(0); }
          10%, 30%, 50%, 70%, 90% { transform: translateX(-4px); }
          20%, 40%, 60%, 80% { transform: translateX(4px); }
        }

        .animate-pulse-subtle {
          animation: pulse-subtle 2s ease-in-out infinite;
        }

        @keyframes pulse-subtle {
          0%, 100% {
            transform: scale(1);
            opacity: 1;
          }
          50% {
            transform: scale(1.05);
            opacity: 0.9;
          }
        }
      `}</style>
    </div>
  );
}

interface ToastContainerProps {
  toasts: Array<{ id: string; message: string; type?: "success" | "error" | "info" }>;
  onRemove: (id: string) => void;
}

export function ToastContainer({ toasts, onRemove }: ToastContainerProps) {
  return (
    <div className="fixed top-6 right-6 z-50 flex flex-col gap-4">
      {toasts.map((toast, index) => (
        <div
          key={toast.id}
          style={{
            animationDelay: `${index * 100}ms`,
          }}
        >
          <Toast
            message={toast.message}
            type={toast.type}
            onClose={() => onRemove(toast.id)}
          />
        </div>
      ))}
    </div>
  );
}
