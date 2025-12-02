"use client";

export function HomePageStyles() {
  return (
    <style jsx global>{`
      @import url('https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700&display=swap');

      @keyframes securityGrid {
        0%, 100% {
          transform: translateY(0) scale(1);
          opacity: 0.03;
        }
        50% {
          transform: translateY(-10px) scale(1.02);
          opacity: 0.06;
        }
      }

      @keyframes fadeInUp {
        from {
          opacity: 0;
          transform: translateY(20px);
        }
        to {
          opacity: 1;
          transform: translateY(0);
        }
      }

      @keyframes pulseGlow {
        0%, 100% {
          box-shadow: 0 0 20px rgba(16, 185, 129, 0.1);
        }
        50% {
          box-shadow: 0 0 30px rgba(16, 185, 129, 0.2);
        }
      }

      .security-grid-bg {
        position: fixed;
        inset: 0;
        background:
          linear-gradient(135deg, #0f172a 0%, #1e293b 50%, #0f172a 100%);
        overflow: hidden;
      }

      .security-grid-bg::before {
        content: '';
        position: absolute;
        inset: 0;
        background-image:
          linear-gradient(rgba(148, 163, 184, 0.05) 1px, transparent 1px),
          linear-gradient(90deg, rgba(148, 163, 184, 0.05) 1px, transparent 1px);
        background-size: 50px 50px;
        animation: securityGrid 8s ease-in-out infinite;
      }

      .security-grid-bg::after {
        content: '';
        position: absolute;
        inset: 0;
        background: radial-gradient(circle at 50% 50%, rgba(16, 185, 129, 0.05) 0%, transparent 70%);
      }

      .card-glass {
        background: rgba(255, 255, 255, 0.95);
        backdrop-filter: blur(20px);
        border: 1px solid rgba(255, 255, 255, 0.2);
        animation: fadeInUp 0.6s ease-out;
      }

      .status-badge {
        animation: fadeInUp 0.6s ease-out 0.2s backwards, pulseGlow 3s ease-in-out infinite;
      }

      .welcome-text {
        animation: fadeInUp 0.6s ease-out 0.3s backwards;
      }

      .user-info {
        animation: fadeInUp 0.6s ease-out 0.4s backwards;
      }

      .action-buttons {
        animation: fadeInUp 0.6s ease-out 0.5s backwards;
      }

      .primary-button {
        position: relative;
        overflow: hidden;
        transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
      }

      .primary-button::before {
        content: '';
        position: absolute;
        top: 50%;
        left: 50%;
        width: 0;
        height: 0;
        border-radius: 50%;
        background: rgba(255, 255, 255, 0.2);
        transform: translate(-50%, -50%);
        transition: width 0.5s, height 0.5s;
      }

      .primary-button:hover::before {
        width: 300px;
        height: 300px;
      }

      .primary-button:hover {
        transform: translateY(-2px);
        box-shadow: 0 20px 40px rgba(16, 185, 129, 0.3);
      }

      .secondary-button {
        transition: all 0.3s ease;
        border: 2px solid rgba(148, 163, 184, 0.3);
      }

      .secondary-button:hover {
        border-color: rgba(148, 163, 184, 0.6);
        background: rgba(148, 163, 184, 0.05);
        transform: translateY(-1px);
      }

      .footer-text {
        animation: fadeInUp 0.6s ease-out 0.6s backwards;
      }
    `}</style>
  );
}
