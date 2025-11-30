import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // CORS is now handled by middleware.ts for dynamic origin checking
  // This allows multiple origins to be configured via ALLOWED_ORIGINS env var
};

export default nextConfig;
