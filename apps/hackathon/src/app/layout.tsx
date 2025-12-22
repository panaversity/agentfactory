import "./global.css";
import { Inter } from "next/font/google";
import { Toaster } from "@/components/ui/sonner";

const inter = Inter({ subsets: ["latin"] });

export const metadata = {
  title: {
    default: process.env.NEXT_PUBLIC_APP_NAME || "Hackathon Platform",
    template: `%s | ${process.env.NEXT_PUBLIC_APP_NAME || "Hackathon Platform"}`,
  },
  description: "AI-first hackathon platform for organizing and managing hackathons",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className={`${inter.className} min-h-screen bg-background antialiased`}>
        {children}
        <Toaster position="top-right" richColors />
      </body>
    </html>
  );
}
