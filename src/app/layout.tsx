import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "RoboLearn - Authentication",
  description: "Sign in or create an account for RoboLearn",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="light">
      <body className="antialiased bg-gray-50 min-h-screen">
        {children}
      </body>
    </html>
  );
}
