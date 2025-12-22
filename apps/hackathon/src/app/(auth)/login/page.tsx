import Link from "next/link";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Trophy, LogIn } from "lucide-react";

export const metadata = {
  title: "Login",
  description: "Log in to your account",
};

export default async function LoginPage({
  searchParams,
}: {
  searchParams: Promise<{ error?: string; returnUrl?: string }>;
}) {
  const { error, returnUrl } = await searchParams;

  return (
    <div className="flex min-h-screen items-center justify-center bg-muted/50 px-4">
      <Card className="w-full max-w-md">
        <CardHeader className="text-center">
          <div className="mx-auto mb-4 flex h-12 w-12 items-center justify-center rounded-full bg-primary">
            <Trophy className="h-6 w-6 text-primary-foreground" />
          </div>
          <CardTitle className="text-2xl">
            {process.env.NEXT_PUBLIC_APP_NAME || "Hackathon Platform"}
          </CardTitle>
          <CardDescription>
            Sign in with your Panaversity account to continue
          </CardDescription>
        </CardHeader>
        <CardContent>
          {error && (
            <div className="mb-4 rounded-md bg-destructive/10 p-3 text-sm text-destructive">
              {getErrorMessage(error)}
            </div>
          )}

          <Button asChild className="w-full" size="lg">
            <a
              href={`/api/auth/login${returnUrl ? `?returnUrl=${encodeURIComponent(returnUrl)}` : ""}`}
            >
              <LogIn className="mr-2 h-4 w-4" />
              Sign in with SSO
            </a>
          </Button>

          <p className="mt-4 text-center text-xs text-muted-foreground">
            Don't have an account?{" "}
            <Link
              href={`${process.env.NEXT_PUBLIC_SSO_URL}/register`}
              className="text-primary underline-offset-4 hover:underline"
            >
              Sign up
            </Link>
          </p>
        </CardContent>
      </Card>
    </div>
  );
}

function getErrorMessage(error: string): string {
  switch (error) {
    case "state_mismatch":
      return "Security check failed. Please try again.";
    case "missing_params":
      return "Invalid response from authentication server.";
    case "no_id_token":
      return "Authentication failed. Please try again.";
    case "auth_failed":
      return "Authentication failed. Please try again.";
    default:
      return "An error occurred during login. Please try again.";
  }
}
