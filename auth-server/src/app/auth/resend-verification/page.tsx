import { Suspense } from "react";
import { ResendVerificationForm } from "@/components/resend-verification-form";

export default function ResendVerificationPage() {
  return (
    <div>
      <h2 className="text-2xl font-semibold text-gray-900 text-center mb-6">
        Resend Verification Email
      </h2>
      <Suspense
        fallback={
          <div className="flex items-center justify-center py-8">
            <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
          </div>
        }
      >
        <ResendVerificationForm />
      </Suspense>
    </div>
  );
}
