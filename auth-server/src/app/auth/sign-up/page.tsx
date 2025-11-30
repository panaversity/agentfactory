import { SignUpForm } from "@/components/sign-up-form";
import { Suspense } from "react";

export default function SignUpPage() {
  return (
    <div>
      <h2 className="text-2xl font-semibold text-gray-900 text-center mb-6">
        Create your account
      </h2>
      <Suspense
        fallback={
          <div className="flex items-center justify-center py-8">
            <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
          </div>
        }
      >
        <SignUpForm />
      </Suspense>
    </div>
  );
}
