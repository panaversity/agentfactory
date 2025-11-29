import { SignInForm } from "@/components/sign-in-form";

export default function SignInPage() {
  return (
    <div>
      <h2 className="text-2xl font-semibold text-gray-900 text-center mb-6">
        Welcome back
      </h2>
      <SignInForm />
    </div>
  );
}
