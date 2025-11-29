import { SignUpForm } from "@/components/sign-up-form";

export default function SignUpPage() {
  return (
    <div>
      <h2 className="text-2xl font-semibold text-gray-900 text-center mb-6">
        Create your account
      </h2>
      <SignUpForm />
    </div>
  );
}
