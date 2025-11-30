import { ForgotPasswordForm } from "@/components/forgot-password-form";

export default function ForgotPasswordPage() {
  return (
    <div>
      <h2 className="text-2xl font-semibold text-gray-900 text-center mb-6">
        Reset your password
      </h2>
      <ForgotPasswordForm />
    </div>
  );
}
