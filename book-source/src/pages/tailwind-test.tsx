import type { ReactNode } from "react";
import Layout from "@theme/Layout";
import TailwindTestComponent from "@/components/TailwindTestComponent";

export default function TailwindTestPage(): ReactNode {
  return (
    <Layout
      title="Tailwind CSS v3 Test"
      description="Testing Tailwind CSS v3 integration with Docusaurus"
    >
      <TailwindTestComponent />
    </Layout>
  );
}
