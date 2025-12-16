import React from "react";
import MDXComponents from "@theme-original/MDXComponents";
import Quiz from "@/components/quiz/Quiz";
import GatedQuiz from "@/components/GatedQuiz";
import GatedSummary from "@/components/GatedSummary";
import InteractivePython from "@/components/InteractivePython";
import PDFViewer from "@/components/PDFViewer";
import ContentGate from "@/components/ContentGate";

export default {
  ...MDXComponents,
  // Original Quiz (ungated) - use when quiz should be freely accessible
  UnlockedQuiz: Quiz,
  // Default Quiz is now gated - requires sign-in
  Quiz: GatedQuiz,
  // Gated summary section for lesson takeaways
  GatedSummary,
  // Generic content gate for wrapping any content
  ContentGate,
  InteractivePython,
  PDFViewer,
};
