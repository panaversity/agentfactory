import type { ReactNode } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { IDEShowcaseSection } from "@/components/HeroIDESimulation";

import styles from "./index.module.css";


import { ThreeDBook } from "@/components/ThreeDBook";
import { Code, Container, Cpu, Terminal, ArrowRight, BookOpen, Layers, GitBranch, Bot, Server, GraduationCap } from "lucide-react";

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx(styles.heroBanner, "relative overflow-hidden border-b border-border/40")}>
      <div className="max-w-[1800px] mx-auto relative z-10 h-full">
        <div className="grid grid-cols-1 lg:grid-cols-[1.2fr_1fr] min-h-[85vh]">

          {/* LEFT COLUMN: Technical Content */}
          <div className="flex flex-col justify-center px-6 md:px-12 lg:px-16 py-12 lg:py-0 border-r border-border/40">

            {/* Semantic Badge */}
            <div className="flex items-center gap-3 mb-8">
              <span className="font-mono text-xs font-bold tracking-widest uppercase text-muted-foreground/80 px-2 py-1 border border-border bg-muted/20">
                v1.0.0 Release
              </span>
              <span className="w-12 h-[1px] bg-border"></span>
              <span className="font-mono text-xs text-muted-foreground/60 tracking-wider">
                ENGINEERING BOOK
              </span>
            </div>

            {/* Heading */}
            <div className="space-y-6 mb-10">
              <h1 className="text-2xl md:text-4xl lg:text-5xl xl:text-6xl font-black tracking-tighter text-foreground leading-[1.0] uppercase">
                AI NATIVE <br />
                <span className="text-primary block text-2xl md:text-4xl lg:text-5xl xl:text-6xl tracking-tight mt-1">
                  SOFTWARE DEVELOPMENT
                </span>
              </h1>
              <p className="text-lg md:text-xl text-muted-foreground font-normal leading-[1.6] max-w-xl">
                The comprehensive engineering book for the <span className="text-foreground font-medium">Agentic Era</span>.
                Move beyond autocomplete to orchestrating intelligent systems.
              </p>
            </div>

            {/* CTA Area */}
            {/* CTA Area */}
            <div className="flex flex-col items-start gap-6 mb-16">
              <div className="flex flex-col sm:flex-row gap-4">
                <Button asChild size="lg" className="h-14 px-8 text-lg font-bold rounded-none bg-primary hover:bg-primary/90 text-primary-foreground transition-all">
                  <Link to="/docs/preface-agent-native" className="flex items-center gap-3">
                    START READING <ArrowRight className="w-5 h-5" />
                  </Link>
                </Button>
                <Button asChild variant="outline" size="lg" className="h-14 px-8 text-lg font-bold rounded-none transition-all">
                  <Link to="https://panaversity.org/">
                    Explore Panaversity
                  </Link>
                </Button>
              </div>
              <div className="text-sm text-muted-foreground font-mono">
                // Open Source
              </div>
            </div>

          </div>

          {/* RIGHT COLUMN: The Artifact (Book) */}
          <div className="relative w-full h-full min-h-[500px] flex items-center justify-center">
            {/* Technical Grid Background */}
            <div className="absolute inset-0 opacity-[0.03]"
              style={{ backgroundImage: 'radial-gradient(circle, currentColor 1px, transparent 1px)', backgroundSize: '40px 40px' }}>
            </div>

            {/* Spotlight Effect */}
            <div className={styles.heroSpotlight} />

            <div className="relative z-10 transform transition-transform duration-700 hover:scale-[1.28] scale-[1.25]">
              <ThreeDBook
                src="/img/book-cover-page.png"
                alt="AI Native Software Development Book Cover"
              />
            </div>
          </div>

        </div>
      </div>
    </header>
  );
}


import { Card, CardContent } from "@/components/ui/card";

function Feature({
  title,
  description,
  icon: Icon,
}: {
  title: string;
  description: string;
  icon: React.ElementType;
}) {
  return (
    <div className="group border border-border bg-card hover:bg-muted/30 transition-colors p-6 flex flex-col items-start gap-4">
      <div className="p-3 bg-primary/10 rounded-none border border-primary/20 group-hover:bg-primary/20 transition-colors">
        <Icon className="w-6 h-6 text-primary stroke-[1.5]" />
      </div>
      <div>
        <h3 className="text-base font-bold text-foreground mb-2 uppercase tracking-wide">{title}</h3>
        <p className="text-muted-foreground leading-relaxed text-sm">{description}</p>
      </div>
    </div>
  );
}

function AISpectrumSection() {
  return (
    <section className="py-24 border-b border-border/40 bg-background">
      <div className="max-w-[1800px] mx-auto px-6 md:px-12 lg:px-16">
        {/* Header */}
        <div className="text-center mb-16">
          <p className="text-sm font-medium text-primary uppercase tracking-widest mb-3">
            Understanding AI Development
          </p>
          <h2 className="text-3xl md:text-4xl font-bold text-foreground mb-4">
            The AI Development Spectrum
          </h2>
          <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
            Three distinct approaches to AI in software development. This book
            teaches you both AI-Driven and AI-Native development.
          </p>
        </div>

        {/* Cards Grid */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6 mb-12">
          {/* AI Assisted */}
          <Card className="flex flex-col border bg-card hover:border-border transition-all duration-300">
            <CardContent className="flex flex-col h-full p-6">
              <div className="mb-4">
                <h3 className="text-xl font-bold text-foreground">AI Assisted</h3>
                <p className="text-sm font-medium text-muted-foreground uppercase tracking-wide">AI as Helper</p>
              </div>
              <p className="text-muted-foreground mb-6 flex-grow">
                AI enhances your productivity with code completion, debugging assistance, and documentation generation.
              </p>
              <ul className="space-y-2 mb-6 text-sm text-muted-foreground">
                <li className="flex items-start"><span className="mr-2 text-primary">•</span>Code completion & suggestions</li>
                <li className="flex items-start"><span className="mr-2 text-primary">•</span>Bug detection & debugging</li>
                <li className="flex items-start"><span className="mr-2 text-primary">•</span>Documentation generation</li>
              </ul>
              <div className="mt-auto pt-4 border-t text-xs text-muted-foreground">
                <strong className="text-foreground">Example:</strong> Using Copilot to build a React website faster
              </div>
            </CardContent>
          </Card>

          {/* AI Driven */}
          <Card className="flex flex-col border-2 border-primary bg-card relative transition-all duration-300 hover:shadow-lg">
            <CardContent className="flex flex-col h-full p-6">
              <Badge variant="outline" className="absolute top-4 right-4 border-primary text-primary">Covered</Badge>
              <div className="mb-4">
                <h3 className="text-xl font-bold text-foreground">AI Driven</h3>
                <p className="text-sm font-medium text-primary uppercase tracking-wide">AI as Co-Creator</p>
              </div>
              <p className="text-muted-foreground mb-6 flex-grow">
                AI generates significant code from specifications. You act as architect, director, and reviewer.
              </p>
              <ul className="space-y-2 mb-6 text-sm text-muted-foreground">
                <li className="flex items-start"><span className="mr-2 text-primary">•</span>Code generation from specs</li>
                <li className="flex items-start"><span className="mr-2 text-primary">•</span>Automated testing & optimization</li>
                <li className="flex items-start"><span className="mr-2 text-primary">•</span>Architecture from requirements</li>
              </ul>
              <div className="mt-auto pt-4 border-t text-xs text-muted-foreground">
                <strong className="text-foreground">Example:</strong> Writing a spec for a REST API, AI generates complete FastAPI backend
              </div>
            </CardContent>
          </Card>

          {/* AI Native */}
          <Card className="flex flex-col border-2 border-primary bg-card relative transition-all duration-300 hover:shadow-lg">
            <CardContent className="flex flex-col h-full p-6">
              <Badge className="absolute top-4 right-4">Ultimate Goal</Badge>
              <div className="mb-4">
                <h3 className="text-xl font-bold text-foreground">AI Native</h3>
                <p className="text-sm font-medium text-primary uppercase tracking-wide">AI IS the Software</p>
              </div>
              <p className="text-muted-foreground mb-6 flex-grow">
                Applications architected around AI capabilities. LLMs and agents are core functional components.
              </p>
              <ul className="space-y-2 mb-6 text-sm text-muted-foreground">
                <li className="flex items-start"><span className="mr-2 text-primary">•</span>Natural language interfaces</li>
                <li className="flex items-start"><span className="mr-2 text-primary">•</span>Intelligent automation & reasoning</li>
                <li className="flex items-start"><span className="mr-2 text-primary">•</span>Agent orchestration systems</li>
              </ul>
              <div className="mt-auto pt-4 border-t text-xs text-muted-foreground">
                <strong className="text-foreground">Example:</strong> Building a customer support agent that autonomously resolves tickets
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Simplified Flow Track */}
        <div className="flex items-center justify-center gap-4">
          <div className="flex items-center gap-2">
            <div className="w-3 h-3 rounded-full bg-muted-foreground/30" />
            <span className="text-sm text-muted-foreground">Helper</span>
          </div>
          <div className="w-12 h-px bg-border" />
          <div className="flex items-center gap-2">
            <div className="w-3 h-3 rounded-full bg-primary" />
            <span className="text-sm font-medium text-foreground">Co-Creator</span>
          </div>
          <div className="w-12 h-px bg-border" />
          <div className="flex items-center gap-2">
            <div className="w-3 h-3 rounded-full bg-primary" />
            <span className="text-sm font-medium text-foreground">Core System</span>
          </div>
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  return (
    <section className="py-24 border-b border-border/40 bg-background">
      <div className="max-w-[1800px] mx-auto px-6 md:px-12 lg:px-16">
        {/* Section Header - Technical Style */}
        <div className="flex flex-col md:flex-row justify-between items-end border-b border-border/40 pb-8 mb-12 gap-6">
          <div className="max-w-2xl">
            <div className="flex items-center gap-2 mb-4">
              <div className="w-2 h-2 bg-primary"></div>
              <span className="font-mono text-xs font-bold text-primary uppercase tracking-widest">
                Core Pillars
              </span>
            </div>
            <h2 className="text-4xl font-black tracking-tight text-foreground uppercase">
              What Makes This Book Different

            </h2>
          </div>
          <p className="text-muted-foreground max-w-lg text-sm font-mono text-right md:text-right hidden md:block">
              // A comprehensive, production-focused approach  <br /> to co-learn with AI in spec-driven way
          </p>
        </div>

        {/* Technical Grid */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-px bg-border/40 border border-border/40">
          <Feature
            icon={Bot}
            title="Co-Learning Philosophy"
            description="Learn alongside AI agents. Not just using AI as a tool, but co-creating where both human and AI learn together."
          />
          <Feature
            icon={Code}
            title="Dual Language Mastery"
            description="Python for reasoning & intelligence, TypeScript for interaction & UI. Master the bilingual AI-native stack."
          />
          <Feature
            icon={GitBranch}
            title="Spec-Driven Development"
            description="Write specifications that both humans and AI understand. Specs become executable blueprints for intelligent systems."
          />
          <Feature
            icon={Layers}
            title="Agentic AI Systems"
            description="Build with OpenAI Agents SDK and Google ADK. Create agents that reason, act, and collaborate autonomously."
          />
          <Feature
            icon={Server}
            title="Production-Ready Architecture"
            description="Cloud-native deployment with Docker, Kubernetes, Dapr, and Ray. Scalable, secure, fault-tolerant systems."
          />
          <Feature
            icon={GraduationCap}
            title="Complete Learning Journey"
            description="46 comprehensive chapters from programming basics to deploying enterprise agentic AI systems in production."
          />
        </div>
      </div>
    </section>
  );
}

function MaturityLevelsSection() {
  const levels = [
    {
      number: 1,
      title: "AI Awareness",
      subtitle: "Experimenting",
      impact: "10-20% productivity gains",
      description: "Individual developers experimenting with AI coding tools. Early AI Assisted Development.",
      approach: "AI Assisted (Individual)",
    },
    {
      number: 2,
      title: "AI Adoption",
      subtitle: "Standardizing",
      impact: "30-40% productivity boost",
      description: "Organization-wide adoption with governance. Established guidelines and security policies.",
      approach: "AI Assisted (Team)",
    },
    {
      number: 3,
      title: "AI Integration",
      subtitle: "Transforming Workflows",
      impact: "2-3x faster development",
      description: "AI-Driven Development practices. Specs become living documentation. Workflows redesigned around AI collaboration.",
      approach: "AI Driven (Workflow)",
    },
    {
      number: 4,
      title: "AI-Native Products",
      subtitle: "Building Intelligence",
      impact: "New capabilities unlocked",
      description: "Products where AI/LLMs are core components. Agent orchestration, natural language interfaces, intelligent systems.",
      approach: "AI Native (Product)",
      focus: true,
    },
    {
      number: 5,
      title: "AI-First Enterprise",
      subtitle: "Living in the Future",
      impact: "10x productivity",
      description: "Entire organization AI-native. Custom models, self-improving systems, AI embedded in every aspect.",
      approach: "AI Native (Enterprise)",
    },
  ];

  return (
    <section className="py-24 border-b border-border/40 bg-background">
      <div className="max-w-[1800px] mx-auto px-6 md:px-12 lg:px-16">
        {/* Header */}
        <div className="text-center mb-16">
          <p className="text-sm font-medium text-primary uppercase tracking-widest mb-3">
            Your AI Journey
          </p>
          <h2 className="text-3xl md:text-4xl font-bold text-foreground mb-4">
            Organizational AI Maturity Levels
          </h2>
          <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
            Where does your organization stand? Understanding these levels helps
            you chart your path forward.
          </p>
        </div>

        {/* Levels List with vertical connector */}
        <div className="relative max-w-5xl mx-auto">
          {/* Vertical connector line */}
          <div className="absolute left-4 md:left-6 top-0 bottom-0 w-px bg-border" />

          <div className="space-y-6">
            {levels.map((level) => (
              <div
                key={level.number}
                className={`relative pl-12 md:pl-16 ${level.focus ? 'py-6 bg-card border-l-2 border-primary -ml-px' : ''}`}
              >
                {/* Number circle */}
                <div className={`absolute left-0 md:left-2 w-8 h-8 rounded-full flex items-center justify-center text-sm font-bold ${level.focus ? 'bg-primary text-primary-foreground' : 'bg-muted text-muted-foreground'}`}>
                  {level.number}
                </div>

                {/* Content */}
                <div className="flex flex-col md:flex-row md:items-start md:justify-between gap-2">
                  <div className="flex-1">
                    <div className="flex items-center gap-3 mb-1">
                      <h3 className="text-lg font-semibold text-foreground">{level.title}</h3>
                      {level.focus && <Badge>Focus</Badge>}
                    </div>
                    <p className="text-sm text-primary font-medium mb-2">{level.subtitle}</p>
                    <p className="text-muted-foreground text-sm mb-2">{level.description}</p>
                    <p className="text-xs text-muted-foreground">
                      <strong className="text-foreground">Approach:</strong> {level.approach}
                    </p>
                  </div>
                  <div className="text-sm font-medium text-primary whitespace-nowrap">
                    {level.impact}
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Bottom CTA */}
        <div className="mt-12 text-center">
          <p className="text-muted-foreground">
            <strong className="text-foreground">This book prepares you for Levels 3-4:</strong> Master
            AI-Driven workflows and build AI-Native products
          </p>
        </div>
      </div>
    </section>
  );
}

function ParadigmShift() {
  const traditionalItems = [
    { title: "Instruction-Based", desc: "Tell computers exactly what to do with precise syntax" },
    { title: "Solo Coding", desc: "Developer writes every line manually" },
    { title: "Documentation as Afterthought", desc: "Specs are static contracts written post-facto" },
    { title: "Linear Learning", desc: "Learn syntax → Build simple projects → Slowly scale" },
    { title: "Code-First", desc: "Focus on implementation details from day one" },
  ];

  const aiNativeItems = [
    { title: "Intent-Based", desc: "Describe what you want; AI reasons how to build it" },
    { title: "Co-Learning Partnership", desc: "You and AI teach each other through iteration" },
    { title: "Specs as Living Blueprints", desc: "Specifications drive code, tests, and documentation" },
    { title: "Production-First Learning", desc: "Build real agentic systems from day one" },
    { title: "Architecture-First", desc: "Design intelligent collaborations, not just code" },
  ];

  return (
    <section className="py-24 border-b border-border/40 bg-background">
      <div className="max-w-[1800px] mx-auto px-6 md:px-12 lg:px-16">
        {/* Section Header */}
        <div className="text-center mb-16">
          <p className="text-sm font-medium text-primary uppercase tracking-widest mb-3">
            The Great Shift
          </p>
          <h2 className="text-3xl md:text-4xl font-bold text-foreground mb-4">
            From Automation to Intelligence
            <br />
            <span className="text-primary">From Coding to Co-Creating</span>
          </h2>
          <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
            AI-native development is not about replacing developers—it's about
            amplifying intelligence. Learn to collaborate with reasoning
            entities that learn with you.
          </p>
        </div>

        {/* Comparison Grid */}
        <div className="grid grid-cols-1 md:grid-cols-[1fr_auto_1fr] gap-6 md:gap-4 items-stretch mb-16">
          {/* Traditional Card */}
          <Card className="border bg-card">
            <CardContent className="p-8">
              <div className="text-center mb-6">
                <h3 className="text-xl font-bold text-foreground">Traditional Development</h3>
                <p className="text-muted-foreground uppercase text-xs font-medium tracking-wider mt-1">The automation era</p>
              </div>
              <ul className="space-y-4">
                {traditionalItems.map((item, i) => (
                  <li key={i} className="text-sm text-muted-foreground">
                    <strong className="block text-foreground mb-1">{item.title}</strong>
                    {item.desc}
                  </li>
                ))}
              </ul>
            </CardContent>
          </Card>

          {/* VS Divider */}
          <div className="hidden md:flex items-center justify-center">
            <div className="flex flex-col items-center gap-2">
              <div className="w-px h-16 bg-border" />
              <span className="text-sm font-bold text-muted-foreground">VS</span>
              <div className="w-px h-16 bg-border" />
            </div>
          </div>

          {/* AI-Native Card */}
          <Card className="border-2 border-primary bg-card relative">
            <CardContent className="p-8">
              <Badge className="absolute top-4 right-4">The Future</Badge>
              <div className="text-center mb-6">
                <h3 className="text-xl font-bold text-foreground">AI-Native Way</h3>
                <p className="text-primary uppercase text-xs font-medium tracking-wider mt-1">The intelligence era</p>
              </div>
              <ul className="space-y-4">
                {aiNativeItems.map((item, i) => (
                  <li key={i} className="text-sm text-muted-foreground">
                    <strong className="block text-primary mb-1">{item.title}</strong>
                    {item.desc}
                  </li>
                ))}
              </ul>
            </CardContent>
          </Card>
        </div>

      </div>
    </section>
  );
}

function FinalCTA() {
  return (
    <section className="py-24 border-b border-border/40 bg-background">
      <div className="max-w-[1800px] mx-auto px-6 md:px-12 lg:px-16 text-center pt-12 pb-12 min-h-[400px] flex flex-col items-center justify-center">
        <h3 className="text-3xl md:text-4xl font-bold text-foreground mb-4">
          Ready to Co-Learn with AI?
        </h3>
        <p className="text-lg md:text-xl text-muted-foreground mb-8 max-w-2xl mx-auto">
          Join the revolution where coding becomes conversation and
          software becomes alive
        </p>
        <Button variant="outline" size="lg" className="h-14 px-8 text-lg font-semibold" asChild>
          <Link to="/docs/preface-agent-native">
            Start Reading
          </Link>
        </Button>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="AI Native Software Development"
      description="Colearning Agentic AI with Python and TypeScript – Spec Driven Reusable Intelligence. Build production-ready intelligent systems."
    >
      <HomepageHeader />
      <AISpectrumSection />
      <FeaturesSection />
      <MaturityLevelsSection />
      <ParadigmShift />
      <IDEShowcaseSection />
      <FinalCTA />
    </Layout>
  );
}
