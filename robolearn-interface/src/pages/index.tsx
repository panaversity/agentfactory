import type { ReactNode } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";

import styles from "./index.module.css";

// Industrial-Kinetic Futurism Components
import HeroSection from "../components/HomepageComponents/HeroSection";
import ModuleCard from "../components/HomepageComponents/ModuleCard";
import FeatureCard from "../components/HomepageComponents/FeatureCard";
import HardwareTier from "../components/HomepageComponents/HardwareTier";
import CTABlock from "../components/HomepageComponents/CTABlock";

// Module data
const modules = [
  {
    moduleNumber: 1,
    icon: "🔌",
    title: "ROS 2 Fundamentals",
    subtitle: "Robot Middleware",
    description: "Master the robotic nervous system with ROS 2 middleware for robot control.",
    topics: ["Nodes, Topics, and Services", "Python Agents with rclpy", "URDF for Humanoids"],
    weekRange: "Weeks 1-5",
  },
  {
    moduleNumber: 2,
    icon: "🎮",
    title: "Digital Twins",
    subtitle: "Simulation",
    description: "Build physics simulations and high-fidelity environments.",
    topics: ["Gazebo Physics Simulation", "Unity Visualization", "Sensor Simulation (LiDAR, IMU)"],
    weekRange: "Weeks 6-7",
  },
  {
    moduleNumber: 3,
    icon: "🧠",
    title: "NVIDIA Isaac",
    subtitle: "AI-Powered",
    description: "Advanced perception, navigation, and sim-to-real transfer.",
    topics: ["Isaac Sim & Synthetic Data", "VSLAM & Navigation", "Reinforcement Learning"],
    weekRange: "Weeks 8-10",
    highlighted: true,
    badge: "AI-Powered",
  },
  {
    moduleNumber: 4,
    icon: "🗣️",
    title: "Vision-Language-Action",
    subtitle: "Capstone",
    description: "Convergence of LLMs and Robotics for conversational control.",
    topics: ["Voice-to-Action (Whisper)", "LLM Cognitive Planning", "Autonomous Humanoid Capstone"],
    weekRange: "Weeks 11-13",
    highlighted: true,
    badge: "Capstone",
  },
];

// Features data
const features = [
  {
    icon: "🤖",
    title: "Embodied Intelligence",
    description: "AI that operates in physical space, not just digital environments. Robots that understand physics and interact with the real world.",
    featured: true,
  },
  {
    icon: "🔧",
    title: "Human-Centered Design",
    description: "Humanoid robots navigate our world without modification. They use human tools, interfaces, and learn from demonstrations.",
  },
  {
    icon: "🎯",
    title: "Production-Ready Skills",
    description: "ROS 2, Gazebo, NVIDIA Isaac, and VLA models. The complete stack for modern robotics development.",
  },
  {
    icon: "🗣️",
    title: "Conversational Robotics",
    description: "Natural language commands translated to robot actions. 'Clean the room' becomes a sequence of coordinated movements.",
  },
  {
    icon: "🌐",
    title: "Sim-to-Real Transfer",
    description: "Train in simulation, deploy to reality. Photorealistic environments and domain randomization bridge the gap.",
  },
  {
    icon: "🚀",
    title: "Interactive Learning",
    description: "RAG-powered chat, personalized content, and hands-on exercises. Learn by doing, not just reading.",
    featured: true,
  },
];

// Hardware tiers data
const hardwareTiers = [
  {
    tierNumber: 1,
    title: "Workstation",
    subtitle: "Full Local Setup",
    description: "RTX 4070 Ti+, 64GB RAM, Ubuntu 22.04. Run Isaac Sim locally with full performance.",
    costEstimate: "~$2,500+ hardware",
    impactLabel: "Best Experience",
  },
  {
    tierNumber: 2,
    title: "Cloud + Edge",
    subtitle: "Hybrid Approach",
    description: "AWS/Azure GPU instances for simulation. Jetson kit for physical deployment.",
    costEstimate: "~$200/quarter cloud + $700 Jetson",
    impactLabel: "Flexible",
    recommended: true,
  },
  {
    tierNumber: 3,
    title: "Simulation Only",
    subtitle: "Learning Focus",
    description: "Cloud-based simulation without physical hardware. Complete the theory and simulation modules.",
    costEstimate: "Cloud compute only",
    impactLabel: "Lowest Cost",
  },
];

function ModulesSection() {
  return (
    <section className={styles.modulesSection} aria-labelledby="modules-heading">
      <div className={styles.container}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionLabel}>Your Journey</span>
          <h2 id="modules-heading" className={styles.sectionTitle}>
            From Zero to Building Robots That Think
          </h2>
          <p className={styles.sectionSubtitle}>
            Each step brings you closer to creating machines that move, sense, and understand
          </p>
        </div>

        <div className={styles.modulesGrid}>
          {modules.map((module, index) => (
            <ModuleCard
              key={module.moduleNumber}
              {...module}
              animationDelay={index * 100}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.featuresSection} aria-labelledby="features-heading">
      <div className={styles.container}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionLabel}>Why This Matters</span>
          <h2 id="features-heading" className={styles.sectionTitle}>
            Build Machines That Free Up Your Time
          </h2>
          <p className={styles.sectionSubtitle}>
            Robots that handle physical tasks while you focus on what matters most
          </p>
        </div>

        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <FeatureCard
              key={feature.title}
              {...feature}
              animationDelay={index * 100}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

function HardwareSection() {
  return (
    <section className={styles.hardwareSection} aria-labelledby="hardware-heading">
      <div className={styles.container}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionLabel}>Start Where You Are</span>
          <h2 id="hardware-heading" className={styles.sectionTitle}>
            No Expensive Hardware Required
          </h2>
          <p className={styles.sectionSubtitle}>
            Begin building today with just your browser. Scale up when you're ready.
          </p>
        </div>

        <div className={styles.hardwareGrid}>
          {hardwareTiers.map((tier, index) => (
            <HardwareTier
              key={tier.tierNumber}
              {...tier}
              animationDelay={index * 100}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="RoboLearn: Physical AI & Humanoid Robotics"
      description="Physical AI & Humanoid Robotics – Bridging the Digital Brain and the Physical Body. Learn ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems."
    >
      <main>
        {/* Skip link for accessibility */}
        <a href="#modules-heading" className={styles.skipLink}>
          Skip to content
        </a>

        {/* Industrial-Kinetic Futurism Sections */}
        <HeroSection />
        <ModulesSection />
        <FeaturesSection />
        <HardwareSection />
        <CTABlock />
      </main>
    </Layout>
  );
}
