import React, { useEffect, useRef } from 'react';
import styles from './styles.module.css';

interface Node {
  x: number;
  y: number;
  vx: number;
  vy: number;
  connections: number[];
  label?: string;
}

// Labels removed for cleaner background - nodes are purely decorative
const NODE_LABELS: string[] = [];

/**
 * NeuralMesh - Animated canvas background showing interconnected nodes
 * representing the platform's knowledge graph (ROS, Isaac, VLA, etc.)
 */
export function NeuralMesh(): React.ReactElement {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationRef = useRef<number>();
  const nodesRef = useRef<Node[]>([]);
  const mouseRef = useRef({ x: 0, y: 0 });

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    const resizeCanvas = () => {
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      ctx.scale(dpr, dpr);
    };
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    // Initialize nodes
    const initNodes = () => {
      const rect = canvas.getBoundingClientRect();
      const nodeCount = 12;
      const nodes: Node[] = [];

      for (let i = 0; i < nodeCount; i++) {
        const node: Node = {
          x: Math.random() * rect.width,
          y: Math.random() * rect.height,
          vx: (Math.random() - 0.5) * 0.3,
          vy: (Math.random() - 0.5) * 0.3,
          connections: [],
          label: i < NODE_LABELS.length ? NODE_LABELS[i] : undefined,
        };
        nodes.push(node);
      }

      // Create connections (each node connects to 2-3 nearest)
      nodes.forEach((node, i) => {
        const distances = nodes
          .map((other, j) => ({ j, dist: Math.hypot(node.x - other.x, node.y - other.y) }))
          .filter((d) => d.j !== i)
          .sort((a, b) => a.dist - b.dist);

        node.connections = distances.slice(0, 2 + Math.floor(Math.random() * 2)).map((d) => d.j);
      });

      nodesRef.current = nodes;
    };
    initNodes();

    // Mouse tracking for parallax
    const handleMouseMove = (e: MouseEvent) => {
      const rect = canvas.getBoundingClientRect();
      mouseRef.current = {
        x: (e.clientX - rect.left) / rect.width,
        y: (e.clientY - rect.top) / rect.height,
      };
    };
    window.addEventListener('mousemove', handleMouseMove);

    // Animation loop
    let dataFlowOffset = 0;
    const animate = () => {
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);

      const nodes = nodesRef.current;
      dataFlowOffset += 0.02;

      // Update node positions
      nodes.forEach((node) => {
        node.x += node.vx;
        node.y += node.vy;

        // Bounce off edges
        if (node.x < 50 || node.x > rect.width - 50) node.vx *= -1;
        if (node.y < 50 || node.y > rect.height - 50) node.vy *= -1;

        // Mouse influence (subtle push away)
        const mouseX = mouseRef.current.x * rect.width;
        const mouseY = mouseRef.current.y * rect.height;
        const dist = Math.hypot(node.x - mouseX, node.y - mouseY);
        if (dist < 150) {
          const force = (150 - dist) / 150 * 0.5;
          node.x += (node.x - mouseX) / dist * force;
          node.y += (node.y - mouseY) / dist * force;
        }
      });

      // Draw connections with data flow effect
      ctx.strokeStyle = 'rgba(0, 212, 255, 0.15)';
      ctx.lineWidth = 1;

      nodes.forEach((node, i) => {
        node.connections.forEach((j) => {
          const other = nodes[j];
          const gradient = ctx.createLinearGradient(node.x, node.y, other.x, other.y);

          // Animated data pulse along the line
          const pulsePos = (dataFlowOffset + i * 0.3) % 1;
          gradient.addColorStop(0, 'rgba(0, 212, 255, 0.05)');
          gradient.addColorStop(Math.max(0, pulsePos - 0.1), 'rgba(0, 212, 255, 0.05)');
          gradient.addColorStop(pulsePos, 'rgba(0, 212, 255, 0.4)');
          gradient.addColorStop(Math.min(1, pulsePos + 0.1), 'rgba(0, 212, 255, 0.05)');
          gradient.addColorStop(1, 'rgba(0, 212, 255, 0.05)');

          ctx.strokeStyle = gradient;
          ctx.beginPath();
          ctx.moveTo(node.x, node.y);
          ctx.lineTo(other.x, other.y);
          ctx.stroke();
        });
      });

      // Draw nodes
      nodes.forEach((node, i) => {
        // Outer glow
        const glowGradient = ctx.createRadialGradient(node.x, node.y, 0, node.x, node.y, 20);
        glowGradient.addColorStop(0, 'rgba(0, 212, 255, 0.3)');
        glowGradient.addColorStop(1, 'rgba(0, 212, 255, 0)');
        ctx.fillStyle = glowGradient;
        ctx.beginPath();
        ctx.arc(node.x, node.y, 20, 0, Math.PI * 2);
        ctx.fill();

        // Node circle
        ctx.fillStyle = node.label ? 'rgba(0, 212, 255, 0.8)' : 'rgba(0, 212, 255, 0.4)';
        ctx.beginPath();
        ctx.arc(node.x, node.y, node.label ? 6 : 4, 0, Math.PI * 2);
        ctx.fill();

        // Label
        if (node.label) {
          ctx.font = '10px "IBM Plex Mono", monospace';
          ctx.fillStyle = 'rgba(0, 212, 255, 0.6)';
          ctx.textAlign = 'center';
          ctx.fillText(node.label, node.x, node.y + 20);
        }
      });

      animationRef.current = requestAnimationFrame(animate);
    };

    animate();

    return () => {
      window.removeEventListener('resize', resizeCanvas);
      window.removeEventListener('mousemove', handleMouseMove);
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, []);

  return (
    <div className={styles.meshContainer}>
      <canvas ref={canvasRef} className={styles.meshCanvas} aria-hidden="true" />
      {/* Scan line overlay */}
      <div className={styles.scanLine} aria-hidden="true" />
    </div>
  );
}

export default NeuralMesh;
