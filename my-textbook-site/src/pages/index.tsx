import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import PhysicalAILanding from '@site/src/components/PhysicalAILanding';

export default function Home(): ReactNode {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="A comprehensive guide to building intelligent physical systems. Learn ROS 2, simulation, and vision-language-action systems.">
      <PhysicalAILanding />
    </Layout>
  );
}
