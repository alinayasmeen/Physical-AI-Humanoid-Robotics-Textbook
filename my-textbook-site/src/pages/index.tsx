import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg margin-right--md"
            to="/course/intro"> {/* Link to your course introduction */}
            Start Learning ⏱️
          </Link>
          <Link
            className="button button--info button--lg margin-left--md"
            to="/course-overview"> {/* Link to your Course Overview page */}
            Course Overview
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Your multi-chapter textbook for Physical AI and Humanoid Robotics.">
      <HomepageHeader />
      <main>
        {/* You can add more sections here, e.g., using HomepageFeatures */}
      </main>
    </Layout>
  );
}
