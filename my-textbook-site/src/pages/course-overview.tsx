import React from 'react';
import Layout from '@theme/Layout';

export default function CourseOverview(): React.ReactNode {
  return (
    <Layout
      title="Course Overview"
      description="An overview of the Physical AI and Humanoid Robotics textbook course.">
      <header className="hero hero--primary">
        <div className="container">
          <h1 className="hero__title">Course Overview</h1>
          <p className="hero__subtitle">
            Explore the chapters, labs, and capstone projects.
          </p>
        </div>
      </header>
      <main>
        <div className="container padding-bottom--xl padding-top--xl">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <h2>Welcome to the Physical AI & Humanoid Robotics Textbook!</h2>
              <p>
                This textbook provides a comprehensive exploration of physical AI
                and humanoid robotics, covering fundamental concepts, advanced
                techniques, and practical applications.
              </p>
              <h3>Course Structure</h3>
              <ul>
                <li><h4>Core Chapters:</h4> Foundational knowledge in robotics, AI, and control systems.</li>
                <li><h4>Labs:</h4> Hands-on exercises to reinforce theoretical concepts.</li>
                <li><h4>Capstone Projects:</h4> Opportunities to apply learned skills to real-world challenges.</li>
              </ul>
              {/* Add more sections, e.g., using custom components for syllabus, learning outcomes, etc. */}
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
