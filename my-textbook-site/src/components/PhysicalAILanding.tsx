import React, { useState } from 'react';
import { Book, Brain, Cpu, Eye, Zap, ChevronRight, Github, MessageCircle, Play, CheckCircle, Rocket } from 'lucide-react';
import styles from './PhysicalAILanding.module.css';

interface Feature {
  icon: React.ComponentType<{ className?: string }>;
  title: string;
  desc: string;
  url?: string;
}

const PhysicalAILanding = () => {
  const [hoveredChapter, setHoveredChapter] = useState<number | null>(null);

  const handleGetStartedClick = () => {
    // Scroll to the chapters section
    const chaptersSection = document.querySelector(`.${styles.chaptersSection}`);
    if (chaptersSection) {
      chaptersSection.scrollIntoView({ behavior: 'smooth' });
    }
  };

  const handleStartReadingClick = () => {
    // Navigate to the first documentation page
    window.location.href = '/docs/intro'; // This should be the actual intro page
  };

  const handleTryChatbotClick = () => {
    // Trigger a custom event that the chatbot component can listen for
    window.dispatchEvent(new CustomEvent('openChatbot', { detail: { source: 'landing-page' } }));
  };

  const handleChapterReadClick = (chapterId: number) => {
    // Navigate to the specific chapter or to the intro page if chapter-specific pages don't exist yet
    const chapterPages = [
      '/docs/intro',
      '/docs/chapters/introduction',
      '/docs/chapters/chapter1',
      '/docs/chapters/chapter2',
      '/docs/chapters/chapter3',
      '/docs/chapters/chapter4',
      '/docs/chapters/hardware-requirements',
      '/docs/chapters/quarter-overview',      // chapter 7
      '/docs/chapters/weekly-breakdown',      // chapter 8
      '/docs/chapters/capstone-intro',        // chapter 9
      '/docs/chapters/labs-intro'             // chapter 10
    ];
    const targetPage = chapterPages[chapterId] || '/docs/intro';
    window.location.href = targetPage;
  };

  const handleCtaPrimaryClick = () => {
    // Navigate to start learning
    window.location.href = '/docs/intro';
  };

  const handleCtaSecondaryClick = () => {
    // Navigate to GitHub
    window.open('https://github.com/alinayasmeen/Physical-AI-Humanoid-Robotics-Textbook', '_blank');
  };

  const chapters = [
    {
      id: 1,
      icon: Brain,
      title: "Introduction to Physical AI",
      description: "Learn the fundamentals of AI systems that interact with the physical world",
      color: "from-purple-500 to-pink-500",
      topics: ["What is Physical AI", "Real-world Applications", "Key Concepts"]
    },
    {
      id: 2,
      icon: Cpu,
      title: "Basics of Humanoid Robotics",
      description: "Explore mechanical design, actuators, sensors, and control systems",
      color: "from-blue-500 to-cyan-500",
      topics: ["Mechanical Design", "Actuators & Sensors", "Control Systems"]
    },
    {
      id: 3,
      icon: Zap,
      title: "ROS 2 Fundamentals",
      description: "Master the industry-standard framework for robot software development",
      color: "from-orange-500 to-red-500",
      topics: ["ROS 2 Architecture", "Nodes & Topics", "Services & Actions"]
    },
    {
      id: 4,
      icon: Eye,
      title: "Digital Twin Simulation",
      description: "Use Gazebo and Isaac Sim to test robots in virtual environments",
      color: "from-green-500 to-emerald-500",
      topics: ["Gazebo Basics", "Isaac Sim", "Virtual Testing"]
    },
    {
      id: 5,
      icon: Eye,
      title: "Vision-Language-Action Systems",
      description: "Build AI models that understand vision, language, and generate actions",
      color: "from-violet-500 to-purple-500",
      topics: ["Computer Vision", "NLP Integration", "Action Models"]
    },
    {
      id: 6,
      icon: Rocket,
      title: "Capstone Project",
      description: "Integrate all concepts to build a complete AI-robot pipeline",
      color: "from-yellow-500 to-orange-500",
      topics: ["Project Planning", "Integration", "Deployment"]
    }
  ];

  const features = [
    { icon: Book, title: "AI-Native Content", desc: "Auto-generated with latest AI technology" },
    { icon: MessageCircle, title: "RAG Chatbot", desc: "Ask questions about any chapter" },
    { icon: CheckCircle, title: "Free Tier", desc: "Optimized for zero-cost deployment" },
    { icon: Github, title: "Open Source", desc: "Full code available on GitHub", url: "https://github.com/alinayasmeen/Physical-AI-Humanoid-Robotics-Textbook" }
  ];

  return (
    <div className={styles.container}>
      {/* Animated Background */}
      <div className={styles.animatedBackground}>
        <div className={styles.purpleBlob}></div>
        <div className={styles.pinkBlob}></div>
        <div className={styles.blueBlob}></div>
      </div>

      {/* Hero Section */}
      <div className={styles.heroSection}>
        <div className={styles.textCenter}>
          <div className={styles.tagContainer}>
            <Play className="w-4 h-4" />
            <span>AI-Native Learning Experience</span>
          </div>
          
          <h1 className={styles.heroTitle}>
            Physical AI &
            <br />
            <span className={styles.gradientText}>
              Humanoid Robotics
            </span>
          </h1>
          
          <p className={styles.heroSubtitle}>
            A comprehensive guide to building intelligent physical systems.
            <br />
            Learn ROS 2, simulation, and vision-language-action systems.
          </p>

          <div className={styles.heroButtons}>
            <button className={styles.primaryButton} onClick={handleStartReadingClick}>
              Start Reading
              <ChevronRight className="w-5 h-5" />
            </button>
            <button className={styles.secondaryButton} onClick={handleTryChatbotClick}>
              <MessageCircle className="w-5 h-5" />
              Try Chatbot
            </button>
          </div>
        </div>

        {/* Stats */}
        <div className={styles.statsGrid}>
          {[
            { number: "6", label: "Chapters" },
            { number: "100%", label: "AI-Generated" },
            { number: "Free", label: "Forever" },
            { number: "RAG", label: "Chatbot" }
          ].map((stat, idx) => (
            <div key={idx} className={styles.statCard}>
              <div className={styles.statNumber}>
                {stat.number}
              </div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>
      </div>

      {/* Features Section */}
      <div className={styles.featuresSection}>
        <h2 className={styles.featuresTitle}>
          Why This Textbook?
        </h2>
        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <div
              key={idx}
              className={styles.featureCard}
              onClick={() => feature.url && window.open(feature.url, '_blank')}
              style={feature.url ? { cursor: 'pointer' } : {}}
            >
              <div className={styles.featureIconContainer}>
                <feature.icon className="w-6 h-6 text-white" />
              </div>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDesc}>{feature.desc}</p>
            </div>
          ))}
        </div>
      </div>

      {/* Chapters Section */}
      <div className={styles.chaptersSection}>
        <div className={styles.chaptersHeader}>
          <h2 className={styles.chaptersTitle}>
            Table of Contents
          </h2>
          <p className={styles.chaptersSubtitle}>
            Six comprehensive chapters covering everything from basics to capstone
          </p>
        </div>

        <div className={styles.chaptersGrid}>
          {chapters.map((chapter) => {
            const Icon = chapter.icon;
            const isHovered = hoveredChapter === chapter.id;

            return (
              <div
                key={chapter.id}
                onMouseEnter={() => setHoveredChapter(chapter.id)}
                onMouseLeave={() => setHoveredChapter(null)}
                className={styles.chapterCard}
                onClick={() => handleChapterReadClick(chapter.id)}
              >
                {/* Gradient Background */}
                <div className={styles.chapterGradient} style={{
                  background: chapter.color ? (() => {
                    const colorParts = chapter.color.split(' ');
                    if (colorParts.length >= 3) {
                      const fromColor = colorParts[0]?.replace('from-', '')?.replace('-500', '') || 'purple';
                      const toColor = colorParts[2]?.replace('to-', '')?.replace('-500', '') || 'pink';
                      return `linear-gradient(135deg, ${fromColor}, ${toColor})`;
                    }
                    return 'linear-gradient(135deg, #a855f7, #ec4899)';
                  })() : 'linear-gradient(135deg, #a855f7, #ec4899)'
                }}></div>

                {/* Content */}
                <div className={styles.chapterContent}>
                  <div className={styles.chapterHeader}>
                    <div className={styles.chapterIconContainer} style={{
                      background: chapter.color ? (() => {
                        const colorParts = chapter.color.split(' ');
                        if (colorParts.length >= 3) {
                          const fromColor = colorParts[0]?.replace('from-', '')?.replace('-500', '') || 'purple';
                          const toColor = colorParts[2]?.replace('to-', '')?.replace('-500', '') || 'pink';
                          return `linear-gradient(135deg, ${fromColor}, ${toColor})`;
                        }
                        return 'linear-gradient(135deg, #a855f7, #ec4899)';
                      })() : 'linear-gradient(135deg, #a855f7, #ec4899)'
                    }}>
                      <Icon className="w-7 h-7 text-white" />
                    </div>
                    <span className={styles.chapterNumber}>
                      {chapter.id}
                    </span>
                  </div>

                  <h3 className={styles.chapterTitle}>
                    {chapter.title}
                  </h3>

                  <p className={styles.chapterDescription}>
                    {chapter.description}
                  </p>

                  {/* Topics */}
                  <div className={`${styles.chapterTopics} ${isHovered ? styles.expanded : styles.hidden}`}>
                    {chapter.topics.map((topic, idx) => (
                      <div key={idx} className={styles.topicItem}>
                        <div className={styles.topicDot}></div>
                        <span>{topic}</span>
                      </div>
                    ))}
                  </div>

                  <button className={styles.readButton} onClick={() => handleChapterReadClick(chapter.id)}>
                    Read Chapter
                    <ChevronRight className="w-4 h-4" />
                  </button>
                </div>
              </div>
            );
          })}
        </div>
      </div>

      {/* CTA Section */}
      <div className={styles.ctaSection}>
        <div className={styles.ctaCard}>
          <h2 className={styles.ctaTitle}>
            Ready to Build the Future?
          </h2>
          <p className={styles.ctaSubtitle}>
            Start your journey into Physical AI and Humanoid Robotics today.
            Everything you need to know, from fundamentals to capstone project.
          </p>
          <div className={styles.ctaButtons}>
            <button className={styles.ctaPrimaryButton} onClick={handleCtaPrimaryClick}>
              Start Learning Now
            </button>
            <button className={styles.ctaSecondaryButton} onClick={handleCtaSecondaryClick}>
              View on GitHub
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default PhysicalAILanding;