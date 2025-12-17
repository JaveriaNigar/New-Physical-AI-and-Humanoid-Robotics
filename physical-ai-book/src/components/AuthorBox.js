import React from 'react';
import clsx from 'clsx';
import styles from './AuthorBox.module.css'; // Import the module-specific CSS
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// AuthorBox Component
const AuthorBox = () => {
  const { siteConfig } = useDocusaurusContext();

  // Author details extracted from user-info
  const author = {
    name: 'Javeria Nigar',
    role: 'Student of GIAIC (Governor Initiative for Artificial Intelligence)',
    bio: 'Enthusiastic web developer and AI enthusiast. Skilled in HTML, CSS, Javascript, TypeScript, Next.js, Python, Open AI SDk and AI project documentation. Passionate about building clean, user-friendly interfaces and exploring innovative AI solutions.',
    links: {
      linkedin: 'https://www.linkedin.com/in/javeria-nigar-252b312b5/',
      github: 'https://github.com/JaveriaNigar',
      portfolio: 'https://portfolio-five-alpha-98.vercel.app/',
      tiktok: 'https://www.tiktok.com/@javeria.nigar01?_r=1&_t=ZS-91MmqRu4Juh',
      facebook: 'https://www.facebook.com/share/176miuVQxw/',
      instagram: 'https://www.instagram.com/javerianigar01?igsh=OWZ0bTg0eW1icjc0',
      threads: 'https://www.threads.com/@javerianigar01',
      twitter: 'https://x.com/Nigarhaide526',
      youtube: 'https://www.youtube.com/@javerianigar',
    }
  };

  return (
    <div className={clsx('margin-bottom--lg', styles.authorBox)}>
      <div className={styles.authorInfo}>
        <h3 className={styles.authorName}>{author.name}</h3>
        <p className={styles.authorRole}>{author.role}</p>
        <p className={styles.authorBio}>{author.bio}</p>

        <div className={styles.socialLinks}>
          <h4>Connect with {author.name.split(' ')[0]}:</h4>
          <ul className={styles.linkList}>
            <li><a href={author.links.github} target="_blank" rel="noopener noreferrer" className={styles.link}>GitHub</a></li>
            <li><a href={author.links.portfolio} target="_blank" rel="noopener noreferrer" className={styles.link}>Portfolio</a></li>
            <li><a href={author.links.linkedin} target="_blank" rel="noopener noreferrer" className={styles.link}>LinkedIn</a></li>
            <li><a href={author.links.twitter} target="_blank" rel="noopener noreferrer" className={styles.link}>Twitter</a></li>
          </ul>
        </div>
      </div>
    </div>
  );
};

export default AuthorBox;