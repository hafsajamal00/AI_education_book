import React from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './Card.module.css';

/**
 * A reusable card component with Docusaurus Link for navigation
 * @param {Object} props - Component properties
 * @param {string} props.title - Card title
 * @param {string} props.description - Card description
 * @param {string} props.to - Link destination
 * @param {string} props.icon - Optional icon component or class name
 * @param {string} props.className - Additional CSS classes
 * @param {Object} props.style - Additional inline styles
 * @returns {JSX.Element} Card component
 */
function Card({ title, description, to, icon, className, style }) {
  return (
    <div className={clsx(styles.card, className)} style={style}>
      <Link to={to} className={styles.cardLink}>
        <div className={styles.cardContent}>
          {icon && <div className={styles.cardIcon}>{icon}</div>}
          <h3 className={styles.cardTitle}>{title}</h3>
          <p className={styles.cardDescription}>{description}</p>
        </div>
      </Link>
    </div>
  );
}

export default Card;