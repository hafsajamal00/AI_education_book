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
 * @param {React.ReactNode} props.icon - Optional icon component or JSX element
 * @param {string} props.className - Additional CSS classes
 * @param {Object} props.style - Additional inline styles
 * @param {string} props.ariaLabel - Accessibility label for the card link
 * @param {number} props.progress - Progress percentage (0-100)
 * @param {string} props.status - Card status: 'completed', 'in-progress', 'locked', or null
 * @param {string} props.image - Optional image URL
 * @returns {JSX.Element} Card component
 */
function Card({
  title,
  description,
  to,
  icon,
  className,
  style,
  ariaLabel,
  progress,
  status,
  image
}) {
  // Determine status classes and text
  const statusClass = status ? styles[`status-${status}`] : '';
  const statusText = status === 'completed' ? 'Completed' :
                    status === 'in-progress' ? 'In Progress' :
                    status === 'locked' ? 'Locked' : null;

  return (
    <Link
      to={to}
      className={clsx(styles.card, statusClass, className)}
      style={style}
      aria-label={ariaLabel || `${title} - ${description}`}
    >
      <div className={styles.cardContent}>
        {/* Card header with icon/image */}
        <div className={styles.cardHeader}>
          {image ? (
            <img src={image} alt={title} className={styles.cardImage} />
          ) : icon ? (
            <div className={styles.cardIcon} aria-hidden="true">{icon}</div>
          ) : null}
        </div>

        {/* Card body */}
        <div className={styles.cardBody}>
          <h3 className={styles.cardTitle}>{title}</h3>
          <p className={styles.cardDescription}>{description}</p>
        </div>

        {/* Progress section if progress is provided */}
        {progress !== undefined && (
          <div className={styles.progressSection}>
            <div className={styles.progressBarContainer}>
              <div
                className={styles.progressBar}
                style={{ width: `${progress}%` }}
                role="progressbar"
                aria-valuenow={progress}
                aria-valuemin="0"
                aria-valuemax="100"
                aria-label={`Progress: ${progress}%`}
              />
            </div>
            <div className={styles.progressText}>{progress}%</div>
          </div>
        )}

        {/* Status badge if status is provided */}
        {statusText && (
          <div className={styles.statusBadge}>
            <span>{statusText}</span>
          </div>
        )}
      </div>
    </Link>
  );
}

export default Card;