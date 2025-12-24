import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from '../index.module.css';

function Footer() {
  return (
    <footer className={clsx('footer', styles.footer)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <div className={styles.footerContent}>
              <div className={styles.footerLinks}>
                <Link to="https://github.com" className={styles.footerLink}>
                  GitHub
                </Link>
              </div>
            </div>
          </div>
        </div>
      </div>
    </footer>
  );
}

export default Footer;