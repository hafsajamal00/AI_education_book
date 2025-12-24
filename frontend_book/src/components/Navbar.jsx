import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useThemeConfig} from '@docusaurus/theme-common';
import useBaseUrl from '@docusaurus/useBaseUrl';

function Navbar() {
  const themeConfig = useThemeConfig();
  const {navbar} = themeConfig;

  return (
    <nav
      className={clsx(
        'navbar',
        'navbar--fixed-top',
        'navbar--dark',
        'navbar--primary'
      )}>
      <div className="navbar__inner">
        <div className="navbar__items">
          <Link className="navbar__brand" to={useBaseUrl('/')}>
            <div className="navbar__logo">
              <img
                src={useBaseUrl('/img/logo.svg')}
                alt="Logo"
                height="32"
                width="32"
              />
            </div>
            <strong className="navbar__title">{navbar.title}</strong>
          </Link>
          {navbar.items.map((item, idx) => (
            <NavbarItem {...item} key={idx} />
          ))}
        </div>
        <div className="navbar__items navbar__items--right">
          <NavbarItem
            href="https://github.com/facebook/docusaurus"
            label="GitHub"
            position="right"
          />
        </div>
      </div>
    </nav>
  );
}

function NavbarItem({type, position, href, label, to}) {
  if (type === 'search') {
    return null; // We'll use the default search
  }

  if (position === 'right') {
    return (
      <a
        href={href}
        target="_blank"
        rel="noopener noreferrer"
        className="navbar__item navbar__link"
        aria-label={label}
      >
        {label}
      </a>
    );
  }

  if (href) {
    return (
      <a href={href} className="navbar__item navbar__link">
        {label}
      </a>
    );
  }

  return (
    <Link to={useBaseUrl(to)} className="navbar__item navbar__link">
      {label}
    </Link>
  );
}

export default Navbar;