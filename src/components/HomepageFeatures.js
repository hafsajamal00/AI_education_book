import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Easy to Learn',
    Svg: require('../../static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Our step-by-step approach makes learning ROS2 easy and accessible for everyone,
        from beginners to experienced developers.
      </>
    ),
  },
  {
    title: 'Practical Examples',
    Svg: require('../../static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Learn with hands-on examples and projects that you can run on real robots or in simulation.
      </>
    ),
  },
  {
    title: 'Comprehensive Coverage',
    Svg: require('../../static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        From basic concepts to advanced topics, our tutorials cover everything you need to know about ROS2.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}