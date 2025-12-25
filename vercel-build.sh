#!/bin/bash
set -e # Exit immediately if a command exits with a non-zero status

echo "Installing dependencies..."
npm install

echo "Building Docusaurus site..."
npm run build

echo "Build completed successfully!"