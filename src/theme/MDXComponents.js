/**
 * Custom MDX components for Docusaurus
 * Register ChatbotWidget as global component
 */

import React from 'react';
// Import the Docusaurus MDXComponents
import MDXComponents from '@theme-original/MDXComponents';
// Import our ChatbotWidget
import ChatbotWidget from '../components/ChatbotWidget';

export default {
  // Re-use the default mapping
  ...MDXComponents,
  // Make ChatbotWidget available in all MDX files
  ChatbotWidget,
};
