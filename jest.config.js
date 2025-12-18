/**
 * Jest configuration for ChatKit testing
 * @type {import('jest').Config}
 */

module.exports = {
  // Test environment
  testEnvironment: 'jsdom',

  // Setup files
  setupFilesAfterEnv: ['<rootDir>/jest.setup.js'],

  // Module paths
  moduleNameMapper: {
    // Handle CSS modules
    '\\.(css|less|scss|sass)$': 'identity-obj-proxy',

    // Handle @site alias (Docusaurus convention)
    '^@site/(.*)$': '<rootDir>/$1',

    // Handle src paths
    '^@/(.*)$': '<rootDir>/src/$1',
  },

  // Transform files
  transform: {
    '^.+\\.(ts|tsx|js|jsx)$': ['babel-jest', {
      presets: [
        '@babel/preset-env',
        '@babel/preset-react',
        '@babel/preset-typescript',
      ],
    }],
  },

  // File extensions
  moduleFileExtensions: ['ts', 'tsx', 'js', 'jsx', 'json'],

  // Test match patterns
  testMatch: [
    '<rootDir>/tests/**/*.test.{ts,tsx,js,jsx}',
    '<rootDir>/src/**/*.test.{ts,tsx,js,jsx}',
  ],

  // Coverage configuration
  collectCoverageFrom: [
    'src/**/*.{ts,tsx,js,jsx}',
    '!src/**/*.d.ts',
    '!src/**/*.test.{ts,tsx,js,jsx}',
    '!src/types/**/*',
  ],

  // Coverage thresholds (optional)
  coverageThresholds: {
    global: {
      branches: 70,
      functions: 70,
      lines: 70,
      statements: 70,
    },
  },

  // Test timeout
  testTimeout: 10000,

  // Clear mocks between tests
  clearMocks: true,
  restoreMocks: true,
};
