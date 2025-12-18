/**
 * Jest setup file
 * Runs before each test suite
 */

// Extend Jest matchers with jest-axe
import { toHaveNoViolations } from 'jest-axe';

expect.extend(toHaveNoViolations);

// Mock crypto.randomUUID if not available (Node < 19)
if (typeof crypto === 'undefined' || !crypto.randomUUID) {
  global.crypto = {
    ...global.crypto,
    randomUUID: () => {
      return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
        const r = (Math.random() * 16) | 0;
        const v = c === 'x' ? r : (r & 0x3) | 0x8;
        return v.toString(16);
      });
    },
  };
}

// Mock localStorage
const localStorageMock = {
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
};

global.localStorage = localStorageMock;

// Mock sessionStorage
const sessionStorageMock = {
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
};

global.sessionStorage = sessionStorageMock;

// Clean up after each test
afterEach(() => {
  jest.clearAllMocks();
  localStorageMock.clear();
  sessionStorageMock.clear();
});
