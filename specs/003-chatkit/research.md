# Research Findings: ChatKit

**Feature**: ChatKit
**Date**: 2025-12-12
**Phase**: Phase 0 Research & Discovery

This document consolidates research findings for 10 technical decisions required for ChatKit implementation.

---

## RT-001: React Component Composition Patterns

**Question**: What is the best practice for composing a multi-component chatbot UI (input, history, error states) in React?

### Decision

Use **hooks-based composition** with a container component pattern.

### Rationale

1. **Modern React Paradigm**: React 17+ favors hooks over class-based components and traditional container/presentational split
2. **State Colocation**: Hooks allow state to live close to where it's used, reducing prop drilling
3. **Reusability**: Custom hooks (e.g., `useChatbot`) encapsulate complex logic and can be reused
4. **Performance**: `useMemo` and `useCallback` optimize re-renders in complex UIs

### Implementation Pattern

```typescript
// Container component: ChatbotWidget/index.tsx
export function ChatbotWidget(props: ChatbotProps) {
  const { messages, loading, error, sendQuery, retryLast } = useChatbot(props);

  return (
    <div className={styles.container}>
      <ChatHistory messages={messages} />
      {loading && <LoadingIndicator />}
      {error && <ErrorMessage error={error} onRetry={retryLast} />}
      <QueryInput onSubmit={sendQuery} disabled={loading} />
    </div>
  );
}

// Custom hook: hooks/useChatbot.ts
export function useChatbot(props: ChatbotProps) {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendQuery = useCallback(async (question: string) => {
    // API call logic
  }, [props.bookId, props.chapterNumber]);

  return { messages, loading, error, sendQuery, retryLast };
}
```

### Alternatives Considered

- **Container/Presentational Split**: More boilerplate, less co-location of logic
- **Class Components**: Deprecated pattern, not recommended for new code
- **Context API**: Overkill for single-component state (no deep nesting)

### Sources

- React Official Docs: [Hooks](https://react.dev/reference/react/hooks)
- React Official Docs: [Sharing State Between Components](https://react.dev/learn/sharing-state-between-components)

---

## RT-002: Docusaurus MDX Component Import Patterns

**Question**: How do authors import and use custom React components in Docusaurus MDX files?

### Decision

Use Docusaurus's `@site/` alias for absolute imports in MDX files.

### Implementation

```mdx
---
id: chapter-1
title: Chapter 1
---

import { ChatbotWidget } from '@site/src/components/ChatbotWidget';

# Chapter 1 Content

<ChatbotWidget bookId="my-book" chapterNumber={1} />
```

### Key Points

1. **@site/ Alias**: Resolves to project root automatically (no configuration needed)
2. **MDX Native Support**: Docusaurus 2.x supports React components in MDX out of the box
3. **No Config Required**: Works without modifying `docusaurus.config.js`
4. **Import Placement**: Imports must come after frontmatter (YAML between `---`)

### Common Gotchas

- **Path Resolution**: Must use `@site/src/...` not relative paths like `../../src/...`
- **Component Exports**: Component must be exported (named or default)
- **JSX Syntax**: Use JSX syntax `<Component />` not Markdown syntax

### Sources

- Docusaurus Docs: [MDX and React](https://docusaurus.io/docs/markdown-features/react)
- Docusaurus Docs: [Using JSX in Markdown](https://docusaurus.io/docs/markdown-features#using-jsx-in-markdown)

---

## RT-003: CSS Modules Scoping in Docusaurus

**Question**: How do CSS Modules work in Docusaurus to avoid theme conflicts?

### Decision

Use CSS Modules with `*.module.css` naming convention (zero configuration required).

### Implementation

```typescript
// ChatbotWidget/index.tsx
import styles from './styles.module.css';

export function ChatbotWidget() {
  return <div className={styles.container}>...</div>;
}
```

```css
/* ChatbotWidget/styles.module.css */
.container {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 1000;
}

.inputField {
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 4px;
}
```

### Key Points

1. **Automatic Scoping**: Docusaurus webpack config handles CSS Modules automatically
2. **Naming Convention**: Files ending in `.module.css` are treated as CSS Modules
3. **Class Name Hashing**: Classes are hashed (e.g., `.container` becomes `.ChatbotWidget_container__3xYZ`)
4. **No Conflicts**: Scoped classes don't conflict with Docusaurus theme or other components
5. **CSS Variables**: Can use Docusaurus CSS variables (e.g., `--ifm-color-*`)

### No Configuration Needed

Docusaurus has postcss and CSS Modules pre-configured. No changes to `docusaurus.config.js` required.

### Sources

- Docusaurus Docs: [Styling and Layout](https://docusaurus.io/docs/styling-layout)
- CSS Modules Spec: [github.com/css-modules/css-modules](https://github.com/css-modules/css-modules)

---

## RT-004: Accessibility Best Practices for Chat Interfaces

**Question**: What ARIA roles, labels, and keyboard navigation patterns are required for WCAG 2.1 AA compliance?

### Decision

Implement ARIA live regions, semantic HTML, keyboard navigation, and WCAG 2.1 AA color contrast.

### Required ARIA Attributes

```typescript
// QueryInput component
<form role="search" aria-label="Ask a question">
  <input
    type="text"
    aria-label="Question input"
    aria-describedby="char-counter"
    aria-required="true"
    maxLength={500}
  />
  <button type="submit" aria-label="Submit question">
    Send
  </button>
  <span id="char-counter" aria-live="polite">
    {charactersRemaining} characters remaining
  </span>
</form>

// ChatHistory component
<div role="log" aria-live="polite" aria-atomic="false">
  {messages.map(msg => (
    <div
      key={msg.id}
      role={msg.type === 'user' ? 'article' : 'article'}
      aria-label={`${msg.type} message`}
    >
      {msg.content}
    </div>
  ))}
</div>

// ErrorMessage component
<div role="alert" aria-live="assertive">
  {errorMessage}
</div>
```

### Keyboard Navigation

| Key | Action |
|-----|--------|
| Tab | Navigate between input, submit button, retry button |
| Enter | Submit query (when focused on input or button) |
| Escape | Close error message or clear input |

### Color Contrast (WCAG 2.1 AA)

- **Normal Text**: Minimum 4.5:1 contrast ratio
- **Large Text (18pt+)**: Minimum 3:1 contrast ratio
- **UI Components**: Minimum 3:1 contrast ratio

### Screen Reader Announcements

- New messages: Use `aria-live="polite"` on chat history
- Errors: Use `aria-live="assertive"` on error messages
- Loading states: Use `aria-busy="true"` on container

### Sources

- WCAG 2.1 Quick Reference: [w3.org/WAI/WCAG21/quickref](https://www.w3.org/WAI/WCAG21/quickref/)
- W3C ARIA Authoring Practices: [Feed Pattern](https://www.w3.org/WAI/ARIA/apg/patterns/feed/) (similar to chat)
- WebAIM: [ARIA Live Regions](https://webaim.org/techniques/aria/#liveregions)

---

## RT-005: Client-Side Caching Strategies for React

**Question**: What is the best approach for implementing 5-minute TTL cache for API responses in React?

### Decision

Use a **custom hook with in-memory cache** (no external library needed).

### Rationale

1. **Simple Requirements**: ChatKit only needs 5-minute TTL, no complex invalidation
2. **Bundle Size**: Adding react-query (~13KB) or SWR (~4KB) is unnecessary
3. **Full Control**: Custom implementation is ~50 lines of code
4. **No Dependencies**: Leverages built-in React hooks

### Implementation

```typescript
// hooks/useCachedQuery.ts
interface CacheEntry<T> {
  data: T;
  timestamp: number;
}

const cache = new Map<string, CacheEntry<any>>();
const TTL = 5 * 60 * 1000; // 5 minutes

export function useCachedQuery<T>(
  key: string,
  fetchFn: () => Promise<T>
): { data: T | null; loading: boolean; error: string | null } {
  const [data, setData] = useState<T | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const cachedEntry = cache.get(key);
    const now = Date.now();

    if (cachedEntry && now - cachedEntry.timestamp < TTL) {
      setData(cachedEntry.data);
      return;
    }

    setLoading(true);
    fetchFn()
      .then(result => {
        cache.set(key, { data: result, timestamp: now });
        setData(result);
        setError(null);
      })
      .catch(err => setError(err.message))
      .finally(() => setLoading(false));
  }, [key]);

  return { data, loading, error };
}
```

### Alternatives Considered

- **react-query**: Powerful but overkill (~13KB, complex API)
- **SWR**: Lightweight (~4KB) but still adds dependency
- **localStorage**: Persistence not needed (session-scoped is fine)

### Sources

- React Docs: [useEffect Hook](https://react.dev/reference/react/useEffect)
- React Docs: [useState Hook](https://react.dev/reference/react/useState)

---

## RT-006: LocalStorage for User ID Persistence

**Question**: What are the best practices for generating and storing persistent user IDs in localStorage?

### Decision

Use `crypto.randomUUID()` with localStorage, fallback to sessionStorage if unavailable.

### Implementation

```typescript
// hooks/useUserId.ts
export function useUserId(): string {
  const [userId, setUserId] = useState<string>('');

  useEffect(() => {
    const STORAGE_KEY = 'chatbot_user_id';

    try {
      // Try localStorage first
      let id = localStorage.getItem(STORAGE_KEY);

      if (!id) {
        // Generate new UUID
        id = crypto.randomUUID();
        localStorage.setItem(STORAGE_KEY, id);
      }

      setUserId(id);
    } catch (error) {
      // Fallback to sessionStorage (Safari private mode)
      let id = sessionStorage.getItem(STORAGE_KEY);

      if (!id) {
        id = crypto.randomUUID();
        sessionStorage.setItem(STORAGE_KEY, id);
      }

      setUserId(id);
    }
  }, []);

  return userId;
}
```

### Key Points

1. **UUID Generation**: `crypto.randomUUID()` is standard (supported in all modern browsers)
2. **Privacy**: UUIDs are pseudonymous (not PII) - GDPR compliant for analytics
3. **Fallback**: sessionStorage for Safari private mode or localStorage quota exceeded
4. **Error Handling**: Try-catch for quota errors and disabled storage

### Browser Support

- `crypto.randomUUID()`: Chrome 92+, Firefox 95+, Safari 15.4+, Edge 92+
- All target browsers support it (as of 2025)

### Sources

- MDN: [crypto.randomUUID()](https://developer.mozilla.org/en-US/docs/Web/API/Crypto/randomUUID)
- MDN: [localStorage](https://developer.mozilla.org/en-US/docs/Web/API/Window/localStorage)
- GDPR Guidelines: Pseudonymous identifiers (not personal data)

---

## RT-007: TypeScript Prop Validation for React Components

**Question**: How do we validate prop types at runtime in TypeScript React components?

### Decision

Use **TypeScript interfaces only** (no runtime validation for ChatKit).

### Rationale

1. **Compile-Time Safety**: TypeScript catches type errors at build time
2. **Zero Runtime Overhead**: No additional bundle size or performance cost
3. **Simple Use Case**: ChatKit props are simple (strings, numbers, enums)
4. **MDX Author Experience**: TypeScript errors show in editor (VS Code)

### Implementation

```typescript
// types/chatbot.ts
export interface ChatbotProps {
  bookId: string;
  chapterNumber: number;
  theme?: 'light' | 'dark';
  position?: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
  welcomeMessage?: string;
}

// ChatbotWidget/index.tsx
export function ChatbotWidget({
  bookId,
  chapterNumber,
  theme = 'light',
  position = 'bottom-right',
  welcomeMessage = 'Ask me anything about this chapter!',
}: ChatbotProps): JSX.Element {
  // TypeScript enforces types at compile time
  // No runtime validation needed
}
```

### Runtime Warnings (Optional)

If runtime warnings are needed (FR-023), use console.warn in development:

```typescript
if (process.env.NODE_ENV === 'development') {
  if (typeof bookId !== 'string') {
    console.warn('ChatbotWidget: bookId must be a string');
  }
  if (theme && !['light', 'dark'].includes(theme)) {
    console.warn(`ChatbotWidget: Invalid theme "${theme}". Use "light" or "dark".`);
  }
}
```

### Alternatives Considered

- **PropTypes**: Deprecated, adds runtime overhead
- **Zod/Yup**: Overkill for simple props, adds ~10-30KB to bundle
- **io-ts**: Functional but complex API, adds ~5KB

### Sources

- TypeScript Docs: [TypeScript for React](https://www.typescriptlang.org/docs/handbook/react.html)
- React Docs: [TypeScript](https://react.dev/learn/typescript)

---

## RT-008: Lazy Loading React Components in Docusaurus

**Question**: How do we implement lazy loading for ChatbotWidget to avoid blocking page load?

### Decision

Use **React.lazy()** with **Suspense** for lazy loading.

### Implementation

**Option 1: Lazy load in MDX (recommended)**

```mdx
import { lazy, Suspense } from 'react';

export const ChatbotWidget = lazy(() => import('@site/src/components/ChatbotWidget'));

<Suspense fallback={<div>Loading chatbot...</div>}>
  <ChatbotWidget bookId="my-book" chapterNumber={1} />
</Suspense>
```

**Option 2: Pre-configured lazy wrapper**

```typescript
// src/components/ChatbotWidget/Lazy.tsx
import { lazy, Suspense } from 'react';

const ChatbotWidgetLazy = lazy(() => import('./index'));

export function ChatbotWidget(props: ChatbotProps) {
  return (
    <Suspense fallback={<div className="loading">Loading chatbot...</div>}>
      <ChatbotWidgetLazy {...props} />
    </Suspense>
  );
}
```

Then in MDX:
```mdx
import { ChatbotWidget } from '@site/src/components/ChatbotWidget/Lazy';

<ChatbotWidget bookId="my-book" chapterNumber={1} />
```

### Key Points

1. **Automatic Code Splitting**: Webpack splits lazy components into separate bundles
2. **On-Demand Loading**: Component JS downloaded only when needed
3. **Fallback UI**: Suspense fallback shows while loading
4. **No Config Needed**: Docusaurus webpack handles code splitting automatically

### Performance Impact

- Initial page load: Faster (ChatbotWidget not in main bundle)
- Time to interactive: Same (component loads in background)
- Lazy load time: ~100-200ms for small component

### Sources

- React Docs: [Code-Splitting with lazy()](https://react.dev/reference/react/lazy)
- React Docs: [Suspense](https://react.dev/reference/react/Suspense)
- Docusaurus Docs: [Code Splitting](https://docusaurus.io/docs/advanced/architecture#code-splitting)

---

## RT-009: Markdown Rendering in React

**Question**: What library should we use to render markdown in chat responses?

### Decision

Use **react-markdown** library.

### Rationale

1. **React Native**: Built for React, uses React components (not dangerouslySetInnerHTML)
2. **Security**: XSS-safe by default (no HTML rendering unless explicitly enabled)
3. **Customizable**: Supports custom components for code blocks, links, etc.
4. **Lightweight**: ~7KB gzipped (acceptable for ChatKit)
5. **Maintained**: Active development, 14M+ weekly downloads

### Implementation

```typescript
import ReactMarkdown from 'react-markdown';

export function ResponseDisplay({ content }: { content: string }) {
  return (
    <div className={styles.response}>
      <ReactMarkdown>{content}</ReactMarkdown>
    </div>
  );
}
```

### Comparison

| Library | Bundle Size | Security | Popularity |
|---------|-------------|----------|------------|
| react-markdown | ~7KB | XSS-safe | 14M+/week |
| marked | ~5KB | Requires sanitization | 9M+/week |
| showdown | ~8KB | Requires sanitization | 400K+/week |

### Security Note

- **react-markdown**: Safe by default (renders to React elements)
- **marked/showdown**: Return HTML strings → need DOMPurify for XSS prevention

### Sources

- npm: [react-markdown](https://www.npmjs.com/package/react-markdown)
- GitHub: [remarkjs/react-markdown](https://github.com/remarkjs/react-markdown)

---

## RT-010: Testing React Components with Accessibility Checks

**Question**: How do we integrate axe-core into Jest/React Testing Library for automated accessibility testing?

### Decision

Use **jest-axe** with React Testing Library.

### Implementation

**Installation:**

```bash
npm install --save-dev jest-axe
```

**Setup (jest.config.js):**

```javascript
module.exports = {
  setupFilesAfterEnv: ['<rootDir>/jest.setup.js'],
};
```

**jest.setup.js:**

```javascript
import { toHaveNoViolations } from 'jest-axe';

expect.extend(toHaveNoViolations);
```

**Test Example:**

```typescript
// tests/accessibility/chatbot.a11y.test.tsx
import { render } from '@testing-library/react';
import { axe, toHaveNoViolations } from 'jest-axe';
import { ChatbotWidget } from '@/components/ChatbotWidget';

expect.extend(toHaveNoViolations);

describe('ChatbotWidget Accessibility', () => {
  it('should have no WCAG 2.1 AA violations', async () => {
    const { container } = render(
      <ChatbotWidget bookId="test" chapterNumber={1} />
    );

    const results = await axe(container, {
      rules: {
        // WCAG 2.1 AA rules
        'color-contrast': { enabled: true },
        'label': { enabled: true },
        'aria-required-attr': { enabled: true },
      },
    });

    expect(results).toHaveNoViolations();
  });

  it('should have proper ARIA labels on input', () => {
    const { getByLabelText } = render(
      <ChatbotWidget bookId="test" chapterNumber={1} />
    );

    expect(getByLabelText('Question input')).toBeInTheDocument();
  });
});
```

### CI/CD Integration

```yaml
# .github/workflows/test.yml
- name: Run accessibility tests
  run: npm test -- --testPathPattern=a11y
```

### Key Points

1. **Automated WCAG Checks**: jest-axe runs 50+ WCAG 2.1 rules automatically
2. **Custom Rules**: Can enable/disable specific rules
3. **Fast**: Runs in milliseconds during test suite
4. **Actionable Errors**: Violations include fix suggestions

### Sources

- GitHub: [jest-axe](https://github.com/nickcolley/jest-axe)
- Deque: [axe-core Documentation](https://github.com/dequelabs/axe-core)
- Testing Library: [Accessibility](https://testing-library.com/docs/queries/about/#accessibility)

---

## Summary of Decisions

| Research Task | Decision | Rationale |
|---------------|----------|-----------|
| RT-001 | Hooks-based composition | Modern, performant, collocates logic |
| RT-002 | `@site/` alias imports | Zero config, Docusaurus standard |
| RT-003 | CSS Modules (`*.module.css`) | Automatic scoping, no conflicts |
| RT-004 | ARIA live regions + roles | WCAG 2.1 AA compliance |
| RT-005 | Custom hook caching | Simple, no external deps |
| RT-006 | `crypto.randomUUID()` + localStorage | Standard, privacy-compliant |
| RT-007 | TypeScript interfaces only | Compile-time safety, zero overhead |
| RT-008 | React.lazy() + Suspense | Automatic code splitting |
| RT-009 | react-markdown | React-native, XSS-safe |
| RT-010 | jest-axe + React Testing Library | Automated WCAG checks |

---

## Open Questions Resolved

1. **Markdown formatting in user questions**: NOT supported (backend handles plain text)
2. **Anonymous users vs. authentication**: Support anonymous with localStorage UUID
3. **Chat state persistence across navigations**: NO (session-scoped only)
4. **Maximum concurrent users**: 100 (per success criteria)
5. **Clear chat history button**: YES (simple utility feature)

---

## Next Steps

1. Create `data-model.md` with detailed entity schemas
2. Generate TypeScript contract definitions in `contracts/chatbot-api.ts`
3. Write `quickstart.md` with setup and usage instructions
4. Update agent context with new technologies
5. Proceed to `/sp.tasks` for task generation
