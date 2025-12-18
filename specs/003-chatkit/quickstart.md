# Quick Start Guide: ChatKit

**Feature**: ChatKit
**Audience**: Authors (component users) and Developers (component implementers)
**Date**: 2025-12-12

This guide provides quick setup and usage instructions for the ChatKit component library.

---

## For Authors (Using ChatbotWidget)

### Prerequisites

- Docusaurus 2.x site already set up
- Backend RAG API running (feature 002-rag-chatbot-education)
- Node.js 16+ and npm installed

### Basic Usage

**Step 1: Import the Component**

Add this import statement to your MDX file (after the frontmatter):

```mdx
---
id: chapter-1-introduction
title: Chapter 1: Introduction
---

import { ChatbotWidget } from '@site/src/components/ChatbotWidget';

# Chapter 1: Introduction

Your chapter content here...

<ChatbotWidget bookId="physical-ai-robotics" chapterNumber={1} />
```

**Step 2: Configure Props**

Required props:
- `bookId`: Your book's unique identifier (string)
- `chapterNumber`: The chapter number (number, 1-indexed)

That's it! The chatbot will appear at the bottom-right of your page.

---

### Customization (Optional)

**Change Theme**

```jsx
<ChatbotWidget
  bookId="physical-ai-robotics"
  chapterNumber={1}
  theme="dark"
/>
```

Options: `"light"` (default) or `"dark"`

**Change Position**

```jsx
<ChatbotWidget
  bookId="physical-ai-robotics"
  chapterNumber={1}
  position="top-left"
/>
```

Options:
- `"bottom-right"` (default)
- `"bottom-left"`
- `"top-right"`
- `"top-left"`

**Custom Welcome Message**

```jsx
<ChatbotWidget
  bookId="physical-ai-robotics"
  chapterNumber={1}
  welcomeMessage="Ask me anything about Physical AI and Robotics!"
/>
```

**Full Customization Example**

```jsx
<ChatbotWidget
  bookId="physical-ai-robotics"
  chapterNumber={2}
  theme="dark"
  position="bottom-left"
  welcomeMessage="Welcome! Ask me about ROS2 fundamentals."
/>
```

---

### Lazy Loading (Recommended)

To improve page load performance, wrap the component in React's `Suspense`:

```mdx
import { ChatbotWidget } from '@site/src/components/ChatbotWidget';
import { Suspense } from 'react';

<Suspense fallback={<div>Loading chatbot...</div>}>
  <ChatbotWidget bookId="physical-ai-robotics" chapterNumber={1} />
</Suspense>
```

---

### Troubleshooting

**Problem: Chatbot doesn't appear**

1. Check console for errors
2. Verify import path uses `@site/` alias
3. Ensure import is after frontmatter (YAML `---` block)
4. Confirm backend API is running at `http://localhost:8000` (or configured URL)

**Problem: Queries fail with error**

1. Check backend API is accessible: `curl http://localhost:8000/health`
2. Verify `bookId` matches backend configuration
3. Check browser console for network errors
4. Ensure CORS is enabled on backend

**Problem: Styling conflicts with site theme**

ChatbotWidget uses CSS Modules for scoped styling, so conflicts should be rare. If issues occur:
1. Check for CSS specificity conflicts
2. Use browser DevTools to inspect element styles
3. Report issue with steps to reproduce

---

## For Developers (Implementing ChatKit)

### Prerequisites

- Docusaurus 2.x project
- TypeScript 4.9+
- React 17+
- Backend RAG API from feature 002-rag-chatbot-education

All dependencies are included in Docusaurus by default (React, TypeScript, webpack). No additional installations required except for development/testing.

---

### Development Setup

**1. Install Development Dependencies**

```bash
npm install --save-dev jest @testing-library/react @testing-library/jest-dom jest-axe
npm install --save-dev @types/jest @types/testing-library__react
```

**2. Install Runtime Dependencies**

```bash
npm install axios react-markdown
```

**3. Configure TypeScript**

Ensure `tsconfig.json` includes:

```json
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "esnext",
    "jsx": "react",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "moduleResolution": "node",
    "resolveJsonModule": true,
    "isolatedModules": true
  }
}
```

---

### Project Structure

```
src/
├── components/
│   └── ChatbotWidget/
│       ├── index.tsx              # Main component
│       ├── QueryInput.tsx         # Input field + submit button
│       ├── ResponseDisplay.tsx    # Response rendering
│       ├── ChatHistory.tsx        # Message list
│       ├── ErrorMessage.tsx       # Error display
│       ├── LoadingIndicator.tsx   # Loading state
│       └── styles.module.css      # Scoped CSS
│
├── hooks/
│   ├── useChatbot.ts              # Main chatbot logic
│   ├── useChatHistory.ts          # Chat history management
│   └── useUserId.ts               # User ID persistence
│
├── services/
│   ├── chatbotApi.ts              # HTTP client
│   └── cache.ts                   # Caching service
│
└── types/
    └── chatbot.ts                 # TypeScript interfaces

tests/
├── components/
│   ├── ChatbotWidget.test.tsx    # Component tests
│   └── QueryInput.test.tsx       # Sub-component tests
│
├── hooks/
│   └── useChatbot.test.tsx       # Hook tests
│
└── accessibility/
    └── chatbot.a11y.test.tsx     # WCAG 2.1 AA tests
```

---

### Running Tests

**Unit Tests**

```bash
npm test
```

**Accessibility Tests**

```bash
npm test -- --testPathPattern=a11y
```

**Watch Mode (Development)**

```bash
npm test -- --watch
```

**Coverage Report**

```bash
npm test -- --coverage
```

---

### Building for Production

**Build Docusaurus Site**

```bash
npm run build
```

**Preview Build**

```bash
npm run serve
```

---

### Environment Variables

Create `.env` file in project root:

```env
# Backend API URL (default: http://localhost:8000)
REACT_APP_API_URL=http://localhost:8000

# Enable development logging (default: false)
REACT_APP_DEBUG=true
```

**Usage in Code:**

```typescript
const apiUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```

---

### API Integration

**ChatbotApiService Usage**

```typescript
import { ChatbotApiService } from '@/services/chatbotApi';

const api = new ChatbotApiService();

// Submit query
const response = await api.submitQuery({
  book_id: 'physical-ai-robotics',
  chapter_number: 1,
  question: 'What is physical AI?',
  user_id: '550e8400-e29b-41d4-a716-446655440000',
});

// Submit feedback
await api.submitFeedback({
  response_id: response.response_id,
  rating: 'helpful',
  user_id: '550e8400-e29b-41d4-a716-446655440000',
});
```

---

### Custom Hook Usage

**useChatbot Hook**

```typescript
import { useChatbot } from '@/hooks/useChatbot';

function ChatbotWidget(props: ChatbotProps) {
  const {
    messages,
    inputValue,
    loading,
    error,
    sendQuery,
    retryLast,
    setInputValue,
    clearError,
  } = useChatbot(props);

  return (
    <div>
      <ChatHistory messages={messages} />
      {error && <ErrorMessage error={error} onRetry={retryLast} />}
      <QueryInput
        value={inputValue}
        onChange={setInputValue}
        onSubmit={sendQuery}
        disabled={loading}
      />
    </div>
  );
}
```

---

### CSS Modules

**Creating Scoped Styles**

```css
/* ChatbotWidget/styles.module.css */
.container {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 350px;
  max-height: 500px;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 1000;
}

.input {
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 4px;
  padding: 8px 12px;
  font-size: 14px;
}

.button {
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 4px;
  padding: 8px 16px;
  cursor: pointer;
}
```

**Using in Component**

```typescript
import styles from './styles.module.css';

export function ChatbotWidget() {
  return (
    <div className={styles.container}>
      <input className={styles.input} />
      <button className={styles.button}>Send</button>
    </div>
  );
}
```

---

### Accessibility Testing

**Example Test with jest-axe**

```typescript
import { render } from '@testing-library/react';
import { axe, toHaveNoViolations } from 'jest-axe';
import { ChatbotWidget } from '@/components/ChatbotWidget';

expect.extend(toHaveNoViolations);

test('ChatbotWidget has no accessibility violations', async () => {
  const { container } = render(
    <ChatbotWidget bookId="test" chapterNumber={1} />
  );

  const results = await axe(container);
  expect(results).toHaveNoViolations();
});
```

---

### Performance Optimization

**1. Lazy Loading**

```typescript
import { lazy, Suspense } from 'react';

const ChatbotWidget = lazy(() => import('./ChatbotWidget'));

export function LazyChat(props) {
  return (
    <Suspense fallback={<div>Loading...</div>}>
      <ChatbotWidget {...props} />
    </Suspense>
  );
}
```

**2. Memoization**

```typescript
import { useMemo, useCallback } from 'react';

function ChatbotWidget(props) {
  const messages = useMemo(() => {
    return filterMessages(allMessages);
  }, [allMessages]);

  const handleSubmit = useCallback((query: string) => {
    submitQuery(query);
  }, [submitQuery]);

  return <ChatHistory messages={messages} onSubmit={handleSubmit} />;
}
```

**3. Caching**

```typescript
// Automatic 5-minute TTL cache
const cachedData = useCachedQuery(cacheKey, fetchFunction);
```

---

### Debugging

**Enable Debug Logging**

Set `REACT_APP_DEBUG=true` in `.env`:

```typescript
if (process.env.REACT_APP_DEBUG === 'true') {
  console.log('[ChatKit]', message, data);
}
```

**Common Issues**

1. **CORS Errors**: Ensure backend allows `http://localhost:3000`
2. **404 on API Calls**: Check `REACT_APP_API_URL` environment variable
3. **Rendering Issues**: Inspect CSS Modules class name generation
4. **State Not Updating**: Check React hooks dependencies arrays

---

### Deployment Checklist

- [ ] All tests passing (`npm test`)
- [ ] Accessibility tests passing (`npm test -- --testPathPattern=a11y`)
- [ ] TypeScript compiles without errors (`npm run build`)
- [ ] Environment variables configured for production
- [ ] Backend API URL updated in production `.env`
- [ ] Bundle size under 50KB gzipped (check with `npm run build`)
- [ ] Browser testing complete (Chrome, Firefox, Safari, Edge)
- [ ] Mobile responsive testing complete (320px to 1920px viewports)

---

## Quick Reference

### Props

| Prop | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| bookId | string | Yes | - | Book identifier |
| chapterNumber | number | Yes | - | Chapter number (1-indexed) |
| theme | 'light' \| 'dark' | No | 'light' | Visual theme |
| position | Position | No | 'bottom-right' | Widget position |
| welcomeMessage | string | No | "Ask me anything..." | Welcome message |

### API Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/v1/query` | POST | Submit question, get answer |
| `/v1/feedback` | POST | Submit feedback on response |
| `/health` | GET | Health check |

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| Tab | Navigate between elements |
| Enter | Submit query |
| Escape | Close error message |

---

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Follow tasks.md for step-by-step implementation
3. Run tests after each task or task group
4. Create PR when User Story 1 (P1) is complete

---

## Support

For issues or questions:
1. Check troubleshooting section above
2. Review `specs/003-chatkit/plan.md` for architecture decisions
3. Review `specs/003-chatkit/research.md` for technical rationale
4. Create GitHub issue with reproduction steps
