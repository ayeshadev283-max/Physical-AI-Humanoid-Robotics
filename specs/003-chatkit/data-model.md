# Data Model: ChatKit

**Feature**: ChatKit
**Date**: 2025-12-12
**Phase**: Phase 1 Design & Contracts

This document defines all data entities, their schemas, relationships, and validation rules for the ChatKit feature.

---

## Entity Overview

ChatKit operates entirely on the client-side (frontend) with no new backend entities. It reuses the existing backend API from feature 002-rag-chatbot-education. The following entities represent client-side state and TypeScript interfaces.

| Entity | Type | Purpose | Validation |
|--------|------|---------|------------|
| ChatbotProps | Interface | Component props | TypeScript compile-time |
| ChatMessage | Interface | Single message in chat history | Runtime (constructor) |
| ChatbotState | Type | Component state shape | TypeScript compile-time |
| QueryRequest | Interface | API request payload | Backend validation |
| QueryResponse | Interface | API response payload | Backend validation |
| FeedbackRequest | Interface | Feedback API payload | Backend validation |

---

## Entity Schemas

### 1. ChatbotProps

**Purpose**: Props interface for the main ChatbotWidget component

**Schema**:

```typescript
interface ChatbotProps {
  // Required props
  bookId: string;          // Unique identifier for the book
  chapterNumber: number;   // Chapter number (1-indexed)

  // Optional customization props (User Story 2)
  theme?: 'light' | 'dark';
  position?: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
  welcomeMessage?: string;

  // Optional analytics props (User Story 3)
  onQuerySubmit?: (query: string) => void;
  onError?: (error: Error) => void;
}
```

**Defaults**:

```typescript
const defaultProps = {
  theme: 'light',
  position: 'bottom-right',
  welcomeMessage: 'Ask me anything about this chapter!',
};
```

**Validation**:

- `bookId`: Non-empty string (TypeScript enforces)
- `chapterNumber`: Positive integer (TypeScript enforces)
- `theme`: Literal type ('light' | 'dark')
- `position`: Literal type (4 values)
- `welcomeMessage`: Any string or undefined

**Usage**:

```tsx
<ChatbotWidget
  bookId="physical-ai-robotics"
  chapterNumber={1}
  theme="dark"
  position="bottom-right"
/>
```

---

### 2. ChatMessage

**Purpose**: Represents a single message in the chat history (user question or assistant answer)

**Schema**:

```typescript
interface ChatMessage {
  id: string;                  // Unique message ID (UUID)
  type: 'user' | 'assistant';  // Message sender
  content: string;             // Message text (supports markdown for assistant)
  timestamp: Date;             // When message was created
  sources?: Source[];          // Citations (assistant messages only)
  error?: string;              // Error message (if request failed)
  responseId?: string;         // Backend response ID (for feedback)
}
```

**Field Details**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | UUID generated with `crypto.randomUUID()` |
| type | 'user' \| 'assistant' | Yes | Distinguishes user questions from AI answers |
| content | string | Yes | Message text; markdown-rendered for assistant messages |
| timestamp | Date | Yes | JavaScript Date object (for display and sorting) |
| sources | Source[] | No | Only present in assistant messages with citations |
| error | string | No | Error message if API call failed |
| responseId | string | No | Backend response ID (required for feedback) |

**Relationships**:

- `sources`: Array of `Source` entities (one-to-many)
- `responseId`: Links to backend `query_responses` table (foreign key concept)

**Example (User Message)**:

```typescript
{
  id: '550e8400-e29b-41d4-a716-446655440000',
  type: 'user',
  content: 'What is physical AI?',
  timestamp: new Date('2025-12-12T10:30:00Z'),
}
```

**Example (Assistant Message with Sources)**:

```typescript
{
  id: '550e8400-e29b-41d4-a716-446655440001',
  type: 'assistant',
  content: 'Physical AI refers to AI systems that interact with the physical world through embodied agents like robots...',
  timestamp: new Date('2025-12-12T10:30:02Z'),
  sources: [
    {
      chapterNumber: 1,
      sectionTitle: 'Introduction to Physical AI',
      url: '/chapters/module-0-foundations/introduction#physical-ai',
    },
  ],
  responseId: '7c9e6679-7425-40de-944b-e07fc1f90ae7',
}
```

**Example (Error Message)**:

```typescript
{
  id: '550e8400-e29b-41d4-a716-446655440002',
  type: 'assistant',
  content: '',
  timestamp: new Date('2025-12-12T10:30:05Z'),
  error: 'Unable to connect to the server. Please try again.',
}
```

---

### 3. Source

**Purpose**: Represents a citation/source reference in an assistant message

**Schema**:

```typescript
interface Source {
  chapterNumber: number;    // Chapter where source is located
  sectionTitle: string;     // Section title for citation
  pageNumber?: number;      // Optional page number (if applicable)
  url: string;              // Link to book section (for navigation)
}
```

**Field Details**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| chapterNumber | number | Yes | Chapter number (1-indexed) |
| sectionTitle | string | Yes | Human-readable section title |
| pageNumber | number | No | Page number (for PDF/print versions) |
| url | string | Yes | Relative or absolute URL to book section |

**Example**:

```typescript
{
  chapterNumber: 2,
  sectionTitle: 'ROS2 Core Concepts',
  pageNumber: 42,
  url: '/chapters/module-1-ros2/core-concepts#topics-and-nodes',
}
```

**URL Generation**:

URLs should follow Docusaurus routing conventions:

```typescript
function generateSourceUrl(source: Source): string {
  // Example: /chapters/module-1-ros2/core-concepts#section-id
  return `/chapters/chapter-${source.chapterNumber}/${slugify(source.sectionTitle)}`;
}
```

---

### 4. ChatbotState

**Purpose**: Defines the shape of React state in the ChatbotWidget component

**Schema**:

```typescript
interface ChatbotState {
  messages: ChatMessage[];     // Chat history (chronological order)
  inputValue: string;          // Current input field value
  loading: boolean;            // Is API request in progress?
  error: string | null;        // Current error message (if any)
  userId: string;              // Persistent user ID (from localStorage)
}
```

**Field Details**:

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| messages | ChatMessage[] | [] | Chat history (oldest first) |
| inputValue | string | '' | Current text in input field |
| loading | boolean | false | True during API call |
| error | string \| null | null | Error message or null |
| userId | string | (generated) | Persistent UUID from localStorage |

**State Transitions**:

```
Initial State:
{
  messages: [],
  inputValue: '',
  loading: false,
  error: null,
  userId: '550e8400-e29b-41d4-a716-446655440000',
}

User Types Question:
{
  messages: [],
  inputValue: 'What is physical AI?',  // Updated
  loading: false,
  error: null,
  userId: '550e8400-e29b-41d4-a716-446655440000',
}

User Submits Query:
{
  messages: [
    { id: '...', type: 'user', content: 'What is physical AI?', ... }
  ],
  inputValue: '',  // Cleared
  loading: true,   // Set to true
  error: null,
  userId: '550e8400-e29b-41d4-a716-446655440000',
}

API Response Received:
{
  messages: [
    { id: '...', type: 'user', content: 'What is physical AI?', ... },
    { id: '...', type: 'assistant', content: 'Physical AI refers to...', ... },
  ],
  inputValue: '',
  loading: false,  // Set to false
  error: null,
  userId: '550e8400-e29b-41d4-a716-446655440000',
}

API Error:
{
  messages: [
    { id: '...', type: 'user', content: 'What is physical AI?', ... },
  ],
  inputValue: '',
  loading: false,
  error: 'Unable to connect to the server.',  // Set
  userId: '550e8400-e29b-41d4-a716-446655440000',
}
```

**Validation Rules**:

- `messages`: Maximum 50 messages (per FR-036, prevents memory bloat)
- `inputValue`: Maximum 500 characters (per FR-006)
- `userId`: Must be valid UUID v4

---

### 5. QueryRequest

**Purpose**: Payload sent to backend API `/v1/query`

**Schema** (from 002-rag-chatbot-education):

```typescript
interface QueryRequest {
  book_id: string;         // Book identifier
  chapter_number: number;  // Chapter number
  question: string;        // User's question (max 500 chars)
  user_id: string;         // Persistent user UUID
}
```

**Validation** (backend enforced):

- `book_id`: Non-empty string
- `chapter_number`: Positive integer
- `question`: Non-empty string, max 500 characters
- `user_id`: Valid UUID v4

**Example**:

```json
{
  "book_id": "physical-ai-robotics",
  "chapter_number": 1,
  "question": "What is the difference between ROS1 and ROS2?",
  "user_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

---

### 6. QueryResponse

**Purpose**: Response from backend API `/v1/query`

**Schema** (from 002-rag-chatbot-education):

```typescript
interface QueryResponse {
  answer: string;                // AI-generated answer (markdown format)
  sources: BackendSource[];      // Source citations
  confidence: number;            // Confidence score (0-1)
  response_id: string;           // UUID for feedback tracking
}

interface BackendSource {
  chapter_number: number;
  section_title: string;
  page_number?: number;
}
```

**Field Details**:

| Field | Type | Description |
|-------|------|-------------|
| answer | string | Markdown-formatted answer text |
| sources | BackendSource[] | Array of source citations |
| confidence | number | AI confidence score (0.0 to 1.0) |
| response_id | string | UUID for feedback linking |

**Example**:

```json
{
  "answer": "ROS2 improves upon ROS1 by using DDS for communication, supporting real-time systems, and providing better security...",
  "sources": [
    {
      "chapter_number": 2,
      "section_title": "ROS2 Core Concepts",
      "page_number": 45
    }
  ],
  "confidence": 0.92,
  "response_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7"
}
```

**Transformation to ChatMessage**:

```typescript
function transformToMessage(response: QueryResponse): ChatMessage {
  return {
    id: crypto.randomUUID(),
    type: 'assistant',
    content: response.answer,
    timestamp: new Date(),
    sources: response.sources.map(s => ({
      chapterNumber: s.chapter_number,
      sectionTitle: s.section_title,
      pageNumber: s.page_number,
      url: generateSourceUrl(s),
    })),
    responseId: response.response_id,
  };
}
```

---

### 7. FeedbackRequest

**Purpose**: Payload sent to backend API `/v1/feedback` (User Story 3)

**Schema** (from 002-rag-chatbot-education):

```typescript
interface FeedbackRequest {
  response_id: string;            // Links to query_responses table
  rating: 'helpful' | 'not_helpful';
  comment?: string;               // Optional user comment
  user_id: string;                // Persistent user UUID
}
```

**Validation** (backend enforced):

- `response_id`: Valid UUID v4
- `rating`: Must be 'helpful' or 'not_helpful'
- `comment`: Optional string, max 500 characters
- `user_id`: Valid UUID v4

**Example**:

```json
{
  "response_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "rating": "helpful",
  "comment": "Clear and concise explanation!",
  "user_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

---

## Validation Rules Summary

### Client-Side Validation

| Rule | Entity | Field | Enforcement |
|------|--------|-------|-------------|
| Non-empty bookId | ChatbotProps | bookId | TypeScript + runtime check |
| Positive chapterNumber | ChatbotProps | chapterNumber | TypeScript + runtime check |
| Valid theme | ChatbotProps | theme | TypeScript literal type |
| Valid position | ChatbotProps | position | TypeScript literal type |
| Max 500 chars | ChatbotState | inputValue | Input maxLength attribute |
| Max 50 messages | ChatbotState | messages | useChatHistory hook enforcement |
| Valid UUID | ChatMessage | id | crypto.randomUUID() guarantees |
| Valid UUID | ChatbotState | userId | useUserId hook generates valid UUID |

### Backend Validation

All backend validation is handled by feature 002-rag-chatbot-education:

- Request payload validation (Pydantic models)
- Rate limiting (60 queries/user/hour)
- Query length limits (500 characters)
- UUID format validation

---

## State Management

### Component State Hierarchy

```
ChatbotWidget (main component)
├── useChatbot (custom hook)
│   ├── messages: ChatMessage[]
│   ├── loading: boolean
│   ├── error: string | null
│   └── sendQuery: (question: string) => Promise<void>
│
├── useUserId (custom hook)
│   └── userId: string (from localStorage)
│
└── useChatHistory (custom hook)
    ├── messages: ChatMessage[]
    └── addMessage: (message: ChatMessage) => void
```

### Data Flow

```
User Input → QueryInput Component
    ↓
Send Query → useChatbot Hook
    ↓
API Call → ChatbotApiService
    ↓
Backend Response → QueryResponse
    ↓
Transform → ChatMessage
    ↓
Update State → ChatbotState.messages
    ↓
Render → ResponseDisplay Component
```

---

## Persistence

| Data | Storage | Lifetime | Purpose |
|------|---------|----------|---------|
| userId | localStorage | Persistent | Track user across sessions |
| messages | React state | Session | Chat history (cleared on refresh) |
| inputValue | React state | Session | Current input text |
| API cache | Memory (Map) | 5 minutes | Reduce duplicate API calls |

**No Backend Persistence**:

ChatKit does not create new database tables. All query logs and analytics are handled by the existing backend (feature 002-rag-chatbot-education).

---

## TypeScript Type Exports

All types will be exported from a central types file:

```typescript
// src/types/chatbot.ts

export interface ChatbotProps { /* ... */ }
export interface ChatMessage { /* ... */ }
export interface Source { /* ... */ }
export interface ChatbotState { /* ... */ }
export interface QueryRequest { /* ... */ }
export interface QueryResponse { /* ... */ }
export interface FeedbackRequest { /* ... */ }
export interface FeedbackResponse { /* ... */ }

// Utility types
export type MessageType = 'user' | 'assistant';
export type Theme = 'light' | 'dark';
export type Position = 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
export type FeedbackRating = 'helpful' | 'not_helpful';
```

---

## Next Steps

1. Create TypeScript contract file: `contracts/chatbot-api.ts`
2. Generate `quickstart.md` with usage examples
3. Update agent context with TypeScript types
4. Proceed to `/sp.tasks` for task generation
