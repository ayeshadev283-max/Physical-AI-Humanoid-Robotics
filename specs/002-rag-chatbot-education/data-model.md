# Data Model: RAG Chatbot for Educational Resources

**Feature**: RAG Chatbot for Educational Resources
**Date**: 2025-12-11
**Purpose**: Define data entities, schemas, relationships, and validation rules

## Overview

This document specifies the data model for the RAG chatbot system, covering both vector database (Qdrant) and relational database (Postgres) entities. The model supports the complete RAG workflow: content chunking, embedding storage, query processing, response generation, feedback collection, and analytics aggregation.

## Entity Relationship Diagram

```
┌─────────────────────┐
│   BookContentChunk  │───┐
│  (Vector Database)  │   │
└─────────────────────┘   │
                          │
                          │ Referenced by
                          │ (chunk_ids)
                          ↓
┌──────────────┐     ┌──────────────────┐     ┌─────────────────┐
│     Query    │────→│ RetrievedContext │←────│  QueryResponse  │
│  (Postgres)  │     │    (Postgres)    │     │   (Postgres)    │
└──────────────┘     └──────────────────┘     └─────────────────┘
      │                                               │
      │                                               │
      │                                               ↓
      │                                        ┌──────────────┐
      │                                        │  UserFeedback│
      │                                        │  (Postgres)  │
      │                                        └──────────────┘
      ↓
┌──────────────────┐
│ AnalyticsAggregate│
│   (Postgres)     │
└──────────────────┘
```

## Core Entities

### 1. BookContentChunk (Vector Database - Qdrant)

**Purpose**: Stores book content segments with vector embeddings for semantic search

**Schema**:

| Field | Type | Description | Required | Constraints |
|-------|------|-------------|----------|-------------|
| id | UUID | Unique chunk identifier | Yes | Primary key |
| vector | float[] | Embedding vector (1536 dimensions) | Yes | OpenAI text-embedding-3-small output |
| content | string | Original text content of chunk | Yes | 100-1000 words (~500 words avg) |
| metadata | object | Chunk metadata (see below) | Yes | JSON object |

**Metadata Structure**:

```json
{
  "book_id": "string",           // Book identifier (e.g., "physical-ai-robotics")
  "book_version": "string",      // Version/edition (e.g., "v1.0.0", "2025-01-15")
  "chapter_number": "integer",   // Chapter number (1, 2, 3...)
  "chapter_title": "string",     // Chapter title
  "section": "string",           // Section heading (e.g., "2.1 Intelligent Tutoring Systems")
  "subsection": "string|null",   // Subsection heading (if applicable)
  "page_number": "integer|null", // Page number (if applicable for PDF versions)
  "chunk_index": "integer",      // Sequential index within chapter (0, 1, 2...)
  "word_count": "integer",       // Number of words in chunk
  "has_code_block": "boolean",   // True if chunk contains code
  "has_math": "boolean",         // True if chunk contains equations
  "created_at": "string",        // ISO 8601 timestamp
  "source_file": "string"        // Original markdown file path
}
```

**Indexes**:
- Vector index (HNSW algorithm) for similarity search
- Payload index on `book_id`, `chapter_number`, `created_at` for filtering

**Validation Rules**:
- `content` length: 100-1000 words (prevent too-small or too-large chunks)
- `vector` dimensions: exactly 1536 (match OpenAI text-embedding-3-small)
- `book_version` format: semantic versioning (vX.Y.Z) or ISO date (YYYY-MM-DD)
- `chapter_number`: positive integer
- `chunk_index`: non-negative integer, unique within (book_id, chapter_number)

**Relationships**:
- Referenced by `RetrievedContext.chunk_ids` (many-to-many via query retrieval)

**Sample Data**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "vector": [0.023, -0.015, 0.042, ...], // 1536 float values
  "content": "Intelligent Tutoring Systems (ITS) are AI-driven platforms that personalize instruction based on student learning patterns. Unlike traditional educational software, ITS adapts to each student's knowledge level, providing targeted guidance and feedback. Research by VanLehn (2011) demonstrates that ITS can be nearly as effective as human tutoring, achieving learning gains of 0.76 standard deviations compared to traditional classroom instruction...",
  "metadata": {
    "book_id": "physical-ai-robotics",
    "book_version": "v1.0.0",
    "chapter_number": 2,
    "chapter_title": "Key AI Applications in K-12 Education",
    "section": "2.1 Intelligent Tutoring Systems (ITS)",
    "subsection": null,
    "page_number": 15,
    "chunk_index": 0,
    "word_count": 487,
    "has_code_block": false,
    "has_math": false,
    "created_at": "2025-12-11T10:30:00Z",
    "source_file": "docs/chapters/02-key-ai-applications.md"
  }
}
```

---

### 2. Query (Relational Database - Postgres)

**Purpose**: Stores user-submitted questions with metadata

**Schema**:

| Field | Type | Description | Required | Constraints |
|-------|------|-------------|----------|-------------|
| query_id | UUID | Unique query identifier | Yes | Primary key, auto-generated |
| user_id | VARCHAR(64) | Anonymized user identifier | Yes | SHA-256 hash of original user ID |
| query_text | TEXT | User's question | Yes | 1-500 characters |
| query_embedding | VECTOR(1536) | Embedding of query text | Yes | pgvector extension |
| selected_text | TEXT | Text highlighted by user (if any) | No | NULL if no selection |
| book_context | JSONB | Book location context | No | See below |
| timestamp | TIMESTAMP WITH TIME ZONE | Query submission time | Yes | Auto-set to NOW() |
| session_id | UUID | User session identifier | No | Groups queries in same session |
| ip_address_hash | VARCHAR(64) | Hashed IP for abuse detection | No | SHA-256 hash |

**Book Context Structure (JSONB)**:
```json
{
  "book_id": "string",
  "chapter_number": "integer|null",
  "page_url": "string"  // URL of page where query was submitted
}
```

**Indexes**:
- Primary key: `query_id`
- Index on `user_id` for user-specific query history
- Index on `timestamp` for time-based analytics
- Index on `(book_context->>'book_id', timestamp)` for book-specific metrics

**Validation Rules**:
- `query_text`: 1-500 characters (prevent empty or extremely long queries)
- `user_id`: 64-character hex string (SHA-256 output format)
- `query_embedding`: exactly 1536 dimensions
- `timestamp`: cannot be in the future

**Relationships**:
- One-to-one with `RetrievedContext` (via `query_id`)
- One-to-one with `QueryResponse` (via `query_id`)

**Sample Data**:
```sql
INSERT INTO queries (query_id, user_id, query_text, query_embedding, selected_text, book_context, timestamp, session_id, ip_address_hash)
VALUES (
  '7f3d5e2a-1b4c-4f8e-9a3d-6c8b2e4a9f1d',
  'a3f2b1c4d5e6f7g8h9i0j1k2l3m4n5o6p7q8r9s0t1u2v3w4x5y6z7a8b9c0d1e2',  -- Hashed user ID
  'How much do test scores improve with ITS?',
  '[0.015, -0.023, 0.041, ...]',  -- 1536 float values
  'students using ITS achieved 10–15% higher test scores',  -- User highlighted this text
  '{"book_id": "physical-ai-robotics", "chapter_number": 2, "page_url": "/docs/chapters/02-key-ai-applications"}',
  '2025-12-11T14:25:30Z',
  '9f8e7d6c-5b4a-3c2d-1e0f-9g8h7i6j5k4l',
  'b2f3c4d5e6f7g8h9i0j1k2l3m4n5o6p7q8r9s0t1u2v3w4x5y6z7a8b9c0d1e2f3'  -- Hashed IP
);
```

---

### 3. RetrievedContext (Relational Database - Postgres)

**Purpose**: Stores retrieved chunks for each query (for reproducibility and analytics)

**Schema**:

| Field | Type | Description | Required | Constraints |
|-------|------|-------------|----------|-------------|
| context_id | UUID | Unique context record identifier | Yes | Primary key, auto-generated |
| query_id | UUID | Associated query | Yes | Foreign key → queries.query_id |
| chunk_ids | UUID[] | Array of retrieved chunk IDs | Yes | 1-10 UUIDs (typically 5) |
| similarity_scores | FLOAT[] | Cosine similarity scores | Yes | Same length as chunk_ids, values 0-1 |
| retrieval_params | JSONB | Retrieval configuration used | Yes | See below |
| timestamp | TIMESTAMP WITH TIME ZONE | Retrieval execution time | Yes | Auto-set to NOW() |

**Retrieval Params Structure (JSONB)**:
```json
{
  "top_k": 5,
  "similarity_threshold": 0.7,
  "filter": {
    "book_id": "physical-ai-robotics",
    "chapter_number": 2  // Optional: if chapter-scoped retrieval
  },
  "retrieval_strategy": "vector_search"  // or "keyword_fallback" if vector search failed
}
```

**Indexes**:
- Primary key: `context_id`
- Index on `query_id` for join with queries table
- Index on `timestamp` for analytics

**Validation Rules**:
- `chunk_ids` length: 1-10 (prevent empty or excessive retrieval)
- `similarity_scores` length: must match `chunk_ids` length
- `similarity_scores` values: 0.0-1.0 (cosine similarity range)
- `retrieval_params.top_k`: 1-20 (reasonable retrieval limits)

**Relationships**:
- Many-to-one with `Query` (via `query_id`)
- References `BookContentChunk` in Qdrant (via `chunk_ids`)

**Sample Data**:
```sql
INSERT INTO retrieved_contexts (context_id, query_id, chunk_ids, similarity_scores, retrieval_params, timestamp)
VALUES (
  'a1b2c3d4-e5f6-7g8h-9i0j-k1l2m3n4o5p6',
  '7f3d5e2a-1b4c-4f8e-9a3d-6c8b2e4a9f1d',  -- Links to query
  ARRAY[
    '550e8400-e29b-41d4-a716-446655440000',
    '660f9500-f39c-52e5-b827-557766551111',
    '770g0600-g40d-63f6-c938-668877662222',
    '880h1700-h51e-74g7-d049-779988773333',
    '990i2800-i62f-85h8-e150-880099884444'
  ],
  ARRAY[0.89, 0.85, 0.82, 0.78, 0.75],  -- Decreasing similarity scores
  '{"top_k": 5, "similarity_threshold": 0.7, "filter": {"book_id": "physical-ai-robotics", "chapter_number": 2}, "retrieval_strategy": "vector_search"}',
  '2025-12-11T14:25:31Z'
);
```

---

### 4. QueryResponse (Relational Database - Postgres)

**Purpose**: Stores AI-generated responses with metadata

**Schema**:

| Field | Type | Description | Required | Constraints |
|-------|------|-------------|----------|-------------|
| response_id | UUID | Unique response identifier | Yes | Primary key, auto-generated |
| query_id | UUID | Associated query | Yes | Foreign key → queries.query_id, UNIQUE |
| response_text | TEXT | AI-generated answer | Yes | 50-2000 characters |
| source_references | JSONB | Citations/sources in response | Yes | Array of source objects |
| generation_params | JSONB | Model configuration used | Yes | See below |
| latency_ms | INTEGER | Response generation time | Yes | Milliseconds (typical: 500-3000) |
| timestamp | TIMESTAMP WITH TIME ZONE | Response generation time | Yes | Auto-set to NOW() |
| confidence_score | FLOAT | Internal confidence estimate | No | 0.0-1.0 (based on retrieval scores) |

**Source References Structure (JSONB)**:
```json
[
  {
    "chapter": "2",
    "section": "2.1 Intelligent Tutoring Systems (ITS)",
    "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
    "citation": "Chapter 2, Section 2.1"
  },
  ...
]
```

**Generation Params Structure (JSONB)**:
```json
{
  "model": "gpt-4o-mini",
  "temperature": 0.0,
  "max_tokens": 500,
  "system_prompt_version": "v1.0",
  "prompt_token_count": 3250,
  "completion_token_count": 187
}
```

**Indexes**:
- Primary key: `response_id`
- Unique index on `query_id` (one response per query)
- Index on `timestamp` for analytics
- Index on `confidence_score` for low-confidence flagging

**Validation Rules**:
- `response_text`: 50-2000 characters (prevent empty or extremely long responses)
- `latency_ms`: 100-10000 ms (typical range; flag outliers)
- `confidence_score`: 0.0-1.0 if present
- `source_references`: minimum 1 source (ensure grounding)

**Relationships**:
- One-to-one with `Query` (via `query_id`)
- One-to-one with `UserFeedback` (via `response_id`)

**Sample Data**:
```sql
INSERT INTO query_responses (response_id, query_id, response_text, source_references, generation_params, latency_ms, timestamp, confidence_score)
VALUES (
  'b2c3d4e5-f6g7-8h9i-0j1k-l2m3n4o5p6q7',
  '7f3d5e2a-1b4c-4f8e-9a3d-6c8b2e4a9f1d',
  'Students using Intelligent Tutoring Systems (ITS) achieved 10-15% higher test scores according to research by Pane et al. (2017). The study found that ITS provides personalized instruction based on student learning patterns, which significantly reduces the need for one-on-one teacher intervention while improving mastery in subjects like mathematics and science. [Source: Chapter 2, Section 2.1]',
  '[{"chapter": "2", "section": "2.1 Intelligent Tutoring Systems (ITS)", "chunk_id": "550e8400-e29b-41d4-a716-446655440000", "citation": "Chapter 2, Section 2.1"}]',
  '{"model": "gpt-4o-mini", "temperature": 0.0, "max_tokens": 500, "system_prompt_version": "v1.0", "prompt_token_count": 3250, "completion_token_count": 187}',
  1847,  -- 1.847 seconds
  '2025-12-11T14:25:33Z',
  0.85  -- High confidence (based on avg similarity score)
);
```

---

### 5. UserFeedback (Relational Database - Postgres)

**Purpose**: Stores user ratings and optional comments on responses

**Schema**:

| Field | Type | Description | Required | Constraints |
|-------|------|-------------|----------|-------------|
| feedback_id | UUID | Unique feedback identifier | Yes | Primary key, auto-generated |
| response_id | UUID | Associated response | Yes | Foreign key → query_responses.response_id, UNIQUE |
| rating | VARCHAR(20) | User rating | Yes | 'helpful' or 'not_helpful' |
| comment | TEXT | Optional feedback text | No | 0-500 characters |
| timestamp | TIMESTAMP WITH TIME ZONE | Feedback submission time | Yes | Auto-set to NOW() |

**Indexes**:
- Primary key: `feedback_id`
- Unique index on `response_id` (one feedback per response)
- Index on `rating` for aggregation queries
- Index on `timestamp` for trend analysis

**Validation Rules**:
- `rating`: must be 'helpful' or 'not_helpful' (enum constraint)
- `comment`: 0-500 characters if present
- `timestamp` must be >= associated `query_response.timestamp` (cannot rate before response generated)

**Relationships**:
- One-to-one with `QueryResponse` (via `response_id`)

**Sample Data**:
```sql
INSERT INTO user_feedbacks (feedback_id, response_id, rating, comment, timestamp)
VALUES (
  'c3d4e5f6-g7h8-9i0j-1k2l-m3n4o5p6q7r8',
  'b2c3d4e5-f6g7-8h9i-0j1k-l2m3n4o5p6q7',
  'helpful',
  'This answer was very clear and included the exact citation I needed!',
  '2025-12-11T14:26:15Z'
);
```

---

### 6. AnalyticsAggregate (Relational Database - Postgres)

**Purpose**: Stores pre-computed analytics metrics for dashboard performance

**Schema**:

| Field | Type | Description | Required | Constraints |
|-------|------|-------------|----------|-------------|
| aggregate_id | UUID | Unique aggregate identifier | Yes | Primary key, auto-generated |
| metric_name | VARCHAR(100) | Metric identifier | Yes | E.g., 'daily_query_count', 'weekly_avg_latency' |
| time_period_start | TIMESTAMP WITH TIME ZONE | Period start | Yes | Beginning of aggregation window |
| time_period_end | TIMESTAMP WITH TIME ZONE | Period end | Yes | End of aggregation window |
| metric_value | JSONB | Aggregated metric data | Yes | See below for structure |
| book_id | VARCHAR(100) | Book identifier (if scoped) | No | NULL for global metrics |
| created_at | TIMESTAMP WITH TIME ZONE | Aggregate computation time | Yes | Auto-set to NOW() |

**Metric Value Structure (JSONB)** - varies by `metric_name`:

For `daily_query_count`:
```json
{
  "total_queries": 1247,
  "unique_users": 387
}
```

For `weekly_avg_latency`:
```json
{
  "p50": 1523,
  "p95": 2847,
  "p99": 3456
}
```

For `top_question_topics`:
```json
{
  "topics": [
    {"topic": "Intelligent Tutoring Systems", "count": 145},
    {"topic": "Adaptive Feedback", "count": 89},
    {"topic": "Knowledge Retrieval", "count": 76}
  ]
}
```

**Indexes**:
- Primary key: `aggregate_id`
- Index on `(metric_name, time_period_start, book_id)` for efficient retrieval
- Index on `time_period_start` for time-series queries

**Validation Rules**:
- `metric_name`: must be from predefined list (enforce via CHECK constraint or enum)
- `time_period_end` > `time_period_start`
- `metric_value`: valid JSON structure for specified `metric_name`

**Sample Data**:
```sql
INSERT INTO analytics_aggregates (aggregate_id, metric_name, time_period_start, time_period_end, metric_value, book_id, created_at)
VALUES (
  'd4e5f6g7-h8i9-0j1k-2l3m-n4o5p6q7r8s9',
  'daily_query_count',
  '2025-12-11T00:00:00Z',
  '2025-12-11T23:59:59Z',
  '{"total_queries": 1247, "unique_users": 387}',
  'physical-ai-robotics',
  '2025-12-12T00:05:00Z'
);
```

---

## Database Schema (Postgres)

### Table Creation SQL

```sql
-- Enable pgvector extension for vector similarity
CREATE EXTENSION IF NOT EXISTS vector;

-- Queries table
CREATE TABLE queries (
  query_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id VARCHAR(64) NOT NULL,
  query_text TEXT NOT NULL CHECK (length(query_text) BETWEEN 1 AND 500),
  query_embedding vector(1536) NOT NULL,
  selected_text TEXT,
  book_context JSONB,
  timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  session_id UUID,
  ip_address_hash VARCHAR(64),
  CONSTRAINT valid_user_id CHECK (length(user_id) = 64)
);

CREATE INDEX idx_queries_user_id ON queries(user_id);
CREATE INDEX idx_queries_timestamp ON queries(timestamp DESC);
CREATE INDEX idx_queries_book_context ON queries((book_context->>'book_id'), timestamp DESC);

-- Retrieved Contexts table
CREATE TABLE retrieved_contexts (
  context_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  query_id UUID NOT NULL REFERENCES queries(query_id) ON DELETE CASCADE,
  chunk_ids UUID[] NOT NULL CHECK (array_length(chunk_ids, 1) BETWEEN 1 AND 10),
  similarity_scores FLOAT[] NOT NULL,
  retrieval_params JSONB NOT NULL,
  timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  CONSTRAINT matching_array_lengths CHECK (array_length(chunk_ids, 1) = array_length(similarity_scores, 1))
);

CREATE INDEX idx_retrieved_contexts_query_id ON retrieved_contexts(query_id);
CREATE INDEX idx_retrieved_contexts_timestamp ON retrieved_contexts(timestamp DESC);

-- Query Responses table
CREATE TABLE query_responses (
  response_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  query_id UUID NOT NULL UNIQUE REFERENCES queries(query_id) ON DELETE CASCADE,
  response_text TEXT NOT NULL CHECK (length(response_text) BETWEEN 50 AND 2000),
  source_references JSONB NOT NULL,
  generation_params JSONB NOT NULL,
  latency_ms INTEGER NOT NULL CHECK (latency_ms BETWEEN 100 AND 10000),
  timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  confidence_score FLOAT CHECK (confidence_score BETWEEN 0.0 AND 1.0)
);

CREATE INDEX idx_query_responses_query_id ON query_responses(query_id);
CREATE INDEX idx_query_responses_timestamp ON query_responses(timestamp DESC);
CREATE INDEX idx_query_responses_confidence ON query_responses(confidence_score);

-- User Feedbacks table
CREATE TABLE user_feedbacks (
  feedback_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  response_id UUID NOT NULL UNIQUE REFERENCES query_responses(response_id) ON DELETE CASCADE,
  rating VARCHAR(20) NOT NULL CHECK (rating IN ('helpful', 'not_helpful')),
  comment TEXT CHECK (length(comment) <= 500),
  timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_user_feedbacks_response_id ON user_feedbacks(response_id);
CREATE INDEX idx_user_feedbacks_rating ON user_feedbacks(rating);
CREATE INDEX idx_user_feedbacks_timestamp ON user_feedbacks(timestamp DESC);

-- Analytics Aggregates table
CREATE TYPE metric_name_enum AS ENUM (
  'daily_query_count',
  'weekly_avg_latency',
  'monthly_feedback_rate',
  'top_question_topics',
  'hourly_concurrent_users'
);

CREATE TABLE analytics_aggregates (
  aggregate_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  metric_name metric_name_enum NOT NULL,
  time_period_start TIMESTAMP WITH TIME ZONE NOT NULL,
  time_period_end TIMESTAMP WITH TIME ZONE NOT NULL,
  metric_value JSONB NOT NULL,
  book_id VARCHAR(100),
  created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  CONSTRAINT valid_time_period CHECK (time_period_end > time_period_start)
);

CREATE INDEX idx_analytics_aggregates_lookup ON analytics_aggregates(metric_name, time_period_start, book_id);
CREATE INDEX idx_analytics_aggregates_time ON analytics_aggregates(time_period_start DESC);
```

---

## Data Lifecycle and Retention

### Data Retention Policies

1. **Queries**: Retain for 12 months, then archive or delete (GDPR compliance)
2. **Retrieved Contexts**: Retain for 12 months (same as queries)
3. **Query Responses**: Retain for 12 months (same as queries)
4. **User Feedbacks**: Retain for 12 months (same as queries)
5. **Analytics Aggregates**: Retain indefinitely (aggregated data, no PII)
6. **Book Content Chunks** (Qdrant): Retain all versions; implement version-based cleanup

### Archival Strategy

- **Monthly Job**: Archive queries, contexts, responses, feedbacks older than 12 months to S3/GCS
- **Partition Tables**: Partition `queries` table by month for efficient archival
- **Analytics Preservation**: Keep aggregated analytics even after raw data archived

---

## Data Migration Strategy

### Initial Data Load

1. **Book Content → Qdrant**:
   - Parse Markdown files from `docs/chapters/`
   - Chunk text (~500 words, 50-word overlap)
   - Generate embeddings via OpenAI API
   - Upsert chunks to Qdrant collection

2. **Schema Setup → Postgres**:
   - Run DDL scripts to create tables and indexes
   - Seed analytics aggregates table with initial metrics (if historical data available)

### Schema Versioning

- Use **Alembic** migrations for Postgres schema changes
- Track migration history in `alembic_version` table
- Test migrations in staging environment before production

### Data Backup

- **Postgres**: Enable Neon automated backups (point-in-time recovery)
- **Qdrant**: Export collections as snapshots weekly (Qdrant snapshot API)

---

## Summary

The data model supports the complete RAG chatbot workflow with:
- **Vector Database** (Qdrant): 1 entity (`BookContentChunk`) for semantic search
- **Relational Database** (Postgres): 5 entities for query processing, analytics, and feedback

Key design decisions:
1. **Separation of Concerns**: Vector data in Qdrant, structured data in Postgres
2. **Reproducibility**: All retrieval and generation parameters logged
3. **Analytics**: Pre-computed aggregates for dashboard performance
4. **Compliance**: Anonymized user IDs, configurable retention policies
5. **Scalability**: Indexed tables, partitioning strategy, efficient queries

All entities support the functional requirements (FR-008 through FR-025) and enable success criteria measurement (SC-001 through SC-010).
