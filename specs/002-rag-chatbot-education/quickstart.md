# Quickstart Guide: RAG Chatbot for Educational Resources

**Feature**: RAG Chatbot for Educational Resources
**Date**: 2025-12-11
**Purpose**: Setup instructions and development workflow for implementing the RAG chatbot

## Prerequisites

Before starting, ensure you have:

### Required Software
- **Python** 3.11+ ([download](https://www.python.org/downloads/))
- **Node.js** 18+ and npm ([download](https://nodejs.org/))
- **Docker** 20+ and Docker Compose ([download](https://www.docker.com/products/docker-desktop/))
- **Git** 2.40+ ([download](https://git-scm.com/downloads))

### Required Accounts
- **OpenAI API Account** ([sign up](https://platform.openai.com/signup))
  - Create API key at https://platform.openai.com/api-keys
  - Note: Will incur costs (~$10-30/month for typical usage)
- **Qdrant Cloud Account** ([sign up](https://cloud.qdrant.io/))
  - Create free tier cluster
  - Note cluster URL and API key
- **Neon Postgres Account** ([sign up](https://neon.tech/))
  - Create free tier database
  - Note connection string

### Optional but Recommended
- **VS Code** with Python and TypeScript extensions
- **Postman** or similar for API testing
- **GitHub** account for version control and deployment

---

## Project Setup

### 1. Clone Repository and Navigate to Project

```bash
# Clone the repository (if not already cloned)
git clone https://github.com/your-organization/physical-ai-book.git
cd physical-ai-book

# Checkout the feature branch
git checkout 002-rag-chatbot-education
```

### 2. Backend Setup (Python + FastAPI)

#### Create Backend Directory Structure

```bash
mkdir -p backend/src/{api,services,models,db,config}
mkdir -p backend/tests/{contract,integration,unit}
mkdir -p backend/scripts
cd backend
```

####  Set Up Python Virtual Environment

```bash
# Create virtual environment
python3.11 -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate
```

#### Install Python Dependencies

Create `requirements.txt`:

```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
pydantic==2.5.0
pydantic-settings==2.1.0
qdrant-client==1.7.0
openai==1.6.1
psycopg[binary,pool]==3.1.15
python-multipart==0.0.6
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
httpx==0.25.2
pytest==7.4.3
pytest-asyncio==0.21.1
pytest-cov==4.1.0
alembic==1.13.0
python-dotenv==1.0.0
```

Install dependencies:

```bash
pip install -r requirements.txt
```

#### Create Environment Configuration

Create `.env` file in `backend/` directory:

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-your-openai-api-key-here
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_GENERATION_MODEL=gpt-4o-mini
OPENAI_MAX_TOKENS=500
OPENAI_TEMPERATURE=0.0

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=book_chunks_v1

# Neon Postgres Configuration
DATABASE_URL=postgresql://user:password@your-cluster.neon.tech/chatbot_db?sslmode=require

# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
API_RELOAD=true
LOG_LEVEL=INFO

# Security Configuration
API_KEY_ENABLED=false  # Set true for production
API_KEY=your-secret-api-key

# RAG Configuration
CHUNK_SIZE=500  # words
CHUNK_OVERLAP=50  # words
TOP_K_RETRIEVAL=5
SIMILARITY_THRESHOLD=0.7

# Rate Limiting
RATE_LIMIT_PER_HOUR=60
```

**Important**: Add `.env` to `.gitignore` to avoid committing secrets!

#### Initialize Database Schema

Create Alembic migration:

```bash
# Initialize Alembic
alembic init alembic

# Create initial migration
alembic revision --autogenerate -m "Initial schema"

# Apply migration
alembic upgrade head
```

---

### 3. Frontend Setup (Docusaurus + React)

#### Install Docusaurus Dependencies

```bash
cd ..  # Return to project root
npm install
```

#### Install Additional Frontend Dependencies

```bash
npm install --save axios
npm install --save-dev @types/node
```

#### Create Chatbot Component Directory

```bash
mkdir -p src/components/ChatbotWidget
mkdir -p src/services
mkdir -p src/hooks
```

---

### 4. Local Development with Docker Compose

Create `docker-compose.yml` in project root for local development:

```yaml
version: '3.8'

services:
  # Local Postgres (alternative to Neon for development)
  postgres:
    image: postgres:15-alpine
    environment:
      POSTGRES_USER: chatbot
      POSTGRES_PASSWORD: development
      POSTGRES_DB: chatbot_db
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data

  # Local Qdrant (alternative to Qdrant Cloud for development)
  qdrant:
    image: qdrant/qdrant:v1.7.0
    ports:
      - "6333:6333"
      - "6334:6334"
    volumes:
      - qdrant_data:/qdrant/storage

  # Backend API
  backend:
    build:
      context: ./backend
      dockerfile: Dockerfile
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=postgresql://chatbot:development@postgres:5432/chatbot_db
      - QDRANT_URL=http://qdrant:6333
    env_file:
      - ./backend/.env
    volumes:
      - ./backend:/app
    depends_on:
      - postgres
      - qdrant
    command: uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload

volumes:
  postgres_data:
  qdrant_data:
```

Create `backend/Dockerfile`:

```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Start services:

```bash
docker-compose up -d
```

---

## Development Workflow

### Step 1: Process Book Content into Chunks

Create script `backend/scripts/chunk_book.py`:

```python
#!/usr/bin/env python3
"""
Chunk book content from Markdown files.
Usage: python chunk_book.py ../docs/chapters
"""
import sys
from pathlib import Path
# Implementation: Parse Markdown, chunk into ~500 words with 50-word overlap
# Output: JSON file with chunks and metadata
```

Run chunking:

```bash
cd backend
python scripts/chunk_book.py ../docs/chapters > chunks.json
```

### Step 2: Generate Embeddings and Upload to Qdrant

Create script `backend/scripts/embed_chunks.py`:

```python
#!/usr/bin/env python3
"""
Generate embeddings for chunks and upload to Qdrant.
Usage: python embed_chunks.py chunks.json
"""
import sys
from openai import OpenAI
from qdrant_client import QdrantClient
# Implementation: Read chunks, call OpenAI embedding API, upsert to Qdrant
```

Run embedding:

```bash
python scripts/embed_chunks.py chunks.json
```

**Note**: This may take 5-10 minutes and will incur OpenAI API costs (~$0.02-$0.10).

### Step 3: Run Backend API

```bash
# From backend/ directory with venv activated
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

API will be available at http://localhost:8000

View API documentation at http://localhost:8000/docs (Swagger UI)

### Step 4: Test API with Sample Query

Using `curl`:

```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How much do test scores improve with ITS?",
    "book_context": {
      "book_id": "physical-ai-robotics",
      "chapter_number": 2
    }
  }'
```

Expected response:

```json
{
  "query_id": "7f3d5e2a-1b4c-4f8e-9a3d-6c8b2e4a9f1d",
  "response_text": "Students using Intelligent Tutoring Systems (ITS) achieved 10-15% higher test scores...",
  "source_references": [
    {
      "chapter": "2",
      "section": "2.1 Intelligent Tutoring Systems (ITS)",
      "citation": "Chapter 2, Section 2.1",
      "chunk_id": "550e8400-e29b-41d4-a716-446655440000"
    }
  ],
  "confidence_score": 0.85,
  "latency_ms": 1847,
  "timestamp": "2025-12-11T14:25:33Z"
}
```

### Step 5: Run Frontend (Docusaurus)

```bash
# From project root
npm start
```

Docusaurus will be available at http://localhost:3000

### Step 6: Integrate Chatbot Widget (Frontend)

Create React component `src/components/ChatbotWidget/index.tsx`:

```typescript
import React, { useState } from 'react';
import axios from 'axios';

interface ChatbotWidgetProps {
  bookId: string;
  chapterNumber?: number;
}

export const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({ bookId, chapterNumber }) => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const res = await axios.post('http://localhost:8000/v1/query', {
        query,
        book_context: { book_id: bookId, chapter_number: chapterNumber }
      });
      setResponse(res.data.response_text);
    } catch (error) {
      console.error('Query failed:', error);
      setResponse('Sorry, something went wrong. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="chatbot-widget">
      <form onSubmit={handleSubmit}>
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask a question about this content..."
          disabled={loading}
        />
        <button type="submit" disabled={loading}>
          {loading ? 'Thinking...' : 'Ask'}
        </button>
      </form>
      {response && <div className="chatbot-response">{response}</div>}
    </div>
  );
};
```

Embed in Docusaurus page (e.g., `docs/chapters/02-key-ai-applications.md`):

```mdx
---
title: Key AI Applications in K-12 Education
---

import {ChatbotWidget} from '@site/src/components/ChatbotWidget';

## 2.1 Intelligent Tutoring Systems

[Your content here...]

<ChatbotWidget bookId="physical-ai-robotics" chapterNumber={2} />
```

---

## Testing

### Unit Tests

Run backend unit tests:

```bash
cd backend
pytest tests/unit/ -v --cov=src
```

### Integration Tests

Run full RAG pipeline tests:

```bash
pytest tests/integration/ -v
```

### Contract Tests

Validate API against OpenAPI spec:

```bash
pytest tests/contract/ -v
```

### End-to-End Tests

Install Playwright:

```bash
npm install --save-dev @playwright/test
```

Run e2e tests:

```bash
npx playwright test
```

---

## Common Issues and Troubleshooting

### Issue: OpenAI API Rate Limit Errors

**Solution**: Implement exponential backoff in `backend/src/services/embedding.py`:

```python
import time
from openai import RateLimitError

def embed_with_retry(text, max_retries=3):
    for attempt in range(max_retries):
        try:
            return openai_client.embeddings.create(input=text, model="text-embedding-3-small")
        except RateLimitError:
            if attempt < max_retries - 1:
                time.sleep(2 ** attempt)  # Exponential backoff: 1s, 2s, 4s
            else:
                raise
```

### Issue: Qdrant Connection Timeout

**Solution**: Check Qdrant Cloud firewall settings and ensure API key is correct.

### Issue: Empty or Irrelevant Responses

**Solution**: Lower similarity threshold in `.env`:

```env
SIMILARITY_THRESHOLD=0.6  # Down from 0.7
```

Or increase top-k retrieval:

```env
TOP_K_RETRIEVAL=7  # Up from 5
```

### Issue: Slow Response Times (>3s)

**Diagnosis**: Check latency breakdown:

```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "test", "book_context": {"book_id": "physical-ai-robotics"}}' \
  -w "\nTime: %{time_total}s\n"
```

**Solutions**:
- **Embedding slow**: Batch multiple queries if possible
- **Retrieval slow**: Check Qdrant cluster performance, consider upgrading tier
- **Generation slow**: Switch to smaller model (gpt-3.5-turbo instead of gpt-4o-mini) or reduce max_tokens

---

## Next Steps

1. **Implement User Stories**: Follow the prioritized user stories from `spec.md`:
   - P1: Student Self-Service Learning (core MVP)
   - P2: Teacher Workload Analytics Dashboard
   - P3: Content Feedback Loop

2. **Run Task Breakdown**: Execute `/sp.tasks` to generate detailed implementation tasks

3. **Set Up CI/CD**: Configure GitHub Actions for automated testing and deployment

4. **Deploy to Production**:
   - Backend: Deploy to Railway, Render, or AWS ECS
   - Frontend: Deploy to GitHub Pages or Vercel
   - Databases: Use Qdrant Cloud + Neon Postgres (already set up)

5. **Monitor and Optimize**:
   - Set up logging aggregation (Datadog, CloudWatch, etc.)
   - Monitor OpenAI API costs
   - Track user feedback and adjust retrieval/generation parameters

---

## Resources

- **FastAPI Documentation**: https://fastapi.tiangolo.com/
- **Qdrant Documentation**: https://qdrant.tech/documentation/
- **OpenAI API Documentation**: https://platform.openai.com/docs
- **Docusaurus Documentation**: https://docusaurus.io/docs
- **Feature Specification**: `spec.md`
- **Implementation Plan**: `plan.md`
- **Data Model**: `data-model.md`
- **API Contracts**: `contracts/openapi.yaml`

---

## Support

For questions or issues:
1. Check this quickstart guide and linked documentation
2. Review the feature specification (`spec.md`) and implementation plan (`plan.md`)
3. Search existing issues in the GitHub repository
4. Open a new issue with reproduction steps and logs

---

**Last Updated**: 2025-12-11
