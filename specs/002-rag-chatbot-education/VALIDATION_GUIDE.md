# Validation Guide: RAG Chatbot User Story 1

**Feature**: RAG Chatbot for Educational Resources
**Date**: 2025-12-12
**Phase**: User Story 1 Validation (Tasks T045-T049)

## Overview

This guide walks through validating the completed User Story 1 (Student Self-Service Learning) implementation. Before running these validation steps, ensure all code implementation tasks (T001-T044) are complete.

## Prerequisites Checklist

Before starting validation, ensure you have:

### Required Accounts and API Keys

- [ ] **OpenAI API Account**
  - Sign up at https://platform.openai.com/signup
  - Create API key at https://platform.openai.com/api-keys
  - Expected cost for validation: ~$0.10 (embeddings + test queries)

- [ ] **Qdrant Vector Database**
  - **Option A (Recommended)**: Qdrant Cloud Free Tier
    - Sign up at https://cloud.qdrant.io/
    - Create cluster and note URL + API key
  - **Option B**: Local Qdrant via Docker
    - Run: `docker-compose up -d qdrant`
    - URL: `http://localhost:6333`
    - No API key needed

- [ ] **PostgreSQL Database**
  - **Option A (Recommended)**: Neon Serverless Postgres
    - Sign up at https://neon.tech/
    - Create database and note connection string
  - **Option B**: Local Postgres via Docker
    - Run: `docker-compose up -d postgres`
    - URL: `postgresql://chatbot:development@localhost:5432/chatbot_db`

### Required Software

- [ ] **Python 3.11+** installed
- [ ] **Node.js 18+** and npm installed
- [ ] **Docker** (if using local Qdrant/Postgres)
- [ ] **Git** for version control

### Environment Configuration

Update `backend/.env` with your actual credentials:

```env
# OpenAI Configuration (REQUIRED)
OPENAI_API_KEY=sk-your-actual-api-key-here
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_GENERATION_MODEL=gpt-4o-mini

# Qdrant Configuration
# Option A: Cloud
QDRANT_URL=https://your-cluster-xxxxx.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Option B: Local
# QDRANT_URL=http://localhost:6333
# QDRANT_API_KEY=

# PostgreSQL Configuration
# Option A: Neon
DATABASE_URL=postgresql://user:password@ep-xxxxx.neon.tech/chatbot_db?sslmode=require

# Option B: Local
# DATABASE_URL=postgresql://chatbot:development@localhost:5432/chatbot_db

# Collection name
QDRANT_COLLECTION_NAME=book_chunks_v1
```

---

## Validation Tasks

### Task T045: Generate Embeddings and Upload to Qdrant

**Purpose**: Process book chunks and upload embeddings to vector database

**Steps**:

1. Activate Python virtual environment:
   ```bash
   cd backend
   # Windows
   venv\Scripts\activate
   # macOS/Linux
   source venv/bin/activate
   ```

2. Verify chunks.json exists:
   ```bash
   ls -lh chunks.json
   # Should show ~380 lines, ~200-300 KB
   ```

3. Run embedding script:
   ```bash
   python scripts/embed_chunks.py chunks.json
   ```

4. Monitor progress:
   ```
   Loading chunks from: chunks.json
   Book: physical-ai-robotics v1.0.0
   Total chunks: 22

   Generating embeddings...
   ✓ Embeddings generated for 22 chunks

   Connecting to Qdrant...
   ✓ Connected to collection: book_chunks_v1

   Preparing chunks for upload...
   ✓ 22 chunks ready for upload

   Uploading chunks to Qdrant...
     → Uploaded 22/22 chunks

   ✓ Upload complete: 22 chunks in book_chunks_v1
   ```

**Expected Duration**: 30-60 seconds
**Expected Cost**: ~$0.02 (OpenAI embedding API)

**Validation**:
- [ ] Script completes without errors
- [ ] All 22 chunks uploaded to Qdrant
- [ ] Qdrant collection `book_chunks_v1` created

**Troubleshooting**:
- **OpenAI rate limit error**: Wait 60 seconds and retry
- **Qdrant connection error**: Check URL and API key in .env
- **Authentication error**: Verify OpenAI API key is valid

---

### Task T046: Start Backend API and Verify Health

**Purpose**: Launch FastAPI server and confirm it's responding

**Steps**:

1. Start backend server (keep terminal open):
   ```bash
   cd backend
   # Ensure venv is activated
   uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
   ```

2. Verify server starts:
   ```
   INFO:     Will watch for changes in these directories: ['/path/to/backend']
   INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
   INFO:     Started reloader process [12345] using StatReload
   INFO:     Started server process [12346]
   INFO:     Waiting for application startup.
   INFO:     Application startup complete.
   ```

3. Test health endpoint:
   ```bash
   # In a new terminal
   curl http://localhost:8000/health
   ```

4. Expected response:
   ```json
   {
     "status": "healthy",
     "version": "1.0.0",
     "services": {
       "qdrant": "connected",
       "postgres": "connected"
     }
   }
   ```

5. View API documentation:
   - Open browser: http://localhost:8000/docs
   - Should see Swagger UI with endpoints

**Validation**:
- [ ] Server starts without errors
- [ ] Health endpoint returns 200 OK
- [ ] Qdrant connection status: "connected"
- [ ] Postgres connection status: "connected"
- [ ] Swagger docs accessible

**Troubleshooting**:
- **Port 8000 already in use**: Kill existing process or use different port
- **Qdrant not connected**: Verify Qdrant URL and API key
- **Postgres not connected**: Check DATABASE_URL in .env
- **Import errors**: Run `pip install -r requirements.txt` again

---

### Task T047: Test POST /v1/query Endpoint

**Purpose**: Validate query processing, retrieval, and response generation

**Steps**:

1. Test basic query:
   ```bash
   curl -X POST http://localhost:8000/v1/query \
     -H "Content-Type: application/json" \
     -d '{
       "query": "What is physical AI?",
       "book_context": {
         "book_id": "physical-ai-robotics",
         "chapter_number": 1
       }
     }'
   ```

2. Expected response structure:
   ```json
   {
     "query_id": "550e8400-e29b-41d4-a716-446655440000",
     "response_text": "Physical AI refers to artificial intelligence systems embodied in physical agents that sense and act upon their environment...",
     "source_references": [
       {
         "chapter": "1",
         "section": "1.1 Embodied Cognition and Physical AI",
         "citation": "Chapter 1, Section 1.1",
         "chunk_id": "abc123..."
       }
     ],
     "confidence_score": 0.87,
     "latency_ms": 1847,
     "timestamp": "2025-12-12T10:30:00Z"
   }
   ```

3. Test additional queries from spec.md:
   ```bash
   # Query 2: Moravec's paradox
   curl -X POST http://localhost:8000/v1/query \
     -H "Content-Type: application/json" \
     -d '{
       "query": "What is Moravec'\''s paradox?",
       "book_context": {"book_id": "physical-ai-robotics"}
     }'

   # Query 3: Embodied cognition
   curl -X POST http://localhost:8000/v1/query \
     -H "Content-Type: application/json" \
     -d '{
       "query": "Explain embodied cognition",
       "book_context": {"book_id": "physical-ai-robotics", "chapter_number": 1}
     }'
   ```

**Validation Criteria**:
- [ ] Latency: p95 < 3000ms (check `latency_ms` field)
- [ ] Source references present (at least 1)
- [ ] Response text is relevant and grounded
- [ ] Confidence score > 0.6
- [ ] No hallucinations detected (manual review)

**Acceptance Scenarios** (from spec.md):

Test these scenarios:
1. ✅ Student asks about concept from specific chapter
2. ✅ Student highlights text and asks follow-up question
3. ✅ Student submits vague query (should get clarification or relevant answer)
4. ✅ Student asks out-of-scope question (should respond with insufficient information)

**Troubleshooting**:
- **500 Internal Server Error**: Check backend logs for Python exceptions
- **High latency (>5s)**: Check OpenAI API status, consider caching
- **Empty source_references**: Lower similarity threshold in .env
- **Irrelevant responses**: Adjust top_k or improve chunking

---

### Task T048: Start Frontend and Verify ChatbotWidget

**Purpose**: Launch Docusaurus dev server and verify chatbot UI renders

**Steps**:

1. Start frontend dev server (keep terminal open):
   ```bash
   # From project root
   npm start
   ```

2. Verify server starts:
   ```
   [INFO] Starting the development server...
   [SUCCESS] Docusaurus website is running at: http://localhost:3000/
   ```

3. Open browser: http://localhost:3000

4. Navigate to a chapter page (e.g., Chapter 1: Introduction)

5. Verify ChatbotWidget renders:
   - Widget should appear on the page
   - Input field should be visible
   - "Ask" button should be present
   - No console errors (check browser DevTools)

**Validation**:
- [ ] Frontend builds without errors
- [ ] ChatbotWidget component renders
- [ ] Input field accepts text
- [ ] No browser console errors
- [ ] Widget styling matches Docusaurus theme

**Troubleshooting**:
- **Port 3000 already in use**: Kill existing process or use different port
- **Component not rendering**: Check import in page MDX file
- **TypeScript errors**: Run `npm install` to ensure dependencies are installed
- **Styling issues**: Check `styles.module.css` import

---

### Task T049: End-to-End Test

**Purpose**: Complete user flow from question submission to response display

**Steps**:

1. Ensure both backend and frontend are running:
   - Backend: http://localhost:8000
   - Frontend: http://localhost:3000

2. In browser, navigate to a chapter page with ChatbotWidget

3. Submit a test query:
   - Type: "What is physical AI?"
   - Click "Ask" button

4. Observe the flow:
   - Loading indicator appears
   - Backend processes query (check backend logs)
   - Response appears with citations
   - No errors in browser console

5. Verify response quality:
   - Response is relevant to the question
   - Source citations are displayed (e.g., "Chapter 1, Section 1.1")
   - Response time is acceptable (<5 seconds)

6. Check database logs:
   ```bash
   # Connect to Postgres and verify logs
   # Query example (adjust for your database)
   SELECT query_text, response_text, latency_ms, timestamp
   FROM queries q
   JOIN query_responses qr ON q.query_id = qr.query_id
   ORDER BY timestamp DESC
   LIMIT 5;
   ```

**Validation Criteria**:
- [ ] Query submits successfully
- [ ] Loading state displays during processing
- [ ] Response appears with formatted text
- [ ] Source citations are clickable/visible
- [ ] Query logged to database
- [ ] Retrieved context logged
- [ ] Response logged to database
- [ ] No frontend errors
- [ ] No backend errors

**Additional Test Scenarios**:

Test these edge cases:
1. **Empty query**: Submit empty input → should show validation error
2. **Very long query**: Submit 600+ character text → should show validation error
3. **Out-of-scope query**: Ask "What's the weather?" → should respond with insufficient info message
4. **Network error**: Stop backend server, submit query → should show error message
5. **Multiple queries**: Submit 3 queries in quick succession → all should work

**Troubleshooting**:
- **CORS errors**: Verify CORS settings in backend/src/main.py
- **Network timeout**: Check axios timeout settings in chatbotApi.ts
- **Empty responses**: Check backend logs for errors
- **Database errors**: Verify DATABASE_URL connection string

---

## Success Criteria Summary

User Story 1 (MVP) is considered validated when:

### Functional Requirements
- ✅ Students can submit natural language queries
- ✅ System returns accurate, source-grounded answers
- ✅ Responses include citations to specific book sections
- ✅ System handles edge cases gracefully (empty query, out-of-scope, etc.)

### Performance Requirements
- ✅ p95 latency < 3 seconds (end-to-end)
- ✅ System supports concurrent queries (test with multiple browser tabs)

### Data Requirements
- ✅ All queries logged to database
- ✅ Retrieved context stored for reproducibility
- ✅ Responses logged with metadata (latency, confidence, etc.)

### Quality Requirements
- ✅ >95% of test queries receive accurate answers (manual review)
- ✅ <1% hallucination rate (responses stay grounded in source material)
- ✅ No unhandled errors or crashes

---

## Next Steps After Validation

Once Tasks T045-T049 are complete and validated:

### Option 1: Deploy MVP (User Story 1)
- Deploy backend to cloud platform (Railway, Render, AWS)
- Deploy frontend to GitHub Pages or Vercel
- Share with initial test users (10-20 students)
- Collect feedback for 1-2 weeks

### Option 2: Continue Development (User Story 2)
- Implement Analytics Dashboard (Tasks T050-T071)
- Build teacher-facing metrics and insights
- Track ROI and usage patterns

### Option 3: Implement Feedback Loop (User Story 3)
- Add feedback buttons (Tasks T072-T090)
- Enable content gap identification
- Close the improvement loop

---

## Validation Checklist

Use this checklist to track validation progress:

**Prerequisites**:
- [ ] OpenAI API key configured
- [ ] Qdrant database accessible (cloud or local)
- [ ] PostgreSQL database accessible (cloud or local)
- [ ] Environment variables (.env) configured

**T045: Embeddings**:
- [ ] chunks.json file exists
- [ ] embed_chunks.py runs successfully
- [ ] All chunks uploaded to Qdrant
- [ ] Collection created in Qdrant

**T046: Backend Health**:
- [ ] Backend server starts without errors
- [ ] Health endpoint returns 200 OK
- [ ] Swagger docs accessible
- [ ] Services connected (Qdrant, Postgres)

**T047: Query Endpoint**:
- [ ] POST /v1/query accepts requests
- [ ] Responses include source_references
- [ ] Latency < 3s for test queries
- [ ] Edge cases handled correctly

**T048: Frontend**:
- [ ] Frontend builds successfully
- [ ] ChatbotWidget renders on page
- [ ] No browser console errors
- [ ] Input field functional

**T049: End-to-End**:
- [ ] Full user flow works (query → response)
- [ ] Database logging confirmed
- [ ] Response quality acceptable
- [ ] All edge cases handled

**Overall**:
- [ ] All validation tasks complete
- [ ] No blocking issues
- [ ] Ready for user testing or next phase

---

**Last Updated**: 2025-12-12
