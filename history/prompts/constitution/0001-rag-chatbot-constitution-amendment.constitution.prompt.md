---
id: 0001
title: RAG Chatbot Constitution Amendment
stage: constitution
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: CW
command: /sp.constitution
labels: ["constitution", "rag-chatbot", "ai-native", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - None
---

## Prompt

   Integrated RAG Chatbot Development
Overview
Retrieval-Augmented Generation (RAG) combines traditional information retrieval with language generation models, enabling AI systems to produce answers grounded in an external knowledge base rather than relying solely on pre-trained language patterns (Lewis et al., 2020). In the context of a digital book created using Docusaurus, a RAG chatbot allows readers to query content interactively, with responses generated directly from the book's text. This ensures accuracy, traceability, and contextual relevance.
System Architecture
The RAG chatbot integration relies on several key components, forming a modular, AI-native architecture:
1.    Frontend (User Interaction Layer)
o    Embedded directly within the Docusaurus book interface.
o    Allows users to highlight or select text and submit queries.
o    Displays responses in real-time, optionally with source references.
2.    Retrieval Layer (Vector Database)
o    Uses Qdrant Cloud Free Tier to store and index book content.
o    Text is converted into embeddings using OpenAI embedding models.
o    When a user submits a query, relevant text chunks are retrieved based on semantic similarity.
3.    Generation Layer (AI Agent)
o    Powered by OpenAI Agents / ChatKit SDKs.
o    Generates coherent responses using the retrieved text as context.
o    Ensures all output is grounded in source material, maintaining reliability.
4.    Backend API Layer (Server Management)
o    Implemented using FastAPI.
o    Handles query requests, manages retrieval calls to Qdrant, and forwards the context to the AI agent.
o    Logs interactions for reproducibility and auditing purposes.
5.    Database Layer (Query Storage and Metadata)
o    Neon Serverless Postgres stores queries, user interactions, and metadata.
o    Supports analytics on usage patterns, performance, and verification of generated answers.
Figure 1: High-Level Architecture of the RAG Chatbot
User Query → FastAPI → Qdrant Retrieval → OpenAI Generation → Response Display
Implementation Workflow
1.    Preprocessing and Embedding Generation
o    The book text is segmented into manageable chunks (~500 words each).
o    Each chunk is converted into embeddings using an OpenAI embedding model.
o    Chunks and embeddings are uploaded to Qdrant for semantic search.
2.    Query Handling
o    Users submit queries via the frontend interface.
o    FastAPI backend receives the query and converts it into an embedding.
o    Qdrant returns the top N relevant text chunks.
3.    Answer Generation
o    Retrieved chunks are passed to the OpenAI Agent via ChatKit SDK.
o    The agent generates a response strictly based on the retrieved content.
o    Optionally, the chatbot includes source references to maintain traceability.
4.    Logging and Reproducibility
o    Every query, retrieved context, and generated response is stored in Neon Postgres.
o    Enables reproducibility, debugging, and verification against the book text.
Evaluation Metrics
•    Accuracy: Percentage of answers correctly grounded in book content.
•    Latency: Response time from query submission to generation.
•    User Satisfaction: Qualitative feedback on answer clarity and relevance.
•    Traceability: Ability to verify responses against the original text.
Discussion
Integrating a RAG chatbot into a Docusaurus-based book represents a fully AI-native approach to educational resources. Users can interactively explore content while ensuring that AI-generated responses remain verifiable and source-based. The modular architecture supports scalability, allowing additional books or chapters to be integrated seamlessly. Challenges include optimizing retrieval performance, managing embedding storage, and ensuring the AI agent strictly adheres to the source content without hallucination.
________________________________________
References (APA style)
•    Lewis, P., Oguz, B., Rinott, R., Riedel, S., & Stoyanov, V. (2020). Retrieval-Augmented Generation for Knowledge-Intensive NLP Tasks. Advances in Neural Information Processing Systems, 33, 9459–9474.
•    OpenAI. (2023). ChatKit SDK Documentation. OpenAI Developer Resources. http

## Response snapshot

Constitution updated from version 1.0.0 to 1.1.0 with comprehensive RAG chatbot integration standards.

Key amendments:
- Added RAG-specific requirements to all four core principles (Accuracy, Clarity, Reproducibility, Rigor)
- Created new "Interactive Learning Standards" subsection under Academic Standards
- Created new "AI-Native Architecture Standards" subsection under Technical Standards
- Added comprehensive "RAG Chatbot Integration Standards" section covering:
  - Retrieval Quality (chunking strategy, embedding quality, retrieval parameters)
  - Generation Quality (grounding mechanisms, response format, safety and accuracy)
  - Logging and Reproducibility (query logging schema, reproducibility requirements, analytics)
- Updated Development Workflow to include RAG Integration Workflow
- Enhanced Quality Gates with RAG Integration Checks
- Added RAG-specific examples to ADR-worthy decisions
- Updated Compliance Review to include monthly RAG quality audits

Technology stack documented:
- Frontend: Docusaurus with embedded chatbot interface
- Vector Database: Qdrant Cloud Free Tier
- Embedding: OpenAI embedding models (~500 word chunks with overlap)
- Generation: OpenAI Agents / ChatKit SDKs
- Backend: FastAPI for query orchestration
- Database: Neon Serverless Postgres for query logs and analytics

Performance targets established:
- Accuracy: >95% grounding in book content
- Latency: <3 seconds p95
- Traceability: All responses must include source references
- Testing: 100+ test queries before deployment

## Outcome

- ✅ Impact: Constitution version bumped to 1.1.0 (MINOR). RAG chatbot standards fully integrated across all relevant sections. Project now has clear governance for AI-native interactive features while maintaining academic rigor and reproducibility.
- 🧪 Tests: No automated tests for constitution amendments. Manual validation performed: all placeholders filled, no backward-incompatible changes, existing templates remain compatible.
- 📁 Files: Modified 1 file (.specify/memory/constitution.md). Created 1 PHR (history/prompts/constitution/0001-rag-chatbot-constitution-amendment.constitution.prompt.md).
- 🔁 Next prompts: Consider running `/sp.adr` to document RAG architecture decisions (Qdrant vs. alternatives, OpenAI vs. open-source models, FastAPI selection, chunking strategy). Consider running `/sp.specify` to create RAG chatbot feature specification.
- 🧠 Reflection: Amendment successfully extends constitution to cover AI-native interactive learning without compromising core academic principles. RAG integration standards are comprehensive yet flexible enough to adapt as technology evolves. Templates remain compatible, no breaking changes required.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (constitution amendment, no automated graders applicable)
- Prompt variant (if applicable): Standard /sp.constitution command with RAG architecture specification as input
- Next experiment (smallest change to try): Test constitution compliance by creating RAG chatbot feature spec using `/sp.specify` and validating against new standards
