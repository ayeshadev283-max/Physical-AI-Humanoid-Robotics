# Quickstart Guide: Contributing to Physical AI & Humanoid Robotics Book

**Date**: 2025-12-05
**Purpose**: Phase 1 output - Guide for contributors to set up environment and add content

## Prerequisites

- Git installed
- Node.js 18.x LTS installed
- Python 3.11+ installed
- Text editor (VS Code recommended)

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/[organization]/physical-ai-book.git
cd physical-ai-book
```

### 2. Install Docusaurus Dependencies

```bash
npm install
```

### 3. Run the Site Locally

```bash
npm run start
```

Site will open at `http://localhost:3000`

### 4. Install Python Dependencies (for code examples)

```bash
pip install -r requirements.txt
```

## Adding New Content

### Adding a Chapter

1. Copy template: `cp specs/001-physical-ai-robotics-book/contracts/chapter-template.md docs/chapters/[XX]-[title].md`
2. Fill in front matter (metadata)
3. Write content following constitution principles (Accuracy, Clarity, Reproducibility, Rigor)
4. Add APA citations inline and to bibliography
5. Add to `sidebars.js` navigation
6. Test locally: `npm run start`

### Adding a Code Example

1. Copy template: `cp specs/001-physical-ai-robotics-book/contracts/code-example-template.md examples/[domain]/[name].py`
2. Implement code with inline comments
3. Add pytest test in `tests/test_[name].py`
4. Run test: `pytest tests/test_[name].py`
5. Document in parent chapter with link

### Adding Citations

1. Add to `docs/references/bibliography.md` in APA format
2. Categorize as Peer-Reviewed or Other
3. Use inline citation in text: `(Author, Year)` or `(Author, Year, p. XX)`
4. Ensure bibliography entry exists for all inline citations

## Constitution Compliance Checklist

Before submitting content:

- [ ] All technical claims verified against primary/authoritative sources
- [ ] Code examples include dependencies, setup, and expected outputs
- [ ] Content clear for graduate CS audience
- [ ] Citations in APA format
- [ ] At least 50% of sources peer-reviewed (for chapters)
- [ ] Speculative content explicitly marked
- [ ] Markdown uses semantic headings
- [ ] Code blocks specify language tags
- [ ] Links use descriptive text
- [ ] All code examples tested and pass

## Validation Before Committing

```bash
# Run linting
npm run lint

# Check links (when implemented)
npm run check-links

# Test all code examples
pytest tests/

# Build site (check for errors)
npm run build
```

## Submitting Contributions

1. Create feature branch: `git checkout -b feature/chapter-XX`
2. Make changes and commit: `git commit -m "Add Chapter XX: [title]"`
3. Push: `git push origin feature/chapter-XX`
4. Create Pull Request on GitHub
5. Ensure CI checks pass (linting, tests, build)

## Style Guide Summary

- **Headings**: H1 for chapter title, H2 for sections, H3 for subsections
- **Code blocks**: Always specify language (```python, ```bash)
- **Citations**: Inline (Author, Year), full APA in bibliography
- **Links**: `[Descriptive text](url)`, not "click here"
- **Images**: Include alt text for accessibility

## Resources

- Constitution: `.specify/memory/constitution.md`
- Spec: `specs/001-physical-ai-robotics-book/spec.md`
- Plan: `specs/001-physical-ai-robotics-book/plan.md`
- Research: `specs/001-physical-ai-robotics-book/research.md`
- Templates: `specs/001-physical-ai-robotics-book/contracts/`

## Getting Help

- Check constitution for quality standards
- Review existing chapters for examples
- Consult spec for feature requirements
- Ask in project discussions/issues
