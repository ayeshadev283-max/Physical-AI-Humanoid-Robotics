# Contributing to Physical AI & Humanoid Robotics Book

Thank you for your interest in contributing to this open-source textbook! This document provides guidelines for contributing content, code examples, and improvements.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How Can I Contribute?](#how-can-i-contribute)
- [Content Guidelines](#content-guidelines)
- [Code Example Guidelines](#code-example-guidelines)
- [Pull Request Process](#pull-request-process)
- [Citation and Attribution](#citation-and-attribution)

## Code of Conduct

This project adheres to principles of academic integrity, inclusivity, and respect. By participating, you are expected to:

- Maintain a respectful and professional tone in all communications
- Provide constructive feedback and welcome constructive criticism
- Focus on what is best for the community and learners
- Acknowledge the contributions of others

## How Can I Contribute?

### Reporting Issues

If you find errors, outdated information, or have suggestions for improvement:

1. Check if the issue already exists in [GitHub Issues](https://github.com/your-organization/physical-ai-book/issues)
2. If not, create a new issue with:
   - A clear, descriptive title
   - The specific section/module where the issue occurs
   - A detailed description of the problem or suggestion
   - (Optional) Screenshots, code snippets, or references

### Suggesting New Content

To suggest new chapters, sections, or topics:

1. Open a [GitHub Discussion](https://github.com/your-organization/physical-ai-book/discussions) to gauge community interest
2. Provide:
   - A clear description of the proposed content
   - Justification for its inclusion (relevance, novelty, educational value)
   - Potential outline or structure
   - Relevant citations or references

### Submitting Pull Requests

For content contributions, code examples, or fixes:

1. Fork the repository
2. Create a new branch from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Make your changes following our guidelines (see below)
4. Commit your changes with clear, descriptive commit messages
5. Push to your fork and submit a pull request

## Content Guidelines

All content contributions must adhere to the project constitution (`.specify/memory/constitution.md`):

### Accuracy (50%+ Peer-Reviewed Sources)

- **Primary Sources**: Prefer peer-reviewed journal articles, conference papers, and authoritative textbooks
- **Secondary Sources**: Technical documentation, official framework docs, and reputable blog posts are acceptable for tool usage
- **Citations**: Use APA format with BibTeX entries in `references/physical-ai-book.bib`
- **Fact-Checking**: Verify all technical claims, formulas, and code snippets before submission

### Clarity and Accessibility

- **Target Audience**: Graduate-level students with basic programming and linear algebra knowledge
- **Writing Style**: Clear, concise, technical but accessible
- **Diagrams**: Include visual aids (Mermaid diagrams, screenshots, or SVG illustrations) for complex concepts
- **Code Comments**: Explain non-obvious logic and design decisions

### Reproducibility

- **Code Examples**: All code must be runnable with clear setup instructions
- **Dependencies**: List all dependencies with specific versions
- **Docker**: Provide Dockerfiles for complex environments (ROS 2, Isaac, etc.)
- **Testing**: Include test cases or validation steps

### Academic Rigor

- **Mathematical Notation**: Use KaTeX for equations, define all symbols
- **Proofs**: Provide sketches or references for key theorems
- **Assumptions**: State assumptions clearly
- **Limitations**: Discuss limitations and edge cases

## Code Example Guidelines

Code examples are licensed under Apache 2.0 and must be:

### Functional and Tested

- Runnable without modification (given correct environment)
- Include unit tests or validation scripts
- Provide expected output or behavior

### Well-Documented

- SPDX license header in all source files:
  ```python
  # SPDX-License-Identifier: Apache-2.0
  # Copyright (C) 2025 Physical AI & Humanoid Robotics Book Contributors
  ```
- Docstrings for functions/classes
- Inline comments for complex logic
- README.md with setup and usage instructions

### Organized

- Follow directory structure:
  ```
  examples/
    ├── ros2-examples/
    ├── gazebo-examples/
    ├── isaac-examples/
    ├── vla-examples/
    ├── capstone-templates/
    └── docker-base/
  ```
- Use descriptive file and variable names
- Consistent formatting (Black for Python, clang-format for C++)

### Secure

- No hardcoded secrets, API keys, or tokens (use `.env` files with `.env.example` templates)
- Validate inputs and handle errors gracefully
- Follow OWASP guidelines for web-facing code

## Pull Request Process

1. **Pre-Submission Checklist**:
   - [ ] All content follows constitutional principles (accuracy, clarity, reproducibility, rigor)
   - [ ] Code examples are tested and documented
   - [ ] Citations are added to `references/physical-ai-book.bib`
   - [ ] No linting errors (`npm run lint`, `black --check .`, etc.)
   - [ ] Docusaurus build succeeds (`npm run build`)
   - [ ] Links are valid (checked with `npm run link-check` or manually)

2. **Pull Request Template**:
   Use the following template when creating a pull request:
   ```markdown
   ## Description
   [Brief description of changes]

   ## Module/Section
   [Which module or section does this affect?]

   ## Type of Change
   - [ ] New content (chapter, section)
   - [ ] Code example
   - [ ] Bug fix (typo, broken link, incorrect code)
   - [ ] Enhancement (improved explanation, diagram)
   - [ ] Documentation

   ## Checklist
   - [ ] I have followed the content/code guidelines
   - [ ] I have added citations for all claims
   - [ ] I have tested code examples
   - [ ] I have built the Docusaurus site locally
   - [ ] My changes generate no new warnings
   - [ ] I have updated the bibliography if needed

   ## Related Issues
   Closes #[issue number]
   ```

3. **Review Process**:
   - Maintainers will review within 5-7 business days
   - Expect constructive feedback and requests for changes
   - Once approved, your PR will be merged and you'll be added to contributors

## Citation and Attribution

### Citing External Sources

All external sources must be cited using BibTeX in `references/physical-ai-book.bib`:

```bibtex
@article{AuthorYear,
  author = {Last, First and Last, First},
  title = {Title of the Paper},
  journal = {Journal Name},
  year = {2024},
  volume = {10},
  number = {2},
  pages = {100--120},
  doi = {10.1234/example},
  keywords = {peer-reviewed, topic1, topic2}
}
```

**Keywords**: Use `peer-reviewed` for journal/conference papers, and topical keywords (e.g., `ros2`, `vla`, `simulation`).

### Inline Citations

Use Pandoc citation syntax in Markdown:

```markdown
Brooks' subsumption architecture [@Brooks1991] demonstrated that intelligence
need not rely on explicit representations.
```

### Your Contributions

By contributing, you agree that:

- Your content will be licensed under **CC BY-SA 4.0** (for content in `docs/`)
- Your code will be licensed under **Apache 2.0** (for code in `examples/`)
- You will be attributed in the contributors list

## Questions?

If you have questions about contributing:

- Check the [FAQ in GitHub Discussions](https://github.com/your-organization/physical-ai-book/discussions)
- Open a new discussion
- Contact the maintainers via GitHub Issues

Thank you for contributing to open robotics education!
