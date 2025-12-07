# Content Templates and Contracts

This directory contains standardized templates for book content, ensuring consistency and constitution compliance.

## Templates

### chapter-template.md
Standard template for all book chapters. Includes metadata structure, section organization, and constitution compliance requirements.

### code-example-template.md
Template for reproducible code examples. Ensures all examples include dependencies, setup, expected outputs, and troubleshooting.

### case-study-template.md
Template for robot case studies. Structured analysis of hardware, perception, planning, control, and lessons learned.

## Usage

Copy the appropriate template when creating new content:

```bash
# New chapter
cp contracts/chapter-template.md docs/chapters/[XX]-[title].md

# New code example
cp contracts/code-example-template.md examples/[domain]/[name].py

# New case study
cp contracts/case-study-template.md docs/chapters/[XX]-case-study-[robot].md
```

Fill in all bracketed placeholders and follow constitution principles (Accuracy, Clarity, Reproducibility, Rigor).

## Constitution Compliance

All templates enforce:
- Verifiable technical claims (Accuracy)
- Clear structure for graduate CS audience (Clarity)
- Reproducible examples with complete setup (Reproducibility)
- APA citations with peer-reviewed sources (Rigor)

See `.specify/memory/constitution.md` for full standards.
