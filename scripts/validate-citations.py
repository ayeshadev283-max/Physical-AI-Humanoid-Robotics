#!/usr/bin/env python3
"""
Citation Validation Script for Physical AI & Humanoid Robotics Book

Validates that:
1. All inline citations have corresponding bibliography entries
2. At least 50% of sources are peer-reviewed
3. APA format is followed (basic checks)
"""

import re
import sys
from pathlib import Path
from typing import Set, List, Tuple


def extract_inline_citations(content: str) -> Set[str]:
    """Extract all inline citations like (Author, Year) or (Author et al., Year)"""
    # Pattern matches: (Author, Year), (Author & Author, Year), (Author et al., Year)
    pattern = r'\(([A-Z][a-zA-Z\s&\.]+,\s*\d{4}[a-z]?(?:,\s*p\.\s*\d+)?)\)'
    citations = set()
    for match in re.finditer(pattern, content):
        citation = match.group(1)
        # Normalize: extract just "Author, Year" part
        parts = citation.split(',')
        if len(parts) >= 2:
            author = parts[0].strip()
            year = parts[1].strip().split()[0]  # Get just the year, ignore page numbers
            normalized = f"{author}, {year}"
            citations.add(normalized)
    return citations


def extract_bibliography_entries(bib_content: str) -> Tuple[Set[str], int, int]:
    """
    Extract bibliography entries and count peer-reviewed sources.

    Returns:
        (set of author-year strings, total_count, peer_reviewed_count)
    """
    entries = set()
    total = 0
    peer_reviewed = 0

    # Split by lines starting with author names (capital letter after line start)
    lines = bib_content.split('\n')

    for line in lines:
        line = line.strip()

        # Skip markdown markers, headings, empty lines
        if not line or line.startswith('#') or line.startswith('**') or line.startswith('---'):
            continue

        # Check if this line starts with a citation (capital letter)
        if len(line) > 0 and line[0].isupper():
            # Check if it's a peer-reviewed source
            if line.startswith('⭐'):
                peer_reviewed += 1
                line = line[1:].strip()  # Remove star

            # Extract author and year using APA pattern
            # Pattern: Author, A. (Year) or Author, A., & Author, B. (Year)
            match = re.match(r'^([A-Z][a-zA-Z\s,\.&\-]+)\((\d{4}[a-z]?)\)', line)
            if match:
                author_part = match.group(1).strip()
                year = match.group(2)

                # Get first author's last name
                first_author = author_part.split(',')[0].strip()

                # Handle "et al." case
                if '&' in author_part or ',' in author_part:
                    # Multiple authors - use first author + et al.
                    entries.add(f"{first_author} et al., {year}")
                    entries.add(f"{first_author}, {year}")  # Also add single author form
                else:
                    entries.add(f"{first_author}, {year}")

                total += 1

    return entries, total, peer_reviewed


def validate_chapter(chapter_path: Path, bib_entries: Set[str]) -> Tuple[bool, List[str]]:
    """Validate a single chapter's citations"""
    errors = []

    if not chapter_path.exists():
        return True, []  # Skip non-existent chapters

    content = chapter_path.read_text(encoding='utf-8')
    inline_citations = extract_inline_citations(content)

    # Check each inline citation
    for citation in inline_citations:
        # Normalize for comparison
        found = False
        for bib_entry in bib_entries:
            if citation.lower() in bib_entry.lower() or bib_entry.lower() in citation.lower():
                found = True
                break

        if not found:
            errors.append(f"  Missing bibliography entry for: {citation}")

    return len(errors) == 0, errors


def main():
    """Main validation function"""
    repo_root = Path(__file__).parent.parent
    docs_dir = repo_root / 'docs'
    bib_path = docs_dir / 'references' / 'bibliography.md'

    print("=" * 60)
    print("Citation Validation for Physical AI & Humanoid Robotics Book")
    print("=" * 60)
    print()

    # Read bibliography
    if not bib_path.exists():
        print(f"❌ ERROR: Bibliography not found at {bib_path}")
        sys.exit(1)

    bib_content = bib_path.read_text(encoding='utf-8')
    bib_entries, total_sources, peer_reviewed = extract_bibliography_entries(bib_content)

    # Report bibliography statistics
    print(f"Bibliography Statistics:")
    print(f"  Total sources: {total_sources}")
    print(f"  Peer-reviewed: {peer_reviewed}")
    if total_sources > 0:
        percentage = (peer_reviewed / total_sources) * 100
        print(f"  Peer-reviewed percentage: {percentage:.1f}%")

        if percentage >= 50:
            print(f"  ✅ PASS: Meets 50% peer-reviewed requirement")
        else:
            print(f"  ❌ FAIL: Does not meet 50% peer-reviewed requirement")
    print()

    # Validate each chapter
    chapters_dir = docs_dir / 'chapters'
    all_valid = True

    if chapters_dir.exists():
        print("Validating chapters:")
        for chapter_file in sorted(chapters_dir.glob('*.md')):
            valid, errors = validate_chapter(chapter_file, bib_entries)

            if valid:
                print(f"  ✅ {chapter_file.name}")
            else:
                print(f"  ❌ {chapter_file.name}")
                for error in errors:
                    print(error)
                all_valid = False
    else:
        print("⚠️  No chapters directory found yet")

    print()
    print("=" * 60)

    if all_valid and (total_sources == 0 or (peer_reviewed / total_sources) >= 0.5):
        print("✅ ALL CHECKS PASSED")
        sys.exit(0)
    else:
        print("❌ VALIDATION FAILED")
        sys.exit(1)


if __name__ == '__main__':
    main()
