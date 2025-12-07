#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2025 Physical AI & Humanoid Robotics Book Contributors
#
# Validate content quality for Physical AI & Humanoid Robotics Book
# Checks: peer-reviewed citation ratio, broken internal links, SPDX headers

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=================================================="
echo "Physical AI Book - Content Validation"
echo "=================================================="

# Check 1: Peer-reviewed citation ratio (must be >= 50%)
echo ""
echo "[1/3] Checking peer-reviewed citation ratio..."

BIB_FILE="$PROJECT_ROOT/references/physical-ai-book.bib"

if [ ! -f "$BIB_FILE" ]; then
    echo "ERROR: Bibliography file not found: $BIB_FILE"
    exit 1
fi

TOTAL_ENTRIES=$(grep -c "^@" "$BIB_FILE" || true)
PEER_REVIEWED=$(grep -c "peer-reviewed" "$BIB_FILE" || true)

if [ "$TOTAL_ENTRIES" -eq 0 ]; then
    echo "WARNING: No bibliography entries found"
    RATIO=0
else
    RATIO=$((PEER_REVIEWED * 100 / TOTAL_ENTRIES))
fi

echo "  Total entries: $TOTAL_ENTRIES"
echo "  Peer-reviewed: $PEER_REVIEWED"
echo "  Ratio: $RATIO%"

if [ "$RATIO" -lt 50 ]; then
    echo "  FAIL: Peer-reviewed ratio ($RATIO%) is below 50% threshold"
    exit 1
else
    echo "  PASS: Peer-reviewed ratio meets 50%+ requirement"
fi

# Check 2: SPDX headers in code examples
echo ""
echo "[2/3] Checking SPDX headers in code examples..."

MISSING_SPDX=0

while IFS= read -r -d '' file; do
    if ! grep -q "SPDX-License-Identifier: Apache-2.0" "$file"; then
        echo "  Missing SPDX header: $file"
        MISSING_SPDX=$((MISSING_SPDX + 1))
    fi
done < <(find "$PROJECT_ROOT/examples" -type f \( -name "*.py" -o -name "*.cpp" -o -name "*.hpp" \) -print0 2>/dev/null || true)

if [ "$MISSING_SPDX" -gt 0 ]; then
    echo "  FAIL: $MISSING_SPDX files missing SPDX headers"
    exit 1
else
    echo "  PASS: All code files have SPDX headers"
fi

# Check 3: Internal link validation
echo ""
echo "[3/3] Checking internal markdown links..."

BROKEN_LINKS=0

while IFS= read -r -d '' md_file; do
    # Extract internal links like [text](relative/path.md)
    while IFS= read -r link; do
        # Resolve relative path
        link_dir=$(dirname "$md_file")
        target="$link_dir/$link"
        
        if [ ! -f "$target" ]; then
            echo "  Broken link in $md_file: $link"
            BROKEN_LINKS=$((BROKEN_LINKS + 1))
        fi
    done < <(grep -oP '\[.*?\]\(\K[^)#]+\.md' "$md_file" 2>/dev/null || true)
done < <(find "$PROJECT_ROOT/docs" -type f -name "*.md" -print0 2>/dev/null || true)

if [ "$BROKEN_LINKS" -gt 0 ]; then
    echo "  FAIL: $BROKEN_LINKS broken internal links found"
    exit 1
else
    echo "  PASS: No broken internal links"
fi

echo ""
echo "=================================================="
echo "All content validation checks passed!"
echo "=================================================="
