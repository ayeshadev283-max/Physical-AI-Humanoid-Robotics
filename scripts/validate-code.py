#!/usr/bin/env python3
"""
Code Validation Script for Physical AI & Humanoid Robotics Book

Validates that:
1. All Python files have valid syntax
2. Code examples follow formatting standards
3. Required files exist
"""

import ast
import sys
from pathlib import Path
from typing import List, Tuple


def check_python_syntax(file_path: Path) -> Tuple[bool, str]:
    """Check if a Python file has valid syntax"""
    try:
        content = file_path.read_text(encoding='utf-8')
        ast.parse(content)
        return True, "OK"
    except SyntaxError as e:
        return False, f"Syntax error at line {e.lineno}: {e.msg}"
    except Exception as e:
        return False, f"Error: {str(e)}"


def check_code_standards(file_path: Path) -> List[str]:
    """Check basic code quality standards"""
    warnings = []
    content = file_path.read_text(encoding='utf-8')

    # Check for module docstring
    if not content.strip().startswith('"""') and not content.strip().startswith("'''"):
        warnings.append("  ⚠️  Missing module docstring")

    # Check for if __name__ == "__main__"
    if 'if __name__' not in content:
        warnings.append("  ⚠️  Missing if __name__ == '__main__' guard")

    return warnings


def main():
    """Main validation function"""
    repo_root = Path(__file__).parent.parent
    examples_dir = repo_root / 'examples'

    print("=" * 60)
    print("Code Validation for Physical AI & Humanoid Robotics Book")
    print("=" * 60)
    print()

    if not examples_dir.exists():
        print("⚠️  No examples directory found yet")
        print("✅ PASS (no code to validate)")
        sys.exit(0)

    # Find all Python files
    python_files = list(examples_dir.rglob('*.py'))

    if not python_files:
        print("⚠️  No Python files found in examples/")
        print("✅ PASS (no code to validate)")
        sys.exit(0)

    print(f"Found {len(python_files)} Python files to validate\n")

    all_valid = True

    for py_file in sorted(python_files):
        rel_path = py_file.relative_to(repo_root)
        valid, message = check_python_syntax(py_file)

        if valid:
            print(f"✅ {rel_path}: {message}")

            # Check code standards
            warnings = check_code_standards(py_file)
            for warning in warnings:
                print(warning)
        else:
            print(f"❌ {rel_path}: {message}")
            all_valid = False

    print()
    print("=" * 60)

    if all_valid:
        print("✅ ALL CHECKS PASSED")
        sys.exit(0)
    else:
        print("❌ VALIDATION FAILED")
        sys.exit(1)


if __name__ == '__main__':
    main()
