#!/usr/bin/env python3
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
errors = []
for path in ROOT.rglob('*'):
    if path.suffix.lower() not in {'.h', '.hpp', '.cpp', '.cc', '.cxx'}:
        continue
    rel_parts = path.relative_to(ROOT).parts
    if any(part.startswith('build') or part == 'third_party' for part in rel_parts):
        continue
    text = path.read_text(encoding='utf-8', errors='ignore')
    for n, line in enumerate(text.splitlines(), 1):
        # Catch the common namespace typo that MSVC can recover as an implicit int return type.
        if re.search(r'(?<!p)ceditor::', line):
            errors.append(f'{path.relative_to(ROOT)}:{n}: suspicious namespace typo: {line.strip()}')

if errors:
    print('\n'.join(errors))
    sys.exit(1)
print('source sanity check: PASS')
