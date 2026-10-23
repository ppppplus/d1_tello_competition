#!/usr/bin/env python3
"""Export an explicit contestant-only allowlist, with no Git history."""
import argparse
from pathlib import Path
import tarfile

FILES = ('CMakeLists.txt', 'package.xml', 'README.md', 'LICENSE',
         'scripts/contestant_node.py', 'docs/COMPETITION_RULES.md',
         'docs/map_top.png', 'docs/entry_facade.png')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    package = root / 'contestant/d1_competition_client'
    # Rules have one maintained source; fail on a stale distribution copy.
    assert (package / 'docs/COMPETITION_RULES.md').read_bytes() == (root / 'organizer/d1_tello_competition/docs/COMPETITION_RULES.md').read_bytes()
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(output, 'w:gz') as archive:
        for name in FILES:
            source = package / name
            if source.is_symlink() or not source.is_file():
                raise RuntimeError('Invalid distribution file: ' + name)
            archive.add(source, arcname='d1_competition_client/' + name, recursive=False)
    print(output)

if __name__ == '__main__':
    main()
