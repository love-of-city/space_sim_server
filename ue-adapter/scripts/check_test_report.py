"""Reject empty, skipped, or failing JUnit runs in an explicit validation layer."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def check(path: Path) -> int:
    root = ET.parse(path).getroot()
    cases = list(root.iter('testcase'))
    if not cases or any(case.find(tag) is not None for case in cases for tag in ('skipped', 'failure', 'error')):
        raise ValueError('Validation did not execute a nonempty, fully passing test set')
    return len(cases)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    print(check(parser.parse_args().report))
