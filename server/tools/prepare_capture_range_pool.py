"""Sample a frozen calibrated capture domain for the standard zmh batch runner."""
import argparse
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT), str(ROOT/'backend')]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--model', type=Path, default=ROOT/'model/SARM/platform/sarm_dual_capture26.xml')
    parser.add_argument('--catalog', type=Path, required=True)
    args=parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    from collection.capture_initial_domain import make_range_pool
    pool=make_range_pool(args.domain,args.output,model=args.model,catalog=args.catalog)
    print(f"Prepared {len(pool['layouts'])} random initial states; no repeated physical run before recording")


if __name__=='__main__': main()
