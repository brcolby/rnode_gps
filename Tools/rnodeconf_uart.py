#!/usr/bin/env python3
"""Run rnodeconf at the fork's UART baud without modifying its installed package."""
import argparse
import sys


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--baud', type=int, default=230400)
    args, rest = parser.parse_known_args()
    import RNS.Utilities.rnodeconf as rnodeconf
    rnodeconf.rnode_baudrate = args.baud
    sys.argv = [sys.argv[0], *rest]
    rnodeconf.main()


if __name__ == '__main__':
    main()
