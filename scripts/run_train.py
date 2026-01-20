"""Training entrypoint."""

import sys

from rl import train_cem


def main():
    train_cem.main(sys.argv[1:])


if __name__ == "__main__":
    main()
