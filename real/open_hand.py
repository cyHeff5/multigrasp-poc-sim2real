"""Open the real AR10 hand to the maximum target on all joints."""

import time

from real.AR10_Extended import hand


def main():
    ar10 = hand()
    ar10.open()
    time.sleep(2.0)
    ar10.close()


if __name__ == "__main__":
    main()
