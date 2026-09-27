"""Entry point for the MaixCAM 2 Stage 1 person tracker."""

from application import TrackerApplication
from config import CONFIG


def main() -> None:
    application = TrackerApplication(CONFIG)
    application.run()


if __name__ == "__main__":
    main()
