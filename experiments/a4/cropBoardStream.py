from pathlib import Path
import sys
import time

from PIL import Image


def main() -> int:
    if len(sys.argv) != 7:
        print("Usage: cropBoardStream.py INPUT OUTPUT LEFT TOP RIGHT BOTTOM", file=sys.stderr)
        return 2

    input_file = Path(sys.argv[1])
    output_file = Path(sys.argv[2])
    box = tuple(int(value) for value in sys.argv[3:7])

    last_mtime = 0.0
    output_file.parent.mkdir(parents=True, exist_ok=True)

    while True:
        try:
            if input_file.exists():
                mtime = input_file.stat().st_mtime
                if mtime > last_mtime:
                    with Image.open(input_file) as image:
                        image.crop(box).save(output_file, quality=95)
                    last_mtime = mtime
        except Exception as exc:
            print(f"cropBoardStream warning: {exc}", file=sys.stderr, flush=True)

        time.sleep(0.15)


if __name__ == "__main__":
    raise SystemExit(main())
