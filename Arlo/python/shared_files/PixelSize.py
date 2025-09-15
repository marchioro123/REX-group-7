#!/usr/bin/env python3
"""
Usage examples:
  python image_size_px.py Example1.jpg
  python image_size_px.py ~/Pictures/*.jpg
  python image_size_px.py --csv sizes.csv Example1.jpg another.png
"""

import argparse
from pathlib import Path
from PIL import Image
import csv
import sys

def get_size(img_path: Path):
    """Open the image and return its width and height in pixels."""
    with Image.open(img_path) as im:
        w, h = im.size
    return w, h

def main():
    # Argument parser for command-line options
    ap = argparse.ArgumentParser(description="Get image sizes in pixels")
    ap.add_argument("paths", nargs="+", help="Path(s) to image files (wildcards *.jpg allowed)")
    ap.add_argument("--csv", help="Save the result to a CSV file")
    args = ap.parse_args()

    rows = []       # List to store results for optional CSV output
    exit_code = 0   # Exit code (0 = success, 1 = error)

    # Iterate over each path provided by the user
    for p in args.paths:
        # Support wildcards like *.jpg
        candidates = Path().glob(p) if any(c in p for c in "*?[]") else [Path(p)]
        for path in sorted(candidates):
            if not path.exists():
                # File does not exist
                print(f"[!] No file: {path}", file=sys.stderr)
                exit_code = 1
                continue
            try:
                # Get image size
                w, h = get_size(path)
                print(f"{path} -> {w} x {h} px")
                # Append result for CSV export
                rows.append({"file": str(path), "width_px": w, "height_px": h})
            except Exception as e:
                # Error opening or reading the image
                print(f"[!] Error reading {path}: {e}", file=sys.stderr)
                exit_code = 1

    # If requested, save results to a CSV file
    if args.csv and rows:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["file", "width_px", "height_px"])
            writer.writeheader()
            writer.writerows(rows)
        print(f"[i] Saved to {args.csv}")

    sys.exit(exit_code)

if __name__ == "__main__":
    main()
