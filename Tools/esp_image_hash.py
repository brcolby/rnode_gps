#!/usr/bin/env python3

"""Print and validate the SHA-256 digest appended to an ESP application image."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


ESP_IMAGE_HEADER_BYTES = 24
ESP_IMAGE_HASH_BYTES = 32
ESP_IMAGE_HASH_APPENDED_OFFSET = 23


def validation_hash(image: bytes) -> bytes:
    minimum_length = ESP_IMAGE_HEADER_BYTES + ESP_IMAGE_HASH_BYTES
    if len(image) < minimum_length:
        raise ValueError("ESP application image is too short")
    if image[ESP_IMAGE_HASH_APPENDED_OFFSET] != 1:
        raise ValueError("ESP application image does not declare an appended hash")

    embedded = image[-ESP_IMAGE_HASH_BYTES:]
    calculated = hashlib.sha256(image[:-ESP_IMAGE_HASH_BYTES]).digest()
    if calculated != embedded:
        raise ValueError("ESP application image has an invalid appended hash")
    return embedded


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    args = parser.parse_args()

    try:
        print(validation_hash(args.image.read_bytes()).hex())
    except (OSError, ValueError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
