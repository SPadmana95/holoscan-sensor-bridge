# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Generate adi_manifest.yaml for an ADI firmware bin file and its license.
# Supports local file paths or remote URLs for both the bin and license files.
#
# Usage:
#   python3 generate_adi_manifest.py -v <version> -b <path.bin> -l <path.txt>
#   python3 generate_adi_manifest.py -v <version> --bin-url <url> --license-url <url>
#

import argparse
import datetime
import hashlib
import os
import urllib.parse
import requests
import yaml


def measure(metadata, content):
    """Compute md5 and size from raw bytes and update the metadata dict."""
    md5 = hashlib.md5(content)
    metadata.update({
        "size": len(content),
        "md5": md5.hexdigest(),
    })


def fetch_url(url):
    """Download a file from a URL, return (filename, metadata, content).
    Raises an exception if the HTTP request fails."""
    p = urllib.parse.urlparse(url)
    image = p.path.split("/")[-1]
    response = requests.get(
        url,
        headers={"Content-Type": "binary/octet-stream"},
        timeout=120,
    )
    if response.status_code != requests.codes.ok:
        raise Exception(
            f'Unable to fetch "{url}"; status={response.status_code}'
        )
    content = response.content
    metadata = {"url": url}
    return image, metadata, content


def fetch_file(filepath):
    """Read a local file, return (filename, metadata, content)."""
    image = os.path.basename(filepath)
    with open(filepath, "rb") as f:
        content = f.read()
    metadata = {"filename": filepath}
    return image, metadata, content


def main():
    parser = argparse.ArgumentParser(
        description="Generate adi_manifest.yaml for an ADI firmware bin and license file.",
        epilog=(
            "Examples:\n"
            "  # Local files only (no url in manifest):\n"
            "  python3 generate_adi_manifest.py -v 1.0.0 -b Fw_Dual_Update_8.1.0.bin -l ADI_Software_License_Agreement.txt\n"
            "  # Local files + URLs (md5/size from local, url stored in manifest, no download):\n"
            "  python3 generate_adi_manifest.py -v 1.0.0 -b Fw_Dual_Update_8.1.0.bin --bin-url https://example.com/adi/1.0.0/Fw_Dual_Update_8.1.0.bin \\\n"
            "      -l ADI_Software_License_Agreement.txt --license-url https://example.com/adi/1.0.0/ADI_Software_License_Agreement.txt\n"
            "  # Remote URLs only (downloads to compute md5/size, url stored in manifest):\n"
            "  python3 generate_adi_manifest.py -v 1.0.0 \\\n"
            "      --bin-url https://example.com/adi/1.0.0/Fw_Dual_Update_8.1.0.bin \\\n"
            "      --license-url https://example.com/adi/1.0.0/ADI_Software_License_Agreement.txt\n"
            "  # Custom strategy and output:\n"
            "  python3 generate_adi_manifest.py -v 1.0.0 -b firmware.bin -l license.txt -s adi_tof -o my_manifest.yaml"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "-v", "--version",
        required=True,
        help='Firmware version string (e.g. "1.0.0").',
    )
    parser.add_argument(
        "-b", "--bin-file",
        default=None,
        metavar="BIN_FILE",
        help="Path to the ADI firmware .bin file (local). Can be combined with --bin-url.",
    )
    parser.add_argument(
        "--bin-url",
        default=None,
        metavar="BIN_URL",
        help="URL of the ADI firmware .bin file. If combined with --bin-file, "
             "the local file is used for md5/size and the URL is stored in the manifest "
             "(no download). If used alone, the file is downloaded to compute md5/size.",
    )
    parser.add_argument(
        "-l", "--license-file",
        default=None,
        metavar="LICENSE_FILE",
        help="Path to the license .txt file (local). Can be combined with --license-url.",
    )
    parser.add_argument(
        "--license-url",
        default=None,
        metavar="LICENSE_URL",
        help="URL of the license .txt file. If combined with --license-file, "
             "the local file is used for md5/size and the URL is stored in the manifest "
             "(no download). If used alone, the file is downloaded to compute md5/size.",
    )
    parser.add_argument(
        "-s", "--strategy",
        default="adi_tof",
        help='Strategy to use with this manifest (default: "adi_tof").',
    )
    parser.add_argument(
        "-o", "--output",
        default="adi_manifest.yaml",
        help='Output YAML file name (default: "adi_manifest.yaml").',
    )
    args = parser.parse_args()

    if not args.bin_file and not args.bin_url:
        parser.error("At least one of --bin-file or --bin-url must be specified.")
    if not args.license_file and not args.license_url:
        parser.error("At least one of --license-file or --license-url must be specified.")

    utc = datetime.timezone.utc
    now = datetime.datetime.now(utc)
    content = {}

    # --- bin file ---
    # Both provided: read local file for md5/size, use supplied URL (no download)
    if args.bin_file and args.bin_url:
        if not os.path.isfile(args.bin_file):
            parser.error(f"--bin-file: file not found: {args.bin_file}")
        bin_name, bin_meta, bin_bytes = fetch_file(args.bin_file)
        bin_meta["url"] = args.bin_url
        del bin_meta["filename"]
    elif args.bin_file:
        if not os.path.isfile(args.bin_file):
            parser.error(f"--bin-file: file not found: {args.bin_file}")
        bin_name, bin_meta, bin_bytes = fetch_file(args.bin_file)
    else:
        bin_name, bin_meta, bin_bytes = fetch_url(args.bin_url)
    measure(bin_meta, bin_bytes)
    content[bin_name] = bin_meta

    # --- license file ---
    # Both provided: read local file for md5/size, use supplied URL (no download)
    if args.license_file and args.license_url:
        if not os.path.isfile(args.license_file):
            parser.error(f"--license-file: file not found: {args.license_file}")
        license_name, license_meta, license_bytes = fetch_file(args.license_file)
        license_meta["url"] = args.license_url
        del license_meta["filename"]
    elif args.license_file:
        if not os.path.isfile(args.license_file):
            parser.error(f"--license-file: file not found: {args.license_file}")
        license_name, license_meta, license_bytes = fetch_file(args.license_file)
    else:
        license_name, license_meta, license_bytes = fetch_url(args.license_url)
    measure(license_meta, license_bytes)
    content[license_name] = license_meta

    manifest = {
        "hololink": {
            "archive": {
                "version": args.version,
                "enrollment_date": now.isoformat(),
            },
            "content": content,
            "images": [
                {
                    "content": bin_name,
                    "context": "adcam",
                }
            ],
            "licenses": [license_name],
            "strategy": args.strategy,
        }
    }

    with open(args.output, "wt") as f:
        yaml.dump(manifest, f, default_flow_style=False, sort_keys=False)

    print(f"Generated {args.output}")
    print(f"  {bin_name}: md5={bin_meta['md5']}  size={bin_meta['size']} bytes")
    print(f"  {license_name}: md5={license_meta['md5']}  size={license_meta['size']} bytes")


if __name__ == "__main__":
    main()
