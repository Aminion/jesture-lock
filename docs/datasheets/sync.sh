#!/usr/bin/env bash
# Check the hand-downloaded reference PDFs and build grep-able text copies.
#   ./sync.sh              list missing files, verify SHA256SUMS, run pdftotext
#   ./sync.sh --update-sums  rewrite SHA256SUMS from the files present (after a new revision)
set -euo pipefail
cd "$(dirname "$0")"

missing=0
present=()
while read -r id url; do
    [[ -z "$id" || "$id" == \#* ]] && continue
    pdf=$(basename "$url")
    if [[ ! -f "$pdf" ]]; then
        echo "missing $id: open $url in a browser, save as docs/datasheets/$pdf"
        missing=1
        continue
    fi
    present+=("$pdf")
    txt="${pdf%.pdf}.txt"
    if [[ ! -f "$txt" || "$pdf" -nt "$txt" ]]; then
        pdftotext -layout "$pdf" "$txt"
        echo "text    $id -> $txt"
    fi
done < manifest.txt

if [[ "${1:-}" == "--update-sums" ]]; then
    sha256sum "${present[@]}" > SHA256SUMS
    echo "wrote SHA256SUMS (${#present[@]} files)"
elif [[ -s SHA256SUMS ]]; then
    sha256sum --check --ignore-missing --quiet SHA256SUMS && echo "checksums OK"
fi
exit "$missing"
