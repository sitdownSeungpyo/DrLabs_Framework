"""Turns a failed `cargo test` log into one GitHub Actions annotation.

Readable from the Checks tab and from the public check-runs API without
auth, which the raw job log is not. Prints the `failures:` block when a test
failed, the compiler or build-script errors when the build did, and the log
tail either way. dtolnay/rust-toolchain sets CARGO_TERM_COLOR=always, so
the ANSI codes are stripped first.

    python3 .github/scripts/annotate_failure.py test.log
"""

import re
import sys

log = open(sys.argv[1], encoding="utf-8", errors="replace").read()
log = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", log).replace("\r", "")
lines = log.split("\n")
out = []
if "failures:" in lines:
    i = lines.index("failures:")
    out += ["== failures =="] + [l for l in lines[i:i + 80] if l]
errs = [n for n, l in enumerate(lines)
        if re.match(r"^(error|warning: build failed|CMake Error|  --- stderr)", l)]
for n in errs[:6]:
    out += ["== %s ==" % lines[n][:80]] + lines[n:n + 14]
out += ["== last 40 lines =="] + lines[-40:]
text = "\n".join(out)[-3800:]
print("::error::" + text.replace("%", "%25").replace("\n", "%0A"))
