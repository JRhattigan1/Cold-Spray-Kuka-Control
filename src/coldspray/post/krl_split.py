"""Split a long KRL program (text from post.krl.to_krl) into subprograms.

Pure text transform. The main program keeps the header and the closing home move
and CALLs parts `<name>_P01`, `_P02`, ... Parts are cut only after a motion line that
is not blended (no C_DIS) while the gun output is off, so no spray is interrupted and
no approximated move straddles a file boundary. $VEL.* and BAS state persist across
CALL, so nothing is re-emitted.
"""
import re

_MOTION = re.compile(r"^(PTP|LIN)\b")
_GUN = re.compile(r"^\$OUT\[\d+\]\s*=\s*(TRUE|FALSE)\b")


class KRLSplitError(Exception):
    pass


def split_krl(text: str, max_lines: int = 2000) -> dict[str, str]:
    """Return {name: source}; the main program is first. One entry (the input) if short."""
    lines = text.split("\n")
    if len(lines) <= max_lines:
        m = re.search(r"^DEF\s+(\w+)", text, re.M)
        return {m.group(1) if m else "main": text}
    d = next((i for i, l in enumerate(lines) if l.startswith("DEF ")), None)
    if d is None:
        raise KRLSplitError("no DEF line")
    name = re.match(r"DEF\s+(\w+)", lines[d]).group(1)
    first = next((i for i, l in enumerate(lines) if _MOTION.match(l)), None)
    end = max(i for i, l in enumerate(lines) if l.strip() == "END")
    if first is None:
        raise KRLSplitError("no motion lines")
    # closing block: the final home PTP (and the blank line before it) stay in main
    last_motion = max(i for i in range(first, end) if _MOTION.match(lines[i]))
    tail_start = last_motion
    while tail_start > first and lines[tail_start - 1].strip() == "":
        tail_start -= 1
    head, body, tail = lines[:first], lines[first:tail_start], lines[tail_start:]
    # the first motion (home) is part of the head so every part starts mid-program
    head, body = head + [body[0]], body[1:]

    chunks, cur, gun = [], [], False
    for i, l in enumerate(body):
        cur.append(l)
        g = _GUN.match(l)
        if g:
            gun = g.group(1) == "TRUE"
        nxt_motion = i + 1 < len(body)
        if (len(cur) >= max_lines and _MOTION.match(l) and "C_DIS" not in l and not gun
                and nxt_motion):
            chunks.append(cur)
            cur = []
    if cur:
        chunks.append(cur)

    base = name[:20]
    names = [f"{base}_P{k + 1:02d}" for k in range(len(chunks))]
    out = {name: "\n".join(head + [f"{n}( )" for n in names] + tail)}
    for n, c in zip(names, chunks):
        out[n] = "\n".join(["&ACCESS RVP", "&REL 1", f"DEF {n}( )", *c, "END", ""])
    return out
