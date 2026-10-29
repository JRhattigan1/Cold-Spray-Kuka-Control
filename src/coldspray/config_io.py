"""Targeted edits of the YAML config files that keep every other line and comment."""
import re
from pathlib import Path

from coldspray.models import CONFIG_DIR, PurgeConfig

_COMMENTS = dict(point="nozzle exit target, WORLD mm", spray_dir="WORLD spray direction",
                 dwell_s="spray on at the point before moving off",
                 every_n_layers="0: only at start / end",
                 approach_mm="lead-in, back from the point along spray_dir")


def _yaml_value(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (tuple, list)):
        return "[" + ", ".join(repr(float(x)) for x in v) + "]"
    return repr(v)


def purge_block(purge: PurgeConfig, nl: str = "\n") -> list[str]:
    """The `purge:` block as YAML lines (with line endings)."""
    out = ["purge:" + nl]
    for name, v in purge.model_dump().items():
        line = f"  {name}: {_yaml_value(v)}"
        if name in _COMMENTS:
            line = f"{line:<36} # {_COMMENTS[name]}"
        out.append(line + nl)
    return out


def _find_block(lines: list[str]) -> tuple[int, int] | None:
    """(start, end) of the live `purge:` block, else of the commented example."""
    for i, line in enumerate(lines):
        if re.match(r"purge:\s*(#.*)?$", line.rstrip("\r\n")):
            j = i + 1
            while j < len(lines) and re.match(r"[ \t]+\S", lines[j]):
                j += 1
            return i, j
    for i, line in enumerate(lines):
        if re.match(r"#\s*purge:\s*$", line.rstrip("\r\n")):
            j = i + 1
            while j < len(lines) and re.match(r"#\s{2,}\S", lines[j]):
                j += 1
            return i, j
    return None


def save_purge(purge: PurgeConfig, path=None) -> Path:
    """Rewrite only the `purge:` block of cell.yaml: replaces a live block, else the
    commented example block, else appends one. All other bytes are kept."""
    path = Path(path or CONFIG_DIR / "cell.yaml")
    with open(path, newline="") as f:
        text = f.read()
    nl = "\r\n" if "\r\n" in text else "\n"
    lines = text.splitlines(keepends=True)
    block = purge_block(purge, nl)
    span = _find_block(lines)
    if span is None:
        if lines and not lines[-1].endswith(("\n", "\r")):
            lines[-1] += nl
        lines += [nl, *block]
    else:
        lines[span[0]:span[1]] = block
    with open(path, "w", newline="") as f:
        f.write("".join(lines))
    return path


def save_home(home_deg, path=None) -> Path:
    """Rewrite only the `home_deg:` line of cell.yaml (a trailing comment is kept);
    appends one if missing. All other bytes are kept."""
    path = Path(path or CONFIG_DIR / "cell.yaml")
    with open(path, newline="") as f:
        text = f.read()
    nl = "\r\n" if "\r\n" in text else "\n"
    lines = text.splitlines(keepends=True)
    value = "[" + ", ".join(f"{float(x):g}" for x in home_deg) + "]"
    for i, line in enumerate(lines):
        m = re.match(r"home_deg:\s*\[[^\]]*\](\s*#.*)?$", line.rstrip("\r\n"))
        if m:
            lines[i] = f"home_deg: {value}{m.group(1) or ''}{nl}"
            break
    else:
        if lines and not lines[-1].endswith(("\n", "\r")):
            lines[-1] += nl
        lines.append(f"home_deg: {value}{nl}")
    with open(path, "w", newline="") as f:
        f.write("".join(lines))
    return path


def _flow(v) -> str:
    """XYZABC as a one-line YAML mapping."""
    return "{" + ", ".join(f"{k}: {float(x):g}" for k, x in v.model_dump().items()) + "}"


def _rewrite(path, edit) -> Path:
    path = Path(path)
    with open(path, newline="") as f:
        text = f.read()
    nl = "\r\n" if "\r\n" in text else "\n"
    lines = edit(text.splitlines(keepends=True), nl)
    with open(path, "w", newline="") as f:
        f.write("".join(lines))
    return path


def save_tools(tool, path=None) -> Path:
    """Write each nozzle's `tool_number` and taught `tcp` into tool.yaml (other lines kept).
    A nozzle with no taught tcp gets no `tcp:` line: the CAD TCP is used."""
    def edit(lines, nl):
        for name, nz in tool.gun.nozzles.items():
            start = next(i for i, ln in enumerate(lines) if re.match(rf"\s+{re.escape(name)}:\s*$", ln.rstrip("\r\n")))
            ind = len(lines[start]) - len(lines[start].lstrip())
            j = start + 1
            while j < len(lines) and lines[j].strip() and len(lines[j]) - len(lines[j].lstrip()) > ind:
                j += 1
            old = [re.match(r"\s+tool_number:\s*[^\s#]*(\s*#.*)?$", ln.rstrip("\r\n")) for ln in lines[start + 1:j]]
            note = next((m.group(1) for m in old if m and m.group(1)), "")   # keep the line's comment
            body = [ln for ln in lines[start + 1:j] if not re.match(r"\s+(tool_number|tcp):", ln)]
            pad = " " * (ind + 2)
            if nz.tool_number is not None:
                body.append(f"{pad}tool_number: {nz.tool_number}{note}{nl}")
            if nz.tcp is not None:
                body.append(f"{pad}tcp: {_flow(nz.tcp)}   # taught TOOL_DATA{nl}")
            lines[start + 1:j] = body
        return lines
    return _rewrite(path or CONFIG_DIR / "tool.yaml", edit)


def save_bases(bases: dict, path=None) -> Path:
    """Write the `bases_taught:` block (BASE_DATA from the pendant) into cell.yaml."""
    def edit(lines, nl):
        block = ["bases_taught:" + (" {}" if not bases else "") + nl]
        block += [f"  {n}: {_flow(v)}{nl}" for n, v in sorted(bases.items())]
        for i, ln in enumerate(lines):
            if re.match(r"bases_taught:", ln):
                j = i + 1
                while j < len(lines) and re.match(r"[ \t]+\S", lines[j]):
                    j += 1
                lines[i:j] = block
                return lines
        if not bases:
            return lines
        if lines and not lines[-1].endswith(("\n", "\r")):
            lines[-1] += nl
        return lines + block
    return _rewrite(path or CONFIG_DIR / "cell.yaml", edit)
