import pytest

from coldspray.post.krl_split import split_krl


def _prog(n, spray=()):
    L = ["&ACCESS RVP", "&REL 1", "DEF demo( )", ";hdr", "$OUT[1] = FALSE", "", "PTP {A1 0}  ;home"]
    for i in range(n):
        if i in spray:
            L += ["WAIT SEC 0", "$OUT[1] = TRUE" if spray[i] else "$OUT[1] = FALSE"]
        L.append(f"LIN {{X {i}}}" + (" C_DIS" if i % 3 == 0 else ""))
    L += ["", "PTP {A1 0}  ;home", "END", ""]
    return "\n".join(L)


def test_short_unchanged():
    t = _prog(5)
    assert split_krl(t, 100) == {"demo": t}


def test_split_preserves_motion_order():
    t = _prog(100)
    parts = split_krl(t, 20)
    assert len(parts) > 3
    main = parts["demo"]
    assert main.rstrip().endswith("END") and main.count("PTP") == 2
    seq = []
    for l in main.split("\n"):
        if l.startswith("demo_P"):
            seq += [x for x in parts[l.split("(")[0]].split("\n") if x.startswith("LIN")]
    assert seq == [x for x in t.split("\n") if x.startswith("LIN")]


def test_no_cut_while_spraying_or_blended():
    t = _prog(60, spray={10: True, 50: False})
    parts = split_krl(t, 10)
    names = [n for n in parts if n != "demo"]
    for n in names[:-1]:
        body = parts[n].split("\n")
        assert "C_DIS" not in [l for l in body if l.startswith("LIN")][-1]
    # spray on at 10, off at 50: no part boundary strictly between
    on = next(n for n in names if "$OUT[1] = TRUE" in parts[n])
    assert "$OUT[1] = FALSE" in parts[on].split("$OUT[1] = TRUE")[1] or on == names[-1] or \
        any("$OUT[1] = FALSE" in parts[on].split("$OUT[1] = TRUE")[1] for _ in [0])
    assert all(len(n) <= 24 for n in parts)


def test_circ_is_a_motion_line():
    """A blended CIRC is never the last line of a part, and CIRCs stay in order."""
    L = ["&ACCESS RVP", "&REL 1", "DEF demo( )", "$OUT[1] = FALSE", "", "PTP {A1 0}  ;home"]
    for i in range(60):
        L.append(f"CIRC {{X {i}}}, {{X {i}.5}} C_DIS" if i % 2 else f"LIN {{X {i}}}")
    L += ["", "PTP {A1 0}  ;home", "END", ""]
    t = "\n".join(L)
    parts = split_krl(t, 10)
    assert len(parts) > 3
    seq = []
    for n, body in parts.items():
        if n == "demo":
            continue
        motion = [l for l in body.split("\n") if l.startswith(("LIN", "CIRC"))]
        assert "C_DIS" not in motion[-1] or n == list(parts)[-1]
        seq += motion
    assert seq == [l for l in L if l.startswith(("LIN", "CIRC"))]
