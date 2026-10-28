"""Pre-flight KRL block: stop before any motion if the controller's tool/base differ from the program's."""


def preflight_lines(tool: int, base: int, check_taught: bool = True) -> list[str]:
    """KRL lines to insert right after `BAS(#BASE, n)` (so $ACT_TOOL/$ACT_BASE reflect the selection).

    HALTs (program stops, operator may resume) when:
      - $ACT_TOOL / $ACT_BASE are not the numbers the program was generated for;
      - check_taught: TOOL_DATA[tool] / BASE_DATA[base] are all-zero (never taught).
    """
    out = [
        f"IF ($ACT_TOOL<>{tool}) OR ($ACT_BASE<>{base}) THEN",
        "  HALT  ;tool/base number differs from the cell config",
        "ENDIF",
    ]
    if check_taught:
        for var, n in (("TOOL_DATA", tool), ("BASE_DATA", base)):
            f = f"{var}[{n}]"
            out += [
                f"IF ({f}.X==0.0) AND ({f}.Y==0.0) AND ({f}.Z==0.0) AND ({f}.A==0.0) AND ({f}.B==0.0) AND ({f}.C==0.0) THEN",
                f"  HALT  ;{f} is not taught (all zero)",
                "ENDIF",
            ]
    return out
