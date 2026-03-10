#!/usr/bin/env python3
import argparse
import re
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--current", required=True)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    input_blocks = Path(args.input).read_text(encoding="utf-8").split("\n\n")
    current_blocks = Path(args.current).read_text(encoding="utf-8").split("\n\n")
    if len(input_blocks) != len(current_blocks):
        raise RuntimeError("input/current block counts do not match")

    replace_with_source = {165, 276, 287, 296, 306, 315, 325, 328, 419, 434}
    deterministic_repairs = {
        111: (
            "As indicated in Figure 2.1, an inertial reference frame "
            "$\\Sigma_{i} = \\left\\lbrack {\\overrightarrow{e}}_{x},{\\overrightarrow{e}}_{y},{\\overrightarrow{e}}_{z} \\right\\rbrack$ "
            "is defined at the origin of the system and a non-inertial (or body) frame "
            "$\\Sigma_{b} = \\left\\lbrack {\\overrightarrow{e}}_{x}^{b},{\\overrightarrow{e}}_{y}^{b},{\\overrightarrow{e}}_{z}^{b} \\right\\rbrack$ "
            "located at the center of mass of the aircraft. Its position relative to the inertial reference frame is represented by "
            "$\\overrightarrow{r} = \\lbrack x,y,z\\rbrack^{T}$, the vector "
            "$\\overrightarrow{v} = \\lbrack u,v,w\\rbrack^{T}$ represents the linear velocity in the inertial reference frame, and the vector "
            "$\\overrightarrow{\\omega} = \\lbrack p,q,r\\rbrack^{T}$ represents its angular velocity in the non-inertial reference frame. "
            "The mass of the vehicle is represented by $M$, while $d$ is the distance between a rotor and its opposite, and $g$ is the gravitational acceleration."
        ),
        140: (
            "To simplify the disturbance notation in Eq. (2.19), making it more convenient for use in the controller, "
            "the disturbance terms that result in the accelerations are summarized as $d_{\\phi}$, $d_{\\theta}$ and $d_{\\psi}$."
        ),
        232: (
            "The goal of the SMC is to drive these variables to zero so that, once in this condition, "
            "the subsystem variables stabilize such that $z \\rightarrow z_{d}$ and $\\psi \\rightarrow \\psi_{d}$. "
            "In fact, when $s_{1} = 0$, one has:"
        ),
        444: (
            "Figure 4.2 - Example of a point-to-point 13th-degree polynomial trajectory with zero derivatives up to the sixth order "
            "for the waypoints $q_{d} = \\lbrack 3,0,2\\rbrack$ at the times $t_{d} = \\lbrack 0,5,10\\rbrack\\ s$."
        ),
        456: (
            "To obtain the two-impulse sequence as illustrated in Figure 4.3, one starts from the description of the residual vibration "
            "of a system with natural frequency $\\omega_{n}$ and damping factor $\\zeta$, given by:"
        ),
        625: (
            "This appendix presents the procedure for obtaining the kinematic transformations between the inertial coordinate system "
            "$\\Sigma_{i}$ and the non-inertial coordinate system $\\Sigma_{b}$, represented in the text by equations (2.1) and (2.2). "
            "It should be noted that the content of this section is based on (BRESCIANI, 2008)."
        ),
    }

    for idx in sorted(replace_with_source):
        current_blocks[idx] = input_blocks[idx]

    for idx, text in deterministic_repairs.items():
        current_blocks[idx] = text

    text = "\n\n".join(current_blocks)
    text = re.sub(r"\bFigura(?=\s+\d)", "Figure", text)
    text = re.sub(r"\bTabela(?=\s+\d)", "Table", text)
    text = re.sub(r"\bAp[eê]ndice(?=\s+[IVXLC\d])", "Appendix", text, flags=re.IGNORECASE)

    Path(args.output).write_text(text, encoding="utf-8")
    print(f"[OK] wrote repaired translation to {args.output}")


if __name__ == "__main__":
    main()
