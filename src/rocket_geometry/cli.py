"""Compatibility CLI for the former ``rocket-geometry`` command."""

from __future__ import annotations

import warnings
import argparse
from pathlib import Path

from rocket_tool.cli import main as tool_main


def main(argv: list[str] | None = None) -> int:
    warnings.warn("rocket-geometry is deprecated; use rocket-tool", DeprecationWarning, stacklevel=2)
    if argv is None:
        import sys

        argv = sys.argv[1:]
    if argv and argv[0] in {"validate", "render", "app"}:
        command = "edit" if argv[0] == "app" else argv[0]
        return tool_main(["model", command, *argv[1:]])
    if argv and argv[0] in {"validate-simulation", "simulate"}:
        from rocket_sim import load_simulation_inputs, run_simulation, write_simulation_outputs

        parser = argparse.ArgumentParser(prog=f"rocket-geometry {argv[0]}")
        parser.add_argument("path", type=Path)
        parser.add_argument("--scenario", type=Path, required=True)
        if argv[0] == "simulate":
            parser.add_argument("--output", type=Path, required=True)
            parser.add_argument("--overwrite", action="store_true")
        arguments = parser.parse_args(argv[1:])
        inputs = load_simulation_inputs(arguments.path, arguments.scenario)
        if argv[0] == "validate-simulation":
            print(f"Valid legacy simulation inputs: {inputs.rocket.rocket.name}")
            return 0
        result = run_simulation(inputs)
        write_simulation_outputs(inputs, result, arguments.output, overwrite=arguments.overwrite)
        print(f"Simulation {result.status}; wrote {arguments.output}")
        return 0
    return tool_main(argv)
