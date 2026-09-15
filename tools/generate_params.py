#!/usr/bin/env python3
"""Validate a parameter YAML against a schema YAML and generate a C++ header."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Any

import yaml


INTEGER_TYPES: dict[str, tuple[int, int]] = {
    "int": (-(2**31), 2**31 - 1),
    "int8_t": (-(2**7), 2**7 - 1),
    "int16_t": (-(2**15), 2**15 - 1),
    "int32_t": (-(2**31), 2**31 - 1),
    "int64_t": (-(2**63), 2**63 - 1),
    "uint8_t": (0, 2**8 - 1),
    "uint16_t": (0, 2**16 - 1),
    "uint32_t": (0, 2**32 - 1),
    "uint64_t": (0, 2**64 - 1),
}
FLOAT_TYPES = {"float", "double"}
SUPPORTED_TYPES = {"bool", *FLOAT_TYPES, *INTEGER_TYPES}
DESCRIPTOR_KEYS = {"cpp_name", "cpp_type", "min", "max", "allowed"}
CPP_NAME_PATTERN = re.compile(r"k[A-Za-z][A-Za-z0-9_]*$")
NAMESPACE_PATTERN = re.compile(
    r"[A-Za-z_][A-Za-z0-9_]*(?:::[A-Za-z_][A-Za-z0-9_]*)*$"
)


def load_mapping(path: Path, description: str) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot load {description} {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"{description} root must be a mapping")
    return value


def flatten(value: Any, prefix: str = "") -> dict[str, Any]:
    if isinstance(value, dict):
        result: dict[str, Any] = {}
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise ValueError(f"parameter key at {prefix or '<root>'} must be a non-empty string")
            path = f"{prefix}.{key}" if prefix else key
            result.update(flatten(child, path))
        return result
    return {prefix: value}


def validate_schema(schema: dict[str, Any]) -> tuple[dict[str, dict[str, Any]], list[dict[str, str]]]:
    if schema.get("schema_version") != 1:
        raise ValueError("schema.schema_version must be 1")

    parameters = schema.get("parameters")
    if not isinstance(parameters, dict) or not parameters:
        raise ValueError("schema.parameters must be a non-empty mapping")

    cpp_names: set[str] = set()
    for path, descriptor in parameters.items():
        if not isinstance(path, str) or not path:
            raise ValueError("every schema parameter path must be a non-empty string")
        if not isinstance(descriptor, dict):
            raise ValueError(f"schema entry {path} must be a mapping")
        unknown_keys = set(descriptor) - DESCRIPTOR_KEYS
        if unknown_keys:
            raise ValueError(f"schema entry {path} has unknown keys: {sorted(unknown_keys)}")

        cpp_name = descriptor.get("cpp_name")
        cpp_type = descriptor.get("cpp_type")
        if not isinstance(cpp_name, str) or not CPP_NAME_PATTERN.fullmatch(cpp_name):
            raise ValueError(f"{path}.cpp_name is not a valid generated constant name")
        if cpp_name in cpp_names:
            raise ValueError(f"duplicate generated constant name: {cpp_name}")
        cpp_names.add(cpp_name)
        if cpp_type not in SUPPORTED_TYPES:
            raise ValueError(f"{path}.cpp_type is unsupported: {cpp_type!r}")
        if "allowed" in descriptor and not isinstance(descriptor["allowed"], list):
            raise ValueError(f"{path}.allowed must be a list")

    raw_constraints = schema.get("constraints", [])
    if not isinstance(raw_constraints, list):
        raise ValueError("schema.constraints must be a list")
    constraints: list[dict[str, str]] = []
    for index, constraint in enumerate(raw_constraints):
        if not isinstance(constraint, dict):
            raise ValueError(f"constraint {index} must be a mapping")
        if set(constraint) != {"kind", "left", "right"}:
            raise ValueError(f"constraint {index} must contain only kind, left and right")
        if constraint["kind"] != "less_than":
            raise ValueError(f"constraint {index} has unsupported kind: {constraint['kind']!r}")
        for operand in ("left", "right"):
            if constraint[operand] not in parameters:
                raise ValueError(f"constraint {index} references unknown parameter {constraint[operand]!r}")
        constraints.append(constraint)

    return parameters, constraints


def validate_scalar(path: str, value: Any, descriptor: dict[str, Any]) -> None:
    cpp_type = descriptor["cpp_type"]
    if cpp_type == "bool":
        if type(value) is not bool:
            raise ValueError(f"{path} must be a boolean")
    else:
        if type(value) not in (int, float) or not math.isfinite(float(value)):
            raise ValueError(f"{path} must be a finite number")
        if cpp_type in INTEGER_TYPES:
            if int(value) != value:
                raise ValueError(f"{path} must be an integer")
            type_minimum, type_maximum = INTEGER_TYPES[cpp_type]
            if value < type_minimum or value > type_maximum:
                raise ValueError(f"{path} does not fit in {cpp_type}")

    if "min" in descriptor and value < descriptor["min"]:
        raise ValueError(f"{path} must be >= {descriptor['min']}")
    if "max" in descriptor and value > descriptor["max"]:
        raise ValueError(f"{path} must be <= {descriptor['max']}")
    if "allowed" in descriptor and value not in descriptor["allowed"]:
        raise ValueError(f"{path} must be one of {descriptor['allowed']}")


def validate(
    values: dict[str, Any],
    parameters: dict[str, dict[str, Any]],
    constraints: list[dict[str, str]],
) -> None:
    missing = sorted(set(parameters) - set(values))
    unknown = sorted(set(values) - set(parameters))
    if missing or unknown:
        raise ValueError(f"missing fields={missing}; unknown fields={unknown}")

    for path, descriptor in parameters.items():
        validate_scalar(path, values[path], descriptor)

    for constraint in constraints:
        left = constraint["left"]
        right = constraint["right"]
        if not values[left] < values[right]:
            raise ValueError(f"{left} must be smaller than {right}")


def format_value(value: Any, cpp_type: str) -> str:
    if cpp_type == "bool":
        return "true" if value else "false"
    if cpp_type in FLOAT_TYPES:
        text = repr(float(value))
        if "." not in text and "e" not in text:
            text += ".0"
        return text + ("F" if cpp_type == "float" else "")
    suffix = "ULL" if cpp_type == "uint64_t" else "U" if cpp_type.startswith("uint") else ""
    return f"{int(value)}{suffix}"


def generate(
    values: dict[str, Any],
    parameters: dict[str, dict[str, Any]],
    namespace: str,
    input_name: str,
    schema_name: str,
) -> str:
    lines = [
        f"/* Generated from {input_name} and {schema_name}. Do not edit. */",
        "#pragma once",
        "",
        "#include <cstdint>",
        "",
        f"namespace {namespace}",
        "{",
    ]
    for path, descriptor in parameters.items():
        cpp_type = descriptor["cpp_type"]
        lines.append(
            f"inline constexpr {cpp_type} {descriptor['cpp_name']} = "
            f"{format_value(values[path], cpp_type)};"
        )
    lines.extend(("", f"}} // namespace {namespace}", ""))
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--schema", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--namespace", required=True)
    args = parser.parse_args()

    try:
        if not NAMESPACE_PATTERN.fullmatch(args.namespace):
            raise ValueError(f"invalid C++ namespace: {args.namespace!r}")
        schema = load_mapping(args.schema, "schema")
        parameters, constraints = validate_schema(schema)
        values = flatten(load_mapping(args.input, "parameter file"))
        validate(values, parameters, constraints)
    except ValueError as error:
        parser.error(str(error))

    output = generate(
        values, parameters, args.namespace, args.input.name, args.schema.name
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not args.output.exists() or args.output.read_text(encoding="utf-8") != output:
        args.output.write_text(output, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
