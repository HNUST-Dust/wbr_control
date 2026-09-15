#!/usr/bin/env python3
"""Generate C++ message declarations and typed transports from lightweight .msg files."""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path

TYPE_MAP = {
    "bool": "bool",
    "int8": "int8_t",
    "uint8": "uint8_t",
    "int16": "int16_t",
    "uint16": "uint16_t",
    "int32": "int32_t",
    "uint32": "uint32_t",
    "int64": "int64_t",
    "uint64": "uint64_t",
    "float32": "float",
    "float64": "double",
}
FIELD_PATTERN = re.compile(
    r"(?P<type>[a-z][a-z0-9]*)(?:\[(?P<count>[A-Za-z_][A-Za-z0-9_]*|[1-9][0-9]*)\])?\s+"
    r"(?P<name>[a-z][a-z0-9_]*)$"
)
CPP_NAME_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*$")


@dataclass(frozen=True)
class Field:
    cpp_type: str
    name: str
    count: int | str | None


@dataclass(frozen=True)
class Message:
    path: Path
    struct_name: str
    latest: tuple[str, ...]
    queues: tuple[tuple[str, int], ...]
    streams: tuple[tuple[str, int], ...]
    zbus: tuple[tuple[str, str], ...]
    enums: tuple[tuple[str, str, tuple[tuple[str, int], ...]], ...]
    constants: tuple[tuple[str, str, int], ...]
    fields: tuple[Field, ...]
    expected_size: int | None


def parse_message(path: Path) -> Message:
    struct_name: str | None = None
    latest: list[str] = []
    queues: list[tuple[str, int]] = []
    streams: list[tuple[str, int]] = []
    zbus: list[tuple[str, str]] = []
    enums: list[tuple[str, str, list[tuple[str, int]]]] = []
    constants: list[tuple[str, str, int]] = []
    current_enum: list[tuple[str, int]] | None = None
    fields: list[Field] = []
    expected_size: int | None = None

    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            directive = line[1:].strip()
            if directive.startswith("@struct "):
                struct_name = directive.removeprefix("@struct ").strip()
                if not CPP_NAME_PATTERN.fullmatch(struct_name):
                    raise ValueError(f"{path}:{line_number}: invalid struct name")
            elif directive.startswith("@latest "):
                channel = directive.removeprefix("@latest ").strip()
                if not CPP_NAME_PATTERN.fullmatch(channel):
                    raise ValueError(f"{path}:{line_number}: invalid channel name")
                latest.append(channel)
            elif directive.startswith("@queue "):
                parts = directive.split()
                if len(parts) != 3 or not CPP_NAME_PATTERN.fullmatch(parts[1]) or not parts[2].isdecimal():
                    raise ValueError(f"{path}:{line_number}: expected '# @queue name depth'")
                queues.append((parts[1], int(parts[2])))
            elif directive.startswith("@byte_stream "):
                parts = directive.split()
                if len(parts) != 3 or not CPP_NAME_PATTERN.fullmatch(parts[1]) or not parts[2].isdecimal():
                    raise ValueError(f"{path}:{line_number}: expected '# @byte_stream name capacity'")
                streams.append((parts[1], int(parts[2])))
            elif directive.startswith("@zbus "):
                parts = directive.split()
                if len(parts) != 3 or not all(CPP_NAME_PATTERN.fullmatch(part) for part in parts[1:]):
                    raise ValueError(f"{path}:{line_number}: expected '# @zbus instance raw_channel'")
                zbus.append((parts[1], parts[2]))
            elif directive.startswith("@enum "):
                parts = directive.split()
                if len(parts) != 3 or not CPP_NAME_PATTERN.fullmatch(parts[1]) or parts[2] not in TYPE_MAP:
                    raise ValueError(f"{path}:{line_number}: expected '# @enum Name uint8'")
                current_enum = []
                enums.append((parts[1], TYPE_MAP[parts[2]], current_enum))
            elif directive.startswith("@value "):
                parts = directive.split()
                if current_enum is None or len(parts) != 3 or not CPP_NAME_PATTERN.fullmatch(parts[1]):
                    raise ValueError(f"{path}:{line_number}: '@value Name integer' must follow @enum")
                try:
                    current_enum.append((parts[1], int(parts[2], 0)))
                except ValueError as error:
                    raise ValueError(f"{path}:{line_number}: invalid enum value") from error
            elif directive.startswith("@constant "):
                parts = directive.split()
                constant_types = {**TYPE_MAP, "size_t": "size_t"}
                if len(parts) != 4 or parts[1] not in constant_types or not CPP_NAME_PATTERN.fullmatch(parts[2]):
                    raise ValueError(f"{path}:{line_number}: expected '# @constant type Name integer'")
                try:
                    constants.append((constant_types[parts[1]], parts[2], int(parts[3], 0)))
                except ValueError as error:
                    raise ValueError(f"{path}:{line_number}: invalid constant value") from error
            elif directive.startswith("@size "):
                size = directive.removeprefix("@size ").strip()
                if not size.isdecimal() or int(size) <= 0:
                    raise ValueError(f"{path}:{line_number}: invalid expected size")
                expected_size = int(size)
            continue

        match = FIELD_PATTERN.fullmatch(line)
        if match is None:
            raise ValueError(f"{path}:{line_number}: invalid field declaration: {line!r}")
        source_type = match.group("type")
        if source_type not in TYPE_MAP:
            raise ValueError(f"{path}:{line_number}: unsupported type {source_type!r}")
        name = match.group("name")
        if any(field.name == name for field in fields):
            raise ValueError(f"{path}:{line_number}: duplicate field {name!r}")
        raw_count = match.group("count")
        count = int(raw_count) if raw_count and raw_count.isdecimal() else raw_count
        if isinstance(count, str) and count not in {name for _, name, _ in constants}:
            raise ValueError(f"{path}:{line_number}: unknown array constant {count!r}")
        fields.append(Field(TYPE_MAP[source_type], name, count))

    if struct_name is None:
        raise ValueError(f"{path}: missing '# @struct Name'")
    if not latest and not queues and not streams and not zbus:
        raise ValueError(f"{path}: at least one transport instance is required")
    if len(set(latest)) != len(latest):
        raise ValueError(f"{path}: duplicate channel name")
    if not fields:
        raise ValueError(f"{path}: message has no fields")
    frozen_enums = tuple((name, base, tuple(values)) for name, base, values in enums)
    return Message(path, struct_name, tuple(latest), tuple(queues), tuple(streams), tuple(zbus),
                   frozen_enums, tuple(constants), tuple(fields), expected_size)


def make_header(message: Message) -> str:
    lines = [
        "/* Generated from " + message.path.name + ". Do not edit. */",
        "#pragma once",
        "",
        "#include <cstdint>",
        "#include <cstddef>",
        "#include <msg/transport/latest_value.hpp>",
        "",
        "namespace msg {",
        "",
    ]
    if message.queues:
        lines.insert(5, "#include <msg/transport/message_queue.hpp>")
    if message.streams:
        lines.insert(5, "#include <msg/transport/byte_stream.hpp>")
    if message.zbus:
        lines.insert(5, "#include <msg/transport/zbus_message.hpp>")
    for enum_name, base_type, values in message.enums:
        lines.append(f"enum {enum_name} : {base_type} {{")
        for value_name, value in values:
            lines.append(f"\t{value_name} = {value},")
        lines.extend(("};", ""))
    for cpp_type, name, value in message.constants:
        suffix = "U" if cpp_type.startswith("uint") or cpp_type == "size_t" else ""
        lines.append(f"inline constexpr {cpp_type} {name} = {value}{suffix};")
    if message.constants:
        lines.append("")
    lines.append(f"struct {message.struct_name} {{")
    for field in message.fields:
        suffix = f"[{field.count}]" if field.count is not None else ""
        lines.append(f"\t{field.cpp_type} {field.name}{suffix};")
    lines.append("};")
    if message.expected_size is not None:
        lines.extend((
            "",
            f"static_assert(sizeof({message.struct_name}) == {message.expected_size}U,",
            f'\t      "Keep {message.struct_name} wire snapshot layout stable");',
        ))
    lines.append("")
    for channel in message.latest:
        lines.append(f"extern LatestValue<{message.struct_name}> {channel};")
    for name, depth in message.queues:
        lines.append(f"extern MessageQueue<{message.struct_name}, {depth}> {name};")
    for name, capacity in message.streams:
        lines.append(f"extern ByteStream<{capacity}> {name};")
    for name, raw_channel in message.zbus:
        lines.append(f"extern ZbusMessage<{message.struct_name}> {name};")
    lines.extend(("", "} // namespace msg", ""))
    for _, raw_channel in message.zbus:
        lines.append(f"ZBUS_CHAN_DECLARE({raw_channel});")
    return "\n".join(lines)


def make_source(messages: list[Message]) -> str:
    lines = ["/* Generated message storage. Do not edit. */", ""]
    for message in messages:
        lines.append(f"#include <msg/{message.path.stem}.hpp>")
    lines.extend(("", "namespace msg {", ""))
    for message in messages:
        for channel in message.latest:
            lines.append(f"LatestValue<{message.struct_name}> {channel};")
        for name, depth in message.queues:
            lines.append(f"MessageQueue<{message.struct_name}, {depth}> {name};")
        for name, capacity in message.streams:
            lines.append(f"ByteStream<{capacity}> {name};")
    lines.extend(("", "} // namespace msg", ""))
    for message in messages:
        for name, raw_channel in message.zbus:
            lines.extend((
                f"ZBUS_CHAN_DEFINE({raw_channel}, msg::{message.struct_name}, NULL, NULL,",
                "\tZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));",
                f"msg::ZbusMessage<msg::{message.struct_name}> msg::{name}(&{raw_channel});",
            ))
    return "\n".join(lines)


def write_if_changed(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text(encoding="utf-8") != content:
        path.write_text(content, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--source-output", required=True, type=Path)
    parser.add_argument("files", nargs="+", type=Path)
    args = parser.parse_args()

    try:
        messages = [parse_message(path) for path in args.files]
        channel_names = [
            name for message in messages
            for name in (
                *message.latest,
                *(name for name, _ in message.queues),
                *(name for name, _ in message.streams),
                *(name for name, _ in message.zbus),
            )
        ]
        if len(set(channel_names)) != len(channel_names):
            raise ValueError("channel instance names must be unique across all message files")
    except (OSError, ValueError) as error:
        parser.error(str(error))

    for message in messages:
        write_if_changed(args.output_dir / f"{message.path.stem}.hpp", make_header(message))
    write_if_changed(args.source_output, make_source(messages))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
