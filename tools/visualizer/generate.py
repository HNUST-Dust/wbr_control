#!/usr/bin/env python3
"""Generate an offline WBR message and chassis-state architecture browser."""

from __future__ import annotations

import argparse
import json
import re
import shutil
from pathlib import Path


DIRECTIVE_RE = re.compile(r"^#\s+@(latest|queue|byte_stream|zbus)\s+(\w+)(?:\s+(\w+))?")
STRUCT_RE = re.compile(r"^#\s+@struct\s+(\w+)")
FIELD_RE = re.compile(r"^(\w+)(?:\[([^]]+)\])?\s+(\w+)$")
USAGE_RE = re.compile(
    r"msg::(?P<storage>\w+)\s*\.\s*(?P<method>"
    r"write|read|Write|Read|TryPush|Push|Pop|Publish|Subscribe)\s*\("
)
STATE_RE = re.compile(r"\bk(Disabled|SafetyStop|DmArming|WaitingFeedback|Recovery|Balance|Flight|Jump|ClimbStairs|ActionFault|TiltFault)\b")
REASON_RE = re.compile(r"ChassisTransitionReason::k(\w+)")
RETURN_RE = re.compile(r"return\s+ChassisControlState::k(\w+)")
CASE_RE = re.compile(r"case\s+ChassisControlState::k(\w+)")

PUBLISH_METHODS = {"write", "Write", "TryPush", "Push", "Publish"}
SUBSCRIBE_METHODS = {"read", "Read", "Pop", "Subscribe"}


def parse_messages(msg_dir: Path) -> tuple[list[dict], dict[str, str]]:
    messages: list[dict] = []
    storage_to_message: dict[str, str] = {}
    for path in sorted(msg_dir.glob("*.msg")):
        struct_name = path.stem
        fields: list[dict] = []
        transports: list[dict] = []
        for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            line = raw.strip()
            match = STRUCT_RE.match(line)
            if match:
                struct_name = match.group(1)
                continue
            match = DIRECTIVE_RE.match(line)
            if match:
                kind, name, detail = match.groups()
                item = {"kind": kind, "name": name, "line": number}
                if detail:
                    item["detail"] = detail
                transports.append(item)
                storage_to_message[name] = struct_name
                continue
            match = FIELD_RE.match(line)
            if match and not line.startswith("#"):
                field_type, array_size, name = match.groups()
                fields.append({"name": name, "type": field_type, "array": array_size, "line": number})
        messages.append({
            "name": struct_name,
            "file": path.as_posix(),
            "fields": fields,
            "transports": transports,
        })
    return messages, storage_to_message


def module_name(path: Path, root: Path) -> str:
    rel = path.relative_to(root)
    parts = rel.parts
    if len(parts) >= 3 and parts[:2] == ("src", "modules"):
        return parts[2]
    if len(parts) >= 3 and parts[:2] == ("src", "protocols"):
        return f"protocol:{parts[2]}"
    if len(parts) >= 3 and parts[:2] == ("platform", "drivers"):
        return f"platform:{path.stem}"
    if parts[0] == "debug":
        return f"debug:{path.stem}"
    if rel.as_posix() == "src/main.cpp":
        return "app"
    return path.parent.name


def scan_usages(root: Path, storage_to_message: dict[str, str]) -> tuple[list[dict], list[dict]]:
    usages: list[dict] = []
    warnings: list[dict] = []
    source_roots = [root / "src", root / "platform", root / "debug"]
    for source_root in source_roots:
        if not source_root.exists():
            continue
        for path in sorted(p for p in source_root.rglob("*") if p.suffix in {".c", ".cc", ".cpp", ".h", ".hpp"}):
            for number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
                for match in USAGE_RE.finditer(line):
                    storage, method = match.group("storage", "method")
                    if storage not in storage_to_message:
                        warnings.append({"file": path.as_posix(), "line": number, "text": f"Unknown storage {storage}"})
                        continue
                    direction = "publish" if method in PUBLISH_METHODS else "subscribe"
                    usages.append({
                        "module": module_name(path, root),
                        "message": storage_to_message[storage],
                        "storage": storage,
                        "direction": direction,
                        "method": method,
                        "file": path.as_posix(),
                        "line": number,
                    })
    return usages, warnings


def parse_states(header: Path, implementation: Path) -> dict:
    states: list[str] = []
    inside = False
    for line in header.read_text(encoding="utf-8").splitlines():
        if "enum class ChassisControlState" in line:
            inside = True
        elif inside and "};" in line:
            break
        elif inside:
            match = STATE_RE.search(line)
            if match:
                states.append(match.group(1))

    transitions: list[dict] = []
    current_sources: list[str] = []
    reason = "None"
    section_has_code = False
    recent: list[str] = []
    for number, line in enumerate(implementation.read_text(encoding="utf-8").splitlines(), 1):
        case = CASE_RE.search(line)
        if case:
            if section_has_code:
                current_sources = []
                recent = []
            current_sources.append(case.group(1))
            section_has_code = False
            continue
        if current_sources and line.strip() and not line.strip().startswith("//"):
            section_has_code = True
            recent.append(line)
            recent = recent[-6:]
        reason_match = REASON_RE.search(line)
        if reason_match:
            reason = reason_match.group(1)
        target = RETURN_RE.search(line)
        if target and current_sources:
            narrowed = []
            condition_text = " ".join(recent[:-1])
            for candidate in STATE_RE.finditer(condition_text):
                if candidate.group(1) in current_sources:
                    narrowed.append(candidate.group(1))
            sources = narrowed[-1:] or current_sources
            for source in sources:
                edge = {"from": source, "to": target.group(1), "reason": reason, "line": number}
                if edge not in transitions:
                    transitions.append(edge)
            reason = "None"
        elif current_sources and re.search(r"return\s+state_\s*;", line):
            current_sources = []
            recent = []
            section_has_code = False

    global_transitions = [
        {"to": "Disabled", "reason": "RemoteDisabled / RemoteTimeout"},
        {"to": "ActionFault", "reason": "Action fault latched"},
        {"to": "SafetyStop", "reason": "ImuLost"},
        {"to": "TiltFault", "reason": "TiltExceeded"},
    ]
    return {
        "states": states,
        "transitions": transitions,
        "globalTransitions": global_transitions,
        "file": implementation.as_posix(),
    }


def build(root: Path) -> dict:
    messages, storage_map = parse_messages(root / "msg")
    usages, warnings = scan_usages(root, storage_map)
    modules = sorted({usage["module"] for usage in usages})
    return {
        "schemaVersion": 1,
        "root": root.as_posix(),
        "modules": [{"name": name} for name in modules],
        "messages": messages,
        "usages": usages,
        "chassisStateMachine": parse_states(
            root / "src/modules/chassis/chassis_types.h",
            root / "src/modules/chassis/chassis_state_machine.cpp",
        ),
        "warnings": warnings,
    }


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=script_dir.parents[1])
    parser.add_argument("--output", type=Path, default=script_dir / "dist")
    args = parser.parse_args()
    root = args.root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    data = build(root)
    (output / "architecture.json").write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for name in ("index.html", "app.js", "style.css"):
        shutil.copyfile(script_dir / "web" / name, output / name)
    index_path = output / "index.html"
    index = index_path.read_text(encoding="utf-8")
    embedded = json.dumps(data, ensure_ascii=False).replace("</", "<\\/")
    index_path.write_text(index.replace("__ARCHITECTURE_JSON__", embedded), encoding="utf-8")
    print(f"Generated {output / 'index.html'}")
    print(f"{len(data['modules'])} modules, {len(data['messages'])} messages, {len(data['usages'])} usages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
