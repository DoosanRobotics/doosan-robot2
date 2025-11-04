#!/usr/bin/env python3
import os
import subprocess
import pathlib


def find_git_root_for_package(here: pathlib.Path):
    pkg_name = None
    for parent in here.parents:
        if parent.name == "lib":
            pkg_name = parent.parent.name
            break

    if pkg_name is None:
        print("[Git Info] Unable to infer package name from path.")
        return None

    ws_root = None
    for parent in here.parents:
        if parent.name == "install":
            ws_root = parent.parent
            break

    if ws_root is None:
        print("[Git Info] Unable to locate workspace root.")
        return None

    pkg_src_path = ws_root / "src" / pkg_name
    if not pkg_src_path.exists():
        print(f"[Git Info] {pkg_src_path} does not exist.")
        return None

    for child in [pkg_src_path, *pkg_src_path.parents]:
        if (child / ".git").exists():
            return child
    return None

def find_any_git_in_src(ws_root: pathlib.Path):
    src_dir = ws_root / "src"
    if not src_dir.exists():
        return None
    for child in src_dir.rglob('*'):
        if (child / ".git").exists():
            return child
    return None

def show_git_info():
    here = pathlib.Path(__file__).resolve()

    git_root = find_git_root_for_package(here)
    if git_root is None:
        for parent in here.parents:
            if parent.name == "install":
                ws_root = parent.parent
                git_root = find_any_git_in_src(ws_root)
                break

    if git_root is None:
        print("[Git Info] .git not found inside src/.")
        return {
            "commit": "unknown",
            "branch": "unknown",
            "user": "unknown",
            "email": "unknown",
        }

    def run(cmd):
        try:
            return subprocess.check_output(cmd, cwd=git_root).decode().strip()
        except Exception:
            return "unknown"

    info = {
        "commit": run(["git", "rev-parse", "--short", "HEAD"]),
        "branch": run(["git", "rev-parse", "--abbrev-ref", "HEAD"]),
        "user": run(["git", "config", "user.name"]),
        "email": run(["git", "config", "user.email"]),
    }
    print(f"\n[Git Info] {info['user']} <{info['email']}> | {info['branch']}@{info['commit']}\n")
    return info

if __name__ == "__main__":
    show_git_info()
