#!/usr/bin/env python3
"""打包 water-level-control skill 为 zip。

用法:
    python pack_skill.py                 # 生成 water-level-control.zip (仓库根目录)
    python pack_skill.py -o dist/x.zip   # 指定输出路径

zip 内以 water-level-control/ 为根目录 (SKILL.md + scripts/),
自动排除 __pycache__、*.pyc、编辑器临时文件等。
"""

import argparse
import os
import sys
import zipfile

# Windows GBK 控制台兼容: 无法编码的字符(如 ✅)替换输出而不是崩溃
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        try:
            _stream.reconfigure(errors="replace")
        except Exception:
            pass

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
SKILL_PARENT = os.path.join(REPO_ROOT, "skill")
SKILL_NAME = "water-level-control"

EXCLUDE_DIRS = {"__pycache__", ".git", ".idea", ".vscode"}
EXCLUDE_EXT = {".pyc", ".pyo", ".swp", ".tmp"}
EXCLUDE_PREFIX = ("~", ".")


def should_skip(name: str, is_dir: bool) -> bool:
    if is_dir:
        return name in EXCLUDE_DIRS or name.startswith(".")
    stem_ext = os.path.splitext(name)
    if stem_ext[1] in EXCLUDE_EXT:
        return True
    return name.startswith(EXCLUDE_PREFIX)


def main():
    ap = argparse.ArgumentParser(description="Pack the water-level-control skill into a zip")
    ap.add_argument("-o", "--out", default=os.path.join(REPO_ROOT, f"{SKILL_NAME}.zip"),
                    help=f"输出 zip 路径 (默认: 仓库根目录 {SKILL_NAME}.zip)")
    args = ap.parse_args()

    skill_dir = os.path.join(SKILL_PARENT, SKILL_NAME)
    if not os.path.isdir(skill_dir):
        print(f"Error: skill 目录不存在: {skill_dir}")
        sys.exit(1)

    # 校验关键文件
    for required in ("SKILL.md", os.path.join("scripts", "water_control_cli.py")):
        if not os.path.isfile(os.path.join(skill_dir, required)):
            print(f"Error: 缺少必需文件 {required}, 拒绝打包")
            sys.exit(1)

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    count = 0
    with zipfile.ZipFile(out_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, dirs, files in os.walk(skill_dir):
            dirs[:] = sorted(d for d in dirs if not should_skip(d, True))
            for fname in sorted(files):
                if should_skip(fname, False):
                    continue
                full = os.path.join(root, fname)
                rel = os.path.relpath(full, SKILL_PARENT)  # 保留 water-level-control/ 前缀
                zf.write(full, rel.replace(os.sep, "/"))
                count += 1
                print(f"  + {rel.replace(os.sep, '/')}")

    size = os.path.getsize(out_path)
    print(f"\n✅ 打包完成: {out_path}  ({count} 个文件, {size / 1024:.1f} KB)")


if __name__ == "__main__":
    main()
