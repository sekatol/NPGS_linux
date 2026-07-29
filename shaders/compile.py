#!/usr/bin/env python3
"""Compile GLSL shaders to SPIR-V using glslc."""

import json
import hashlib
import os
import subprocess
import sys
from pathlib import Path

SHADER_SRC  = Path(__file__).resolve().parent / ".." / "src" / "Engine" / "Shaders"
SHADER_OUT  = Path(__file__).resolve().parent / ".." / "assets" / "Shaders"
META_FILE   = SHADER_OUT / "ShaderMeta.json"

# Variants defined in original CompileShaders.cfg
VARIANTS = [
    ("Scene.vert.glsl", "Scene.vert.spv", []),
    ("Scene.frag.glsl", "Scene.frag.spv", []),
    ("Scene.frag.glsl", "Scene_Lamp.frag.spv", ["LAMP_BOX"]),
    ("Bloom.frag.glsl", "PreBloom.frag.spv", ["GENERATE_MIPMAP"]),
    ("Bloom.frag.glsl", "GaussBlur.frag.spv", ["GAUSS_BLUR"]),
    ("Bloom.comp.glsl", "PreBloom.comp.spv", ["GENERATE_MIPMAP"]),
    ("Bloom.comp.glsl", "GaussBlur.comp.spv", ["GAUSS_BLUR"]),
]

# Non-variant shaders (compile source name -> output .spv name)
NON_VARIANT_SHADERS = {
    "ScreenQuad.vert.glsl": "ScreenQuad.vert.spv",
    "BlackHole_prepass.frag.glsl": "BlackHole_prepass.frag.spv",
    "BlackHole_composite.frag.glsl": "BlackHole_composite.frag.spv",
    "BlackHole.frag.glsl": "BlackHole.frag.spv",
    "BlackHole.comp.glsl": "BlackHole.comp.spv",
    "ColorBlend.frag.glsl": "ColorBlend.frag.spv",
    "Skybox.vert.glsl": "Skybox.vert.spv",
    "Skybox.frag.glsl": "Skybox.frag.spv",
    "PostProcess.vert.glsl": "PostProcess.vert.spv",
    "PostProcess.frag.glsl": "PostProcess.frag.spv",
    "ShadowMap.vert.glsl": "ShadowMap.vert.spv",
    "ShadowMap.frag.glsl": "ShadowMap.frag.spv",
}

def get_included_files(source_path, visited=None):
    """Recursively find all files #included by a GLSL file."""
    if visited is None:
        visited = set()
    if source_path in visited:
        return visited
    visited.add(source_path)
    if not source_path.exists():
        return visited
    try:
        with open(source_path, "r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith('#include "'):
                    inc_name = stripped.split('"')[1]
                    inc_path = source_path.parent / inc_name
                    if inc_path.exists():
                        get_included_files(inc_path.resolve(), visited)
    except Exception:
        pass
    return visited

def file_hash(filepath):
    return hashlib.md5(open(filepath, "rb").read()).hexdigest()

def needs_recompile(src_name, out_name, defines):
    src_path = SHADER_SRC / src_name
    if not src_path.exists():
        print(f"  SKIP: {src_name} not found")
        return False

    out_path = SHADER_OUT / out_name
    if not out_path.exists():
        return True

    # Load meta
    meta = {}
    if META_FILE.exists():
        meta = json.loads(META_FILE.read_text())

    key = out_name
    cached_hash = meta.get(key)
    if not cached_hash:
        return True

    # Hash source + all includes
    files = get_included_files(src_path)
    combined = "".join(file_hash(f) for f in sorted(files, key=str))
    # Include defines in hash
    combined += "|".join(sorted(defines))
    current_hash = hashlib.md5(combined.encode()).hexdigest()

    return current_hash != cached_hash

def compile_shader(src_name, out_name, defines):
    src_path = SHADER_SRC / src_name
    out_path = SHADER_OUT / out_name

    SHADER_OUT.mkdir(parents=True, exist_ok=True)

    extra_args = []
    for d in defines:
        extra_args += [f"-D{d}"]

    opt = "-O"
    # Shaders that include BlackHole_common.glsl need all bindings preserved
    if "prepass" in src_name.lower() or "composite" in src_name.lower():
        opt = "-O0"
    result = subprocess.run(
        ["glslc", "--target-env=vulkan1.4", opt,
         str(src_path), "-o", str(out_path)] + extra_args,
        capture_output=True, text=True
    )

    if result.returncode != 0:
        print(f"  FAIL: {out_name}")
        print(result.stderr)
        return False

    print(f"  OK:   {out_name}")
    return True

def main():
    print("Compiling shaders...")

    # Non-variant shaders
    for src_name, out_name in NON_VARIANT_SHADERS.items():
        if not needs_recompile(src_name, out_name, []):
            continue
        compile_shader(src_name, out_name, [])

    # Variant shaders
    for src_name, out_name, defines in VARIANTS:
        if not needs_recompile(src_name, out_name, defines):
            continue
        compile_shader(src_name, out_name, defines)

    # Update hash cache
    meta = {}
    if META_FILE.exists():
        meta = json.loads(META_FILE.read_text())

    for out_name in list(NON_VARIANT_SHADERS.values()) + [v[1] for v in VARIANTS]:
        src_name = None
        defines = []
        for s, o, d in VARIANTS:
            if o == out_name:
                src_name = s
                defines = d
                break
        if src_name is None:
            for s, o in NON_VARIANT_SHADERS.items():
                if o == out_name:
                    src_name = s
                    break

        if src_name and (SHADER_SRC / src_name).exists():
            src_path = SHADER_SRC / src_name
            files = get_included_files(src_path)
            combined = "".join(file_hash(f) for f in sorted(files, key=str))
            combined += "|".join(sorted(defines))
            meta[out_name] = hashlib.md5(combined.encode()).hexdigest()

    META_FILE.write_text(json.dumps(meta, indent=2))
    print("Done.")

if __name__ == "__main__":
    main()
