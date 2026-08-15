#!/usr/bin/env python3
"""Bake Burk-e visual transforms into link-local binary STL assets.

The source CAD exports are intentionally preserved.  Generated ``*_link.stl``
files contain metre-scale vertices in the corresponding URDF link frame, so
all URDF consumers can render them with an identity visual transform.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import struct
import sys


@dataclass(frozen=True)
class MeshTransform:
    source: str
    output: str
    scale: float
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


PI = math.pi
TRANSFORMS = (
    MeshTransform("MiR1350_reduced.stl", "MiR1350_link.stl", 1.0, (0.0, 0.0, 0.000230), (0.0, 0.0, PI / 2.0)),
    MeshTransform("LIFTKIT_1.stl", "LIFTKIT_1_link.stl", 0.001, (0.0, 0.0, 0.0), (PI / 2.0, 0.0, 0.0)),
    MeshTransform("LIFTKIT_2.stl", "LIFTKIT_2_link.stl", 0.001, (0.0, 0.0, -0.275), (PI / 2.0, 0.0, 0.0)),
    MeshTransform("LIFTKIT_3.stl", "LIFTKIT_3_link.stl", 0.001, (0.0, 0.0, -0.500), (PI / 2.0, 0.0, 0.0)),
    MeshTransform("UR8L_PART_1.stl", "UR8L_PART_1_link.stl", 0.001, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    MeshTransform("UR8L_PART_2.stl", "UR8L_PART_2_link.stl", 0.001, (0.0, 0.0, 0.0), (PI / 2.0, 0.0, -PI / 2.0)),
    MeshTransform("UR8L_PART_3.stl", "UR8L_PART_3_link.stl", 0.001, (0.0, 0.0, 0.2212), (PI / 2.0, -PI / 2.0, 0.0)),
    MeshTransform("UR8L_PART_4.stl", "UR8L_PART_4_link.stl", 0.001, (0.0, 0.0, 0.0463), (PI / 2.0, -PI / 2.0, 0.0)),
    MeshTransform("UR8L_PART_5.stl", "UR8L_PART_5_link.stl", 0.001, (0.0, 0.0, -0.00025), (PI / 2.0, -PI / 2.0, 0.0)),
    MeshTransform("UR8L_PART_6.stl", "UR8L_PART_6_link.stl", 0.001, (0.0, 0.0, 0.0641), (PI / 2.0, 0.0, -PI)),
    MeshTransform("UR8L_PART_7.stl", "UR8L_PART_7_link.stl", 0.001, (0.0, 0.0, -0.0512), (-PI / 2.0, PI / 2.0, 0.0)),
)


def rotation_matrix(rpy: tuple[float, float, float]) -> tuple[tuple[float, float, float], ...]:
    """Return the URDF fixed-axis RPY matrix Rz(yaw) * Ry(pitch) * Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def rotate(matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(sum(row[index] * vector[index] for index in range(3)) for row in matrix)  # type: ignore[return-value]


def read_binary_stl(path: Path) -> tuple[bytes, list[tuple[tuple[float, ...], int]]]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path} is too short to be a binary STL")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(data) != expected_size:
        raise ValueError(f"{path} is not a supported binary STL: expected {expected_size} bytes, found {len(data)}")
    triangles = []
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack_from("<12f", data, offset)
        attribute = struct.unpack_from("<H", data, offset + 48)[0]
        triangles.append((values, attribute))
        offset += 50
    return data[:80], triangles


def transformed_triangles(transform: MeshTransform, source: Path) -> list[tuple[tuple[float, ...], int]]:
    _, triangles = read_binary_stl(source)
    matrix = rotation_matrix(transform.rpy)
    output = []
    for values, attribute in triangles:
        normal = rotate(matrix, values[0:3])
        transformed = list(normal)
        for start in (3, 6, 9):
            scaled = tuple(transform.scale * value for value in values[start:start + 3])
            rotated = rotate(matrix, scaled)
            transformed.extend(rotated[index] + transform.xyz[index] for index in range(3))
        output.append((tuple(transformed), attribute))
    return output


def write_binary_stl(path: Path, transform: MeshTransform, triangles: list[tuple[tuple[float, ...], int]]) -> None:
    label = f"Burk-e link-local mesh derived from {transform.source}".encode("ascii")[:80]
    with path.open("wb") as stream:
        stream.write(label.ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        for values, attribute in triangles:
            stream.write(struct.pack("<12fH", *values, attribute))


def verify(transform: MeshTransform, expected: list[tuple[tuple[float, ...], int]], output_path: Path) -> None:
    _, actual = read_binary_stl(output_path)
    if len(actual) != len(expected):
        raise ValueError(f"{output_path} triangle count differs from its source")
    tolerance = 2e-6
    for triangle_index, ((expected_values, expected_attribute), (actual_values, actual_attribute)) in enumerate(zip(expected, actual)):
        if expected_attribute != actual_attribute or any(
            abs(expected_value - actual_value) > tolerance
            for expected_value, actual_value in zip(expected_values, actual_values)
        ):
            raise ValueError(f"{output_path} differs from the baked transform at triangle {triangle_index}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="create or replace the derived link-local meshes")
    args = parser.parse_args()
    mesh_directory = Path(__file__).resolve().parents[1] / "cad" / "stl"

    for transform in TRANSFORMS:
        source_path = mesh_directory / transform.source
        output_path = mesh_directory / transform.output
        expected = transformed_triangles(transform, source_path)
        if args.write:
            write_binary_stl(output_path, transform, expected)
        if not output_path.is_file():
            raise FileNotFoundError(f"missing derived mesh {output_path}; run with --write")
        verify(transform, expected, output_path)
        print(f"verified {transform.output}: {len(expected)} triangles")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
