#!/usr/bin/env python3
"""Generate a concave aircraft footprint union and its buffered perimeter."""

import argparse
import struct
from pathlib import Path

import cv2
import numpy as np
import yaml


def read_projected_triangles(mesh_path: Path) -> list[np.ndarray]:
    """Read binary STL triangles and project them into aircraft XY."""
    data = mesh_path.read_bytes()
    triangle_count = struct.unpack_from('<I', data, 80)[0]
    aircraft_center_stl_z = (3657.599854 + 24332.212891) / 2.0
    triangles = []
    for index in range(triangle_count):
        values = struct.unpack_from('<12f', data, 84 + 50 * index)
        vertices = np.asarray((values[3:6], values[6:9], values[9:12]))
        triangles.append(np.column_stack((
            (aircraft_center_stl_z - vertices[:, 2]) * 0.001,
            -vertices[:, 0] * 0.001)))
    return triangles


def raster_union(triangles: list[np.ndarray], scale: int):
    """Rasterize the union of all projected triangles and return its metadata."""
    all_points = np.concatenate(triangles)
    # Leave room for the circular buffer; otherwise dilation clips at the
    # raster edge and silently destroys the requested clearance.
    lower = all_points.min(axis=0) - 2.0
    upper = all_points.max(axis=0) + 2.0
    size = np.ceil((upper - lower) * scale).astype(int) + 1
    mask = np.zeros((size[1], size[0]), dtype=np.uint8)
    for triangle in triangles:
        pixels = np.rint((triangle - lower) * scale).astype(np.int32)
        pixels[:, 1] = size[1] - 1 - pixels[:, 1]
        cv2.fillPoly(mask, [pixels], 255)
    return mask, lower, size


def contour_points(contour, lower, size, scale, epsilon_m):
    """Convert an OpenCV exterior contour to clockwise aircraft coordinates."""
    simplified = cv2.approxPolyDP(contour, epsilon_m * scale, True).reshape(-1, 2)
    points = [
        (float(x / scale + lower[0]),
         float((size[1] - 1 - y) / scale + lower[1]))
        for x, y in simplified]
    if signed_area(points) > 0.0:
        points.reverse()
    return points


def signed_area(points):
    """Return the signed area of a closed polygon."""
    return 0.5 * sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]))


def densify(points, maximum_spacing):
    """Densify a closed contour so no adjacent output points exceed spacing."""
    result = []
    for start, end in zip(points, points[1:] + points[:1]):
        distance = np.linalg.norm(np.asarray(end) - np.asarray(start))
        count = max(1, int(np.ceil(distance / maximum_spacing)))
        result.extend(
            (start[0] + (end[0] - start[0]) * step / count,
             start[1] + (end[1] - start[1]) * step / count)
            for step in range(count))
    return result


def rotate_to_nose(points):
    """Start the perimeter at the positive-X nose point."""
    index = min(range(len(points)), key=lambda i: (-points[i][0], abs(points[i][1])))
    return points[index:] + points[:index]


def generate(mesh_path: Path, clearance: float, maximum_clearance: float,
             maximum_spacing: float, scale: int = 100):
    """Generate the union exterior and a raster-buffered perimeter."""
    triangles = read_projected_triangles(mesh_path)
    mask, lower, size = raster_union(triangles, scale)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    footprint = contour_points(max(contours, key=cv2.contourArea), lower, size, scale, 0.01)

    radius_pixels = int(round((clearance + 0.2) * scale))
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * radius_pixels + 1, 2 * radius_pixels + 1))
    buffered = cv2.dilate(mask, kernel)
    contours, _ = cv2.findContours(buffered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    perimeter = contour_points(
        max(contours, key=cv2.contourArea), lower, size, scale, 0.02)
    nose_index = max(range(len(perimeter)), key=lambda index: perimeter[index][0])
    perimeter[nose_index] = (
        max(point[0] for point in footprint) + clearance + 0.2, 0.0)
    perimeter = rotate_to_nose(densify(perimeter, maximum_spacing))
    return {
        'perimeter_path': {
            'frame_id': 'aircraft',
            'closed': True,
            'footprint': footprint,
            'waypoints': perimeter,
            'clearance_m': clearance,
            'maximum_clearance_m': maximum_clearance,
            'construction_radius_m': clearance + 0.2,
            'maximum_pose_spacing_m': maximum_spacing,
            'generation': {
                'source_mesh': str(mesh_path),
                'method': 'project all collision triangles, raster union, external contour, '
                          'circular raster buffer',
                'raster_resolution_px_per_m': scale,
            },
        }
    }


def main():
    """Generate a perimeter YAML file from the collision mesh."""
    parser = argparse.ArgumentParser()
    parser.add_argument('mesh', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = generate(args.mesh, 1.0, 1.5, 1.0)
    args.output.write_text(yaml.safe_dump(result, sort_keys=False))


if __name__ == '__main__':
    main()
