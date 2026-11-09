"""Sample the exposed convex finger surface against the original finger STL."""

import json
from pathlib import Path

import numpy as np
import trimesh


def validate(side, root, sample_count=30000):
    """Measure exposed convex surface distance from the source mesh.

    Args:
        side: Finger side, left or right.
        root: Shared robot description package directory.
        sample_count: Number of deterministic convex surface samples.

    Returns:
        Dictionary containing sample count, maximum and percentile errors in meters.
    """
    path = root / 'meshes/gripper/ms42dc'
    manifest = json.loads((path / 'convex/manifest.json').read_text(encoding='utf-8'))
    original = trimesh.load_mesh(path / 'split' / f'ms42dc_{side}_finger.stl')
    original.apply_scale(0.001)
    hulls = [trimesh.load_mesh(path / 'convex' / name) for name in manifest['fingers'][side]['parts']]
    combined = trimesh.util.concatenate(hulls)
    points, faces = trimesh.sample.sample_surface(combined, sample_count, seed=0)
    outside = points + combined.face_normals[faces] * 0.00005
    hidden = np.zeros(len(points), dtype=bool)
    for hull in hulls:
        inside_bounds = np.all((outside >= hull.bounds[0]) & (outside <= hull.bounds[1]), axis=1)
        indices = np.flatnonzero(~hidden & inside_bounds)
        if len(indices):
            hidden[indices[hull.contains(outside[indices])]] = True
    exposed = points[~hidden]
    distances = trimesh.proximity.closest_point(original, exposed)[1]
    return {'side': side, 'exposed_samples': len(exposed),
            'p99_error_m': float(np.percentile(distances, 99)),
            'maximum_error_m': float(np.max(distances)),
            'above_2mm': int(np.sum(distances > 0.002))}


def main():
    """Report the deterministic surface sample and fail on errors above 2 mm.

    Returns:
        Process status, zero if the sampled surfaces meet the limit.
    """
    root = Path(__file__).resolve().parents[2] / 'common/third_party/embodied_nexus_sim_description'
    results = [validate(side, root) for side in ('left', 'right')]
    print(json.dumps(results, indent=2))
    return int(any(item['above_2mm'] for item in results))


if __name__ == '__main__':
    raise SystemExit(main())
