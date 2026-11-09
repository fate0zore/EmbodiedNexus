"""Generate rigid convex finger collision meshes from the shared STL assets."""

import hashlib
import json
from importlib.metadata import version
from pathlib import Path

import coacd
import numpy as np
import trimesh


def main():
    """Generate both finger meshes and record reproducible provenance.

    Returns:
        None.
    """
    root = Path(__file__).resolve().parents[2] / 'common/third_party/embodied_nexus_sim_description'
    source = root / 'meshes/gripper/ms42dc/split'
    output = root / 'meshes/gripper/ms42dc/convex'
    output.mkdir(parents=True, exist_ok=True)
    options = {'threshold': 0.001, 'real_metric': True, 'seed': 0}
    manifest = {'tool': 'CoACD', 'version': version('coacd'), 'options': options, 'fingers': {}}
    for side in ('left', 'right'):
        path = source / f'ms42dc_{side}_finger.stl'
        mesh = trimesh.load_mesh(path, process=True)
        mesh.apply_scale(0.001)
        hulls = coacd.run_coacd(coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces)), **options)
        files = []
        for index, (vertices, faces) in enumerate(hulls):
            name = f'{side}_{index:02d}.stl'
            trimesh.Trimesh(vertices=vertices, faces=faces, process=False).export(output / name)
            files.append(name)
        manifest['fingers'][side] = {'source': path.name, 'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                      'parts': files}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
