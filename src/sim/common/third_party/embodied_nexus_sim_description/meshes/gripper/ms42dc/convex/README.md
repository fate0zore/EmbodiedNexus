# MS42DC convex collision meshes

These collision meshes are derived from the adjacent `split/` MS42DC finger
STLs. The original Arachne description and its MIT attribution are documented
in this package's `UPSTREAM_README.md`. `manifest.json` records each input hash,
the output part names and the decomposition settings.

The meshes were generated offline in meters using CoACD 1.0.14 (MIT), with
`threshold=0.001`, `real_metric=True`, `seed=0` and other arguments at the
version's defaults. CoACD performs automatic preprocessing because the input
STLs are not consistently manifold. Run `decompose_ms42dc_fingers.py` from
the repository's `src/sim/mujoco/examples/` to regenerate them. The tool is
not needed at runtime. The neighboring validation example samples exposed
surfaces; it does not establish an exact Hausdorff bound.
