Robot model assets owned by the simulation end.

- `marvin/`: original Marvin M6 URDF and STL meshes. Source, import adaptation
  and collision boundaries are documented in `marvin/README.md`.

`physics/assets.py` resolves each model from this directory. Scenario recipes
select `robot.assembly`; they do not carry model file paths.
