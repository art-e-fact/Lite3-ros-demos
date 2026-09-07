## Remote assets

Large world meshes are not stored in this repository. They live in the
[Artefacts/quadruped-environments](https://huggingface.co/datasets/Artefacts/quadruped-environments)
dataset on the Hugging Face Hub and are downloaded on first use.

Any config field that takes a path also accepts an `hf://` URI:

```
hf://datasets/<owner>/<name>[@<revision>]/<path/to/file.xml>
```

The whole containing folder is fetched, so a scene XML arrives together with the
meshes it references. Only that folder is downloaded, which is why each Karuizawa
part lives in its own directory: running `part1` pulls ~171 MB rather than the full
~481 MB dataset. Files are cached in `~/.cache/huggingface`, so the download only
happens once per revision.

The Karuizawa scene is pinned to a specific dataset commit in `pixi.toml`. To pick up
new world uploads, update that revision (or drop `@<revision>` to track `main`).

```bash
pixi run sim-newton-m20-karuizawa part2
```

No Git LFS setup is needed to *use* these assets. It is only relevant when publishing
new ones — see below.

### Publishing world updates

The Hub dataset is a normal Git repository, cloned outside this project:

```bash
git clone https://huggingface.co/datasets/Artefacts/quadruped-environments
```

Export from Blender into `mjcf/<world>/<part>/`, keeping each part's `.xml` and `.obj`
side by side, then commit and push. Files over 10 MB are tracked with Git LFS
automatically. The Blender source (`parts.blend`) is committed one level above the part
directories, so it is versioned but never downloaded at runtime.