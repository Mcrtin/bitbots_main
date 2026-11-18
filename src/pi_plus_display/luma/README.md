# luma.core (vendored)

Copy of [luma.core](https://github.com/rm-hull/luma.core) 2.6.0 trimmed down to `legacy/` (MIT licence, see `core/LICENSE.rst`),
so the status display runs without an additional pip dependency. Only
`luma.core.legacy.text`/`textsize` and the bitmap fonts from `luma.core.legacy.font` are used; Pillow is enough for that.

Updating: install the new version with pip, copy `luma/core/legacy` here (without `__pycache__`);
in `legacy/__init__.py`, remove `show_message` and the imports of `render`/`virtual` again.
