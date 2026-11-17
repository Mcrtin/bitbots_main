# luma.core (eingebunden)

Auf `legacy/` gekuerzte Kopie von [luma.core](https://github.com/rm-hull/luma.core) 2.6.0 (MIT-Lizenz, siehe `core/LICENSE.rst`),
damit die Statusanzeige ohne zusaetzliche pip-Abhaengigkeit laeuft. Genutzt werden nur
`luma.core.legacy.text`/`textsize` und die Bitmap-Fonts aus `luma.core.legacy.font`; dafuer reicht Pillow.

Aktualisieren: neue Version per pip installieren, `luma/core/legacy` hierher kopieren (ohne `__pycache__`);
in `legacy/__init__.py` wieder `show_message` und die Imports von `render`/`virtual` entfernen.
