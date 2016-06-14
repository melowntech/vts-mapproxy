vts/mapproxy

---

generatevrtwo: Generate virtual GDAL dataset with overviews.

Dataset format:
* directory with:
 * original: symlink to original dataset (anything user provided)
 * dataset: virtual dataset itself (VRT driver); use this file as an entry point
 * 0-N: directory with overview level
  * ovr.vrt: virtual dataset encapsulating individual tiles
  * x-y.tif: file with tile at (x, y)
  * bg.solid: (optional) dataset with uniform vaulu used in place of missing tiles

NB: symlink to original dataset is used to ease remapping of "vrtwo" dataset
when moving around filesystems/machines.
