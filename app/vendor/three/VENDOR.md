# three.js 0.186.1 (MIT, see LICENSE)

From the npm tarball `three-0.186.1.tgz` (sha512-blFeqb49wRCSGUGj7gtpfnSGHy2lwDk94RhUmS1c/hTby70kvChbWpkJ4Pm1390LqzzvTmzgXKHPEafJwCb8jA==,
matches the registry). `build/three.module.js` and `build/three.core.js` are unmodified. `addons/postprocessing/Pass.js`
(from `examples/jsm/`, needed by Spark) has its `'three'` import rewritten to `'../../build/three.module.js'`, because
import maps do not apply inside module workers. Pinned: `app/js/twin` patches Spark internals that depend on it.
