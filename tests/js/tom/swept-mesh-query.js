// Tom's Suite — SWEPT QUERY vs MESH.
// World.shapeIntersect (the swept-box query used for collide-and-slide) used to return NOTHING for a
// MeshShape body: getContact routes mesh/compound contacts through the addContact CALLBACK and returns
// undefined, while shapeIntersect only read the direct return value. So a swept box cast straight through
// a static mesh reported zero hits — a swept body would pass clean through static mesh geometry.
// (Dynamic meshes masked it: the body got shoved on penetration, which registered as a collision.)
// Oracle: a box swept through a static cube MESH returns at least one intersection whose object IS that
// mesh, with a sane surface normal — proving the query sees the mesh at all.
//
// Pure query (no simulation needed); deterministic, headless == browser.
(function (Runner, U) {
	Runner.suite('tom');

	// A unit cube as a triangle mesh (8 verts, 12 triangles), centered at the origin, half-extent H.
	function cubeMesh(H) {
		var v = [
			[-H,-H,-H],[ H,-H,-H],[ H, H,-H],[-H, H,-H], // back  (z=-H)
			[-H,-H, H],[ H,-H, H],[ H, H, H],[-H, H, H]  // front (z=+H)
		];
		var f = [
			0,1,2, 0,2,3,   // back
			4,6,5, 4,7,6,   // front
			0,4,5, 0,5,1,   // bottom
			3,2,6, 3,6,7,   // top
			0,3,7, 0,7,4,   // left
			1,5,6, 1,6,2    // right
		];
		return { v: v, f: f };
	}

	Runner.test('swept-mesh-query', 'shapeIntersect sees a static mesh', function (t) {
		t.log('Sweep a box straight through a static cube MESH and assert shapeIntersect returns it.');

		var w = t.makeWorld({ gravity: 0 });
		var H = 1;
		var cube = cubeMesh(H);
		// static (mass 0) mesh cube centered at the origin
		var meshBody = t.mesh(w, cube.v, cube.f, 0, U.withMat({ pos: [0, 0, 0], color: '#889' }));

		// Sweep a small box from -x, THROUGH the cube, to +x, at cube center height.
		var box = new t.Goblin.BoxShape(0.3, 0.3, 0.3);
		var start = t.vec(-3, 0, 0), end = t.vec(3, 0, 0);

		// One criterion, evaluated as a live predicate (runs each tick; the query is tick-independent so it
		// resolves on tick 0). Assert: a returned intersection whose object is the mesh body, with a
		// finite, non-degenerate normal.
		t.expect('swept box returns the static mesh as a hit', function () {
			var hits = w.shapeIntersect(box, start, end);
			var meshHits = hits.filter(function (h) { return h.object === meshBody; });
			var ok = false, detail = 'meshHits=' + meshHits.length + ' totalHits=' + hits.length;
			if (meshHits.length) {
				var n = meshHits[0].normal, nlen = Math.sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
				ok = nlen > 0.5 && isFinite(nlen);
				detail += ' normal=[' + n.x.toFixed(2) + ',' + n.y.toFixed(2) + ',' + n.z.toFixed(2) + '] |n|=' + nlen.toFixed(2);
			}
			return { ok: ok, detail: detail };
		});

		t.simulate(w, 2);
	}, {
		visual: false, steps: 0, page: 'swept-mesh-query',
		description:
			"World.shapeIntersect (the swept-box query used for collide-and-slide) must return a MeshShape " +
			"body it sweeps through. The regression: mesh/compound contacts route through the addContact " +
			"callback and getContact returns undefined, so shapeIntersect saw zero mesh hits and a swept " +
			"body would pass through static meshes. PASS: a box swept through a static cube mesh returns " +
			"at least one hit whose object is that mesh, with a sane surface normal."
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
