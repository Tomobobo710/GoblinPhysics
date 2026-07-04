// Chandler's convexshape.html — ported. ConvexShape volume / center_of_mass / support points for a unit
// cube and a pyramid. Volume checked with strict ===; the rest with the default EPSILON (1e-5). Each
// test builds its own convex body (via t.loneBody, so the hull draws in the viewer). The support-point
// tests also expose t.support so the viewer draws the query direction + the found point on the hull.
(function (Runner) {
	Runner.suite('chandler');

	var BOX = [[-1,-1,-1],[1,-1,-1],[1,-1,1],[-1,-1,1],[-1,1,-1],[1,1,-1],[1,1,1],[-1,1,1]];
	var PYR = [[-1,-1,-1],[1,-1,-1],[1,-1,1],[-1,-1,1],[0,2,0]];
	var EPS = 1e-5;
	var D = "A ConvexShape is built from a cloud of vertices (any convex hull). From just the points it " +
		"has to compute the right volume, the right center of mass (needed for correct physics), and the " +
		"right support points (needed for collision). Checked on a unit cube (volume 8, centered) and a " +
		"pyramid (volume 4, center of mass pulled down toward the wide base at y=-0.25).";

	function makeConvex(t, verts, color) {
		return t.loneBody(new t.Goblin.ConvexShape(verts.map(function (v) { return t.vec(v[0], v[1], v[2]); })), { mass: 0, color: color });
	}

	// --- volume / center-of-mass: just draw the hull (no point to mark) ---
	Runner.test('convexshape', 'box: volume = 8', function (t) {
		var b = makeConvex(t, BOX, '#45B7D1');
		t.checkEqual(b.shape.volume, 8, 'unit cube volume = 8');
	}, { visual: true, steps: 0, page: 'convexshape', description: D });

	Runner.test('convexshape', 'box: center of mass at origin', function (t) {
		var b = makeConvex(t, BOX, '#45B7D1');
		t.check(b.shape.center_of_mass.distanceTo(t.vec(0, 0, 0)), 0, EPS, 'center of mass at (0,0,0)');
	}, { visual: true, steps: 0, page: 'convexshape', description: D });

	Runner.test('convexshape', 'pyramid: volume = 4', function (t) {
		var b = makeConvex(t, PYR, '#F4D35E');
		t.checkEqual(b.shape.volume, 4, 'pyramid volume = 4');
	}, { visual: true, steps: 0, page: 'convexshape', description: D });

	Runner.test('convexshape', 'pyramid: center of mass at (0,-0.25,0)', function (t) {
		var b = makeConvex(t, PYR, '#F4D35E');
		t.check(b.shape.center_of_mass.distanceTo(t.vec(0, -0.25, 0)), 0, EPS, 'center of mass pulled toward the base');
	}, { visual: true, steps: 0, page: 'convexshape', description: D });

	// --- support points: draw the hull + the query direction + the found point ---
	function supportTest(name, verts, color, dir, expected, label) {
		Runner.test('convexshape', name, function (t) {
			var b = makeConvex(t, verts, color);
			var p = t.vec(0, 0, 0);
			b.findSupportPoint(t.vec(dir[0], dir[1], dir[2]), p);
			t.support = { dir: dir, point: [p.x, p.y, p.z] };
			t.check(p.distanceTo(t.vec(expected[0], expected[1], expected[2])), 0, EPS, label);
		}, { visual: true, steps: 0, page: 'convexshape', description: D });
	}

	supportTest('box: support point (corner)', BOX, '#45B7D1', [-1, -1, -1], [-1, -1, -1], 'support toward a corner');
	supportTest('box: support point (face)', BOX, '#45B7D1', [1, 0, 0], [1, -1, 1], 'support toward a face');
	supportTest('box: support point (edge)', BOX, '#45B7D1', [0, 1, 1], [1, 1, 1], 'support toward an edge');
	supportTest('pyramid: support point (corner)', PYR, '#F4D35E', [-1, -1, -1], [-1, -1, -1], 'support toward a base corner');
	supportTest('pyramid: support point (apex)', PYR, '#F4D35E', [0, 1, 0], [0, 2, 0], 'support straight up = the apex');
	supportTest('pyramid: support point (edge)', PYR, '#F4D35E', [0, -1, 1], [1, -1, 1], 'support toward a base edge');

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
