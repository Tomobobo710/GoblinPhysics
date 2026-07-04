// Chandler's gjk_spheres.html — ported. Two unit spheres in four collision configs; GjkEpa.testCollision
// returns the contact between them, which we check three ways: how deep they overlap (penetration
// depth), which way the contact pushes them apart (normal direction), and where they touch (contact
// point). EPSILON 0.01. Each config is a standalone test with fresh bodies and explicit transforms.
// These are instant geometric queries — no simulation — but the 3D view draws the two spheres so you
// can see the configuration.
(function (Runner) {
	Runner.suite('chandler');
	var EPS = 0.01;
	var DESC = "GJK+EPA is the collision algorithm that answers, for two convex shapes: are they " +
		"touching, how deeply do they overlap, which direction separates them, and at what point? Here two " +
		"unit spheres are placed in a specific arrangement and the reported contact is checked against the " +
		"known-correct penetration depth, normal direction, and contact point (within 0.01).";

	function collide(t, s1cfg, s2cfg, depth, normal, dotExp, point) {
		var s1 = t.loneBody(new t.Goblin.SphereShape(1), { mass: 1, pos: s1cfg.pos, rot: s1cfg.rot, color: '#F4D35E' });
		var s2 = t.loneBody(new t.Goblin.SphereShape(1), { mass: 1, pos: s2cfg.pos, rot: s2cfg.rot, color: '#45B7D1' });
		var c = t.Goblin.GjkEpa.testCollision(s1, s2);
		t.checkTrue(c != null, 'a contact is found');
		if (!c) return;
		t.check(c.penetration_depth, depth, EPS, 'penetration depth ≈ ' + depth);
		t.check(c.contact_normal.dot(t.vec(normal[0], normal[1], normal[2])), dotExp, EPS, 'contact normal points the right way');
		t.check(c.contact_point.distanceTo(t.vec(point[0], point[1], point[2])), 0, EPS, 'contact point is where the surfaces meet');
	}

	Runner.test('gjk/spheres', 'collision 1: near-touching, normal points down', function (t) {
		collide(t, { pos: [0, 0.9999, 0] }, { pos: [0, -1, 0] }, 0.01, [0, -1, 0], 0.977, [0, 0, 0]);
	}, { visual: true, steps: 0, page: 'gjk_spheres', description: DESC });

	Runner.test('gjk/spheres', 'collision 2: deep overlap', function (t) {
		collide(t, { pos: [0, 0.5, 0] }, { pos: [0, -1, 0] }, 0.5, [0.2, -0.97, 0], 0.99, [0, -0.25, 0]);
	}, { visual: true, steps: 0, page: 'gjk_spheres', description: DESC });

	Runner.test('gjk/spheres', 'collision 3: one sphere rotated', function (t) {
		collide(t, { pos: [-2, 1, 0], rot: [1, 0, 0, 1] }, { pos: [-2, -0.5, 0] }, 0.5, [0.1486, -0.98, -0.1486], 0.93, [-2, 0.25, 0]);
	}, { visual: true, steps: 0, page: 'gjk_spheres', description: DESC });

	Runner.test('gjk/spheres', 'collision 4: other sphere rotated', function (t) {
		collide(t, { pos: [-2, 0.75, 0] }, { pos: [-2, -0.75, 0], rot: [0, 5, -3, 1] }, 0.5, [-0.1486, -0.98, 0.1486], 0.96, [-2, 0, 0]);
	}, { visual: true, steps: 0, page: 'gjk_spheres', description: DESC });

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
