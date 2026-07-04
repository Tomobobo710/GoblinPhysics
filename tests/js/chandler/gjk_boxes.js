// Chandler's gjk_boxes.html — ported. box1(2,1,2) vs box2(0,0.5,0.5) in four configs; GjkEpa reports
// their contact — penetration depth, normal, contact point — checked within 0.01. Each config is a
// standalone test with fresh bodies and explicit transforms. Instant query; the 3D view draws the boxes.
(function (Runner) {
	Runner.suite('chandler');
	var EPS = 0.01;
	var DESC = "GJK+EPA collision between two boxes: given a specific arrangement it reports how deep " +
		"they overlap, which direction pushes them apart, and where they touch. Box-box contacts are " +
		"trickier than spheres (flat faces, edges, corners), so this checks the algorithm on flat stacks, " +
		"offset overlaps, and rotated boxes. Each result is checked against the known-correct values (0.01).";

	function collide(t, b1cfg, b2cfg, depth, normal, dotExp, point) {
		var b1 = t.loneBody(new t.Goblin.BoxShape(2, 1, 2), { mass: 1, pos: b1cfg.pos, rot: b1cfg.rot, color: '#F4D35E' });
		var b2 = t.loneBody(new t.Goblin.BoxShape(0, 0.5, 0.5), { mass: 1, pos: b2cfg.pos, rot: b2cfg.rot, color: '#45B7D1' });
		var c = t.Goblin.GjkEpa.testCollision(b1, b2);
		t.checkTrue(c != null, 'a contact is found');
		if (!c) return;
		t.check(c.penetration_depth, depth, EPS, 'penetration depth ≈ ' + depth);
		t.check(c.contact_normal.dot(t.vec(normal[0], normal[1], normal[2])), dotExp, EPS, 'contact normal points the right way');
		t.check(c.contact_point.distanceTo(t.vec(point[0], point[1], point[2])), 0, EPS, 'contact point is where the surfaces meet');
	}

	Runner.test('gjk/boxes', 'collision 1: flat stack', function (t) {
		collide(t, { pos: [0, 0, 0] }, { pos: [0, 1.49, 0] }, 0.02, [0, 1, 0], 1, [0, 1, 0.4]);
	}, { visual: true, steps: 0, page: 'gjk_boxes', description: DESC });

	Runner.test('gjk/boxes', 'collision 2: offset overlap', function (t) {
		collide(t, { pos: [2, 0, 0] }, { pos: [1.5, 1.25, 0] }, 0.26, [0, 1, 0], 1, [1.5, 0.875, 0.5]);
	}, { visual: true, steps: 0, page: 'gjk_boxes', description: DESC });

	Runner.test('gjk/boxes', 'collision 3: both rotated about y', function (t) {
		collide(t, { pos: [0, 0, 0], rot: [0, -0.415, 0, 1] }, { pos: [0, 1.49, 0], rot: [0, 0.415, 0, 1] }, 0.02, [0, 1, 0], 1, [0.2829, 1, 0.2829]);
	}, { visual: true, steps: 0, page: 'gjk_boxes', description: DESC });

	Runner.test('gjk/boxes', 'collision 4: one box rotated about x', function (t) {
		collide(t, { pos: [2, -1, 0] }, { pos: [2, 0.7, 0], rot: [0.415, 0, 0, 1] }, 0.017, [0, 1, 0], 1, [2, 0, 0]);
	}, { visual: true, steps: 0, page: 'gjk_boxes', description: DESC });

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
