// Chandler's sweptshapes.html — ported. A LineSweptShape is a base shape (a box, a sphere) dragged
// along a line segment — the volume it sweeps out, used for continuous/swept collision. We check its
// bounding box and its support points. The viewer draws the base shape at BOTH endpoints (faint) so you
// can see the sweep, and the support-point tests draw the query direction + found point.
(function (Runner) {
	Runner.suite('chandler');
	var G = typeof module !== 'undefined' && module.exports ? require('../../../build/goblin.js') : window.Goblin;
	var V3 = G.Vector3;
	var DESC = "A swept shape is a base shape (a box, a sphere) dragged along a line segment — the volume " +
		"it sweeps out. It's how continuous collision avoids a fast object tunnelling through a wall. These " +
		"check two fundamentals of the swept shape: its axis-aligned bounding box (min/max corners) and its " +
		"support points (farthest point in a direction) are computed exactly. The viewer shows the base " +
		"shape at the start and end of the sweep.";

	function sweptBox() { return new G.LineSweptShape(new V3(0, -2, 3), new V3(5, 0, 0), new G.BoxShape(1, 1, 1)); }
	function sweptSphere() { return new G.LineSweptShape(new V3(-2, 0, 3), new V3(-2, 1, -1), new G.SphereShape(2)); }

	// expose the swept shape for the viewer to draw (base shape at start + end)
	function show(t, s) { t.swept = { shape: s.shape, start: [s.start.x, s.start.y, s.start.z], end: [s.end.x, s.end.y, s.end.z] }; }

	Runner.test('sweptshapes', 'swept box: bounding box is correct', function (t) {
		var s = sweptBox(); show(t, s);
		t.checkEqual(s.aabb.min.x, -1, 'aabb.min.x'); t.checkEqual(s.aabb.min.y, -3, 'aabb.min.y'); t.checkEqual(s.aabb.min.z, -1, 'aabb.min.z');
		t.checkEqual(s.aabb.max.x, 6, 'aabb.max.x'); t.checkEqual(s.aabb.max.y, 1, 'aabb.max.y'); t.checkEqual(s.aabb.max.z, 4, 'aabb.max.z');
	}, { visual: true, steps: 0, page: 'sweptshapes', description: DESC });

	Runner.test('sweptshapes', 'swept box: support points are correct', function (t) {
		var s = sweptBox(); show(t, s); var p = new V3();
		s.findSupportPoint(new V3(0, -1, 0), p); t.support = { dir: [0, -1, 0], point: [p.x, p.y, p.z] };
		t.checkEqual(p.distanceTo(new V3(1, -3, 4)), 0, 'support down');
		s.findSupportPoint(new V3(0, 1, 0), p); t.checkEqual(p.distanceTo(new V3(6, 1, 1)), 0, 'support up');
		s.findSupportPoint(new V3(-1, 0, 1), p); t.checkEqual(p.distanceTo(new V3(-1, -1, 4)), 0, 'support diagonal');
	}, { visual: true, steps: 0, page: 'sweptshapes', description: DESC });

	Runner.test('sweptshapes', 'swept sphere: bounding box is correct', function (t) {
		var s = sweptSphere(); show(t, s);
		t.checkEqual(s.aabb.min.x, -4, 'aabb.min.x'); t.checkEqual(s.aabb.min.y, -2, 'aabb.min.y'); t.checkEqual(s.aabb.min.z, -3, 'aabb.min.z');
		t.checkEqual(s.aabb.max.x, 0, 'aabb.max.x'); t.checkEqual(s.aabb.max.y, 3, 'aabb.max.y'); t.checkEqual(s.aabb.max.z, 5, 'aabb.max.z');
	}, { visual: true, steps: 0, page: 'sweptshapes', description: DESC });

	Runner.test('sweptshapes', 'swept sphere: support points are correct', function (t) {
		var s = sweptSphere(); show(t, s); var p = new V3();
		s.findSupportPoint(new V3(0, -1, 0), p); t.support = { dir: [0, -1, 0], point: [p.x, p.y, p.z] };
		t.checkEqual(p.distanceTo(new V3(-2, -2, 3)), 0, 'support down');
		s.findSupportPoint(new V3(0, 1, 0), p); t.checkEqual(p.distanceTo(new V3(-2, 3, -1)), 0, 'support up');
		s.findSupportPoint(new V3(-1, 0, 1), p); t.checkEqual(p.distanceTo(new V3(-3.414213562373095, 0, 4.414213562373095)), 0, 'support diagonal');
	}, { visual: true, steps: 0, page: 'sweptshapes', description: DESC });

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
