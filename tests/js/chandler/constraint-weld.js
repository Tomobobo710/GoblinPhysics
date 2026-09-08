// Chandler's constraint-weld.html — ported exactly.
// A base box at y=2 and a top box at y=4, welded face-to-face by a WeldConstraint, falling onto a
// ground plane at y=0 as one rigid unit. No behavioral assertions yet — just runs for 600 ticks so it
// can be watched.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('constraint-weld', 'two boxes welded together fall as one unit', function (t) {
		t.log('Welding two stacked boxes together, then dropping them; just watching it run for now.');

		var w = t.makeWorld({ gravity: -9.8 });

		t.plane(w, 1, 20, 20, 0, { color: '#243B2A' });

		var base = t.box(w, 1, 1, 1, 10, { pos: [0, 2, 0], color: '#B08968' });
		var top = t.box(w, 1, 1, 1, 10, { pos: [0, 4, 0], color: '#4af' });

		var constraint = new t.Goblin.WeldConstraint(base, t.vec(0, 1, 0), top, t.vec(0, -1, 0));
		w.addConstraint(constraint);

		t.expect('scene ran to completion (this test is for watching, not asserting)', function (world) {
			return world.ticks >= 600;
		});
		t.simulate(w, 600);
	}, {
		visual: true, steps: 600, page: 'constraint-weld',
		description:
			"A base box and a top box are welded face-to-face by a WeldConstraint and dropped onto a ground " +
			"plane, matching Chandler's constraint-weld.html example exactly. No pass/fail criteria yet — it " +
			"just runs for 600 ticks so the welded pair's behavior can be watched in the viewer."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
