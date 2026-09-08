// Chandler's constraint-slider.html — ported exactly.
// A tall base box and a smaller top box, offset on X, linked by a SliderConstraint along the world Y
// axis, both falling onto a ground plane at y=-3. No behavioral assertions yet — just runs for 600
// ticks so it can be watched.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('constraint-slider', 'two boxes linked by a slider constraint', function (t) {
		t.log('Linking a base and top box with a SliderConstraint along Y, then dropping them; just watching it run for now.');

		var w = t.makeWorld({ gravity: -9.8 });

		t.plane(w, 1, 20, 20, 0, { pos: [0, -3, 0], color: '#243B2A' });

		var base = t.box(w, 1, 5, 1, 10, { pos: [0, 5, 0], color: '#B08968' });
		var top = t.box(w, 1, 2, 1, 10, { pos: [3, 10, 0], color: '#4af' });

		var constraint = new t.Goblin.SliderConstraint(base, t.vec(0, 1, 0), top);
		w.addConstraint(constraint);

		t.expect('scene ran to completion (this test is for watching, not asserting)', function (world) {
			return world.ticks >= 600;
		});
		t.simulate(w, 600);
	}, {
		visual: true, steps: 600, page: 'constraint-slider',
		description:
			"A tall base box and an X-offset top box are linked by a SliderConstraint along the world Y axis " +
			"and dropped onto a ground plane, matching Chandler's constraint-slider.html example exactly. " +
			"No pass/fail criteria yet — it just runs for 600 ticks so the slider's behavior can be watched."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
