// Chandler's spheres.html — ported exactly.
// A 5x5 static base level of spheres, then dynamic 4x4/3x3/2x2 levels stacked above, then a single
// heavy sphere dropped from y=20 with a downward impulse. No behavioral assertions yet — just runs
// for 600 ticks so it can be watched.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('spheres', 'sphere pyramid with a heavy sphere dropped on top', function (t) {
		t.log('Building a sphere pyramid, then dropping a heavy sphere onto it; just watching it run for now.');

		var w = t.makeWorld({ gravity: -9.8 });
		var i, j;

		// Base spheres (static)
		for (i = 0; i < 5; i++) {
			for (j = 0; j < 5; j++) {
				t.sphere(w, 1, 0, { pos: [i * 2 - 2.5, 0, j * 2 - 2.5], color: '#8899AA' });
			}
		}
		// 2nd level
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				t.sphere(w, 1, 10, { pos: [i * 2 - 1.5, 2, j * 2 - 1.5], color: '#8899AA' });
			}
		}
		// 3rd level
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				t.sphere(w, 1, 10, { pos: [i * 2 - 0.5, 4, j * 2 - 0.5], color: '#8899AA' });
			}
		}
		// 4th level
		for (i = 0; i < 2; i++) {
			for (j = 0; j < 2; j++) {
				t.sphere(w, 1, 10, { pos: [i * 2 + 0.5, 6, j * 2 + 0.5], color: '#8899AA' });
			}
		}
		// 5th level: single heavy sphere dropped with a downward impulse
		var heavy = t.sphere(w, 1.5, 300, { pos: [1.5, 20, 1.5], color: '#E85D4D' });
		heavy.applyImpulse(new t.Goblin.Vector3(0, -10, 0));

		t.expect('scene ran to completion (this test is for watching, not asserting)', function (world) {
			return world.ticks >= 600;
		});
		t.simulate(w, 600);
	}, {
		visual: true, steps: 600, page: 'spheres',
		description:
			"A 5x5 static sphere base with 4x4/3x3/2x2 dynamic levels stacked above, then a heavy sphere " +
			"dropped from y=20 with a downward impulse, matching Chandler's spheres.html example exactly. " +
			"No pass/fail criteria yet — it just runs for 600 ticks so the pile's behavior can be watched."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
