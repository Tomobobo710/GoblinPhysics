// Chandler's constraint-point.html — ported exactly.
// A rope-bridge of 11 planks linked end-to-end by paired PointConstraints (front + back), anchored at
// both ends (first/last plank are static/mass 0), over a ground plane at y=-5. A sphere is spawned
// every 60 ticks and dropped onto the bridge from y=8, just like the example's stepStart listener.
// No behavioral assertions yet — just runs for 600 ticks so it can be watched.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('constraint-point', 'plank bridge held together by point constraints', function (t) {
		t.log('Building a plank bridge from PointConstraint pairs, anchored at both ends; just watching it run for now.');

		var w = t.makeWorld({ gravity: -9.8 });
		w.solver.warmstarting_factor = 0.4;
		w.solver.sor_weight = 1;

		t.plane(w, 1, 20, 20, 0, { pos: [0, -5, 0], color: '#243B2A' });

		var previous_plank = null,
			plank_count = 11,
			plank_separation = 0.2,
			plank_width = 2,
			plank_height = 0.4,
			plank_length = 6,
			plank_mass = 1,
			plank_space = plank_width + plank_separation,
			i, constraint,
			right_point_front = t.vec(plank_space / 2, 0, plank_length / -6),
			left_point_front = t.vec(plank_space / -2, 0, plank_length / -6),
			right_point_back = t.vec(plank_space / 2, 0, plank_length / 6),
			left_point_back = t.vec(plank_space / -2, 0, plank_length / 6);

		for (i = 0; i < plank_count; i++) {
			var isAnchor = (i === 0 || i + 1 === plank_count);
			var plank = t.box(w, plank_width / 2, plank_height / 2, plank_length / 2, isAnchor ? 0 : plank_mass, {
				pos: [i * plank_space - (plank_count / 2 * plank_space) + plank_width / 2, 3, 0],
				color: isAnchor ? '#888' : '#B08968'
			});

			if (previous_plank) {
				constraint = new t.Goblin.PointConstraint(previous_plank, right_point_back, plank, left_point_back);
				constraint.breaking_threshold = 8;
				w.addConstraint(constraint);

				constraint = new t.Goblin.PointConstraint(previous_plank, right_point_front, plank, left_point_front);
				constraint.breaking_threshold = 8;
				w.addConstraint(constraint);
			}

			previous_plank = plank;
		}

		t.onTick(function (world, tick) {
			if (tick % 60 === 1) {
				t.sphere(world, 1, 1, { pos: [Math.random() * 4 - 2, 8, 0], color: '#4af' });
			}
		});

		t.expect('scene ran to completion (this test is for watching, not asserting)', function (world) {
			return world.ticks >= 600;
		});
		t.simulate(w, 600);
	}, {
		visual: true, steps: 600, page: 'constraint-point',
		description:
			"An 11-plank bridge is linked end-to-end with paired PointConstraints, anchored at both ends " +
			"over a ground plane, and a sphere is dropped onto it every 60 ticks, matching Chandler's " +
			"constraint-point.html example exactly. No pass/fail criteria yet — it just runs for 600 ticks " +
			"so the bridge's sag and stability can be watched in the viewer."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
