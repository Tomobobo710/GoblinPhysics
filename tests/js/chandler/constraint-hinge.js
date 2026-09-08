// Chandler's constraint-hinge.html — ported exactly.
// A plank hinged to a fixed point in the world (single-body HingeConstraint, no object_b) over a
// ground plane at y=-10, with a limit and a motor driving it. A sphere is spawned every 240 ticks at
// (-3, 5, 0), just like the example's stepStart listener. No behavioral assertions yet — just runs
// for 600 ticks so it can be watched.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('constraint-hinge', 'motorized hinge plank catches dropped spheres', function (t) {
		t.log('Hinging a plank to a fixed point with a motor, then dropping spheres onto it; just watching it run for now.');

		var w = t.makeWorld({ gravity: -9.8 });

		t.plane(w, 1, 20, 20, 0, { pos: [0, -10, 0], color: '#243B2A' });

		var plank = t.box(w, 5, 0.3, 2, 1, { color: '#B08968' });

		var constraint = new t.Goblin.HingeConstraint(
			plank,
			t.vec(0, 0, 1),   // axis of allowed rotation in plank's reference frame
			t.vec(-4, 0, 0)   // point on plank that is the hinge
		);
		constraint.limit.set(-Math.PI / 8, 0);
		constraint.motor.set(40, 1);
		w.addConstraint(constraint);

		t.onTick(function (world, tick) {
			if (tick % 240 === 0) {
				t.sphere(world, 1, 1, { pos: [-3, 5, 0], color: '#4af' });
			}
		});

		t.expect('scene ran to completion (this test is for watching, not asserting)', function (world) {
			return world.ticks >= 600;
		});
		t.simulate(w, 600);
	}, {
		visual: true, steps: 600, page: 'constraint-hinge',
		description:
			"A plank is hinged to a fixed world point (limit + motor) over a ground plane, and a sphere is " +
			"dropped onto it every 240 ticks, matching Chandler's constraint-hinge.html example exactly. " +
			"No pass/fail criteria yet — it just runs for 600 ticks so the hinge's behavior can be watched."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
