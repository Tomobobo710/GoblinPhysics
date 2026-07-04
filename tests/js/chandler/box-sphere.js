// Chandler's box-sphere.html — ported.
// A static box (half-extents 0.5) with a sphere (radius 1) dropped from y=5 under default gravity. It
// falls, contacts the box top, and comes to rest with its center at y ≈ 1.5 (box top 0.5 + radius 1).
//
// Live predicate: flips green the tick the sphere is resting on the box (at y ≈ 1.5 and no longer
// moving) — you watch it fall and settle.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('box-sphere', 'sphere rests on a static box at y ≈ 1.5', function (t) {
		t.log('A sphere is dropped onto a fixed box; does it land and rest on top?');

		var w = t.makeWorld({ gravity: -9.8 });
		var box = t.box(w, 0.5, 0.5, 0.5, 0, { color: '#888' });
		var sphere = t.sphere(w, 1, 1, { pos: [0, 5, 0], color: '#45B7D1' });

		t.expect('sphere lands and rests on the box (center y ≈ 1.5)', function () {
			var atRest = Math.abs(sphere.position.y - 1.5) <= 0.05 && Math.abs(sphere.linear_velocity.y) < 0.1;
			return { ok: atRest, detail: 'y=' + sphere.position.y.toFixed(3) + ' vy=' + sphere.linear_velocity.y.toFixed(2) };
		});

		t.log('It falls under gravity and should stop cleanly on the box top (0.5 + radius 1 = 1.5).');
		t.simulate(w, 150);
	}, {
		visual: true, steps: 150, page: 'box-sphere',
		description:
			"A sphere of radius 1 is dropped from y=5 onto a fixed box whose top is at y=0.5. Sphere-on-box " +
			"contact should catch it and hold it: its center comes to rest at y = 0.5 + 1 = 1.5, not sinking " +
			"in or bouncing away. The check flips green the frame the sphere is at 1.5 and no longer moving. " +
			"PASS: sphere rests at y ≈ 1.5."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
