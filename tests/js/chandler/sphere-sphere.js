// Chandler's sphere-sphere.html — ported.
// Gravity off. Two e=0.2 spheres approach head-on (s1 at origin moving up +2, s2 above at y=3 moving
// down -2). After the (mostly inelastic) collision they separate slowly: s1 -> -0.4, s2 -> +0.4.
//
// Live predicate: each ball's criterion flips green the tick it reaches its post-collision velocity —
// you watch them close, hit, and rebound apart at the reduced speed.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('sphere-sphere', 'e=0.2 head-on: s1 -> -0.4, s2 -> +0.4', function (t) {
		t.log('Two low-restitution balls collide head-on; they should bounce apart slowly (e=0.2).');

		var w = t.makeWorld({ gravity: 0 });
		var s1 = t.sphere(w, 1, 1, { pos: [0, 0, 0], vel: [0, 2, 0], restitution: 0.2, color: '#F4D35E' });
		var s2 = t.sphere(w, 1, 1, { pos: [0, 3, 0], vel: [0, -2, 0], restitution: 0.2, color: '#45B7D1' });

		function reachesVy(b, v) { return function () { return { ok: Math.abs(b.linear_velocity.y - v) <= 0.001, detail: 'vy=' + b.linear_velocity.y.toFixed(3) }; }; }
		t.expect('bottom ball ends moving down slowly (vy → -0.4)', reachesVy(s1, -0.4));
		t.expect('top ball ends moving up slowly (vy → +0.4)', reachesVy(s2, 0.4));

		t.log('They meet, collide, and rebound at 20% of the closing speed — each flips when it arrives.');
		t.simulate(w, 120);
	}, {
		visual: true, steps: 120, page: 'sphere-sphere',
		description:
			"Two equal balls fly straight at each other at 2 u/s, each with restitution 0.2 (barely bouncy). " +
			"A head-on collision at e=0.2 keeps only 20% of the closing speed, so they rebound apart slowly: " +
			"the bottom ball ends drifting down at -0.4 and the top ball up at +0.4. Each check flips green " +
			"when its ball reaches that velocity. PASS: both land on ±0.4."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
