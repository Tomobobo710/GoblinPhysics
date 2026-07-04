// Chandler's restitution.html — ported.
// Gravity off. Four restitution sub-tests share ONE world, laid out along x (0/3/6/9). We keep them in
// one standalone test because the solver's cross-body state is part of his result — an isolated world
// lands ~0.0003 off his 0.0001 epsilon. Building his exact shared world reproduces his numbers exactly.
//
// The criteria are LIVE predicates checked every tick against the running world. Each flips green the
// moment the ball reaches the velocity it should have after the bounce — not at a tick we picked.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('restitution', 'four restitution sub-tests (shared world)', function (t) {
		t.log('Restitution = "bounciness": the fraction of approach speed that survives a collision.');
		t.log('Gravity is off, so the only thing that changes a ball\'s velocity is the hit itself.');

		var w = t.makeWorld({ gravity: 0 });
		var t1_stat = t.sphere(w, 1, 0, { pos: [0, 0, 0], restitution: 1, color: '#888' });
		var t1_dyn = t.sphere(w, 1, 1, { pos: [0, 5, 0], vel: [0, -3, 0], restitution: 1, color: '#F4D35E' });
		var t2_stat = t.sphere(w, 1, 0, { pos: [3, 0, 0], restitution: 0.2, color: '#888' });
		var t2_dyn = t.sphere(w, 1, 1, { pos: [3, 5, 0], vel: [0, -3, 0], restitution: 0.2, color: '#EE964B' });
		var t3_a = t.sphere(w, 1, 1, { pos: [6, 0, 0], restitution: 1, color: '#45B7D1' });
		var t3_b = t.sphere(w, 1, 1, { pos: [6, 3, 0], vel: [0, -2, 0], restitution: 1, color: '#45B7D1' });
		var t4_a = t.sphere(w, 1, 1, { pos: [9, 0, 0], color: '#8367C7' });
		var t4_b = t.sphere(w, 1, 1, { pos: [9, 3, 0], vel: [0, -2, 0], color: '#8367C7' });

		// "reaches velocity v (±eps)" — checked live each tick.
		function reachesVy(b, v, eps) { return function () { var d = Math.abs(b.linear_velocity.y - v); return { ok: d <= eps, detail: 'vy=' + b.linear_velocity.y.toFixed(4) }; }; }
		function sepSpeed(a, b, v, eps) { return function () { var s = a.linear_velocity.length() + b.linear_velocity.length(); return { ok: Math.abs(s - v) <= eps, detail: 'sep=' + s.toFixed(4) }; }; }

		t.expect('Test 1 — elastic ball bounces off the wall at full speed (vy → +3)', reachesVy(t1_dyn, 3, 0.0001));
		t.expect('Test 2 — soft ball (e=0.2) keeps 20% after the bounce (vy → +0.6)', reachesVy(t2_dyn, 0.6, 0.0001));
		t.expect('Test 3 — elastic Newton\'s-cradle transfers all motion (separating speed → 2)', sepSpeed(t3_a, t3_b, 2, 0.0001));
		t.expect('Test 4 — default restitution conserves the closing speed (separating speed → 2)', sepSpeed(t4_a, t4_b, 2, 0.0001));

		t.log('Watching each ball bounce — a check goes green the tick it reaches its post-bounce speed.');
		t.simulate(w, 150);
	}, {
		visual: true, steps: 150, page: 'restitution',
		description:
			"Restitution is how bouncy a collision is: e=1 keeps all the closing speed (perfect bounce), " +
			"e=0 keeps none (dead stop). Gravity is off, so the only thing that changes a ball's velocity " +
			"is the hit itself. Four cases run in one world side-by-side: (1) an elastic ball off a fixed " +
			"wall rebounds at full speed (+3); (2) an e=0.2 ball keeps 20% (+0.6); (3) & (4) one ball " +
			"strikes an equal resting ball and hands over its motion (Newton's cradle), so the total " +
			"separating speed still equals the 2 u/s that came in. Each check flips green the moment its " +
			"ball reaches the right post-bounce speed."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
