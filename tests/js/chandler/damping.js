// Chandler's damping.html — ported.
// One world at default gravity (-9.8), nine spheres along x, each isolating a damping case. Damping
// bleeds velocity over time: 0 = none, 0.5 = moderate, 0.9 = strong. Bodies that test gravity keep world
// gravity; the rest zero their own and get a linear impulse or spin. His expected values are the
// velocities each reaches after ~2s. Tolerance 0.25.
//
// Live predicate: each case flips green the tick its velocity reaches the expected damped value — you
// watch the damped ones bleed down toward theirs while the un-damped ones hold steady.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('damping', 'nine damping cases (gravity/linear/angular × none/0.5/0.9)', function (t) {
		t.log('Damping bleeds velocity away over time: 0 = none, 0.5 = moderate, 0.9 = strong.');

		var w = t.makeWorld();   // default gravity -9.8, like his page
		var b1 = t.sphere(w, 1, 1, { pos: [-12, 0, 0], color: '#F4D35E' });
		var b2 = t.sphere(w, 1, 1, { pos: [-9, 0, 0], noGravity: true, color: '#F4D35E' }); b2.applyImpulse(t.vec(0, -10, 0));
		var b3 = t.sphere(w, 1, 1, { pos: [-6, 0, 0], noGravity: true, avel: [0, 0, 5], color: '#F4D35E' });
		var b4 = t.sphere(w, 1, 1, { pos: [-3, 0, 0], linear_damping: 0.5, color: '#EE964B' });
		var b5 = t.sphere(w, 1, 1, { pos: [0, 0, 0], noGravity: true, linear_damping: 0.5, color: '#EE964B' }); b5.applyImpulse(t.vec(0, -10, 0));
		var b6 = t.sphere(w, 1, 1, { pos: [3, 0, 0], noGravity: true, angular_damping: 0.5, avel: [0, 0, 5], color: '#EE964B' });
		var b7 = t.sphere(w, 1, 1, { pos: [6, 0, 0], linear_damping: 0.9, color: '#45B7D1' });
		var b8 = t.sphere(w, 1, 1, { pos: [9, 0, 0], noGravity: true, linear_damping: 0.9, color: '#45B7D1' }); b8.applyImpulse(t.vec(0, -10, 0));
		var b9 = t.sphere(w, 1, 1, { pos: [12, 0, 0], noGravity: true, angular_damping: 0.9, avel: [0, 0, 5], color: '#45B7D1' });

		function vy(b, v) { return function () { return { ok: Math.abs(b.linear_velocity.y - v) <= 0.25, detail: 'vy=' + b.linear_velocity.y.toFixed(2) }; }; }
		function wz(b, v) { return function () { return { ok: Math.abs(b.angular_velocity.z - v) <= 0.25, detail: 'wz=' + b.angular_velocity.z.toFixed(2) }; }; }

		t.expect('no damping — gravity pulls to vy ≈ -19.6', vy(b1, -19.6));
		t.expect('no damping — pushed body holds vy ≈ -10', vy(b2, -10));
		t.expect('no damping — spin holds wz ≈ 5', wz(b3, 5));
		t.expect('0.5 damping — gravity fights drag, settles vy ≈ -10.54', vy(b4, -10.54));
		t.expect('0.5 damping — pushed body bleeds to vy ≈ -2.5', vy(b5, -10 / 4));
		t.expect('0.5 damping — spin bleeds to wz ≈ 1.25', wz(b6, 5 / 4));
		t.expect('0.9 strong damping — gravity barely wins, vy ≈ -4.13', vy(b7, -4.13));
		t.expect('0.9 strong damping — pushed body nearly stops, vy ≈ -0.01', vy(b8, -10 / 1000));
		t.expect('0.9 strong damping — spin nearly stops, wz ≈ 0.005', wz(b9, 5 / 1000));

		t.log('Damped cases bleed toward their value; un-damped ones hold — each flips when it arrives.');
		t.simulate(w, 130);
	}, {
		visual: true, steps: 130, page: 'damping',
		description:
			"Damping is drag that bleeds a body's velocity away over time. Nine spheres isolate every " +
			"combination: linear vs angular motion, driven by gravity / a push / a spin, with no damping, " +
			"moderate (0.5), or strong (0.9) drag. No-damping bodies hold their motion; damped ones decay " +
			"toward a lower steady value (strong damping nearly to a stop). Each check flips green when its " +
			"body reaches the expected velocity (±0.25). PASS: all nine land on their damped values."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
