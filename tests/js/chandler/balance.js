// Chandler's balance.html — ported.
// A static sphere (radius 1) at the origin with two dynamic unit spheres dropped straight down the Y
// axis from y=3 and y=7. They should stack into a 3-ball column: centers at y=0 / 2 / 4.
//
// The criteria are LIVE predicates, checked every tick against the running world. Each goes green the
// moment the physics satisfies it — the middle ball flips when it actually comes to rest at y=2, the top
// when it rests at y=4 — not at any tick we chose. If a ball never gets there, its criterion goes red.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('balance', 'two spheres stack on a static one (y=2, y=4)', function (t) {
		t.log('Dropping two balls onto a fixed ball; do they stack into a stable column?');

		var w = t.makeWorld({ gravity: -9.8 });
		var stat = t.sphere(w, 1, 0, { pos: [0, 0, 0], color: '#888' });
		var d1 = t.sphere(w, 1, 1, { pos: [0, 3, 0], color: '#F4D35E' });   // middle
		var d2 = t.sphere(w, 1, 1, { pos: [0, 7, 0], color: '#45B7D1' });   // top

		// A ball is "resting at height h" when it's at h (±0.25, his tolerance) AND barely moving.
		function restingAt(b, h) {
			return function () {
				var dy = Math.abs(b.position.y - h), slow = Math.abs(b.linear_velocity.y) < 0.1;
				return { ok: dy <= 0.25 && slow, detail: 'y=' + b.position.y.toFixed(2) + ' vy=' + b.linear_velocity.y.toFixed(2) };
			};
		}
		t.expect('middle ball comes to rest on the static one (y ≈ 2)', restingAt(d1, 2));
		t.expect('top ball comes to rest on the middle one (y ≈ 4)', restingAt(d2, 4));

		t.log('Watching the column form — each check goes green when its ball actually settles.');
		t.simulate(w, 180);   // give them time to fall and settle; checks flip whenever they're satisfied
	}, {
		visual: true, steps: 180, page: 'balance',
		description:
			"Two balls are dropped straight down onto a fixed ball. A correct solver stacks them into a " +
			"steady 3-ball column (centers at y=0, 2, 4) instead of letting them slip off or sink through. " +
			"Each check is watched live: the middle ball's turns green the instant it settles at y=2, the " +
			"top ball's when it settles at y=4. PASS: both balls come to rest at the right heights."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
