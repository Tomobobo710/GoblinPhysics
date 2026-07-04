// Chandler's gravity.html — ported.
// One world at g=-10, four bodies spread along x (never touching). Under constant gravity every body
// accelerates downward at the same rate regardless of mass, reaching vy = -20 after 2s (120 frames).
//
// Live predicate: each body's criterion flips green the tick its downward speed reaches 20 — you watch
// them all cross the line together (same acceleration), proving gravity is mass-independent.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('gravity', 'four bodies free-fall to vy=-20 (g=-10, t=2s)', function (t) {
		t.log('Constant gravity accelerates every body downward at the same rate, whatever its mass.');

		var w = t.makeWorld({ gravity: -10 });
		var sphere1 = t.sphere(w, 1, 1, { pos: [0, 0, 0], color: '#F4D35E' });
		var sphere2 = t.sphere(w, 1, 10, { pos: [3, 0, 0], color: '#EE964B' });
		var box1 = t.box(w, 1, 1, 1, 1, { pos: [6, 0, 0], color: '#45B7D1' });
		var box2 = t.box(w, 1, 1, 1, 0.01, { pos: [9, 0, 0], color: '#8367C7' });

		// "reaches vy = -20" — true the tick the body's downward speed hits 20 (his sample value at t=2s).
		function reachesMinus20(b) { return function () { return { ok: Math.abs(b.linear_velocity.y + 20) <= 1e-5, detail: 'vy=' + b.linear_velocity.y.toFixed(3) }; }; }
		t.expect('sphere (mass 1) reaches vy = -20', reachesMinus20(sphere1));
		t.expect('sphere (mass 10) reaches vy = -20 — same as mass 1', reachesMinus20(sphere2));
		t.expect('box (mass 1) reaches vy = -20', reachesMinus20(box1));
		t.expect('box (mass 0.01) reaches vy = -20 — mass does not matter', reachesMinus20(box2));

		t.log('Watch all four cross -20 on the same frame — that\'s mass-independence.');
		t.simulate(w, 130);
	}, {
		visual: true, steps: 130, page: 'gravity',
		description:
			"Four bodies of very different mass (spheres of mass 1 and 10, boxes of mass 1 and 0.01) fall " +
			"under gravity g=-10. Galileo's point: gravity accelerates everything equally, so after 2 " +
			"seconds every body — heavy or light — is falling at exactly vy = -20. Each check flips green " +
			"the frame its body reaches -20, and they all cross together. PASS: all four hit -20."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
