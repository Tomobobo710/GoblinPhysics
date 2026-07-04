// Chandler's math/quaternion.html — ported. Pure math, exact-value checks. The Quaternion constructor
// normalizes, so instantiate/multiply results are the normalized values his test expects.
(function (Runner) {
	Runner.suite('chandler');
	var G = typeof module !== 'undefined' && module.exports ? require('../../../build/goblin.js') : window.Goblin;
	var Q = G.Quaternion;
	var DESC = "Quaternions encode rotation without the gimbal-lock problems of angles. This checks the " +
		"core operations the engine spins bodies with: construction (which auto-normalizes to a valid unit " +
		"rotation), multiplication (composing rotations), normalization, and inversion (undoing a rotation) " +
		"— all against exact expected values.";
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'quaternion', description: DESC }); }

	test('math/quaternion', 'instantiate (identity / normalized-on-construct)', function (t) {
		var a = new Q(); t.checkEqual(a.x, 0); t.checkEqual(a.y, 0); t.checkEqual(a.z, 0); t.checkEqual(a.w, 1);
		var b = new Q(1, 3, 5.5, 8);
		t.checkEqual(b.x, 0.09794042137487835); t.checkEqual(b.y, 0.29382126412463505);
		t.checkEqual(b.z, 0.5386723175618309); t.checkEqual(b.w, 0.7835233709990268);
	});
	test('math/quaternion', 'multiply', function (t) {
		var q1 = new Q(1, 3, 5.5, 8), q2 = new Q(0, 0, 0, 1); q1.multiply(q2);
		t.checkEqual(q1.x, 0.09794042137487835); t.checkEqual(q1.y, 0.29382126412463505);
		t.checkEqual(q1.z, 0.5386723175618309); t.checkEqual(q1.w, 0.7835233709990268);
		q2 = new Q(0.5, 1, 1, 0.5); q1.multiply(q2);
		t.checkEqual(q1.x, 0.12388592261650222); t.checkEqual(q1.y, 0.6968583147178248);
		t.checkEqual(q1.z, 0.6349153534095736); t.checkEqual(q1.w, -0.30971480654125544);
	});
	test('math/quaternion', 'multiplyQuaternions', function (t) {
		var q1 = new Q(1, 3, 5.5, 8), q2 = new Q(0, 0, 0, 1), q3 = new Q(); q3.multiplyQuaternions(q1, q2);
		t.checkEqual(q3.x, 0.09794042137487835); t.checkEqual(q3.y, 0.29382126412463505);
		t.checkEqual(q3.z, 0.5386723175618309); t.checkEqual(q3.w, 0.7835233709990268);
		q2 = new Q(0.5, 1, 1, 0.5); q3.multiplyQuaternions(q1, q2);
		t.checkEqual(q3.x, 0.12388592261650222); t.checkEqual(q3.y, 0.6968583147178248);
		t.checkEqual(q3.z, 0.6349153534095736); t.checkEqual(q3.w, -0.30971480654125544);
	});
	test('math/quaternion', 'normalize', function (t) {
		var a = new Q(); a.normalize();
		t.checkEqual(a.x, 0); t.checkEqual(a.y, 0); t.checkEqual(a.z, 0); t.checkEqual(a.w, 1);
		var b = new Q(1, 0, 0, 1); b.normalize();
		t.checkEqual(b.x, 0.7071067811865476); t.checkEqual(b.y, 0); t.checkEqual(b.z, 0); t.checkEqual(b.w, 0.7071067811865476);
		var c = new Q(2, 1, 1, 1); c.normalize();
		t.checkEqual(c.x, 0.7559289460184546); t.checkEqual(c.y, 0.3779644730092273);
		t.checkEqual(c.z, 0.3779644730092273); t.checkEqual(c.w, 0.3779644730092273);
	});
	test('math/quaternion', 'invertQuaternion', function (t) {
		var q1 = new Q(), q2 = new Q(); q2.invertQuaternion(q1);
		t.checkEqual(q2.x, 0); t.checkEqual(q2.y, 0); t.checkEqual(q2.z, 0); t.checkEqual(q2.w, 1);
		q1 = new Q(1, 0, 0, 1); q2.invertQuaternion(q1);
		t.checkEqual(q2.x, -0.7071067811865476); t.checkEqual(q2.y, 0); t.checkEqual(q2.z, 0); t.checkEqual(q2.w, 0.7071067811865476);
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
