// Chandler's math/vector3.html — ported. Pure math, exact-value checks (his expect().property() and
// .to.equal() are strict ===). Each of his it()s is genuinely independent, so each is its own test.
(function (Runner) {
	Runner.suite('chandler');
	var G = typeof module !== 'undefined' && module.exports ? require('../../../build/goblin.js') : window.Goblin;
	var V3 = G.Vector3;
	var DESC = "The Vector3 math the whole engine is built on: construction, add/subtract/multiply, " +
		"scaling, length and normalization, dot and cross products, and distance. Every result is checked " +
		"against its exact expected value — the bedrock that positions, velocities, and forces rely on.";
	// local wrapper: tag every test in this file with its page + shared description
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'vector3', description: DESC }); }

	test('math/vector3', 'instantiate (no values / with values)', function (t) {
		var a = new V3(); t.checkEqual(a.x, 0); t.checkEqual(a.y, 0); t.checkEqual(a.z, 0);
		var b = new V3(5, 3, 2.5); t.checkEqual(b.x, 5); t.checkEqual(b.y, 3); t.checkEqual(b.z, 2.5);
	});
	test('math/vector3', 'copy (leaves source intact)', function (t) {
		var a = new V3(1, 3, 5.5), b = new V3(); b.copy(a); a.scale(3);
		t.checkEqual(a.x, 3); t.checkEqual(a.y, 9); t.checkEqual(a.z, 16.5);
		t.checkEqual(b.x, 1); t.checkEqual(b.y, 3); t.checkEqual(b.z, 5.5);
	});
	test('math/vector3', 'add / addVectors', function (t) {
		var a = new V3(1, 3, 5.5), b = new V3(-2, 0.5, 4); a.add(b);
		t.checkEqual(a.x, -1); t.checkEqual(a.y, 3.5); t.checkEqual(a.z, 9.5);
		var c = new V3(1, 3, 5.5), d = new V3(-2, 0.5, 4), e = new V3(); e.addVectors(c, d);
		t.checkEqual(e.x, -1); t.checkEqual(e.y, 3.5); t.checkEqual(e.z, 9.5);
	});
	test('math/vector3', 'subtract / subtractVectors', function (t) {
		var a = new V3(1, 3, 5.5), b = new V3(-2, 0.5, 4); a.subtract(b);
		t.checkEqual(a.x, 3); t.checkEqual(a.y, 2.5); t.checkEqual(a.z, 1.5);
		var c = new V3(1, 3, 5.5), d = new V3(-2, 0.5, 4), e = new V3(); e.subtractVectors(c, d);
		t.checkEqual(e.x, 3); t.checkEqual(e.y, 2.5); t.checkEqual(e.z, 1.5);
	});
	test('math/vector3', 'multiply / multiplyVectors', function (t) {
		var a = new V3(1, 3, 5.5), b = new V3(-2, 0.5, 4); a.multiply(b);
		t.checkEqual(a.x, -2); t.checkEqual(a.y, 1.5); t.checkEqual(a.z, 22);
		var c = new V3(1, 3, 5.5), d = new V3(-2, 0.5, 4), e = new V3(); e.multiplyVectors(c, d);
		t.checkEqual(e.x, -2); t.checkEqual(e.y, 1.5); t.checkEqual(e.z, 22);
	});
	test('math/vector3', 'scale / scaleVector', function (t) {
		var a = new V3(1, 3, 5.5); a.scale(3.5);
		t.checkEqual(a.x, 3.5); t.checkEqual(a.y, 10.5); t.checkEqual(a.z, 19.25);
		var b = new V3(1, 3, 5.5), c = new V3(); c.scaleVector(b, 3.5);
		t.checkEqual(c.x, 3.5); t.checkEqual(c.y, 10.5); t.checkEqual(c.z, 19.25);
	});
	test('math/vector3', 'lengthSquared / length', function (t) {
		t.checkEqual(new V3(1, 3, 5.5).lengthSquared(), 40.25);
		t.checkEqual(new V3(1, 3, 5.5).length(), 6.34428877022476);
	});
	test('math/vector3', 'normalize (incl zero vector)', function (t) {
		var a = new V3(1, 3, 5.5); a.normalize();
		t.checkEqual(a.x, 0.15762208124782012); t.checkEqual(a.y, 0.47286624374346037); t.checkEqual(a.z, 0.8669214468630106);
		var b = new V3(); b.normalize();
		t.checkEqual(b.x, 0); t.checkEqual(b.y, 0); t.checkEqual(b.z, 0);
	});
	test('math/vector3', 'dot', function (t) {
		t.checkEqual(new V3(1, 3, 5.5).dot(new V3(-2, 0.5, 4)), 21.5);
	});
	test('math/vector3', 'cross / crossVectors', function (t) {
		var a = new V3(1, 3, 5.5), b = new V3(-2, 0.5, 4); a.cross(b);
		t.checkEqual(a.x, 9.25); t.checkEqual(a.y, -15); t.checkEqual(a.z, 6.5);
		var c = new V3(1, 3, 5.5), d = new V3(-2, 0.5, 4), e = new V3(); e.crossVectors(c, d);
		t.checkEqual(e.x, 9.25); t.checkEqual(e.y, -15); t.checkEqual(e.z, 6.5);
	});
	test('math/vector3', 'distanceTo', function (t) {
		t.checkEqual(new V3(1, 1, 1).distanceTo(new V3(3, 3, 2)), 3);
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
