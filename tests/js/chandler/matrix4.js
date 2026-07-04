// Chandler's math/matrix4.html — ported. Pure math, exact-value checks against his full-precision
// literals. all() lists e00..e33 in the row-major order his tests write them.
(function (Runner) {
	Runner.suite('chandler');
	var G = typeof module !== 'undefined' && module.exports ? require('../../../build/goblin.js') : window.Goblin;
	var M4 = G.Matrix4, Q = G.Quaternion, V3 = G.Vector3;

	function all(t, m, v) {
		var k = ['e00','e01','e02','e03','e10','e11','e12','e13','e20','e21','e22','e23','e30','e31','e32','e33'];
		for (var i = 0; i < 16; i++) t.checkEqual(m[k[i]], v[i]);
	}
	var DESC = "A Matrix4 is a 4×4 transform combining rotation AND translation — how a body's full pose " +
		"is represented. This checks identity, makeTransform (from a rotation + position), transforming and " +
		"rotating vectors (rotate ignores the translation part), and inversion (turning a world→local " +
		"transform into local→world), all against exact expected values.";
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'matrix4', description: DESC }); }

	test('math/matrix4', 'instantiate (all zero)', function (t) { all(t, new M4(), [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]); });
	test('math/matrix4', 'identity', function (t) { var m = new M4(); m.identity(); all(t, m, [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]); });
	test('math/matrix4', 'makeTransform', function (t) {
		var m = new M4();
		m.makeTransform(new Q(0, 0, 0, 1), new V3(0, 0, 0)); all(t, m, [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]);
		m.makeTransform(new Q(0, 0, 0, 1), new V3(1, 3.5, 5)); all(t, m, [1,0,0,1, 0,1,0,3.5, 0,0,1,5, 0,0,0,1]);
		m.makeTransform(new Q(0, 1, 0, 1), new V3(1, 0, 0));
		all(t, m, [2.220446049250313e-16,0,0.9999999999999998,1, 0,1,0,0,
			-0.9999999999999998,0,2.220446049250313e-16,0, 0,0,0,1]);
	});
	test('math/matrix4', 'transformVector3', function (t) {
		var m = new M4(); m.identity();
		var v = new V3(1, 3, 5.5); m.transformVector3(v); t.checkEqual(v.x,1); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5);
		v = new V3(1, 3, 5.5); m.makeTransform(new Q(0, 0, 0, 1), new V3(0, 0, 0)); m.transformVector3(v);
		t.checkEqual(v.x,1); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5);
		v = new V3(1, 3, 5.5); m.makeTransform(new Q(0, 0, 0, 1), new V3(-1, 0, 0)); m.transformVector3(v);
		t.checkEqual(v.x,0); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5);
		v = new V3(1, 3, 5.5); m.makeTransform(new Q(0, 1, 0, 1), new V3(-1, 0, 0)); m.transformVector3(v);
		t.checkEqual(v.x,4.499999999999999); t.checkEqual(v.y,3); t.checkEqual(v.z,-0.9999999999999986);
	});
	test('math/matrix4', 'transformVector3Into', function (t) {
		var m = new M4(), v = new V3(1, 3, 5.5), dest = new V3(); m.identity(); m.transformVector3Into(v, dest);
		t.checkEqual(dest.x,1); t.checkEqual(dest.y,3); t.checkEqual(dest.z,5.5);
		m.makeTransform(new Q(0, 0, 0, 1), new V3(0, 0, 0)); m.transformVector3Into(v, dest);
		t.checkEqual(dest.x,1); t.checkEqual(dest.y,3); t.checkEqual(dest.z,5.5);
		m.makeTransform(new Q(0, 0, 0, 1), new V3(-1, 0, 0)); m.transformVector3Into(v, dest);
		t.checkEqual(dest.x,0); t.checkEqual(dest.y,3); t.checkEqual(dest.z,5.5);
		m.makeTransform(new Q(0, 1, 0, 1), new V3(-1, 0, 0)); m.transformVector3Into(v, dest);
		t.checkEqual(dest.x,4.499999999999999); t.checkEqual(dest.y,3); t.checkEqual(dest.z,-0.9999999999999986);
	});
	test('math/matrix4', 'rotateVector3', function (t) {
		var m = new M4(); m.identity();
		var v = new V3(1, 3, 5.5); m.rotateVector3(v); t.checkEqual(v.x,1); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5);
		v = new V3(1, 3, 5.5); m.makeTransform(new Q(0, 0, 0, 1), new V3(0, 0, 0)); m.rotateVector3(v);
		t.checkEqual(v.x,1); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5);
		v = new V3(1, 3, 5.5); m.makeTransform(new Q(0, 0, 0, 1), new V3(-1, 0, 0)); m.rotateVector3(v);
		t.checkEqual(v.x,1); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5); // rotate ignores translation
		v = new V3(1, 3, 5.5); m.makeTransform(new Q(0, 1, 0, 1), new V3(-1, 0, 0)); m.rotateVector3(v);
		t.checkEqual(v.x,5.499999999999999); t.checkEqual(v.y,3); t.checkEqual(v.z,-0.9999999999999986);
	});
	test('math/matrix4', 'invert', function (t) {
		var m = new M4(); m.makeTransform(new Q(0, 0, 0, 1), new V3(1, 3, 5.5)); m.invert();
		var v = new V3(0, 0, 0);
		t.checkEqual(m.e03,-1); t.checkEqual(m.e13,-3); t.checkEqual(m.e23,-5.5);
		m.transformVector3(v); t.checkEqual(v.x,-1); t.checkEqual(v.y,-3); t.checkEqual(v.z,-5.5);
		m.makeTransform(new Q(1, 0, 0, 1), new V3(1, 3, 5.5)); m.invert();
		v = new V3(18, -3, 12); m.transformVector3(v);
		t.checkEqual(v.x,17); t.checkEqual(v.y,6.500000000000001); t.checkEqual(v.z,6.000000000000003);
	});
	test('math/matrix4', 'invertInto', function (t) {
		var m = new M4(), dest = new M4(); m.makeTransform(new Q(0, 0, 0, 1), new V3(1, 3, 5.5)); m.invertInto(dest);
		var v = new V3(0, 0, 0);
		t.checkEqual(dest.e03,-1); t.checkEqual(dest.e13,-3); t.checkEqual(dest.e23,-5.5);
		dest.transformVector3(v); t.checkEqual(v.x,-1); t.checkEqual(v.y,-3); t.checkEqual(v.z,-5.5);
		m.makeTransform(new Q(1, 0, 0, 1), new V3(1, 3, 5.5)); m.invertInto(dest);
		v = new V3(18, -3, 12); dest.transformVector3(v);
		t.checkEqual(v.x,17); t.checkEqual(v.y,6.500000000000001); t.checkEqual(v.z,6.000000000000003);
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
