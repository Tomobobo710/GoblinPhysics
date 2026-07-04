// Chandler's math/matrix3.html — ported. Pure math, exact-value checks against his full-precision
// literals (including the float-noise ones like 2.220446049250313e-16 from quaternion-derived matrices).
(function (Runner) {
	Runner.suite('chandler');
	var G = typeof module !== 'undefined' && module.exports ? require('../../../build/goblin.js') : window.Goblin;
	var M3 = G.Matrix3, Q = G.Quaternion, V3 = G.Vector3;

	function all(t, m, v) { // v in order e00,e10,e20,e01,e11,e21,e02,e12,e22
		var k = ['e00','e10','e20','e01','e11','e21','e02','e12','e22'];
		for (var i = 0; i < 9; i++) t.checkEqual(m[k[i]], v[i]);
	}
	var DESC = "A Matrix3 is a 3×3 rotation/scale matrix — how the engine transforms directions and " +
		"inertia. This checks identity, building one from a quaternion, transforming a vector by it, " +
		"transpose, inverse, and matrix multiplication, all against exact expected values (including the " +
		"tiny floating-point residues like 2.2e-16 that a real rotation produces).";
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'matrix3', description: DESC }); }

	test('math/matrix3', 'instantiate (all zero)', function (t) { all(t, new M3(), [0,0,0,0,0,0,0,0,0]); });
	test('math/matrix3', 'identity', function (t) { var m = new M3(); m.identity(); all(t, m, [1,0,0,0,1,0,0,0,1]); });
	test('math/matrix3', 'fromQuaternion', function (t) {
		var m = new M3(); m.fromQuaternion(new Q(0, 0, 0, 1));
		t.checkEqual(m.e00,1); t.checkEqual(m.e01,0); t.checkEqual(m.e02,0);
		t.checkEqual(m.e10,0); t.checkEqual(m.e11,1); t.checkEqual(m.e12,0);
		t.checkEqual(m.e20,0); t.checkEqual(m.e21,0); t.checkEqual(m.e22,1);
		m.fromQuaternion(new Q(1, 0, 0, 1));
		t.checkEqual(m.e00,1); t.checkEqual(m.e01,0); t.checkEqual(m.e02,0);
		t.checkEqual(m.e10,0); t.checkEqual(m.e11,2.220446049250313e-16); t.checkEqual(m.e12,-0.9999999999999998);
		t.checkEqual(m.e20,0); t.checkEqual(m.e21,0.9999999999999998); t.checkEqual(m.e22,2.220446049250313e-16);
	});
	test('math/matrix3', 'transformVector3', function (t) {
		var m = new M3(); m.identity();
		var v = new V3(1, 3, 5.5); m.transformVector3(v); t.checkEqual(v.x,1); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5);
		v = new V3(1, 3, 5.5); m.fromQuaternion(new Q(0, 0, 0, 1)); m.transformVector3(v);
		t.checkEqual(v.x,1); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5);
		m.fromQuaternion(new Q(0, 1, 0, 1)); m.transformVector3(v);
		t.checkEqual(v.x,5.499999999999999); t.checkEqual(v.y,3); t.checkEqual(v.z,-0.9999999999999986);
	});
	test('math/matrix3', 'transformVector3Into', function (t) {
		var m = new M3(), dest = new V3(); m.identity();
		var v = new V3(1, 3, 5.5); m.transformVector3Into(v, dest); t.checkEqual(dest.x,1); t.checkEqual(dest.y,3); t.checkEqual(dest.z,5.5);
		v = new V3(1, 3, 5.5); m.fromQuaternion(new Q(0, 0, 0, 1)); m.transformVector3Into(v, dest);
		t.checkEqual(dest.x,1); t.checkEqual(dest.y,3); t.checkEqual(dest.z,5.5);
		m.fromQuaternion(new Q(0, 1, 0, 1)); m.transformVector3Into(v, dest);
		t.checkEqual(dest.x,5.499999999999999); t.checkEqual(dest.y,3); t.checkEqual(dest.z,-0.9999999999999986);
	});
	test('math/matrix3', 'transpose (transposeInto)', function (t) {
		var m = new M3(1, 2, 3, 4, 5, 6, 7, 8, 9), dest = new M3(); m.transposeInto(dest); all(t, dest, [1,2,3,4,5,6,7,8,9]);
	});
	test('math/matrix3', 'invert', function (t) {
		var m = new M3(); m.fromQuaternion(new Q(0, 0, 0, 1)); m.invert();
		var v = new V3(1, 3, 5.5); m.transformVector3(v); t.checkEqual(v.x,1); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5);
		m.fromQuaternion(new Q(0, 1, 0, 1)); m.invert();
		v = new V3(1, 3, 5.5); m.transformVector3(v);
		t.checkEqual(v.x,-5.500000000000001); t.checkEqual(v.y,3); t.checkEqual(v.z,1.0000000000000016);
	});
	test('math/matrix3', 'invertInto', function (t) {
		var m = new M3(), dest = new M3(); m.fromQuaternion(new Q(0, 0, 0, 1)); m.invertInto(dest);
		var v = new V3(1, 3, 5.5); dest.transformVector3(v); t.checkEqual(v.x,1); t.checkEqual(v.y,3); t.checkEqual(v.z,5.5);
		m.fromQuaternion(new Q(0, 1, 0, 1)); m.invertInto(dest);
		v = new V3(1, 3, 5.5); dest.transformVector3(v);
		t.checkEqual(v.x,-5.500000000000001); t.checkEqual(v.y,3); t.checkEqual(v.z,1.0000000000000016);
	});
	test('math/matrix3', 'multiply', function (t) {
		var a = new M3(1, 0, 0, 0, 1, 0, 0, 0, 1), b = new M3(1, 2, 3, 4, 5, 6, 7, 8, 9); b.multiply(a);
		all(t, b, [1,4,7,2,5,8,3,6,9]); // note: all() order is e00,e10,e20,e01,e11,e21,e02,e12,e22
		a.fromQuaternion(new Q(1, 0, 0, 1)); b = new M3(1, 2, 3, 4, 5, 6, 7, 8, 9); b.multiply(a);
		t.checkEqual(b.e00,1); t.checkEqual(b.e10,4); t.checkEqual(b.e20,7);
		t.checkEqual(b.e01,2.9999999999999996); t.checkEqual(b.e11,5.999999999999999); t.checkEqual(b.e21,9);
		t.checkEqual(b.e02,-1.999999999999999); t.checkEqual(b.e12,-4.999999999999998); t.checkEqual(b.e22,-7.9999999999999964);
	});
	test('math/matrix3', 'multiplyFrom', function (t) {
		var a = new M3(1, 0, 0, 0, 1, 0, 0, 0, 1), b = new M3(1, 2, 3, 4, 5, 6, 7, 8, 9), c = new M3(); c.multiplyFrom(b, a);
		all(t, c, [1,4,7,2,5,8,3,6,9]);
		a.fromQuaternion(new Q(1, 0, 0, 1)); b = new M3(1, 2, 3, 4, 5, 6, 7, 8, 9); c.multiplyFrom(b, a);
		t.checkEqual(c.e00,1); t.checkEqual(c.e10,4); t.checkEqual(c.e20,7);
		t.checkEqual(c.e01,2.9999999999999996); t.checkEqual(c.e11,5.999999999999999); t.checkEqual(c.e21,9);
		t.checkEqual(c.e02,-1.999999999999999); t.checkEqual(c.e12,-4.999999999999998); t.checkEqual(c.e22,-7.9999999999999964);
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
