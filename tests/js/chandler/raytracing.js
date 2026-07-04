// Chandler's raytracing.html — ported. rayIntersect(start, stop, list) for every shape, checking the
// first hit point (37 cases). EPSILON 0.01. Like support-points, his page reused one body per shape but
// set pos/rot explicitly where needed (resetting to origin for rotated cases), so a fresh body with the
// config's explicit transform (origin/identity if unset) reproduces his state — each is standalone.
// Each test records ctx.ray (start/stop/hit) so the browser viewer can draw the ray and hit marker.
(function (Runner) {
	Runner.suite('chandler');
	var EPS = 0.01, S2 = Math.sqrt(2), S45 = Math.sqrt(4.5);
	var PYR = [[-1,-1,-1],[1,-1,-1],[1,-1,1],[-1,-1,1],[0,2,0]];

	function mkShape(G, key) {
		switch (key) {
			case 'sphere':   return new G.SphereShape(2);
			case 'box':      return new G.BoxShape(3, 4, 5);
			case 'cylinder': return new G.CylinderShape(3, 2.5);
			case 'cone':     return new G.ConeShape(3, 2.5);
			case 'plane_x':  return new G.PlaneShape(0, 5, 10);
			case 'convex':   return new G.ConvexShape(PYR.map(function (v) { return new G.Vector3(v[0], v[1], v[2]); }));
			case 'compound':
				var cs = new G.CompoundShape();
				cs.addChildShape(new G.SphereShape(2), new G.Vector3(0, 2, -1), new G.Quaternion(0, 0, 1, 1));
				return cs;
		}
	}
	// [group, name, key, start[3], stop[3], expected[3], pos[3]|null, rot[4]|null]
	var R = [
		['Sphere','Base Sphere 1','sphere',[-4,0,0],[4,0,0],[-2,0,0],null,null],
		['Sphere','Base Sphere 2','sphere',[0,2,2],[0,-2,-2],[0,S2,S2],null,null],
		['Sphere','Base Sphere 3','sphere',[0,0,0],[2,0,-2],[S2,0,-S2],null,null],
		['Sphere','Translated Sphere 1','sphere',[2,3,0],[2,1,0],[2,1,0],[2,3,0],null],
		['Sphere','Translated Sphere 2','sphere',[0,5,0],[4,5,0],[2,5,0],[2,3,0],null],
		['Sphere','Rotated Sphere 1','sphere',[-4,0,0],[4,0,0],[-2,0,0],[0,0,0],[1,0,0,1]],
		['Sphere','Rotated Sphere 2','sphere',[0,2,2],[0,-2,-2],[0,S2,S2],[0,0,0],[0,1,0,1]],
		['Sphere','Translated & Rotated Sphere 1','sphere',[-10,0,1],[10,0,1],[4.13,0,1],[5,1.5,0],[1,0,0.5,1]],
		['Box','Base Box 1','box',[-5,0,0],[0,0,0],[-3,0,0],null,null],
		['Box','Base Box 2','box',[-2,5,0],[0,0,0],[-1.6,4,0],null,null],
		['Box','Translated Box','box',[5,-10,9],[5,0,9],[5,-7,9],[2,-3,4],null],
		['Box','Rotated Box','box',[0,0,10],[0,0,0],[0,0,3],[0,0,0],[0,1,0,1]],
		['Box','Translated & Rotated Box','box',[0,-2,-10],[0,-2,0],[0,-2,-4],[2,-3,0],[1,0,0,1]],
		['Plane','Base Plane','plane_x',[-10,2,0],[10,2,0],[0,2,0],null,null],
		['Plane','Translated Plane','plane_x',[-10,6,0],[10,6,0],[2,6,0],[2,1,0],null],
		['Plane','Rotated Plane','plane_x',[2,0,10],[2,0,-10],[2,0,0],[0,0,0],[0,1,0,1]],
		['Plane','Translated & Rotated Plane','plane_x',[0,6,10],[0,6,-10],[0,6,0],[0,1,0],[0,1,0,1]],
		['Cylinder','Base Cylinder 1','cylinder',[0,-10,0],[0,10,0],[0,-2.5,0],null,null],
		['Cylinder','Base Cylinder 2','cylinder',[-10,2,10],[10,1,-10],[-S45,1.6,S45],null,null],
		['Cylinder','Translated Cylinder 1','cylinder',[0,-10,0],[0,10,0],[0,-1.5,0],[2,1,0],null],
		['Cylinder','Translated Cylinder 2','cylinder',[-5,-10,-3],[5,10,-3],[0.25,0.5,-3],[3,3,-3],null],
		['Cylinder','Rotated Cylinder 1','cylinder',[0,-10,0],[0,10,0],[0,-2.5,0],[0,0,0],[0,1,0,1]],
		['Cylinder','Rotated Cylinder 2','cylinder',[0,-10,0],[0,10,0],[0,-3,0],[0,0,0],[1,0,0,1]],
		['Cylinder','Translated & Rotated Cylinder','cylinder',[-10,2,10],[10,1,-10],[-0.62,1.53,0.62],[2,3,0],[1,0,0,1]],
		['Cone','Base Cone 1','cone',[-10,-2.5,0],[10,-2.5,0],[-3,-2.5,0],null,null],
		['Cone','Base Cone 2','cone',[2.12,-10,-2.12],[2.12,0,-2.12],[2.12,-2.5,-2.12],null,null],
		['Cone','Base Cone 3','cone',[0,1,10],[0,1,-10],[0,1,0.9],null,null],
		['Cone','Translated Cone 1','cone',[0,1,10],[0,1,-10],[0,1,0.12],[1,1,-1],null],
		['Cone','Rotated Cone 1','cone',[0,1,10],[0,1,-10],[0,1,0.9],[0,0,0],[0,1,0,1]],
		['Cone','Rotated Cone 2','cone',[0,0,-10],[0,0,10],[0,0,-2.5],[0,0,0],[1,0,0,1]],
		['Cone','Translated & Rotated Cone 1','cone',[0,0,10],[0,0,-10],[0,0,-0.86],[1,1,-1],[1,0,0,1]],
		['Compound','Base Compound 1','compound',[0,10,0],[0,-10,0],[0,3.73,0],null,null],
		['Compound','Base Compound 2','compound',[0,10,-1.5],[0,-10,-1.5],[0,3.93,-1.5],null,null],
		['Compound','Translated Compound 1','compound',[0,10,-1.5],[0,-10,-1.5],[0,4.66,-1.5],[1,1,-1],null],
		['Compound','Rotated Compound 1','compound',[0,10,1],[0,-10,1],[0,4,1],[0,0,0],[0,1,0,0]],
		['Compound','Translated & Rotated Compound 1','compound',[0,10,0],[0,-10,0],[0,2,0],[1,-2,0],[0,1,0,1]],
		['Convex','Base Convex','convex',[-4,0,0],[4,0,0],[-0.67,0,0],null,null]
	];

	var DESC = "Raytracing shoots a line segment through a shape and asks where it first pierces the " +
		"surface. It's the basis for picking, line-of-sight, and ground checks. A ray is fired at each " +
		"shape (sphere, box, plane, cylinder, cone, a compound of shapes, and a convex hull), moved and " +
		"rotated in various ways, and the first hit point is checked against the known-correct location " +
		"(within 0.01). The 3D view draws the ray (blue) and the hit point (red).";

	R.forEach(function (row) {
		var group = row[0], name = row[1], key = row[2], start = row[3], stop = row[4], exp = row[5], pos = row[6], rot = row[7];
		Runner.test('raytracing/' + group.toLowerCase(), name, function (t) {
			var opts = { mass: key === 'sphere' ? 0 : 1 };
			if (pos) opts.pos = pos;
			if (rot) opts.rot = rot;
			var b = t.loneBody(mkShape(t.Goblin, key), opts);
			var list = [];
			b.rayIntersect(t.vec(start[0], start[1], start[2]), t.vec(stop[0], stop[1], stop[2]), list);
			// expose the ray + hit for the viewer (drawn from the SAME run that is asserted)
			t.ray = { start: start, stop: stop, hit: list.length ? [list[0].point.x, list[0].point.y, list[0].point.z] : null };
			t.checkTrue(list.length > 0, 'the ray hits the shape');
			if (list.length) t.check(list[0].point.distanceTo(t.vec(exp[0], exp[1], exp[2])), 0, EPS,
				'first hit is at (' + exp.map(function (n) { return +n.toFixed(2); }).join(',') + ')');
		}, { visual: true, steps: 0, page: 'raytracing', description: DESC });
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
