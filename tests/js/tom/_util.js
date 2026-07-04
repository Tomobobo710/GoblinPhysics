// Tom's Suite — shared helpers (no tests here).
//
// These build the standard scene every scene here shares — a big static ground box with
// its top face at y=0 — and the settle oracle that most of Tom's regression tests share.
//
// Everything is a LIVE predicate: fed to t.expect(label, fn), evaluated every tick against the running
// world, identical headless and in-browser. Nothing is precomputed.
(function (root, factory) {
	var mod = factory();
	if (typeof module !== 'undefined' && module.exports) module.exports = mod;
	else root.TomUtil = mod;
})(typeof window !== 'undefined' ? window : this, function () {

	// Tom's material preset — the material these scenes put on their bodies. The high angular_damping is
	// deliberate and load-bearing: friction alone cannot stop a ROLLING contact (it opposes sliding at
	// the contact point, not spin), so without it any spin picked up on landing never bleeds off and round
	// shapes roll/buzz forever. A moderate linear_damping does the same for slow lateral creep. Tests that
	// must MEASURE raw bounce or contact energy (bounce-energy) override these back to bare values so the
	// damping doesn't mask the signal.
	var MAT = { friction: 3.0, restitution: 0.33, linear_damping: 0.1, angular_damping: 0.9 };

	// Merge Tom's material preset into an opts object without clobbering anything the caller set.
	function withMat(opts) {
		opts = opts || {};
		for (var k in MAT) if (Object.prototype.hasOwnProperty.call(MAT, k) && opts[k] == null) opts[k] = MAT[k];
		return opts;
	}

	// The 40x1x40 floor these scenes all use: a static box, top face at y=0. Carries the FULL preset —
	// the material is applied to every body regardless of mass, so the static floor gets the same friction
	// AND restitution as everything else (damping is inert on a zero-mass body but set for consistency).
	// The floor's restitution governs every bounce/settle against it, so it must match, not fall back to a
	// bare default.
	function ground(t, w) {
		return t.box(w, 20, 0.5, 20, 0, withMat({ pos: [0, -0.5, 0], color: '#243B2A' }));
	}

	// linear + angular speed of a raw Goblin body
	function speed(b) { var v = b.linear_velocity; return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
	function spin(b) { var a = b.angular_velocity; return Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

	// A settle predicate factory. Returns fn(world) for t.expect.
	//   b        the body that must come to rest
	//   opts.v   linear-speed threshold (default 0.05)
	//   opts.w   angular-speed threshold (default 0.05)
	//   opts.hold consecutive ticks it must stay under threshold before it counts (default 20)
	//   opts.minY the body must never sink below this (tunnel guard; default -2)
	//   opts.maxY the body must never climb above this (explode guard; default +50)
	// The predicate carries its own hold-counter in a closure, so a momentary zero-crossing does NOT
	// pass — it must genuinely be at rest for `hold` ticks in a row.
	function settles(b, opts) {
		opts = opts || {};
		var vThresh = opts.v != null ? opts.v : 0.05;
		var wThresh = opts.w != null ? opts.w : 0.05;
		var hold = opts.hold != null ? opts.hold : 20;
		var minY = opts.minY != null ? opts.minY : -2;
		var maxY = opts.maxY != null ? opts.maxY : 50;
		var run = 0, blown = false;
		return function () {
			var y = b.position.y, sv = speed(b), sw = spin(b);
			if (y < minY || y > maxY) blown = true;   // tunneled or exploded — can never pass
			if (blown) return { ok: false, detail: 'y=' + y.toFixed(2) + ' LEFT THE FLOOR (tunnel/explode)' };
			if (sv < vThresh && sw < wThresh) run++; else run = 0;
			return { ok: run >= hold, detail: 'y=' + y.toFixed(2) + ' |v|=' + sv.toFixed(3) + ' |w|=' + sw.toFixed(3) + ' rest=' + run + '/' + hold };
		};
	}

	// Quaternion for a rotation of `ang` radians about a unit axis (x,y,z). Normalized by t.quat.
	function axisAngle(t, x, y, z, ang) {
		var s = Math.sin(ang / 2);
		return [x * s, y * s, z * s, Math.cos(ang / 2)];
	}

	// Build a dynamic mesh body that is actually SIMULABLE, instead of feeding authored vertices straight
	// into Goblin.MeshShape. Two things go wrong with the naive path, and both are fixed here:
	//
	//   1. WRONG PIVOT. A RigidBody's position is its center of mass / rotation pivot. Authored meshes
	//      usually have their local origin at the base (y=0) for placement convenience, not at their
	//      geometric center. Pivoting about an off-center origin injects a small persistent torque every
	//      rotation step — a resting body slowly rocks and eventually tips upright on its own. Fix:
	//      recenter every vertex to the vertex centroid and spawn the body at pos + centroid, so it lands
	//      in the same place but pivots about its true mass center.
	//
	//   2. NEGATIVE INERTIA -> EXPLOSION. MeshShape.getInertiaTensor derives inertia from a divergence-
	//      theorem VOLUME integral that assumes ONE closed, consistently-wound, non-self-overlapping
	//      solid. Authored props are the opposite: several overlapping boxes plus flat/double-sided sheets
	//      merged into one vertex buffer. For that input the integral is not a valid rigid body and
	//      routinely yields a NEGATIVE moment of inertia. A negative moment of inertia is physically
	//      impossible: contact torque then accelerates spin the WRONG way, and the constraint solver
	//      "lowers energy" by spinning the body faster at every contact — so it launches and spins without
	//      bound. (A thin, tall, or offset-COM prop is exactly the marginal case that tips negative.)
	//      Fix: keep the full concave mesh for COLLISION, but compute INERTIA from the mesh vertices
	//      treated as equal point masses about their centroid. That respects the true anisotropy (a tall
	//      prop resists tumbling more than spinning about its long axis), needs no closed/consistent
	//      topology, and is guaranteed positive-definite — so the body is always physically simulable.
	//
	//   t        the test context      verts  [[x,y,z],...]   faces  flat index triples
	//   mass, opts  as for t.mesh (opts.pos is the intended spawn origin, pre-recenter)
	// Returns the raw Goblin RigidBody.
	function meshBody(t, w, verts, faces, mass, opts) {
		opts = withMat(opts || {});
		var n = verts.length, cx = 0, cy = 0, cz = 0;
		for (var i = 0; i < n; i++) { cx += verts[i][0]; cy += verts[i][1]; cz += verts[i][2]; }
		if (n > 0) { cx /= n; cy /= n; cz /= n; }
		var centered = verts.map(function (v) { return [v[0] - cx, v[1] - cy, v[2] - cz]; });

		// Spawn at pos + centroid so the recentered body still lands where intended.
		var pos = opts.pos || [0, 0, 0];
		var o2 = {}; for (var k in opts) if (Object.prototype.hasOwnProperty.call(opts, k)) o2[k] = opts[k];
		o2.pos = [pos[0] + cx, pos[1] + cy, pos[2] + cz];

		var body = t.mesh(w, centered, faces, mass, o2);

		// Point-cloud inertia about the centroid (verts are already centered, so centroid ≈ 0).
		if (mass !== 0 && mass !== Infinity && isFinite(mass)) {
			var pm = mass / n, Ixx = 0, Iyy = 0, Izz = 0, Ixy = 0, Ixz = 0, Iyz = 0;
			for (var j = 0; j < n; j++) {
				var x = centered[j][0], y = centered[j][1], z = centered[j][2];
				Ixx += pm * (y * y + z * z); Iyy += pm * (x * x + z * z); Izz += pm * (x * x + y * y);
				Ixy -= pm * x * y; Ixz -= pm * x * z; Iyz -= pm * y * z;
			}
			var I = body.inertiaTensor;
			I.e00 = Ixx; I.e01 = Ixy; I.e02 = Ixz;
			I.e10 = Ixy; I.e11 = Iyy; I.e12 = Iyz;
			I.e20 = Ixz; I.e21 = Iyz; I.e22 = Izz;
			I.invertInto(body.inverseInertiaTensor);
			if (body.updateDerived) body.updateDerived();
		}
		return body;
	}

	return { ground: ground, settles: settles, speed: speed, spin: spin, axisAngle: axisAngle, MAT: MAT, withMat: withMat, meshBody: meshBody };
});
