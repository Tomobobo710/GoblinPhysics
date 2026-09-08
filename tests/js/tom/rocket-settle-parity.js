// Tom's Suite — ROCKET SETTLE PARITY (solver oracle).
//
// A rocket mesh body is spawned from ONE fixed pose — the exact position / orientation / velocity the
// `special shapes` rocket has at tick 300 under the XPBD (PBDSolver) run, a ~56-degree half-toppled
// lean, essentially at rest. From that identical starting state the scene runs another 300 ticks.
//
// The point is a side-by-side: pick the solver in the suite dropdown and watch this same start play out.
//   - PGS (IterativeSolver): the lean completes - the rocket tips the rest of the way and lies down,
//     nose to the floor, within ~1-2 seconds, the way a real half-fallen rocket would.
//   - XPBD (PBDSolver): the lean STALLS. The nose hangs in the air and the tilt barely changes for the
//     whole run - the fall crawls or stops outright.
//
// So this is a real regression gate for the XPBD "slow / stalled toppling" bug, not just something to
// watch: the asserts require the nose to actually come down and the body to actually reach rest lying
// over. It passes under PGS today and fails under XPBD, and should pass under both once the bug is fixed.
//
// The rocket() mesh builder below is copied verbatim from special-shapes.js (it is private to that
// file's closure) so the shape, and therefore U.meshBody's recenter + point-cloud inertia, are byte-
// identical to what the capture was taken from.
(function (Runner, U) {
	Runner.suite('tom');

	var CONTINUE_TICKS = 300;

	// Captured from `special shapes` -> `rocket (octagon body + nose + fins)` at tick 300, PBDSolver.
	// position is the recentered body's COM (U.meshBody spawns at pos + centroid; this is the live
	// position after the run, so it is fed back in directly, NOT through meshBody's pos option).
	var CAPTURE = {
		position: [0.20979075069289646, 1.0286649175052776, 0.09440595107286276],
		rotation: [0.2895472598207968, 0.09358949015262917, -0.3582318451294227, 0.882651310993423],
		linear_velocity: [0, 0, 0],
		angular_velocity: [-0.00012449005933608617, 0.0007279673799533971, -0.00007763687427537989]
	};

	// ---- rocket() — verbatim from special-shapes.js ----
	function Builder() { this.ref = { x: 0, y: 0, z: 0 }; }
	Builder.prototype.setReferencePoint = function (p) { this.ref = { x: p.x, y: p.y, z: p.z }; };
	Builder.prototype.createTriangle = function (idx, pos, a, b, c, forceFlip, doubleSided) {
		var ax = pos[a * 3], ay = pos[a * 3 + 1], az = pos[a * 3 + 2];
		var bx = pos[b * 3], by = pos[b * 3 + 1], bz = pos[b * 3 + 2];
		var cx = pos[c * 3], cy = pos[c * 3 + 1], cz = pos[c * 3 + 2];
		var ux = bx - ax, uy = by - ay, uz = bz - az;
		var vx = cx - ax, vy = cy - ay, vz = cz - az;
		var nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
		var gx = (ax + bx + cx) / 3, gy = (ay + by + cy) / 3, gz = (az + bz + cz) / 3;
		var dot = nx * (gx - this.ref.x) + ny * (gy - this.ref.y) + nz * (gz - this.ref.z);
		var eps = 1e-6; if (Math.abs(dot) < eps) dot = dot >= 0 ? eps : -eps;
		var v1, v2, v3;
		if (dot >= 0) { v1 = a; v2 = b; v3 = c; } else { v1 = a; v2 = c; v3 = b; }
		if (forceFlip) { var tmp = v2; v2 = v3; v3 = tmp; }
		idx.push(v1, v2, v3);
		if (doubleSided) idx.push(v1, v3, v2);
	};
	function finish(v, idx) {
		var verts = [];
		for (var i = 0; i < v.length; i += 3) verts.push([v[i], v[i + 1], v[i + 2]]);
		return { verts: verts, faces: idx };
	}
	function rocket() {
		var b = new Builder(); var v = [], idx = [];
		var SIDES = 8, R = 0.34, BODY_H = 1.6, NOSE_H = 0.9;
		var push = function (px, py, pz) { v.push(px, py, pz); return v.length / 3 - 1; };
		var ring = function (yy, r) { var ids = []; for (var i = 0; i < SIDES; i++) { var a = (i / SIDES) * Math.PI * 2; ids.push(push(Math.cos(a) * r, yy, Math.sin(a) * r)); } return ids; };
		b.setReferencePoint({ x: 0, y: BODY_H / 2, z: 0 });
		var bot = ring(0, R), top = ring(BODY_H, R);
		for (var i = 0; i < SIDES; i++) { var j = (i + 1) % SIDES; b.createTriangle(idx, v, bot[i], bot[j], top[j]); b.createTriangle(idx, v, bot[i], top[j], top[i]); }
		var botCtr = push(0, 0, 0);
		for (var c = 0; c < SIDES; c++) { var cj = (c + 1) % SIDES; b.createTriangle(idx, v, botCtr, bot[c], bot[cj]); }
		b.setReferencePoint({ x: 0, y: BODY_H + NOSE_H / 2, z: 0 });
		var noseRing = ring(BODY_H, R);
		var apex = push(0, BODY_H + NOSE_H, 0);
		for (var n = 0; n < SIDES; n++) { var nj = (n + 1) % SIDES; b.createTriangle(idx, v, noseRing[n], noseRing[nj], apex); }
		for (var f = 0; f < 4; f++) {
			var fa = (f / 4) * Math.PI * 2, cx = Math.cos(fa), sz = Math.sin(fa);
			b.setReferencePoint({ x: cx * (R + 0.3), y: 0.3, z: sz * (R + 0.3) });
			var p1 = push(cx * R, 0.8, sz * R), p2 = push(cx * R, 0, sz * R), p3 = push(cx * (R + 0.6), 0, sz * (R + 0.6));
			b.createTriangle(idx, v, p1, p2, p3, false, true);
		}
		return finish(v, idx);
	}

	// Nose apex in the AUTHORED (pre-recenter) frame: (0, BODY_H + NOSE_H, 0) = (0, 2.5, 0).
	var NOSE_AUTHORED = [0, 2.5, 0];

	Runner.test('rocket parity', 'half-toppled rocket finishes falling', function (t) {
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);

		var m = rocket();

		// Build exactly as special-shapes does (recenter to centroid + point-cloud inertia). The pos we
		// pass here is a placeholder; we overwrite the full transform with the capture immediately after.
		var body = U.meshBody(t, w, m.verts, m.faces, 2, { pos: [0, 0, 0], color: '#d9dde6' });

		// meshBody recentered the verts to the centroid; recover that centroid so we can express the
		// nose apex in the same recentered local frame the body now uses.
		var n = m.verts.length, cx = 0, cy = 0, cz = 0;
		for (var i = 0; i < n; i++) { cx += m.verts[i][0]; cy += m.verts[i][1]; cz += m.verts[i][2]; }
		cx /= n; cy /= n; cz /= n;
		var noseLocal = t.vec(NOSE_AUTHORED[0] - cx, NOSE_AUTHORED[1] - cy, NOSE_AUTHORED[2] - cz);

		// Slam the captured pose in.
		body.position.set(CAPTURE.position[0], CAPTURE.position[1], CAPTURE.position[2]);
		body.rotation = t.quat(CAPTURE.rotation[0], CAPTURE.rotation[1], CAPTURE.rotation[2], CAPTURE.rotation[3]);
		body.linear_velocity.set(CAPTURE.linear_velocity[0], CAPTURE.linear_velocity[1], CAPTURE.linear_velocity[2]);
		body.angular_velocity.set(CAPTURE.angular_velocity[0], CAPTURE.angular_velocity[1], CAPTURE.angular_velocity[2]);
		if (body.updateDerived) body.updateDerived();

		function tiltDeg() {
			var qw = Math.abs(body.rotation.w); if (qw > 1) qw = 1;
			return 2 * Math.acos(qw) * 180 / Math.PI;
		}
		function noseY() {
			var v = t.vec(noseLocal.x, noseLocal.y, noseLocal.z);
			body.transform.transformVector3(v);
			return v.y;
		}

		var startTilt = tiltDeg();
		var startNose = noseY();
		t.log('Rocket starts at the XPBD tick-300 pose: tilt=' + startTilt.toFixed(0) + 'deg, nose ' + startNose.toFixed(2) + 'm up, at rest.');
		t.log('A body this far past balance must keep falling. PGS finishes the topple; XPBD stalls with the nose in the air.');

		var minNoseEver = startNose;
		var maxTiltEver = startTilt;
		var ticks = 0;
		t.onTick(function (world, tk) {
			ticks = tk;
			var ny = noseY(); if (ny < minNoseEver) minNoseEver = ny;
			var tl = tiltDeg(); if (tl > maxTiltEver) maxTiltEver = tl;
			if (tk % 60 === 0) {
				t.log('t' + tk + ': tilt=' + tl.toFixed(0) + 'deg  noseY=' + ny.toFixed(2) + '  |w|=' + U.spin(body).toFixed(3));
			}
			// tint: blue = still toppling, green once the nose is basically down
			body._color = ny < 0.25 ? '#3fb950' : '#8ac';
		});

		// The lean must actually progress: from ~56deg it has to get near lying-down.
		t.expect('the rocket keeps toppling (tilt reaches 80deg)', function () {
			if (ticks < CONTINUE_TICKS) return { ok: maxTiltEver >= 80, detail: 'tilt so far max=' + maxTiltEver.toFixed(0) + 'deg (start ' + startTilt.toFixed(0) + ')' };
			return { ok: maxTiltEver >= 80, detail: 'max tilt reached=' + maxTiltEver.toFixed(0) + 'deg' };
		});

		// And the nose must come down.
		t.expect('the nose comes down to the floor (noseY < 0.25)', function () {
			return { ok: minNoseEver < 0.25, detail: 'lowest nose=' + minNoseEver.toFixed(2) + 'm (start ' + startNose.toFixed(2) + ')' };
		});

		// And it must settle there, lying over, not still creeping at the end.
		t.expect('it comes to rest lying down by the end', function () {
			if (ticks < CONTINUE_TICKS) return false;
			var atRest = U.speed(body) < 0.08 && U.spin(body) < 0.08;
			var down = noseY() < 0.35 && tiltDeg() > 75;
			return { ok: atRest && down, detail: 'end: tilt=' + tiltDeg().toFixed(0) + 'deg noseY=' + noseY().toFixed(2) + ' |v|=' + U.speed(body).toFixed(3) + ' |w|=' + U.spin(body).toFixed(3) };
		});

		t.simulate(w, CONTINUE_TICKS);
	}, {
		visual: true, steps: CONTINUE_TICKS, page: 'rocket parity',
		description: 'A rocket spawned from the exact half-toppled pose the special-shapes rocket has at tick 300 under XPBD, then run 300 more ticks. PGS finishes the fall and lies it down; XPBD stalls with the nose hung in the air. Asserts the topple completes — passes under PGS, fails under XPBD until the slow-toppling bug is fixed.'
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
