/**
 * Tom's Suite — FPSCharacterController RAMP matrix (R1-R18).
 */
(function (Runner, PBF, Goblin) {
	Runner.suite('tom');

	var DT = PBF.DT;

	// rampWorld(deg): world with a 20x2x30 ramp at y=10, rotated +deg about X. NOT scaled.
	function rampWorld(deg) {
		var w = PBF.makeWorld();
		var rot = PBF.axisAngleQuat(1, 0, 0, deg * Math.PI / 180);
		var ramp = PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot);
		w.addRigidBody(ramp);
		return { w: w, ramp: ramp };
	}
	// buriedRamp: steep ramp buried through the floor (R13/R14/R15/R16 geometry).
	function buriedRamp() {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(25, 0.2, 25, { x: 0, y: -0.2, z: 0 }));
		var rot = PBF.axisAngleQuat(1, 0, 0, 0.96);
		var ramp = PBF.staticBox(2.5, 0.2, 4, { x: 0, y: 3, z: -3 }, '#665544', null, rot);
		w.addRigidBody(ramp);
		return { w: w, ramp: ramp };
	}
	function drift(p, z0, y0) {
		return Math.hypot(p.body.position.z - z0, p.body.position.y - y0);
	}
	function rampFaceDistance(deg, worldY, worldZ) {
		var rad = deg * Math.PI / 180;
		var ny = Math.cos(rad), nz = Math.sin(rad);
		var faceY = 10 + ny, faceZ = 0 + nz;
		return ny * (worldY - faceY) + nz * (worldZ - faceZ);
	}

	// ---- R1: hold below-limit 30s (worst drift over 10/25/40/44) ----
	PBF.scaleTest('fps/ramp', 'R1', 'hold below-limit (zero drift)', function (t, S) {
		var angles = [10, 25, 40, 44], BLOCK = 300;
		var w = PBF.makeWorld();
		var z0 = null, y0 = null, worst = {};

		function build(world, idx, deg) {
			var rot = PBF.axisAngleQuat(1, 0, 0, deg * Math.PI / 180);
			world.addRigidBody(PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot));
			z0 = null; y0 = null;
			return S.spawn(world, { x: 0, y: 16, z: 0 }, {});
		}

		var seq = PBF.sequentialBlocks(t, w, angles, BLOCK, build, function (local, idx, deg, curP) {
			if (local === 120) { z0 = curP.body.position.z; y0 = curP.body.position.y; }
			if (local > 120 && z0 != null) worst[deg] = Math.max(worst[deg] || 0, drift(curP, z0, y0));
		});

		t.log('Idle on below-limit ramps 10°→25°→40°→44°, ONE at a time. Each must HOLD dead still — if it moves at ALL, it slipped.');
		t.expect('holds dead still (zero movement)', function () {
			if (seq.tickOf() < angles.length * BLOCK) return { ok: false, detail: 'holding ' + angles[Math.min(angles.length - 1, Math.floor(seq.tickOf() / BLOCK))] + '°…' };
			var wv = 0, wa = 0; angles.forEach(function (d) { if ((worst[d] || 0) > wv) { wv = worst[d]; wa = d; } });
			return { ok: +wv.toFixed(6) === 0, detail: 'maxMovement=' + wv.toFixed(6) + '@' + wa + '° (any movement = slip)' };
		});
		t.simulate(w, angles.length * BLOCK);
	}, { page: 'fps/ramp', steps: 1200, description: 'Idle on below-limit ramps (10/25/40/44°) one at a time, ~5s each; the character must hold DEAD STILL — any movement is a slip.' });

	// ---- R2: slide off above-limit (47/55/70/89 all slide > 2) ----
	PBF.scaleTest('fps/ramp', 'R2', 'slide off above-limit', function (t, S) {
		var angles = [47, 55, 70, 89], BLOCK = 360;
		var w = PBF.makeWorld();
		var z0 = null, y0 = null, slid = {}, fellThrough = {}, launched = {}, groundedLastTick = false;

		function build(world, idx, deg) {
			world.addRigidBody(PBF.staticBox(60, 0.5, 60, { x: 0, y: -0.5, z: 0 }));
			var rot = PBF.axisAngleQuat(1, 0, 0, deg * Math.PI / 180);
			world.addRigidBody(PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot));
			var rad = deg * Math.PI / 180;
			var ny = Math.cos(rad), nz = Math.sin(rad);
			var faceY = 10 + ny, faceZ = 0 + nz;
			var standoff = 10;
			var curP = S.spawn(world, { x: 0, y: faceY + ny * standoff, z: faceZ + nz * standoff }, {});
			var tossSpeed = 8;
			curP.body.linear_velocity.x = 0;
			curP.body.linear_velocity.y = tossSpeed * 0.3;
			curP.body.linear_velocity.z = -nz * tossSpeed;
			z0 = null; y0 = null; groundedLastTick = false;
			return curP;
		}

		var seq = PBF.sequentialBlocks(t, w, angles, BLOCK, build, function (local, idx, deg, curP) {
			var wasJustGrounded = curP.grounded && !groundedLastTick;
			groundedLastTick = curP.grounded;
			if (local === 1) { z0 = curP.body.position.z; y0 = curP.body.position.y; }
			if (local > 1 && z0 != null) {
				slid[deg] = +drift(curP, z0, y0).toFixed(1);
				var feet = curP.body.position.y - curP.height / 2;
				var overRamp = Math.abs(curP.body.position.z) < 15;
				var faceDist = rampFaceDistance(deg, feet, curP.body.position.z);
				if (overRamp && faceDist < -1) fellThrough[deg] = true;
				if (wasJustGrounded && curP.body.linear_velocity.y > 2) launched[deg] = true;
			}
		});

		t.log('Idle on above-limit ramps 47°→55°→70°→89°, ONE at a time. Each must SLIDE DOWN THE FACE (> 2u), NEVER drop through the slab, and NEVER launch upward on landing.');
		t.expect('all angles slide down the face (> 2u)', function () {
			if (seq.tickOf() < angles.length * BLOCK) return { ok: false, detail: 'sliding ' + angles[Math.min(angles.length - 1, Math.floor(seq.tickOf() / BLOCK))] + '°…' };
			var ok = angles.every(function (d) { return (slid[d] || 0) > 2; });
			var m = {}; angles.forEach(function (d) { m[d] = slid[d] || 0; });
			return { ok: ok, detail: JSON.stringify(m) };
		});
		t.expect('no angle falls through the slab', function () {
			if (seq.tickOf() < angles.length * BLOCK) return { ok: false, detail: 'sliding…' };
			var ok = angles.every(function (d) { return !fellThrough[d]; });
			var m = {}; angles.forEach(function (d) { m[d] = !!fellThrough[d]; });
			return { ok: ok, detail: JSON.stringify(m) };
		});
		t.expect('no angle launches upward on landing', function () {
			if (seq.tickOf() < angles.length * BLOCK) return { ok: false, detail: 'sliding…' };
			var ok = angles.every(function (d) { return !launched[d]; });
			var m = {}; angles.forEach(function (d) { m[d] = !!launched[d]; });
			return { ok: ok, detail: JSON.stringify(m) };
		});
		t.simulate(w, angles.length * BLOCK);
	}, { page: 'fps/ramp', steps: 1440, description: 'Idle on above-limit ramps (47/55/70/89°) one at a time; each must slide DOWN THE FACE, not tunnel through the slab, and not launch upward on landing.' });

	// ---- R3: boundary 44 holds / 47 slides ----
	PBF.scaleTest('fps/ramp', 'R3', 'boundary 44 holds / 47 slides', function (t, S) {
		var BLOCK = 240;
		var w = PBF.makeWorld();
		var z0 = null, y0 = null, hold44 = false, slid47 = false;
		var blockAngles = [44, 47];

		function build(world, idx, deg) {
			var rot = PBF.axisAngleQuat(1, 0, 0, deg * Math.PI / 180);
			world.addRigidBody(PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot));
			z0 = null; y0 = null;
			return S.spawn(world, { x: 0, y: 16, z: 0 }, {});
		}

		var seq = PBF.sequentialBlocks(t, w, blockAngles, BLOCK, build, function (local, idx, deg, curP) {
			if (local === 60) { z0 = curP.body.position.z; y0 = curP.body.position.y; }
			if (local === BLOCK) {
				if (idx === 0) hold44 = curP.grounded;
				else slid47 = drift(curP, z0, y0) > 2;
			}
		});

		t.log('The stand/slide boundary: first 44° (character HOLDS, grounded), then 47° (SLIDES off). Watch both in turn.');
		t.expect('44° holds (grounded)', function () {
			if (seq.tickOf() < 2 * BLOCK) return { ok: false, detail: seq.tickOf() <= BLOCK ? 'holding 44°…' : 'sliding 47°…' };
			return { ok: hold44, detail: 'hold44=' + hold44 };
		});
		t.expect('47° slides', function () {
			if (seq.tickOf() < 2 * BLOCK) return { ok: false, detail: seq.tickOf() <= BLOCK ? 'holding 44°…' : 'sliding 47°…' };
			return { ok: slid47, detail: 'slid47=' + slid47 };
		});
		t.simulate(w, 2 * BLOCK);
	}, { page: 'fps/ramp', steps: 480, description: 'The stand/slide boundary sits between 44° (holds) and 47° (slides), shown one after the other.' });

	// ---- R4: limit-disabled holds 55° (maxSlopeAngle:90) ----
	PBF.scaleTest('fps/ramp', 'R4', 'limit-disabled holds 55deg', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, { maxSlopeAngle: 90 });
		PBF.renderables(t, p);
		var z0 = null, y0 = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 120) { z0 = p.body.position.z; y0 = p.body.position.y; }
			return {};
		});
		t.log('maxSlopeAngle:90 disables the limit — the character HOLDS on a 55° ramp, grounded, < 0.5u drift.');
		t.expect('stays grounded', function () {
			if (z0 == null) return { ok: false, detail: 'settling…' };
			return { ok: p.grounded, detail: 'grounded=' + p.grounded };
		});
		t.expect('drift < 0.5', function () {
			if (z0 == null) return { ok: false, detail: 'settling…' };
			var d = drift(p, z0, y0);
			return { ok: d < 0.5, detail: 'drift=' + d.toFixed(3) };
		});
		t.simulate(w, 420);
	}, { page: 'fps/ramp', steps: 420, description: 'With the slope limit disabled the character stands on a 55° ramp.' });

	// ---- R5: walk up ramp climbs no launch (25°) ----
	PBF.scaleTest('fps/ramp', 'R5', 'walk up ramp climbs no launch', function (t, S) {
		var rw = rampWorld(25);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var y0 = null, launched = false, leftRamp = false;
		PBF.drive(t, p, function (tick) {
			if (tick === 60) y0 = p.body.position.y;
			if (tick > 60) {
				if (p.body.position.z < -12) leftRamp = true;
				if (!leftRamp && !p.grounded && p.body.linear_velocity.y > 2) launched = true;
			}
			return tick > 60 ? { forward: -1, sprint: true } : {};
		});
		t.log('Sprint up a 25° ramp for ~40 ticks. It should CLIMB (> 0.5u) without launching off the face.');
		t.expect('climbs > 0.5', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			var climbed = p.body.position.y - y0;
			return { ok: climbed > 0.5, detail: 'climbed=' + climbed.toFixed(2) };
		});
		t.expect('never launches off the face', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: !launched, detail: 'launched=' + launched };
		});
		t.simulate(w, 100);
	}, { page: 'fps/ramp', steps: 100, description: 'Sprint up a 25° ramp; the character climbs without being launched.' });

	// ---- R6: walk down stays clamped (25°) ----
	PBF.scaleTest('fps/ramp', 'R6', 'walk down stays clamped', function (t, S) {
		var rw = rampWorld(25);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var airborneTicks = 0, stopped = false;
		PBF.drive(t, p, function (tick) {
			if (tick > 60 && !stopped) {
				if (p.body.position.z > 12) stopped = true;
				else if (!p.grounded) airborneTicks++;
			}
			return tick > 60 ? { forward: 1, sprint: true } : {};
		});
		t.log('Sprint DOWN a 25° ramp. Ground clamp keeps it planted — < 8 airborne ticks over the descent.');
		t.expect('airborne ticks < 8', function () {
			return { ok: airborneTicks < 8, detail: 'airborneTicks=' + airborneTicks + '/40' };
		});
		t.simulate(w, 100);
	}, { page: 'fps/ramp', steps: 100, description: 'Sprint down a 25° ramp; the ground clamp keeps the character planted.' });

	// ---- R7: walk across fall line no jitter (30°) ----
	PBF.scaleTest('fps/ramp', 'R7', 'walk across fall line no jitter', function (t, S) {
		var rw = rampWorld(30);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var maxJit = 0, prev = null;
		PBF.drive(t, p, function (tick) {
			if (tick > 60) {
				var s = PBF.hsp(p);
				if (prev !== null) maxJit = Math.max(maxJit, Math.abs(s - prev));
				prev = s;
			}
			return tick > 60 ? { right: 1, sprint: true } : {};
		});
		t.log('Strafe ACROSS the fall line of a 30° ramp. Speed should be smooth — per-tick delta < 2.');
		t.expect('max per-tick speed delta < 2', function () {
			if (prev === null) return { ok: false, detail: 'settling…' };
			return { ok: maxJit < 2, detail: 'maxPerTickDelta=' + maxJit.toFixed(3) };
		});
		t.simulate(w, 180);
	}, { page: 'fps/ramp', steps: 180, description: 'Strafe across the fall line of a 30° ramp; horizontal speed stays smooth.' });

	// ---- R8: ramp->flat seam no spike ----
	PBF.scaleTest('fps/ramp', 'R8', 'ramp->flat seam no spike', function (t, S) {
		var w = S.flat();
		var rot = PBF.axisAngleQuat(1, 0, 0, 25 * Math.PI / 180);
		w.addRigidBody(PBF.staticBox(10, 0.3, 8, { x: 0, y: 2.2, z: -6 }, '#665544', null, rot));
		var p = S.feetSpawn(w, 0, 4, {});
		PBF.renderables(t, p);
		var maxSpike = 0, prev = null, local = 0;
		PBF.drive(t, p, function (tick) {
			if (tick <= 40) return {};
			if (prev === null) prev = PBF.hsp(p);
			local++;
			var s = PBF.hsp(p);
			if (local > 3) maxSpike = Math.max(maxSpike, Math.abs(s - prev));
			prev = s;
			return { forward: -1, sprint: true };
		});
		t.log('Walk from flat onto a shallow ramp across the seam. No velocity spike (< sc(3)) as you cross.');
		t.expect('max seam spike < sc(3)', function () {
			if (prev === null) return { ok: false, detail: 'settling…' };
			return { ok: maxSpike < S.sc(3), detail: 'maxSpike=' + maxSpike.toFixed(3) + ' limit=' + S.sc(3).toFixed(2) };
		});
		t.simulate(w, 130);
	}, { page: 'fps/ramp', steps: 130, description: 'Walk from flat onto a shallow ramp; crossing the seam produces no velocity spike.' });

	// ---- R9: cannot climb steep slope (default), 55° ----
	PBF.scaleTest('fps/ramp', 'R9', 'cannot climb steep slope (default)', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 4, z: 6 }, {});
		PBF.renderables(t, p);
		var y0 = null, worstClimb = -99;
		PBF.drive(t, p, function (tick) {
			if (tick === 30) y0 = p.body.position.y;
			if (tick > 30 && y0 != null) worstClimb = Math.max(worstClimb, p.body.position.y - y0);
			return tick > 30 ? { forward: -1, sprint: true } : {};
		});
		t.log('Default settings: sprint into a 55° ramp. It should NOT climb — height gained stays < 2u.');
		t.expect('climbed < 2 (stays low)', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: worstClimb < 2, detail: 'climbedY=' + (worstClimb === -99 ? '0.00' : worstClimb.toFixed(2)) };
		});
		t.simulate(w, 150);
	}, { page: 'fps/ramp', steps: 150, description: 'By default the character cannot walk up a too-steep 55° ramp.' });

	// ---- R10: climbSteepSlopes opts into climbing, 55° ----
	PBF.scaleTest('fps/ramp', 'R10', 'climbSteepSlopes opts into climbing', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 4, z: 6 }, { climbSteepSlopes: true });
		PBF.renderables(t, p);
		var y0 = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 30) y0 = p.body.position.y;
			return tick > 30 ? { forward: -1, sprint: true } : {};
		});
		t.log('climbSteepSlopes:true — sprint into a 55° ramp should ASCEND it (> 4u climbed).');
		t.expect('climbs > 4 units', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			var climbed = p.body.position.y - y0;
			return { ok: climbed > 4, detail: 'climbedY=' + climbed.toFixed(2) };
		});
		t.simulate(w, 150);
	}, { page: 'fps/ramp', steps: 150, description: 'With climbSteepSlopes on, sprint into a 55° ramp and walk UP the too-steep face.' });

	// ---- R11: steep slip is movement not slide (55°) ----
	PBF.scaleTest('fps/ramp', 'R11', 'steep slip is movement not slide', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var y0 = null, everSliding = false, gainedSpeed = false;
		PBF.drive(t, p, function (tick) {
			if (tick === 45) y0 = p.body.position.y;
			if (tick > 45) {
				if (p._sliding) everSliding = true;
				if (PBF.hsp(p) > 2) gainedSpeed = true;
			}
			return {};
		});
		t.log('Idle on a 55° ramp: it slips down and builds speed, but the crouch-slide flag never engages.');
		t.expect('gains speed (> 2)', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: gainedSpeed, detail: 'gainedSpeed=' + gainedSpeed };
		});
		t.expect('never engages the crouch-slide flag', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: !everSliding, detail: 'slidFlag=' + everSliding };
		});
		t.expect('drops > 0.5 below base', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			var dropped = p.body.position.y < y0 - 0.5;
			return { ok: dropped, detail: 'droppedY=' + (y0 - p.body.position.y).toFixed(2) };
		});
		t.simulate(w, 105);
	}, { page: 'fps/ramp', steps: 105, description: 'Idle on a 55° ramp; the character slips down under gravity without engaging the crouch-slide.' });

	// ---- R12: cannot walk up steep (net descent, 55°) ----
	PBF.scaleTest('fps/ramp', 'R12', 'cannot walk up steep (net descent)', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var y0 = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 30) y0 = p.body.position.y;
			return tick > 30 ? { forward: -1, sprint: true } : {};
		});
		t.log('Hold forward uphill into a 55° ramp — gravity wins, net result is DESCENT (final y < start).');
		t.expect('net descent (final y < base)', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: p.body.position.y < y0, detail: 'netClimbY=' + (p.body.position.y - y0).toFixed(2) };
		});
		t.simulate(w, 120);
	}, { page: 'fps/ramp', steps: 120, description: 'Holding forward uphill into a 55° ramp nets a descent.' });

	// ---- R13: buried-ramp base stabilizes (no wedge/glitch) ----
	PBF.scaleTest('fps/ramp', 'R13', 'buried-ramp base stabilizes (no wedge/glitch)', function (t, S) {
		var rw = buriedRamp();
		var w = rw.w;
		var p = S.feetSpawn(w, 0, 2, {});
		PBF.renderables(t, p);
		var maxSpeedLate = 0, minY = 99, everAirborneLate = false, local = 0;
		PBF.drive(t, p, function (tick) {
			if (tick <= 25) return {};
			local++;
			if (local > 50) {
				maxSpeedLate = Math.max(maxSpeedLate, Math.hypot(p.body.linear_velocity.x, p.body.linear_velocity.y, p.body.linear_velocity.z));
				minY = Math.min(minY, p.body.position.y);
				if (!p.grounded) everAirborneLate = true;
			}
			return { forward: -1, sprint: true };
		});
		t.log('Walk on flat into the base of a steep buried ramp. It must STABILIZE — rests, grounded, no wedge-fall.');
		t.expect('speed settles low (< sc(0.5))', function () {
			if (local <= 50) return { ok: false, detail: 'approaching…' };
			return { ok: maxSpeedLate < S.sc(0.5), detail: 'lateSpeed=' + maxSpeedLate.toFixed(2) };
		});
		t.expect('never airborne late (no wedge-fall)', function () {
			if (local <= 50) return { ok: false, detail: 'approaching…' };
			return { ok: !everAirborneLate, detail: 'airborneLate=' + everAirborneLate };
		});
		t.expect('stays above the floor (minY > sc(0.5))', function () {
			if (local <= 50) return { ok: false, detail: 'approaching…' };
			return { ok: minY > S.sc(0.5), detail: 'minY=' + minY.toFixed(2) };
		});
		t.simulate(w, 105);
	}, { page: 'fps/ramp', steps: 105, description: 'Walk into the base of a steep buried ramp; the character stabilizes with no wedge-fall.' });

	// ---- R14: no mount-pop bob on too-steep toe ----
	PBF.scaleTest('fps/ramp', 'R14', 'no mount-pop bob on too-steep toe', function (t, S) {
		var rw = buriedRamp();
		var w = rw.w;
		var p = S.feetSpawn(w, 0, 2, {});
		PBF.renderables(t, p);
		var steepFlips = 0, prevSteep = null, yMin = 99, yMax = -99, maxUpVz = 0, local = 0;
		PBF.drive(t, p, function (tick) {
			if (tick <= 25) return {};
			local++;
			var steep = p.groundNormal.y < p._minStandableNormalY;
			if (local > 40) {
				if (prevSteep !== null && steep !== prevSteep) steepFlips++;
				yMin = Math.min(yMin, p.body.position.y);
				yMax = Math.max(yMax, p.body.position.y);
				if (-p.body.linear_velocity.z > maxUpVz) maxUpVz = -p.body.linear_velocity.z;
			}
			prevSteep = steep;
			return { forward: -1, sprint: true };
		});
		t.log('At the too-steep toe: the steep flag must not flip-flop and y must not bob (no mount-pop).');
		t.expect('steep flag does not flip-flop (<= 1 flip)', function () {
			if (local <= 40) return { ok: false, detail: 'approaching…' };
			return { ok: steepFlips <= 1, detail: 'steepFlips=' + steepFlips };
		});
		t.expect('no y-bob (< 0.05)', function () {
			if (local <= 40) return { ok: false, detail: 'approaching…' };
			return { ok: (yMax - yMin) < 0.05, detail: 'yBob=' + (yMax - yMin).toFixed(3) };
		});
		t.expect('no mount-pop upward velocity (< 1.0)', function () {
			if (local <= 40) return { ok: false, detail: 'approaching…' };
			return { ok: maxUpVz < 1.0, detail: 'maxUpVz=' + maxUpVz.toFixed(2) };
		});
		t.simulate(w, 105);
	}, { page: 'fps/ramp', steps: 105, description: 'Holding forward into a too-steep toe does not mount-and-pop.' });

	// ---- R15: backside approach no passthrough ----
	PBF.scaleTest('fps/ramp', 'R15', 'backside approach no passthrough', function (t, S) {
		var rw = buriedRamp();
		var w = rw.w;
		var p = S.feetSpawn(w, 0, -8, {});
		p.yaw = 0;
		PBF.renderables(t, p);
		var maxZ = -99, started = false;
		PBF.drive(t, p, function (tick) {
			if (tick <= 20) return {};
			started = true;
			maxZ = Math.max(maxZ, p.body.position.z);
			return { forward: 1, sprint: true, yaw: 0 };
		});
		t.log('Approach the steep ramp from the high back side. It must STOP against the face, never tunnel past z=-1.');
		t.expect('stops against the face (maxZ < -1.0)', function () {
			if (!started) return { ok: false, detail: 'settling…' };
			return { ok: maxZ < -1.0, detail: 'maxZ=' + maxZ.toFixed(3) };
		});
		t.expect('ends grounded', function () {
			if (!started) return { ok: false, detail: 'settling…' };
			return { ok: p.grounded, detail: 'grounded=' + p.grounded };
		});
		t.simulate(w, 140);
	}, { page: 'fps/ramp', steps: 140, description: 'Walk into the steep ramp from the tall back side; the character stops against the face.' });

	// ---- R16: sprint into ramp from flat climbs (no toe stall) ----
	PBF.scaleTest('fps/ramp', 'R16', 'sprint into ramp from flat climbs (no toe stall)', function (t, S) {
		var starts = [0.7, 1.0, 1.5, 2.0, 2.5, 3.0], BLOCK = 170;
		var w = PBF.makeWorld();
		var y0 = null, climbed = {};

		function build(world, idx, startZ) {
			world.addRigidBody(PBF.staticBox(25, 0.2, 25, { x: 0, y: -0.2, z: 0 }));
			var rot = PBF.axisAngleQuat(1, 0, 0, 0.96);
			world.addRigidBody(PBF.staticBox(2.5, 0.2, 4, { x: 0, y: 3, z: -3 }, '#665544', null, rot));
			y0 = null;
			return S.feetSpawn(world, 0, startZ, { climbSteepSlopes: true });
		}

		var seq = PBF.sequentialBlocks(t, w, starts, BLOCK, build, function (local, idx, startZ, curP) {
			if (local === 10) y0 = curP.body.position.y;
			if (local > 10 && y0 != null) climbed[startZ] = Math.max(climbed[startZ] != null ? climbed[startZ] : -99, curP.body.position.y - y0);
		}, function (local) { return local >= 10 ? { forward: -1, sprint: true } : {}; });

		t.log('Sprint from flat into a steep buried-lip ramp from 6 approach positions, ONE at a time. EVERY approach climbs > 3u (no toe stall).');
		t.expect('all approaches climb > 3', function () {
			if (seq.tickOf() < starts.length * BLOCK) return { ok: false, detail: 'approach z=' + starts[Math.min(starts.length - 1, Math.floor(seq.tickOf() / BLOCK))] + '…' };
			var worst = 99, worstZ = 0; starts.forEach(function (z) { var c = climbed[z] != null ? climbed[z] : 0; if (c < worst) { worst = c; worstZ = z; } });
			var allClimbed = starts.every(function (z) { return (climbed[z] != null ? climbed[z] : 0) >= 3; });
			return { ok: allClimbed, detail: 'worstPeakClimb=' + worst.toFixed(2) + '@z' + worstZ };
		});
		t.simulate(w, starts.length * BLOCK);
	}, { page: 'fps/ramp', steps: 1020, description: 'Sprint from flat into a steep buried-lip ramp from 6 positions, one at a time; every approach climbs.' });

	// ---- R17: slip under a steep ramp, then get stuck ----
	function steepRamp() {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(25, 0.2, 25, { x: 0, y: -0.2, z: 0 }));
		var rot = PBF.axisAngleQuat(1, 0, 0, -0.96);
		var ramp = PBF.staticBox(2.5, 0.2, 4, { x: -15, y: 3, z: 3 }, '#665544', null, rot);
		w.addRigidBody(ramp);
		return { w: w, ramp: ramp };
	}
	PBF.scaleTest('fps/ramp', 'R17', 'slip under ramp then get axis-locked', function (t, S) {
		var rw = steepRamp();
		var w = rw.w;
		var spawnZ = S.SC <= 0.5 ? 1.5 : 1.88;
		var p = S.feetSpawn(w, -19.68, spawnZ, {});
		p.yaw = Math.PI / 2;
		PBF.renderables(t, p);
		var turnDelta = S.SC <= 0.5 ? 0.15 : (S.SC >= 2 ? 0.35 : 0.10);
		var turnYaw = Math.PI / 2 - turnDelta;
		var rampToeX = -17.5;
		var flushStopX = null, flushStopZ = null;
		var slideStartZ = null, maxSlid = 0;
		var everBlocked = false, everSliding = false;
		var wedgeTick = null, wedgeX = null, wedgeZ = null;
		var pushTicks = null, settledTick = null, settleX = null, settleZ = null;
		var lockZ = null, isWedged = false, freeZ = null, movedAway = 0, tick0 = 0;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			if (tick <= 20) return {};
			if (tick <= 90) {
				if (tick === 90) { flushStopX = p.body.position.x; flushStopZ = p.body.position.z; }
				return { forward: 1, yaw: Math.PI / 2 };
			}
			if (wedgeTick == null) {
				if (slideStartZ == null) slideStartZ = p.body.position.z;
				maxSlid = Math.max(maxSlid, Math.abs(p.body.position.z - slideStartZ));
				var vx = p.body.linear_velocity.x, vz = p.body.linear_velocity.z;
				var blocked = Math.abs(vx) < S.sc(0.5);
				var sliding = Math.abs(vz) > S.sc(0.5);
				if (blocked) everBlocked = true;
				if (everBlocked && blocked && sliding) everSliding = true;
				if (everSliding && !blocked) {
					wedgeTick = tick; wedgeX = p.body.position.x; wedgeZ = p.body.position.z;
					return { forward: 1, yaw: Math.PI / 2 };
				}
				return { forward: 1, yaw: turnYaw };
			}
			if (pushTicks == null) {
				var distToMid = Math.max(0, -15 - p.body.position.x);
				pushTicks = Math.max(1, Math.round((distToMid / p.sprintSpeed) * 60));
				settledTick = wedgeTick + pushTicks;
			}
			if (tick <= settledTick) return { forward: 1, yaw: Math.PI / 2 };
			if (settleX == null) { settleX = p.body.position.x; settleZ = p.body.position.z; }
			if (tick <= settledTick + 120) return {};
			if (lockZ == null) lockZ = p.body.position.z;
			if (tick <= settledTick + 240) {
				if (Math.abs(p.body.position.z - lockZ) > S.sc(0.02)) isWedged = false; else isWedged = true;
				return { yaw: Math.PI / 2 };
			}
			if (freeZ == null) freeZ = p.body.position.z;
			movedAway = Math.max(movedAway, p.body.position.z - freeZ);
			return { right: 1, yaw: Math.PI / 2 };
		});
		t.log('Walk into the steep ramp\'s face like a wall, look right just enough to start sliding under, then look back to dead-on and settle centered under the ramp — then strafe away.');
		t.expect('stopped at the ramp face, close to flush (like a wall)', function () {
			if (flushStopX == null) return { ok: false, detail: 'approaching…' };
			var gap = rampToeX - (flushStopX + p.width / 2);
			return { ok: Math.abs(gap) < S.sc(0.15), detail: 'gap=' + gap.toFixed(3) };
		});
		t.expect('slid along the ramp face after turning', function () {
			if (slideStartZ == null) return { ok: false, detail: 'sliding…' };
			return { ok: maxSlid > S.sc(0.3), detail: 'slid=' + maxSlid.toFixed(3) };
		});
		t.expect('found a low-clearance point to wedge under while sliding', function () {
			if (slideStartZ == null) return { ok: false, detail: 'sliding…' };
			return { ok: wedgeTick != null, detail: wedgeTick == null ? 'never wedged (everBlocked=' + everBlocked + ' everSliding=' + everSliding + ')' : 'wedgeTick=' + wedgeTick };
		});
		t.expect('did not blow straight through and out the far side', function () {
			if (wedgeTick == null) return { ok: false, detail: 'never wedged' };
			var blewThrough = wedgeX >= (-15 + 2.5);
			return { ok: !blewThrough, detail: 'wedgeX=' + wedgeX.toFixed(3) + ' wedgeZ=' + wedgeZ.toFixed(3) };
		});
		t.expect('settled where expected (under the ramp, past the wedge point)', function () {
			if (wedgeTick == null) return { ok: false, detail: 'never wedged' };
			if (settleX == null) return { ok: false, detail: 'settling…' };
			var inFootprint = settleX > (-15 - 2.5) && settleX < (-15 + 2.5);
			var advanced = settleX > wedgeX;
			return { ok: inFootprint && advanced, detail: 'settleX=' + settleX.toFixed(3) + ' settleZ=' + settleZ.toFixed(3) };
		});
		t.expect('does not get stuck strafing away', function () {
			if (wedgeTick == null) return { ok: false, detail: 'never wedged' };
			if (freeZ == null) return { ok: false, detail: 'settling…' };
			if (tick0 < wedgeTick + 400) return { ok: false, detail: 'checking…' };
			var freedItself = movedAway > S.sc(0.5);
			return { ok: !isWedged || freedItself,
				detail: 'lockZ=' + (lockZ == null ? '?' : lockZ.toFixed(3)) + ' isWedged=' + isWedged + ' movedAway=' + movedAway.toFixed(3) };
		});
		t.simulate(w, 700);
	}, { page: 'fps/ramp', steps: 700, description: 'Walk into a steep ramp face (-0.96 rad) like a wall, look right just enough to slip under, revert to dead-on to settle centered under it, then strafe away. Regression guard for a lock-up under the ramp.' });

	// R18: entry-side companion to R17.
	PBF.scaleTest('fps/ramp', 'R18', 'never rests standing in a too-short pocket', function (t, S) {
		var rw = steepRamp();
		var w = rw.w;
		var p = S.feetSpawn(w, -19.68, 1.88, {});
		p.yaw = Math.PI / 2;
		PBF.renderables(t, p);
		var worstDeficit = 0, worstTick = 0;
		PBF.drive(t, p, function (tick) {
			if (tick <= 20) return {};
			if (tick <= 90) return { forward: 1, yaw: Math.PI / 2 };
			var turnDelta = S.SC <= 0.5 ? 0.15 : (S.SC >= 2 ? 0.64 : 0.46);
			var turnYaw = Math.PI / 2 - turnDelta;
			if (tick <= 120) return { forward: 1, yaw: turnYaw };
			if (p.grounded) {
				var feetY = p.body.position.y - p.height / 2;
				var clr = p._ceilingClearanceAt(p.body.position.x, p.body.position.z, feetY);
				var need = p.height + S.sc(0.02);
				var deficit = isFinite(clr) ? (need - clr) : -Infinity;
				if (deficit > worstDeficit) { worstDeficit = deficit; worstTick = tick; }
			}
			return { yaw: turnYaw };
		});
		t.log('Walk into the same real ramp overhang R17 tests. The controller must never let a full-height character come to rest somewhere shorter than its own height clears.');
		t.expect('never comes to rest under-clearance while standing height', function () {
			return { ok: worstDeficit <= 0,
				detail: 'worstDeficit=' + worstDeficit.toFixed(3) + ' at tick=' + worstTick };
		});
		t.simulate(w, 400);
	}, { page: 'fps/ramp', steps: 400, description: 'Companion to R17: walk into the same real ramp overhang and confirm the controller never lets a standing-height character settle in a pocket with less than standing-height clearance.' });

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/goblin.js') : window.Goblin
);
