/**
 * Tom's Suite — FPSCharacterController PLATFORM tests (PL1-PL3).
 *
 * One controller per test, all 3 scales. Exercises the base-velocity mechanism (endStep acquires a
 * platform's linear_velocity when the ground probe lands on an isPlatform body; beginStep adds it into
 * the swept move; _updateVertical adds its vertical component into a jump ADDITIVELY, so jumping off a
 * rising platform flings you higher).
 */
(function (Runner, PBF, Goblin) {
	Runner.suite('tom');

	var G = 'fps/platform', P = 'fps/platform';

	// ---- PL1: ride an elevator through a FULL up+down cycle, staying planted, no jitter ----
	PBF.scaleTest(G, 'PL1', 'ride a vertical elevator through a full up+down cycle, no jitter', function (t, S) {
		var w = S.flat();
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: 0 }, { x: 0, y: 5, z: 0 }, 2.5, '#4a7ab0');
		var startY = mover.position.y + (0.3 * S.SC) / 2;
		var p = S.spawn(w, { x: 0, y: startY + 0.9 * S.SC + 0.001, z: 0 }, {});
		PBF.renderables(t, p, [mover]);

		var groundedFlips = 0, lastGrounded = null, maxGap = 0, settledTick = 3;
		var sawUp = false, sawDown = false;
		var peakY = -Infinity, minYAfterPeak = Infinity;
		var everUngroundedWhilePlatformMoving = false;

		PBF.drive(t, p, function (tick) {
			var settled = tick > settledTick;
			if (settled) {
				if (lastGrounded !== null && p.grounded !== lastGrounded) groundedFlips++;
				lastGrounded = p.grounded;

				if (!p.grounded) everUngroundedWhilePlatformMoving = true;
				else {
					var platTopY = mover.position.y + (0.3 * S.SC) / 2;
					var feetY = p.body.position.y - p.height / 2;
					var gap = Math.abs(feetY - platTopY);
					if (gap > maxGap) maxGap = gap;
				}
			}

			if (mover.linear_velocity.y < 0) sawDown = true; else sawUp = true;
			if (p.body.position.y > peakY) peakY = p.body.position.y;
			if (sawDown && p.body.position.y < minYAfterPeak) minYAfterPeak = p.body.position.y;

			return {}; // no input — pure ride, isolates the base-velocity mechanism
		}, [mover]);

		t.log('Stand still on an elevator through a FULL ascent + descent (the mover reverses mid-test) — ' +
			'must stay grounded and tracking the platform the whole time, with no grounded-state flicker ' +
			'and no growing gap between feet and platform surface.');
		t.expect('never went airborne while riding (stayed planted)', function () {
			return { ok: !everUngroundedWhilePlatformMoving, detail: 'everUngrounded=' + everUngroundedWhilePlatformMoving };
		});
		t.expect('no grounded-state flicker (0 flips while riding)', function () {
			return { ok: groundedFlips === 0, detail: 'groundedFlips=' + groundedFlips };
		});
		t.expect('feet-to-platform gap stayed tight the whole ride (no jitter)', function () {
			return { ok: maxGap < S.sc(0.05), detail: 'maxGap=' + maxGap.toFixed(4) };
		});
		t.expect('rode both legs — ascent then descent — not just one direction', function () {
			return { ok: sawUp && sawDown, detail: 'sawUp=' + sawUp + ' sawDown=' + sawDown };
		});
		t.expect('actually came back down after the peak (real descent, not stuck at the top)', function () {
			return { ok: minYAfterPeak < peakY - S.sc(1.0), detail: 'peakY=' + peakY.toFixed(2) + ' minYAfterPeak=' + minYAfterPeak.toFixed(2) };
		});
		t.simulate(w, 260);
	}, { page: P, steps: 260, description: 'Riding an elevator through a full up+down cycle stays grounded the whole time with no jitter.' });

	// ---- PL2: genuinely ride a horizontal platform across a gap ----
	PBF.scaleTest(G, 'PL2', 'ride a horizontal moving platform across a gap and dismount on the far side', function (t, S) {
		var w = S.flat();
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0, z: -3 }, { x: 0, y: 0, z: 3 }, 1.2, '#4a7ab0');
		var platY = mover.position.y + (0.3 * S.SC) / 2;
		var platformFrontZ = mover.position.z - S.sc(1);
		var p = S.spawn(w, { x: 0, y: platY + 0.9 * S.SC + 0.001, z: platformFrontZ - S.sc(2) }, {});
		PBF.renderables(t, p, [mover]);

		var onPlatform = function () {
			var cands = p._probeGroundCandidates(p.stepDownDist);
			for (var i = 0; i < cands.length; i++) if (cands[i].object && cands[i].object.isPlatform) return true;
			return false;
		};

		var boardedAt = -1, stoppedWalkingAt = -1, everFellOffMidRide = false, reachedFarSideWhileAboard = false;
		var farSideZ = S.sc(3); // the platform's OTHER endpoint (matches {x:0,y:0,z:3} above)
		PBF.drive(t, p, function (tick) {
			var standing = p.grounded && onPlatform();
			if (standing && boardedAt < 0) boardedAt = tick;
			// Keep walking until actually near the platform's OWN center in Z, not a fixed tick count —
			// walk speed varies with scale, so a fixed tick budget can (and did) carry the player clean
			// across the platform's whole 2-unit width and off the far edge before ever settling aboard.
			var nearMiddle = standing && Math.abs(p.body.position.z - mover.position.z) < S.sc(0.3);
			if (nearMiddle && stoppedWalkingAt < 0) stoppedWalkingAt = tick;
			if (boardedAt >= 0) {
				if (!standing && p.grounded && !reachedFarSideWhileAboard && p.body.position.z < farSideZ - S.sc(0.2)) {
					everFellOffMidRide = true;
				}
				if (standing && p.body.position.z >= farSideZ - S.sc(0.2)) reachedFarSideWhileAboard = true;
			}
			return { forward: stoppedWalkingAt < 0 ? 1 : 0, yaw: 0 };
		}, [mover]);

		t.log('Walk onto a horizontal platform, then stand still — the base-velocity mechanism (not the ' +
			'player\'s own momentum) must carry the rider all the way to the platform\'s far endpoint ' +
			'while still aboard, never sliding off partway through the crossing.');
		t.expect('boarded the platform', function () {
			return { ok: boardedAt > 0, detail: 'boardedAt=' + boardedAt };
		});
		// A genuine walk-on should take roughly as long as covering the spawn-to-platform approach
		// distance at moveSpeed — a few dozen ticks, not hundreds. A test that only checks "eventually
		// boards" within a huge tick budget can hide a real bug behind a technical pass.
		t.expect('boarded within a reasonable time (not a multi-second chase/stall)', function () {
			if (boardedAt < 0) return { ok: false, detail: 'not boarded yet' };
			return { ok: boardedAt <= 60, detail: 'boardedAt=' + boardedAt + ' (must be <= 60 ticks / 1s)' };
		});
		t.expect('settled near the platform\'s middle (didn\'t walk clean over it)', function () {
			return { ok: stoppedWalkingAt > 0, detail: 'stoppedWalkingAt=' + stoppedWalkingAt };
		});
		t.expect('did not fall off mid-ride (no friction-drag-then-drop)', function () {
			return { ok: !everFellOffMidRide, detail: 'everFellOffMidRide=' + everFellOffMidRide };
		});
		t.expect('rode all the way to the far side while still aboard', function () {
			return { ok: reachedFarSideWhileAboard, detail: 'reachedFarSideWhileAboard=' + reachedFarSideWhileAboard + ' finalZ=' + p.body.position.z.toFixed(2) + ' farSideZ=' + farSideZ.toFixed(2) };
		});
		t.simulate(w, 500);
	}, { page: P, steps: 500, description: 'Walking onto a horizontal platform and standing still, the base-velocity mechanism carries the rider all the way to the far side without falling off partway.' });

	// ---- PL3: ride an elevator from the bottom, no jitter, jump near the top, fling higher than normal ----
	PBF.scaleTest(G, 'PL3', 'ride from the bottom, jump near the top, and fling higher than a normal jump', function (t, S) {
		var w = S.flat();
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: 0 }, { x: 0, y: 8, z: 0 }, 6, '#4a7ab0');
		var startY = mover.position.y + (0.3 * S.SC) / 2;
		var p = S.spawn(w, { x: 0, y: startY + 0.9 * S.SC + 0.001, z: 0 }, {});
		PBF.renderables(t, p, [mover]);

		// The theoretical peak rise of the SAME jump (jumpSpeed) from a dead stop — v^2 = 2*g*h —
		// computed analytically rather than run in a second world, so this stays a single,
		// live-drivable, watchable world (one elevator, one ride, one jump, one fling).
		var g = -w.gravity.y;
		var normalJumpRise = (p.jumpSpeed * p.jumpSpeed) / (2 * g);

		var groundedFlips = 0, lastGrounded = null, maxGap = 0, settledTick = 6;
		// Jump near the very END of the ascent (8 units at 6 u/s = ~80 ticks to the top), not mid-ride —
		// that's the moment the fling is most dramatic and most clearly distinguishable from "just
		// riding," since the platform is about to stop climbing right after.
		var jumpTick = 72, wasGroundedBeforeJump = false, jumpedAt = -1;
		var jumpVelYAtJump = null, jumpPeakY = -Infinity, jumpStartY = null;
		var landedAfterFling = false;
		PBF.drive(t, p, function (tick) {
			if (tick === settledTick + 1) lastGrounded = p.grounded;
			if (tick > settledTick) {
				if (p.grounded !== lastGrounded) groundedFlips++;
				lastGrounded = p.grounded;
				if (p.grounded) {
					var platTopY = mover.position.y + (0.3 * S.SC) / 2;
					var feetY = p.body.position.y - p.height / 2;
					var gap = Math.abs(feetY - platTopY);
					if (gap > maxGap) maxGap = gap;
				}
			}

			if (tick === jumpTick) {
				wasGroundedBeforeJump = p.grounded;
				jumpVelYAtJump = mover.linear_velocity.y;
				jumpStartY = p.body.position.y;
			}
			if (tick === jumpTick && wasGroundedBeforeJump) jumpedAt = jumpTick;
			if (jumpedAt > 0 && p.body.position.y > jumpPeakY) jumpPeakY = p.body.position.y;
			if (jumpedAt > 0 && jumpPeakY > jumpStartY && p.grounded && p.body.position.y < jumpPeakY) {
				landedAfterFling = true;
			}

			return { jumpPressed: tick === jumpTick };
		}, [mover]);

		t.log('Ride the elevator up from the bottom (must stay planted with no jitter), then jump near the ' +
			'very END of the ascent — the jump should reach measurably higher than jumpSpeed alone would ' +
			'produce from a dead stop, because the platform\'s rising base velocity is added additively ' +
			'into the jump, not overwritten by it — then fall back and land.');
		t.expect('rode the elevator with no grounded-state flicker before the jump', function () {
			return { ok: groundedFlips === 0, detail: 'groundedFlips=' + groundedFlips };
		});
		t.expect('feet-to-platform gap stayed tight while riding (no jitter)', function () {
			return { ok: maxGap < S.sc(0.05), detail: 'maxGap=' + maxGap.toFixed(4) };
		});
		t.expect('jumped while still riding partway up', function () {
			return { ok: jumpedAt > 0, detail: 'jumpedAt=' + jumpedAt };
		});
		t.expect('platform was genuinely rising at the moment of the jump', function () {
			if (jumpVelYAtJump == null) return { ok: false, detail: 'settling…' };
			return { ok: jumpVelYAtJump > S.sc(0.5), detail: 'jumpVelYAtJump=' + jumpVelYAtJump.toFixed(3) };
		});
		t.expect('the fling rose measurably higher than a normal jump from a dead stop would', function () {
			if (jumpStartY == null) return { ok: false, detail: 'settling…' };
			var actualRise = jumpPeakY - jumpStartY;
			return {
				ok: actualRise > normalJumpRise + S.sc(0.15),
				detail: 'actualRise=' + actualRise.toFixed(3) + ' normalJumpRise=' + normalJumpRise.toFixed(3)
			};
		});
		t.expect('fell back down and landed after the fling', function () {
			return { ok: landedAfterFling, detail: 'landedAfterFling=' + landedAfterFling };
		});
		t.simulate(w, 340);
	}, { page: P, steps: 340, description: 'Riding an elevator from the bottom with no jitter, jumping near the top of the ascent flings the player measurably higher than a normal jump, then they fall back and land.' });

	// ---- PL4: sprint-slide down a lane, cross a platform heading the other way, keep going normally ----
	PBF.scaleTest(G, 'PL4', 'sliding across an oncoming platform behaves normally, not a boost pad', function (t, S) {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(S.sc(15), S.sc(1), S.sc(30), { x: 0, y: -S.sc(1), z: S.sc(10) }, '#333'));
		// Platform starts at the FAR end of the lane and travels toward the player (who starts at the
		// near end), flush with the floor, so a sprint+slide down the lane meets it head-on and
		// crosses over it — matching the real in-game scenario directly. Thickness (0.3, pre-scale)
		// matches game.js's actual shuttle platform — it must stay under the controller's stepHeight
		// (0.4 pre-scale) so the character steps up onto it instead of clipping/phasing at its edge.
		var mover = S.splatform(w, 4, 0.3, { x: 0, y: 0.15, z: 15 }, { x: 0, y: 0.15, z: -15 }, 3, '#4a7ab0');
		var p = S.feetSpawn(w, 0, -S.sc(10), {});
		PBF.renderables(t, p, [mover]);

		// t.expect(label, fn) is the ONLY mechanism that works both headless AND live in the browser
		// viewer — the live render loop (render.js) drives ticks itself and re-evaluates every
		// t.expect() predicate each tick via ctx.evalTick; code placed after t.simulate() in the test
		// body runs exactly once, synchronously, at t=0, before the browser ever steps a single tick
		// (see render.js's captureSetup, which stubs out simulate() and runs the rest of the test
		// function immediately) — a plain t.checkTrue() there always sees the untouched initial state
		// in the live viewer even though it works fine headless. So: accumulate state every tick in
		// PBF.drive's per-tick callback as before, but each PREDICATE below only resolves (ok:true or
		// ok:false) once the run has reached its final tick — before that it stays pending — so it
		// still gets ONE check against the FULLY accumulated result, the same as before, just via the
		// mechanism that also works live.
		var enteredSlide = false, slidingAtBoard = false, boardedAt = -1, leftAt = -1,
			ownAtBoard = -1, prevOwn = -1, grewAfterBoardTick = false, worstGrowth = 0,
			stalledAfterBoard = false, minPzAdvanceAfterBoard = Infinity, prevPz = null;
		var TOTAL_TICKS = 220, curTick = 0;
		PBF.drive(t, p, function (tick) {
			curTick = tick;
			if (p.sliding) { enteredSlide = true; }
			var onPlatform = p._baseVelocity.z !== 0 || p._baseVelocity.x !== 0;
			if (onPlatform && boardedAt < 0) { boardedAt = tick; slidingAtBoard = p.sliding; }
			if (!onPlatform && boardedAt >= 0 && leftAt < 0) { leftAt = tick; }
			var standing = p.grounded && p.sliding && onPlatform;
			if (standing) {
				var own = Math.hypot(p._ownVelocityX, p._ownVelocityZ);
				if (ownAtBoard < 0) { ownAtBoard = own; }
				else if (prevOwn >= 0 && own > prevOwn) {
					grewAfterBoardTick = true;
					if (own - prevOwn > worstGrowth) worstGrowth = own - prevOwn;
				}
				prevOwn = own;
			}
			// "Slid over, not stopped dead": WHILE ABOARD (between boardedAt and leftAt only — not for
			// the rest of the run after dismount, where ordinary end-of-slide friction decay also
			// slows the per-tick advance and would otherwise be misread as a stall), the player must
			// keep making forward progress every tick, not flatline in place (the boost-pad bug's
			// stall symptom). Measured directly: broken crawls ~0.01-0.11 units/tick while aboard;
			// correctly-fixed advances ~0.14-0.17+ units/tick throughout the crossing.
			if (boardedAt >= 0 && leftAt < 0 && prevPz !== null) {
				var advance = p.body.position.z - prevPz;
				if (advance < minPzAdvanceAfterBoard) { minPzAdvanceAfterBoard = advance; }
				if (advance <= S.sc(0.06)) { stalledAfterBoard = true; }
			}
			prevPz = p.body.position.z;
			// Sprint long enough to actually close most of the distance to the platform before
			// crouching — triggering the slide right away (as tick<=20 did) launches it at the start
			// line, so it's fully decayed well before ever reaching the platform. Real play sprints
			// most of the approach, then slides in at the last moment.
			if (tick <= 45) { return { forward: 1, sprint: true, yaw: 0 }; }
			return { forward: 1, sprint: true, crouch: true, yaw: 0 };
		}, [mover]);

		t.log('Sprint then crouch-slide straight down a lane; a platform travels the same lane toward the player and they cross over it. Expect: nothing special happens — own speed keeps decaying from friction the whole time, same as sliding on plain ground, and the player slides all the way OVER the platform rather than stalling dead partway across. Any tick where own speed goes UP instead of down while aboard is the platform\'s motion leaking into the character\'s own momentum — the boost-pad bug.');
		t.expect('entered a slide at some point', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'enteredSlide=' + enteredSlide + ' (pending…)' }; }
			return { ok: enteredSlide, detail: 'enteredSlide=' + enteredSlide };
		});
		t.expect('was still sliding when it reached/boarded the platform', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'boardedAt=' + boardedAt + ' (pending…)' }; }
			return { ok: boardedAt > 0 && slidingAtBoard, detail: 'boardedAt=' + boardedAt + ' slidingAtBoard=' + slidingAtBoard };
		});
		t.expect('slid all the way over the platform, never stalled dead', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'boardedAt=' + boardedAt + ' leftAt=' + leftAt + ' (pending…)' }; }
			return { ok: boardedAt > 0 && leftAt > boardedAt && !stalledAfterBoard,
				detail: 'boardedAt=' + boardedAt + ' leftAt=' + leftAt + ' stalledAfterBoard=' + stalledAfterBoard +
					' minPzAdvanceAfterBoard=' + (isFinite(minPzAdvanceAfterBoard) ? minPzAdvanceAfterBoard.toFixed(4) : 'n/a') };
		});
		t.expect('own speed never grows while aboard, no boost-pad compounding', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'worstGrowth=' + worstGrowth.toFixed(3) + ' (pending…)' }; }
			return { ok: !grewAfterBoardTick, detail: 'worstGrowth=' + worstGrowth.toFixed(3) + ' ownAtBoard=' + ownAtBoard.toFixed(2) };
		});
		t.simulate(w, TOTAL_TICKS);
	}, { page: P, steps: 220, description: 'Sprint-sliding down a lane while a platform travels toward the player down the same lane — crossing it must feel like normal ground, not a boost pad.' });

	// ---- PL5: sprint-slide down a lane, catch a platform heading the SAME way — must NOT boost ----
	PBF.scaleTest(G, 'PL5', 'catching an outrunning platform from behind must not launch the player faster', function (t, S) {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(S.sc(15), S.sc(1), S.sc(30), { x: 0, y: -S.sc(1), z: S.sc(10) }, '#333'));
		// Platform starts just ahead of the player and travels AWAY down the same lane the player is
		// sprinting/sliding — the player catches up from behind and boards while it's still moving away.
		// The bug: touching the platform launches the player to a way-higher speed than they already
		// had. That must NOT happen — total speed right after boarding should stay close to whatever
		// speed the player already had, not spike.
		var mover = S.splatform(w, 4, 0.3, { x: 0, y: 0.15, z: 5 }, { x: 0, y: 0.15, z: 25 }, 3, '#4a7ab0');
		var p = S.feetSpawn(w, 0, -S.sc(10), {});
		PBF.renderables(t, p, [mover]);

		var boardedAt = -1, speedBeforeBoard = -1, worstSpikeAfterBoard = 0;
		var TOTAL_TICKS = 220, curTick = 0;
		PBF.drive(t, p, function (tick) {
			curTick = tick;
			var onPlatform = p._baseVelocity.z !== 0 || p._baseVelocity.x !== 0;
			var total = Math.hypot(p.body.linear_velocity.x, p.body.linear_velocity.z);
			if (!onPlatform) { speedBeforeBoard = total; }
			if (onPlatform) {
				if (boardedAt < 0) { boardedAt = tick; }
				var spike = total - speedBeforeBoard;
				if (spike > worstSpikeAfterBoard) { worstSpikeAfterBoard = spike; }
			}
			if (tick <= 45) { return { forward: 1, sprint: true, yaw: 0 }; }
			return { forward: 1, sprint: true, crouch: true, yaw: 0 };
		}, [mover]);

		t.log('Sprint then crouch-slide down a lane, catching a platform from behind that\'s moving away down the same lane. The bug: touching the platform hauls the player to a much higher speed than they already had. Expect: boarding must NOT launch the player faster — speed while aboard stays close to whatever speed they already had going in.');
		t.expect('boarded the platform from behind', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'boardedAt=' + boardedAt + ' (pending…)' }; }
			return { ok: boardedAt > 0, detail: 'boardedAt=' + boardedAt };
		});
		t.expect('touching the platform did not launch the player faster', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'worstSpikeAfterBoard=' + worstSpikeAfterBoard.toFixed(2) + ' (pending…)' }; }
			return { ok: worstSpikeAfterBoard <= S.sc(1), detail: 'speedBeforeBoard=' + speedBeforeBoard.toFixed(2) +
				' worstSpikeAfterBoard=' + worstSpikeAfterBoard.toFixed(2) };
		});
		t.simulate(w, TOTAL_TICKS);
	}, { page: P, steps: 220, description: 'Sprint-sliding down a lane and catching a platform from behind, moving away down the same lane — boarding it must not launch the player to a higher speed.' });
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/goblin.js') : window.Goblin
);
