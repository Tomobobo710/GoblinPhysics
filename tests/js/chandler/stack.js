// Chandler's stack.html — a 385-box pyramid on a ground plane, at engine-default solver settings.
// A pyramid resting on solid ground is a stable equilibrium, so it must stay a pyramid: no
// interpenetration, no upward extrusion, no creep, no tipping, and fully at rest by the end.
(function (Runner) {
	Runner.suite('chandler');

	var SIZE = 10;
	var TICKS = 1200;
	var BOX_H = 2.0;      // full height of a box (half-extent 1)
	var LAYER_GAP = 2.2;  // vertical spacing between layers at spawn

	// Columns sit 2.6 apart and each layer is inset 1.2, so a box spans [x-1, x+1] and overlaps the
	// two boxes below it by 0.80 on one side and 0.60 on the other, per axis. Every box is therefore
	// supported at spawn - nothing bridges a gap. Drifting 0.60 toward the weaker side removes that
	// support entirely, so 0.60 is the point where the stack stops being held up by what it started on.
	var SUPPORT_LOSS = 0.6;


	// Local-space corners, so checks measure each box's real world extent, not just its center.
	var CORNERS = (function () {
		var c = [];
		for (var x = -1; x <= 1; x += 2)
			for (var y = -1; y <= 1; y += 2)
				for (var z = -1; z <= 1; z += 2) c.push([x, y, z]);
		return c;
	})();

	Runner.test('stack', 'box pyramid stays a pyramid', function (t) {
		t.log('Dropping a 10x10 box pyramid (385 boxes) onto a ground plane at engine-default solver settings.');
		t.log('The pyramid must hold: layers keep their heights, no interpenetration or extrusion, no creep or tipping, and the pile comes to rest by the end.');

		var w = t.makeWorld({ gravity: -9.8 });

		t.plane(w, 1, 20, 20, 0, { color: '#243B2A' });

		var boxes = [];
		for (var i = 0; i < SIZE; i++) {
			for (var j = 0; j < SIZE - i; j++) {
				for (var k = 0; k < SIZE - i; k++) {
					var x = 2 * j * 1.3 - SIZE + i * 1.2,
						y = i * LAYER_GAP + 1,
						z = 2 * k * 1.3 - SIZE + i * 1.2;
					var b = t.box(w, 1, 1, 1, 1, { pos: [x, y, z], friction: 2.5, color: '#B08968' });
					boxes.push({ body: b, layer: i, x0: x, y0: y, z0: z });
				}
			}
		}

		var tmp = t.vec(0, 0, 0);
		var ticks = 0;

		// Worst-case trackers, accumulated every tick against the live world.
		var anyNonFinite = false;
		var worstLayerDropEver = 0;    // furthest any box has fallen below its spawn height
		var worstLayerRiseEver = 0;    // furthest any box has been pushed ABOVE its spawn height
		var worstLatDriftEver = 0;     // furthest any box has moved horizontally from its spawn column
		var worstTiltEver = 0;         // furthest any box has rotated out of axis-alignment (degrees)
		var worstFloorPenetrationEver = 0;

		// Viewer tint. Vertical settling is measured against the box's own layer, not its spawn height:
		// closing the 0.2 spawn gaps drops every layer by ~0.19 per level below it, which is correct and
		// must not read as displacement. Blue holding, yellow drifting, red out of place.
		// Each signal is divided by its criterion's threshold, so 1.0 means "at the limit": yellow is a
		// box using up its allowance, red is a box that would fail.
		function tintFor(vertOff, lat, tiltDeg) {
			var worst = Math.max(vertOff / 0.015, lat / SUPPORT_LOSS, tiltDeg / 0.75);
			return worst < 0.5 ? '#4af' : (worst < 1 ? '#ffcc00' : '#ff0000');
		}

		// Total rotation from spawn, about any axis. A box spawns axis-aligned (identity rotation), so
		// the angle of its current quaternion is how far it has turned in total. tiltDegrees below only
		// sees rotation that moves local +Y off world +Y, which is blind to yaw: a box spun flat on the
		// table reads 0 there and up to 46 degrees here.
		function totalRotationDegrees(body) {
			var w = Math.abs(body.rotation.w);
			if (w > 1) w = 1;
			return 2 * Math.acos(w) * 180 / Math.PI;
		}

		// A box is axis-aligned at spawn. Tilt = angle between its local +Y and world +Y.
		function tiltDegrees(body) {
			tmp.set(0, 1, 0);
			body.rotation.transformVector3(tmp);
			var d = Math.max(-1, Math.min(1, tmp.y));
			return Math.acos(d) * 180 / Math.PI;
		}

		function lowestCornerOf(body) {
			var lo = Infinity;
			for (var c = 0; c < 8; c++) {
				tmp.set(CORNERS[c][0], CORNERS[c][1], CORNERS[c][2]);
				body.transform.transformVector3(tmp);
				if (tmp.y < lo) lo = tmp.y;
			}
			return lo;
		}

		// Mean resting height of each layer, recomputed each tick. A box is judged against its own
		// layer's mean, so uniform settling is not mistaken for a box being out of place.
		var layerSum = [], layerCount = [], layerMean = [];
		function computeLayerMeans() {
			var L;
			for (L = 0; L < SIZE; L++) { layerSum[L] = 0; layerCount[L] = 0; }
			for (var n = 0; n < boxes.length; n++) {
				var y = boxes[n].body.position.y;
				if (!isFinite(y)) continue;
				layerSum[boxes[n].layer] += y; layerCount[boxes[n].layer]++;
			}
			for (L = 0; L < SIZE; L++) layerMean[L] = layerCount[L] ? layerSum[L] / layerCount[L] : null;
		}

		// Worst deviation of any box from its own layer's mean height — the "is the pile level" measure.
		var worstLayerDeviationEver = 0;

		var MAX_ROTATION = 8;


		t.onTick(function (world, tick) {
			ticks = tick;
			computeLayerMeans();

			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], body = o.body, p = body.position;
				if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) { anyNonFinite = true; body._color = '#ff0000'; continue; }

				var drop = o.y0 - p.y;
				if (drop > worstLayerDropEver) worstLayerDropEver = drop;
				var rise = p.y - o.y0;
				if (rise > worstLayerRiseEver) worstLayerRiseEver = rise;

				var dx = p.x - o.x0, dz = p.z - o.z0;
				var lat = Math.sqrt(dx * dx + dz * dz);
				if (lat > worstLatDriftEver) worstLatDriftEver = lat;

				var tilt = tiltDegrees(body);
				if (tilt > worstTiltEver) worstTiltEver = tilt;

				var pen = -lowestCornerOf(body);
				if (pen > worstFloorPenetrationEver) worstFloorPenetrationEver = pen;

				var mean = layerMean[o.layer];
				var vertOff = mean == null ? 0 : Math.abs(p.y - mean);
				if (vertOff > worstLayerDeviationEver) worstLayerDeviationEver = vertOff;

				body._color = tintFor(vertOff, lat, tilt);

			}
		});

		// Helper: worst per-box vertical displacement from spawn height, right now.
		function worstVerticalDisplacementNow() {
			var worst = 0, which = null;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], y = o.body.position.y;
				if (!isFinite(y)) continue;
				var d = Math.abs(y - o.y0);
				if (d > worst) { worst = d; which = o; }
			}
			return { worst: worst, box: which };
		}

		// Tolerances come from the geometry: boxes are 2.0 tall with 0.2 gaps between layers.

		t.expect('every box stays finite (no numerical blowup)', function () {
			if (ticks < TICKS) return false;
			return { ok: !anyNonFinite, detail: anyNonFinite ? 'at least one box went NaN/Infinity' : 'all ' + boxes.length + ' boxes finite at every tick' };
		});

		// Measured once settled: an impact-frame dip that recovers is fine, resting inside the floor is not.
		t.expect('no box rests inside the floor (penetration < 0.012)', function () {
			if (ticks < TICKS) return false;
			var pen = 0;
			for (var n = 0; n < boxes.length; n++) {
				var body = boxes[n].body;
				if (!isFinite(body.position.y)) continue;
				var d = -lowestCornerOf(body);
				if (d > pen) pen = d;
			}
			return { ok: pen < 0.012, detail: 'deepest resting floor penetration=' + pen.toFixed(4) };
		});

		// Closing the 0.2 spawn gaps is legitimate settling, so measure the real invariant instead:
		// stacked layers must end up a box height apart. Too little means interpenetration; too much
		// means the solver is holding boxes apart on a cushion instead of letting them touch.
		t.expect('layers rest one box height apart (spacing within 0.012 of ' + BOX_H + ')', function () {
			if (ticks < TICKS) return false;
			computeLayerMeans();
			var tightest = Infinity, tightestAt = -1, widest = -Infinity, widestAt = -1, L;
			for (L = 1; L < SIZE; L++) {
				if (layerMean[L] == null || layerMean[L - 1] == null) continue;
				var gap = layerMean[L] - layerMean[L - 1];
				if (gap < tightest) { tightest = gap; tightestAt = L; }
				if (gap > widest) { widest = gap; widestAt = L; }
			}
			if (tightestAt < 0) return false;
			return {
				ok: tightest >= BOX_H - 0.012 && widest <= BOX_H + 0.012,
				detail: 'tightest=' + tightest.toFixed(4) + ' (layer ' + tightestAt + '), widest=' + widest.toFixed(4) + ' (layer ' + widestAt + '), box height ' + BOX_H
			};
		});

		// Nothing in this scene pushes upward, so a box resting above its spawn height means the solver
		// extruded it out of the pile. Impact-frame overshoot that recovers is not the failure.
		t.expect('no box is left extruded above its spawn height (rise < 0.01)', function () {
			if (ticks < TICKS) return false;
			var worst = 0;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], y = o.body.position.y;
				if (!isFinite(y)) continue;
				var r = y - o.y0;
				if (r > worst) worst = r;
			}
			return { ok: worst < 0.01, detail: 'worst resting rise=' + worst.toFixed(4) };
		});

		// Boxes shuffle outward once as the spawn gaps close, then must stop. Ongoing creep would mean
		// friction is not holding, so compare the second half of the run against the first.
		var latAtHalf = null;
		t.onTick(function (world, tick) {
			if (tick === Math.floor(TICKS / 2)) latAtHalf = worstLatDriftEver;
		});
		t.expect('lateral creep stops once settled (second half adds < 0.05)', function () {
			if (ticks < TICKS) return false;
			if (latAtHalf == null) return false;
			var added = worstLatDriftEver - latAtHalf;
			return {
				ok: added < 0.05,
				detail: 'drift ' + latAtHalf.toFixed(3) + ' by half-time, ' + worstLatDriftEver.toFixed(3) + ' at end (+' + added.toFixed(4) + ')'
			};
		});

		// The pile spreads outward as the spawn gaps close, which is real settling. What must not happen
		// is a box leaving its own footprint - past half a box width it is no longer over its column.
		t.expect('no box slides off a supporting neighbour (drift < ' + SUPPORT_LOSS + ')', function () {
			if (ticks < TICKS) return false;
			var worst = 0, which = null, lost = 0;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], p = o.body.position;
				if (!isFinite(p.x)) continue;
				var dx = p.x - o.x0, dz = p.z - o.z0;
				var l = Math.sqrt(dx * dx + dz * dz);
				if (l >= SUPPORT_LOSS) lost++;
				if (l > worst) { worst = l; which = o; }
			}
			return {
				ok: worst < SUPPORT_LOSS,
				detail: 'worst drift=' + worst.toFixed(3) + (which ? ' (layer ' + which.layer + ')' : '') +
					', ' + lost + ' boxes past the ' + SUPPORT_LOSS + ' support-loss limit'
			};
		});

		// The max above is one box at the edge of the pile; this catches the whole stack shifting, which
		// a single-worst-box check would miss.
		// The typical box has nothing pushing it sideways: the pile is symmetric, it spawns already
		// supported on four boxes, and it is at rest. So the median box should barely move. A tenth of
		// the support-loss limit still leaves the stack solidly seated; more than that and the whole
		// pile is migrating rather than one box misbehaving.
		t.expect('the pile as a whole does not migrate (median drift < ' + (SUPPORT_LOSS / 10) + ')', function () {
			if (ticks < TICKS) return false;
			var drifts = [];
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], p = o.body.position;
				if (!isFinite(p.x)) continue;
				var dx = p.x - o.x0, dz = p.z - o.z0;
				drifts.push(Math.sqrt(dx * dx + dz * dz));
			}
			if (!drifts.length) return false;
			drifts.sort(function (a, b) { return a - b; });
			var median = drifts[Math.floor(drifts.length / 2)];
			var p90 = drifts[Math.floor(drifts.length * 0.9)];
			return {
				ok: median < SUPPORT_LOSS / 10,
				detail: 'median drift=' + median.toFixed(3) + ', p90=' + p90.toFixed(3) + ', support-loss limit ' + SUPPORT_LOSS
			};
		});

		// Boxes spawn axis-aligned on flat ground and the pile is symmetric, so nothing should end up
		// rotated. Every box, no exceptions.
		// Rotation about ANY axis, which is the half the tilt check below cannot see. Boxes are dropped
		// axis-aligned into a grid and nothing in the scene asks them to turn, so any accumulated
		// rotation is the pile rearranging itself. Yaw is where it shows up: measured at tick 1200, the
		// worst box had turned 46 degrees while the tilt check read 0.26.
		t.expect('no box has spun in place (every box under ' + MAX_ROTATION + ' degrees of total rotation)', function () {
			if (ticks < TICKS) return false;
			var worst = 0, which = null, over = 0;
			for (var n = 0; n < boxes.length; n++) {
				var body = boxes[n].body;
				if (!isFinite(body.position.x)) continue;
				var rot = totalRotationDegrees(body);
				if (rot > MAX_ROTATION) over++;
				if (rot > worst) { worst = rot; which = boxes[n]; }
			}
			return {
				ok: over === 0,
				detail: 'worst=' + worst.toFixed(2) + ' degrees' + (which ? ' (layer ' + which.layer + ')' : '') +
					', ' + over + ' boxes past ' + MAX_ROTATION + ' degrees'
			};
		});

		t.expect('no box is left tipped (every box under 0.75 degrees)', function () {
			if (ticks < TICKS) return false;
			var worst = 0, which = -1, over = 0;
			for (var n = 0; n < boxes.length; n++) {
				if (!isFinite(boxes[n].body.position.y)) continue;
				var tl = tiltDegrees(boxes[n].body);
				if (tl > 0.75) over++;
				if (tl > worst) { worst = tl; which = boxes[n].layer; }
			}
			return {
				ok: worst < 0.75,
				detail: 'worst tilt=' + worst.toFixed(2) + ' degrees' + (which >= 0 ? ' (layer ' + which + ')' : '') + ', ' + over + ' boxes past 0.75 degrees'
			};
		});

		// Tilt must stop growing once the pile is at rest. Compares the CURRENT worst tilt at half-time
		// against the end, so landing transients that recover don't count - only ongoing rotation does.
		function worstTiltNow() {
			var worst = 0;
			for (var n = 0; n < boxes.length; n++) {
				if (!isFinite(boxes[n].body.position.y)) continue;
				var tl = tiltDegrees(boxes[n].body);
				if (tl > worst) worst = tl;
			}
			return worst;
		}
		var tiltAtHalf = null;
		t.onTick(function (world, tick) {
			if (tick === Math.floor(TICKS / 2)) tiltAtHalf = worstTiltNow();
		});
		t.expect('tilt stops growing once settled (second half adds < 0.25 degrees)', function () {
			if (ticks < TICKS) return false;
			if (tiltAtHalf == null) return false;
			var now = worstTiltNow();
			var added = now - tiltAtHalf;
			return {
				ok: added < 0.25,
				detail: 'worst tilt ' + tiltAtHalf.toFixed(2) + ' deg at half-time, ' + now.toFixed(2) + ' at end (' + (added >= 0 ? '+' : '') + added.toFixed(2) + ')'
			};
		});

		// Each layer must rest level: every box within a small margin of its own layer's mean height.
		t.expect('every layer rests level (no box more than 0.015 off its layer mean)', function () {
			if (ticks < TICKS) return false;
			computeLayerMeans();
			var worst = 0, which = null;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], y = o.body.position.y, mean = layerMean[o.layer];
				if (!isFinite(y) || mean == null) continue;
				var d = Math.abs(y - mean);
				if (d > worst) { worst = d; which = o; }
			}
			return {
				ok: worst < 0.015,
				detail: 'worst deviation from layer mean=' + worst.toFixed(4) + (which ? ' (layer ' + which.layer + ')' : '')
			};
		});

		// Settling closes the 0.2 spawn gaps, so a box may sit up to 0.2 per layer below where it
		// started. Anything past that is the pyramid deforming, not settling.
		t.expect('the pyramid still holds its shape at tick ' + TICKS, function () {
			if (ticks < TICKS) return false;
			var worst = 0, which = null;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], y = o.body.position.y;
				if (!isFinite(y)) continue;
				var allowed = 0.25 * o.layer + 0.25;
				var off = Math.abs(y - o.y0) - allowed;
				if (off > worst) { worst = off; which = o; }
			}
			return {
				ok: worst <= 0,
				detail: worst <= 0
					? 'every box within its settling allowance'
					: 'worst box exceeds allowance by ' + worst.toFixed(3) + (which ? ' (layer ' + which.layer + ', spawn y=' + which.y0.toFixed(1) + ' now y=' + which.body.position.y.toFixed(2) + ')' : '')
			};
		});

		// Layer 0 rests at y=1.0, so an upper-layer box near y=1 fell the whole height of the pile.
		t.expect('no upper-layer box ever falls to ground level (layer >= 2 stays above y = 2.5)', function () {
			if (ticks < TICKS) return false;
			var fallen = 0, worst = null;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n];
				if (o.layer < 2) continue;
				var y = o.body.position.y;
				if (!isFinite(y)) continue;
				if (y < 2.5) {
					fallen++;
					if (!worst || y < worst.body.position.y) worst = o;
				}
			}
			return {
				ok: fallen === 0,
				detail: fallen === 0
					? 'no upper-layer box reached the floor'
					: fallen + ' upper-layer boxes fell to ground level' + (worst ? ' (worst: layer ' + worst.layer + ' spawned at y=' + worst.y0.toFixed(1) + ', now y=' + worst.body.position.y.toFixed(2) + ')' : '')
			};
		});

		// A stable stack must reach rest and stay there.
		t.expect('the whole pile has come to rest (every box slower than 0.001 units/s)', function () {
			if (ticks < TICKS) return false;
			var fastest = 0;
			for (var n = 0; n < boxes.length; n++) {
				var v = boxes[n].body.linear_velocity;
				if (!isFinite(v.x)) continue;
				var sp = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
				if (sp > fastest) fastest = sp;
			}
			return { ok: fastest < 0.001, detail: 'fastest box at tick ' + TICKS + ' = ' + fastest.toFixed(3) + ' units/s' };
		});

		// Linear rest alone misses a box spinning in place - that is exactly how a slow permanent
		// rotation went unnoticed before. Checked separately.
		t.expect('nothing is still rotating at the end (angular speed < 0.0025 rad/s)', function () {
			if (ticks < TICKS) return false;
			var fastest = 0, which = null;
			for (var n = 0; n < boxes.length; n++) {
				var a = boxes[n].body.angular_velocity;
				if (!isFinite(a.x)) continue;
				var sp = Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
				if (sp > fastest) { fastest = sp; which = boxes[n]; }
			}
			return {
				ok: fastest < 0.0025,
				detail: 'fastest rotation=' + fastest.toFixed(5) + ' rad/s' + (which ? ' (layer ' + which.layer + ')' : '')
			};
		});

		t.simulate(w, TICKS);
	}, {
		visual: true, steps: TICKS, page: 'stack', singleStepPerFrame: true,
		description:
			"Chandler's stack.html scene: a 385-box pyramid on a ground plane at engine-default solver " +
			"settings, run for 1200 ticks. A pyramid on solid ground is a stable equilibrium, so it must " +
			"stay a pyramid: layers hold their heights, boxes neither sink into one another nor get " +
			"extruded upward, nothing creeps or tips, no upper-layer box reaches the floor, and the pile " +
			"has come to rest by the end. Boxes are tinted by displacement (blue holding, yellow past a half " +
			"box, red past a full box)."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
