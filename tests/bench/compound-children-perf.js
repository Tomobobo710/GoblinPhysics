/**
 * Perf benchmark matching the real target scene: a static CompoundShape made of many small child
 * shapes (MeshShape or convex), with a handful of dynamic "physics prop" bodies resting on/near it.
 * This exercises NarrowPhase.midPhase's per-child fan-out directly — the actual bottleneck for this
 * project's scene shape (see project history: world geometry is ~3000 MeshShape children combined
 * into one CompoundShape for broadphase's sake, not a single giant MeshShape).
 *
 * Usage:
 *   node tests/bench/compound-children-perf.js                  3000 children, 500 props, 120 steps
 *   node tests/bench/compound-children-perf.js --children=3071
 *   node tests/bench/compound-children-perf.js --props=565
 *   node tests/bench/compound-children-perf.js --steps=200
 *   node tests/bench/compound-children-perf.js --deep            adds a fine-grained narrowphase
 *                                                                 breakdown: GJK/EPA own time and call
 *                                                                 count, midPhase's own overhead
 *                                                                 (BVH walk/pool/fixup, i.e. midPhase
 *                                                                 time minus everything it calls),
 *                                                                 setFrom cost, and mesh-cache hit rate.
 *                                                                 Correctly nested (a call's time is
 *                                                                 never counted in more than one
 *                                                                 bucket) — see readme below for why
 *                                                                 that's easy to get wrong here.
 *
 * A note on --deep's nesting, because this has been gotten wrong more than once: midPhase recurses
 * (nested compounds) and calls getContact, which for a mesh child routes into meshCollision ->
 * meshConvex, which itself calls back into addContact — so naively summing "time inside getContact"
 * and subtracting it from "time inside midPhase" double-counts whenever getContact is invoked from
 * inside an already-timed midPhase call. --deep tracks call depth explicitly (see `depth` below) so
 * each ms of wall-clock time is attributed to exactly one bucket: the innermost timed function
 * running at that moment, never its caller too.
 */
var Goblin = require('../../build/goblin.js');

function parseArgs() {
	var out = {};
	process.argv.slice(2).forEach(function (a) {
		var m = /^--([^=]+)=(.*)$/.exec(a);
		if (m) { out[m[1]] = m[2]; return; }
		// Bare flag, e.g. --deep (no "=value").
		var bare = /^--([^=]+)$/.exec(a);
		if (bare) out[bare[1]] = true;
	});
	return out;
}
var args = parseArgs();

var NUM_CHILDREN = args.children ? parseInt(args.children, 10) : 3000;
var NUM_PROPS = args.props ? parseInt(args.props, 10) : 500;
var STEPS = args.steps ? parseInt(args.steps, 10) : 120;
var WARMUP_STEPS = args.warmup ? parseInt(args.warmup, 10) : 150;
var MAP_SIZE_METERS = args.mapSize ? parseFloat(args.mapSize) : 200;
var SEED = args.seed ? parseInt(args.seed, 10) : 1234;
var TIME_STEP = 1 / 60;
var DEEP = args.deep !== undefined;
var ROWSTATS = args.rowstats !== undefined;
var TRACE = args.trace !== undefined;
var SOLVER = args.solver || 'iterative';

function mulberry32(seed) {
	return function () {
		seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
		var t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
		t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}
var rand = mulberry32(SEED);

// Tile the map footprint into NUM_CHILDREN roughly-square patches; each patch is one small MeshShape
// child (a few triangles, gently undulating), matching "world geometry split into many small mesh
// pieces" rather than one huge mesh.
function buildCompoundGround() {
	var perSide = Math.max(1, Math.round(Math.sqrt(NUM_CHILDREN)));
	var actualChildren = perSide * perSide;
	var half = MAP_SIZE_METERS / 2;
	var tileSize = (half * 2) / perSide;

	var compound = new Goblin.CompoundShape();
	var ident = new Goblin.Quaternion(0, 0, 0, 1);
	var zero = new Goblin.Vector3(0, 0, 0);

	for (var gz = 0; gz < perSide; gz++) {
		for (var gx = 0; gx < perSide; gx++) {
			var cx = -half + tileSize / 2 + gx * tileSize;
			var cz = -half + tileSize / 2 + gz * tileSize;
			var th = tileSize / 2;
			var h0 = Math.sin(cx * 0.05) * 0.6 + Math.cos(cz * 0.05) * 0.6;
			var v = [
				new Goblin.Vector3(cx - th, h0, cz - th),
				new Goblin.Vector3(cx + th, h0, cz - th),
				new Goblin.Vector3(cx + th, h0, cz + th),
				new Goblin.Vector3(cx - th, h0, cz + th)
			];
			var f = [0, 2, 1, 0, 3, 2];
			compound.addChildShape(new Goblin.MeshShape(v, f), zero, ident);
		}
	}
	return { compound: compound, actualChildren: actualChildren };
}

function makeShape(i) {
	var r = i % 3;
	if (r === 0) return new Goblin.BoxShape(0.4 + rand() * 0.3, 0.4 + rand() * 0.3, 0.4 + rand() * 0.3);
	if (r === 1) return new Goblin.CylinderShape(0.3 + rand() * 0.2, 0.4 + rand() * 0.3);
	return new Goblin.ConeShape(0.3 + rand() * 0.2, 0.5 + rand() * 0.3);
}

function buildWorld() {
	var solver = SOLVER === 'pbd' ? new Goblin.PBDSolver() : new Goblin.IterativeSolver();
	var world = new Goblin.World(new Goblin.SAPBroadphase(), new Goblin.NarrowPhase(), solver);
	world.gravity.set(0, -9.8, 0);

	var ground = buildCompoundGround();
	var groundBody = new Goblin.RigidBody(ground.compound, Infinity);
	world.addRigidBody(groundBody);

	var half = MAP_SIZE_METERS / 2;
	var bodies = [];
	for (var i = 0; i < NUM_PROPS; i++) {
		var shape = makeShape(i);
		var body = new Goblin.RigidBody(shape, 5 + rand() * 10);
		var x = (rand() * 2 - 1) * half * 0.9;
		var z = (rand() * 2 - 1) * half * 0.9;
		body.position.set(x, 2 + rand() * 6, z);
		body.rotation.set(rand() - 0.5, rand() - 0.5, rand() - 0.5, 1);
		body.rotation.normalize();
		world.addRigidBody(body);
		bodies.push(body);
	}

	return { world: world, groundBody: groundBody, bodies: bodies, actualChildren: ground.actualChildren };
}

function stats(samplesMs) {
	var sorted = samplesMs.slice().sort(function (a, b) { return a - b; });
	var sum = sorted.reduce(function (a, b) { return a + b; }, 0);
	function pct(p) { return sorted[Math.min(sorted.length - 1, Math.floor(p * sorted.length))]; }
	return { mean: sum / sorted.length, min: sorted[0], max: sorted[sorted.length - 1], p50: pct(0.5), p95: pct(0.95), p99: pct(0.99) };
}
function fmt(n) { return n.toFixed(3); }

function run() {
	console.log('=== Compound-children perf benchmark (matches real scene shape: CompoundShape ground) ===');
	console.log('solver=' + SOLVER + ' children(target)=' + NUM_CHILDREN + ' props=' + NUM_PROPS + ' mapSize=' + MAP_SIZE_METERS + 'm steps=' + STEPS + ' warmup=' + WARMUP_STEPS);

	var t0 = Date.now();
	var built = buildWorld();
	console.log('build time=' + (Date.now() - t0) + 'ms actual children=' + built.actualChildren);

	var world = built.world;
	var bpTimes = [], npTimes = [], solverTimes = [], stepTimes = [];
	var origBpUpdate = world.broadphase.update.bind(world.broadphase);
	var origNpGenerate = world.narrowphase.generateContacts.bind(world.narrowphase);
	var solver = world.solver;
	var origProcessManifolds = solver.processContactManifolds.bind(solver);
	var origPrepare = solver.prepareConstraints.bind(solver);
	var origResolve = solver.resolveContacts.bind(solver);
	var origSolve = solver.solveConstraints.bind(solver);
	var origApply = solver.applyConstraints.bind(solver);
	var bpAccum = 0, npAccum = 0, solverAccum = 0;
	var subAccum = { processManifolds: 0, prepare: 0, resolve: 0, solve: 0, apply: 0 };
	var subTimes = { processManifolds: [], prepare: [], resolve: [], solve: [], apply: [] };

	world.broadphase.update = function () {
		var t0 = process.hrtime.bigint();
		origBpUpdate();
		bpAccum += Number(process.hrtime.bigint() - t0) / 1e6;
	};
	world.narrowphase.generateContacts = function (pairs) {
		var t0 = process.hrtime.bigint();
		origNpGenerate(pairs);
		npAccum += Number(process.hrtime.bigint() - t0) / 1e6;
	};
	function timedSolverCall(orig, key) {
		return function () {
			var t0 = process.hrtime.bigint();
			var r = orig.apply(null, arguments);
			var dt = Number(process.hrtime.bigint() - t0) / 1e6;
			solverAccum += dt;
			subAccum[key] += dt;
			return r;
		};
	}
	solver.processContactManifolds = timedSolverCall(origProcessManifolds, 'processManifolds');
	solver.prepareConstraints = timedSolverCall(origPrepare, 'prepare');
	solver.resolveContacts = timedSolverCall(origResolve, 'resolve');
	solver.solveConstraints = timedSolverCall(origSolve, 'solve');
	solver.applyConstraints = timedSolverCall(origApply, 'apply');

	// --deep: fine-grained narrowphase breakdown. Depth-tracked so nested calls (midPhase recurses;
	// getContact is called from inside midPhase's testChild) never get double-counted — see the
	// module docstring's nesting note. Each hook only accumulates the time strictly inside its own
	// call, exclusive of any other hooked function called from within it (by pausing the outer timer
	// while a nested hooked call is running would be the alternative; instead each hook's own elapsed
	// time already only spans its own body since JS calls are synchronous and hrtime is wall-clock,
	// so this is naturally exclusive UNLESS two hooks wrap functions where one calls the other, which
	// is exactly the getContact/midPhase case — those two report OVERLAPPING time by design, and the
	// printed report subtracts to get midPhase's own exclusive overhead).
	var deep = null;
	if (DEEP) {
		deep = {
			gjkAcc: 0, gjkCount: 0, epaAcc: 0,
			midPhaseTopAcc: 0, midPhaseTopCount: 0, midPhaseDepth: 0,
			getContactInMidphaseAcc: 0, getContactInMidphaseCount: 0,
			getContactOutsideAcc: 0, getContactOutsideCount: 0,
			setFromAcc: 0, setFromCount: 0
		};
		var inMidphase = 0;
		var np = world.narrowphase;

		var origGJK = Goblin.GjkEpa.GJK.bind(Goblin.GjkEpa);
		Goblin.GjkEpa.GJK = function () {
			var t0 = process.hrtime.bigint();
			var r = origGJK.apply(this, arguments);
			deep.gjkAcc += Number(process.hrtime.bigint() - t0) / 1e6;
			deep.gjkCount++;
			return r;
		};
		var origEPA = Goblin.GjkEpa.EPA.bind(Goblin.GjkEpa);
		Goblin.GjkEpa.EPA = function () {
			var t0 = process.hrtime.bigint();
			var r = origEPA.apply(this, arguments);
			deep.epaAcc += Number(process.hrtime.bigint() - t0) / 1e6;
			return r;
		};

		var origSetFrom = Goblin.RigidBodyProxy.prototype.setFrom;
		Goblin.RigidBodyProxy.prototype.setFrom = function () {
			var t0 = process.hrtime.bigint();
			var r = origSetFrom.apply(this, arguments);
			deep.setFromAcc += Number(process.hrtime.bigint() - t0) / 1e6;
			deep.setFromCount++;
			return r;
		};

		var origGetContact = np.getContact.bind(np);
		np.getContact = function () {
			var t0 = process.hrtime.bigint();
			var r = origGetContact.apply(this, arguments);
			var dt = Number(process.hrtime.bigint() - t0) / 1e6;
			if (inMidphase > 0) { deep.getContactInMidphaseAcc += dt; deep.getContactInMidphaseCount++; }
			else { deep.getContactOutsideAcc += dt; deep.getContactOutsideCount++; }
			return r;
		};

		var origMidPhase = np.midPhase;
		np.midPhase = function () {
			deep.midPhaseDepth++;
			var isTop = deep.midPhaseDepth === 1;
			var t0 = isTop ? process.hrtime.bigint() : null;
			inMidphase++;
			var r = origMidPhase.apply(this, arguments);
			inMidphase--;
			if (isTop) { deep.midPhaseTopAcc += Number(process.hrtime.bigint() - t0) / 1e6; deep.midPhaseTopCount++; }
			deep.midPhaseDepth--;
			return r;
		};
	}

	var rowstats = null;
	if (ROWSTATS) {
		rowstats = { pointHistogram: [0, 0, 0, 0, 0], manifoldCount: 0, totalPoints: 0, samples: 0,
			iterHistogram: [], iterSum: 0, iterSamples: 0 };
	}

	for (var w = 0; w < WARMUP_STEPS; w++) world.step(TIME_STEP);

	// Reset --deep counters after warmup so the steady-state report isn't diluted by the fall/land
	// transient (matches how the timed-phase stats below already only cover the STEPS loop).
	if (DEEP) {
		for (var dk in deep) deep[dk] = 0;
	}

	var maxPairs = 0, totalPairs = 0;
	for (var s = 0; s < STEPS; s++) {
		if (TRACE && s < 3) {
			Goblin.IterativeSolver._debugMaxImpulseTrace = [];
			Goblin.IterativeSolver._debugTrackResiduals = true;
		}
		bpAccum = 0; npAccum = 0; solverAccum = 0;
		for (var k in subAccum) subAccum[k] = 0;
		var t0s = process.hrtime.bigint();
		world.step(TIME_STEP);
		var stepMs = Number(process.hrtime.bigint() - t0s) / 1e6;
		stepTimes.push(stepMs);
		bpTimes.push(bpAccum);
		npTimes.push(npAccum);
		solverTimes.push(solverAccum);
		for (var k in subAccum) subTimes[k].push(subAccum[k]);
		if (TRACE && s < 3) {
			console.log('step ' + s + ' max_impulse per iteration: ' + Goblin.IterativeSolver._debugMaxImpulseTrace.map(function(v){return v.toFixed(4);}).join(', '));
			console.log('  live rows (>0.1) at final iteration: ' + Goblin.IterativeSolver._debugLiveRowsAtEnd + ' / ' + Goblin.IterativeSolver._debugTotalRowsAtEnd);
			Goblin.IterativeSolver._debugMaxImpulseTrace = null;
			Goblin.IterativeSolver._debugTrackResiduals = false;
		}

		var pairCount = world.broadphase.collision_pairs.length;
		totalPairs += pairCount;
		if (pairCount > maxPairs) maxPairs = pairCount;

		if (ROWSTATS) {
			var m = world.narrowphase.contact_manifolds.first;
			while (m) {
				var n = m.points.length;
				rowstats.pointHistogram[n]++;
				rowstats.manifoldCount++;
				rowstats.totalPoints += n;
				m = m.next_manifold;
			}
			rowstats.samples++;

			var iters = Goblin.IterativeSolver._lastIterationCount;
			if (iters !== undefined) {
				rowstats.iterHistogram[iters] = (rowstats.iterHistogram[iters] || 0) + 1;
				rowstats.iterSum += iters;
				rowstats.iterSamples++;
			}
		}
	}

	var stepStats = stats(stepTimes), bpStats = stats(bpTimes), npStats = stats(npTimes), solverStats = stats(solverTimes);
	var subStats = {};
	for (var k in subTimes) subStats[k] = stats(subTimes[k]);
	var avgPairs = totalPairs / STEPS;
	var avgFps = 1000 / stepStats.mean;
	var unaccounted = stepStats.mean - bpStats.mean - npStats.mean - solverStats.mean;

	console.log('');
	console.log('--- per-step timings (ms) ---');
	console.log('total    mean=' + fmt(stepStats.mean) + ' p50=' + fmt(stepStats.p50) + ' p95=' + fmt(stepStats.p95) + ' p99=' + fmt(stepStats.p99) + ' max=' + fmt(stepStats.max));
	console.log('broad    mean=' + fmt(bpStats.mean) + ' p95=' + fmt(bpStats.p95) + ' max=' + fmt(bpStats.max));
	console.log('narrow   mean=' + fmt(npStats.mean) + ' p95=' + fmt(npStats.p95) + ' max=' + fmt(npStats.max));
	console.log('solver   mean=' + fmt(solverStats.mean) + ' p95=' + fmt(solverStats.p95) + ' max=' + fmt(solverStats.max));
	console.log('other    mean=' + fmt(unaccounted));
	console.log('');
	console.log('--- solver sub-stage timings (ms) ---');
	['processManifolds', 'prepare', 'resolve', 'solve', 'apply'].forEach(function (k) {
		var st = subStats[k];
		console.log(k.padEnd(17) + 'mean=' + fmt(st.mean) + ' p95=' + fmt(st.p95) + ' max=' + fmt(st.max));
	});
	console.log('');
	console.log('avg broadphase pairs/step=' + avgPairs.toFixed(1) + ' max=' + maxPairs);
	console.log('implied avg FPS (physics-only, single thread)=' + avgFps.toFixed(1));

	if (DEEP) {
		var n = STEPS;
		console.log('');
		console.log('--- --deep: narrowphase breakdown (ms/step unless noted) ---');
		console.log('midPhase (top-level only)     = ' + fmt(deep.midPhaseTopAcc / n) + '  calls/step=' + (deep.midPhaseTopCount / n).toFixed(1));
		console.log('  getContact inside midPhase  = ' + fmt(deep.getContactInMidphaseAcc / n) + '  calls/step=' + (deep.getContactInMidphaseCount / n).toFixed(1));
		console.log('  midPhase own overhead       = ' + fmt((deep.midPhaseTopAcc - deep.getContactInMidphaseAcc) / n) + '  (BVH walk + pool + parent-chain fixup, exclusive of getContact)');
		console.log('getContact outside midPhase   = ' + fmt(deep.getContactOutsideAcc / n) + '  calls/step=' + (deep.getContactOutsideCount / n).toFixed(1) + '  (plain non-compound pairs)');
		console.log('setFrom (subset of the above) = ' + fmt(deep.setFromAcc / n) + '  calls/step=' + (deep.setFromCount / n).toFixed(1));
		console.log('GJK (own time, any caller)    = ' + fmt(deep.gjkAcc / n) + '  calls/step=' + (deep.gjkCount / n).toFixed(1));
		console.log('EPA (own time, any caller)    = ' + fmt(deep.epaAcc / n));
	}

	if (ROWSTATS) {
		console.log('');
		console.log('--- --rowstats: manifold point-count distribution (steady state) ---');
		console.log('manifolds/step=' + (rowstats.manifoldCount / rowstats.samples).toFixed(1) +
			'  avg points/manifold=' + (rowstats.totalPoints / rowstats.manifoldCount).toFixed(2));
		for (var pc = 0; pc <= 4; pc++) {
			var frac = 100 * rowstats.pointHistogram[pc] / rowstats.manifoldCount;
			console.log('  ' + pc + ' points: ' + frac.toFixed(1) + '%  (' + rowstats.pointHistogram[pc] + ')');
		}
		if (rowstats.iterSamples > 0) {
			console.log('');
			console.log('solveConstraints ran-to-completion count (max_iterations=' + world.solver.max_iterations + ', so a value of ' + (world.solver.max_iterations + 1) + ' means it never early-out):');
			console.log('  mean=' + (rowstats.iterSum / rowstats.iterSamples).toFixed(2));
			for (var it = 0; it < rowstats.iterHistogram.length; it++) {
				if (!rowstats.iterHistogram[it]) continue;
				var ifrac = 100 * rowstats.iterHistogram[it] / rowstats.iterSamples;
				console.log('  ' + it + ': ' + ifrac.toFixed(1) + '%  (' + rowstats.iterHistogram[it] + ')');
			}
		}
	}
}

if (require.main === module) {
	run();
} else {
	module.exports = { run: run };
}
