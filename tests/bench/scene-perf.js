/**
 * Performance benchmark: approximates the real target scene — a 200m x 200m map footprint (Blender/
 * Goblin units treated as meters, matching the engine's -9.8 gravity default) carrying 1,000,000
 * triangles of static world geometry as one MeshShape, plus a field of prop_physics-style convex
 * bodies (boxes/cylinders/cones mixed, 500 by default) resting on/near it. Reports per-step timing
 * broken down by broadphase / narrowphase / solver, so we can find where the frame budget actually
 * goes and track the floor as fixes land.
 *
 * Target is 200 FPS (5ms/step budget), not 60 — this is a physics-thread budget check, not a
 * render-thread one.
 *
 * Usage:
 *   node tests/bench/scene-perf.js                       500 mixed bodies, 200m map, 1M tris, 120 steps
 *   node tests/bench/scene-perf.js --bodies=200           override body count
 *   node tests/bench/scene-perf.js --tris=250000          override mesh triangle count (density, not map size)
 *   node tests/bench/scene-perf.js --mapSize=500          override map footprint in meters
 *   node tests/bench/scene-perf.js --steps=300            override steps measured
 *   node tests/bench/scene-perf.js --shape=cylinders      cylinders|boxes|cones|mixed
 *   node tests/bench/scene-perf.js --fps=200              override target FPS for the budget check
 *   node tests/bench/scene-perf.js --seed=42              deterministic body placement
 */
var Goblin = require('../../build/goblin.js');

function parseArgs() {
	var out = {};
	process.argv.slice(2).forEach(function (a) {
		var m = /^--([^=]+)=(.*)$/.exec(a);
		if (m) out[m[1]] = m[2];
	});
	return out;
}
var args = parseArgs();

var NUM_BODIES = args.bodies ? parseInt(args.bodies, 10) : 500;
var TARGET_TRIS = args.tris ? parseInt(args.tris, 10) : 1000000;
var STEPS = args.steps ? parseInt(args.steps, 10) : 120;
// 500 bodies dropped from a few meters up take well over 100 steps to actually settle onto the mesh
// (measured: manifold count / step time both still climbing at step 100, plateaus around step 120-140).
// A too-low warmup measures free-fall, which is nearly free — steady-state resting contact is the real
// cost and is 2x+ higher, so this needs to be high enough that the measured window is genuinely settled.
var WARMUP_STEPS = args.warmup ? parseInt(args.warmup, 10) : 150;
var SHAPE_MODE = args.shape || 'mixed'; // cylinders | boxes | mixed
var SEED = args.seed ? parseInt(args.seed, 10) : 1234;
var TARGET_FPS = args.fps ? parseFloat(args.fps) : 200;
var TIME_STEP = 1 / 60;

// Deterministic PRNG (mulberry32) so runs are comparable across code changes.
function mulberry32(seed) {
	return function () {
		seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
		var t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
		t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}
var rand = mulberry32(SEED);

// Real-world anchor for the benchmark, per the actual scene being modeled: a 200m x 200m x 200m map
// (Blender units == meters == Goblin units, since the engine's default gravity of -9.8 assumes meters)
// containing 1,000,000 triangles of static world geometry. Map footprint (X/Z) is fixed at this size —
// growing/shrinking --tris changes triangle DENSITY on this fixed footprint, it does not change map
// size. That's intentional: "how does perf change with LOD/geo complexity on the actual map" is the
// question this benchmark answers; a separate map-size question would need extent as its own argument.
var MAP_SIZE_METERS = args.mapSize ? parseFloat(args.mapSize) : 200;

/**
 * Builds a heightfield-style ground mesh spanning the fixed MAP_SIZE_METERS footprint, subdivided to
 * hit ~targetTris triangles, plus scattered box-like "building/prop" bumps, so the BVH has real depth
 * and reflects genuine world-geometry density rather than an arbitrarily shrunk or stretched patch.
 */
function buildMapMesh(targetTris, seedRand) {
	var quads = Math.max(1, Math.round(targetTris / 2));
	var res = Math.max(2, Math.round(Math.sqrt(quads)));
	var halfExtent = MAP_SIZE_METERS / 2;
	var verts = [], faces = [];
	var step = (halfExtent * 2) / res;

	for (var z = 0; z <= res; z++) {
		for (var x = 0; x <= res; x++) {
			var wx = -halfExtent + x * step;
			var wz = -halfExtent + z * step;
			// Gentle rolling height + occasional raised "structure" plateaus to break flatness.
			var h = Math.sin(wx * 0.05) * 0.6 + Math.cos(wz * 0.05) * 0.6;
			if (seedRand() < 0.01) h += 1.5 + seedRand() * 2;
			verts.push(new Goblin.Vector3(wx, h, wz));
		}
	}
	function idx(x, z) { return z * (res + 1) + x; }
	for (z = 0; z < res; z++) {
		for (x = 0; x < res; x++) {
			var a = idx(x, z), b = idx(x + 1, z), c = idx(x + 1, z + 1), d = idx(x, z + 1);
			faces.push(a, b, c);
			faces.push(a, c, d);
		}
	}
	return { verts: verts, faces: faces };
}

// prop_physics-style mix: crates (boxes), barrels/pipes (cylinders), and traffic-cone-like convex
// shapes (cones), roughly evenly split so the mesh sees varied GJK/EPA support-point geometry rather
// than one shape repeated 500 times.
function makeShape(i) {
	var mode = SHAPE_MODE;
	if (mode === 'mixed') {
		var r = i % 3;
		mode = r === 0 ? 'boxes' : (r === 1 ? 'cylinders' : 'cones');
	}
	if (mode === 'boxes') {
		return new Goblin.BoxShape(0.4 + rand() * 0.3, 0.4 + rand() * 0.3, 0.4 + rand() * 0.3);
	}
	if (mode === 'cones') {
		return new Goblin.ConeShape(0.3 + rand() * 0.2, 0.5 + rand() * 0.3);
	}
	return new Goblin.CylinderShape(0.3 + rand() * 0.2, 0.4 + rand() * 0.3);
}

function buildWorld() {
	var world = new Goblin.World(new Goblin.SAPBroadphase(), new Goblin.NarrowPhase(), new Goblin.IterativeSolver());
	world.gravity.set(0, -9.8, 0);

	var halfExtent = MAP_SIZE_METERS / 2;
	var mapData = buildMapMesh(TARGET_TRIS, rand);
	var actualTris = mapData.faces.length / 3;
	var mapShape = new Goblin.MeshShape(mapData.verts, mapData.faces);
	var mapBody = new Goblin.RigidBody(mapShape, Infinity);
	world.addRigidBody(mapBody);

	var bodies = [];
	for (var i = 0; i < NUM_BODIES; i++) {
		var shape = makeShape(i);
		var body = new Goblin.RigidBody(shape, 5 + rand() * 10);
		var x = (rand() * 2 - 1) * halfExtent * 0.9;
		var z = (rand() * 2 - 1) * halfExtent * 0.9;
		var dropHeight = 2 + rand() * 6;
		body.position.set(x, dropHeight, z);
		body.rotation.set(rand() - 0.5, rand() - 0.5, rand() - 0.5, 1);
		body.rotation.normalize();
		world.addRigidBody(body);
		bodies.push(body);
	}

	return { world: world, mapBody: mapBody, bodies: bodies, actualTris: actualTris };
}

function percentile(sorted, p) {
	var idx = Math.min(sorted.length - 1, Math.floor(p * sorted.length));
	return sorted[idx];
}

function stats(samplesMs) {
	var sorted = samplesMs.slice().sort(function (a, b) { return a - b; });
	var sum = sorted.reduce(function (a, b) { return a + b; }, 0);
	return {
		mean: sum / sorted.length,
		min: sorted[0],
		max: sorted[sorted.length - 1],
		p50: percentile(sorted, 0.5),
		p95: percentile(sorted, 0.95),
		p99: percentile(sorted, 0.99)
	};
}

function fmt(n) { return n.toFixed(3); }

function run() {
	console.log('=== Goblin scene perf benchmark ===');
	console.log('bodies=' + NUM_BODIES + ' shape=' + SHAPE_MODE + ' targetTris=' + TARGET_TRIS +
		' mapSize=' + MAP_SIZE_METERS + 'm steps=' + STEPS + ' warmup=' + WARMUP_STEPS + ' seed=' + SEED);

	var built = buildWorld();
	var world = built.world;
	console.log('actual mesh triangles=' + built.actualTris);

	// Instrument: wrap broadphase.update, narrowphase.generateContacts, and the solver stages to time
	// each independently without altering World.step's control flow. Solver stages are summed into one
	// "solver" bucket since step() calls them as four separate small methods back-to-back.
	var bpTimes = [], npTimes = [], solverTimes = [], stepTimes = [];
	var origBpUpdate = world.broadphase.update.bind(world.broadphase);
	var origNpGenerate = world.narrowphase.generateContacts.bind(world.narrowphase);
	var solver = world.solver;
	var origPrepare = solver.prepareConstraints.bind(solver);
	var origResolve = solver.resolveContacts.bind(solver);
	var origSolve = solver.solveConstraints.bind(solver);
	var origApply = solver.applyConstraints.bind(solver);
	var bpAccum = 0, npAccum = 0, solverAccum = 0;

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
	function timedSolverCall(orig) {
		return function () {
			var t0 = process.hrtime.bigint();
			var r = orig.apply(null, arguments);
			solverAccum += Number(process.hrtime.bigint() - t0) / 1e6;
			return r;
		};
	}
	solver.prepareConstraints = timedSolverCall(origPrepare);
	solver.resolveContacts = timedSolverCall(origResolve);
	solver.solveConstraints = timedSolverCall(origSolve);
	solver.applyConstraints = timedSolverCall(origApply);

	// Warmup: let bodies fall and settle so steady-state has real resting mesh contacts, not just
	// free-fall broadphase-only steps. Warmup isn't measured.
	for (var w = 0; w < WARMUP_STEPS; w++) {
		world.step(TIME_STEP);
	}

	var maxPairs = 0, totalPairs = 0;

	for (var s = 0; s < STEPS; s++) {
		bpAccum = 0; npAccum = 0; solverAccum = 0;
		var t0 = process.hrtime.bigint();
		world.step(TIME_STEP);
		var stepMs = Number(process.hrtime.bigint() - t0) / 1e6;

		stepTimes.push(stepMs);
		bpTimes.push(bpAccum);
		npTimes.push(npAccum);
		solverTimes.push(solverAccum);

		var pairCount = world.broadphase.collision_pairs.length;
		totalPairs += pairCount;
		if (pairCount > maxPairs) maxPairs = pairCount;
	}

	var stepStats = stats(stepTimes);
	var bpStats = stats(bpTimes);
	var npStats = stats(npTimes);
	var solverStats = stats(solverTimes);
	var avgPairs = totalPairs / STEPS;
	var avgFps = 1000 / stepStats.mean;

	var unaccounted = stepStats.mean - bpStats.mean - npStats.mean - solverStats.mean;

	console.log('');
	console.log('--- per-step timings (ms) ---');
	console.log('total    mean=' + fmt(stepStats.mean) + ' p50=' + fmt(stepStats.p50) +
		' p95=' + fmt(stepStats.p95) + ' p99=' + fmt(stepStats.p99) + ' max=' + fmt(stepStats.max));
	console.log('broad    mean=' + fmt(bpStats.mean) + ' p95=' + fmt(bpStats.p95) + ' max=' + fmt(bpStats.max));
	console.log('narrow   mean=' + fmt(npStats.mean) + ' p95=' + fmt(npStats.p95) + ' max=' + fmt(npStats.max));
	console.log('solver   mean=' + fmt(solverStats.mean) + ' p95=' + fmt(solverStats.p95) + ' max=' + fmt(solverStats.max));
	console.log('other    mean=' + fmt(unaccounted) + '  (gravity/integrate/updateDerived/ghost/emit)');
	console.log('');
	console.log('avg broadphase pairs/step=' + avgPairs.toFixed(1) + ' max=' + maxPairs);
	console.log('implied avg FPS (physics-only, single thread)=' + avgFps.toFixed(1));

	var budgetMs = 1000 / TARGET_FPS;
	console.log('');
	console.log('budget check: ' + fmt(budgetMs) + 'ms/step (' + TARGET_FPS + 'Hz target) -> ' +
		(stepStats.mean <= budgetMs ? 'PASS' : 'FAIL') +
		'  (mean is ' + (stepStats.mean / budgetMs).toFixed(2) + 'x budget)');

	return {
		bodies: NUM_BODIES, tris: built.actualTris, steps: STEPS,
		step: stepStats, broadphase: bpStats, narrowphase: npStats, solver: solverStats, avgPairs: avgPairs
	};
}

if (require.main === module) {
	run();
} else {
	module.exports = { run: run };
}
