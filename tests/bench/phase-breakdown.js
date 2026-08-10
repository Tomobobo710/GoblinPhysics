/**
 * Phase-bucketed fine-grained breakdown: same instrumentation as narrowphase-breakdown.js, but
 * measured across the ENTIRE drop-to-settle run and bucketed per-step into three phases instead of
 * only steady-state:
 *   falling  — zero broadphase pairs yet, nothing has touched the mesh
 *   impact   — pairs/manifolds actively climbing (bodies landing, first-contact penetration resolving)
 *   settled  — pairs/manifolds have plateaued (steady-state resting)
 * Reports mean ms/step AND percentage-of-total breakdown separately for each phase, so we can see
 * whether the impact spike is the same bottleneck at higher volume, or something impact-specific
 * (EPA doing real iterative work on deep fresh penetrations, more solver iterations, manifold churn).
 *
 * Usage:
 *   node tests/bench/phase-breakdown.js
 *   node tests/bench/phase-breakdown.js --bodies=500 --tris=1000000 --mapSize=200 --steps=260
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
var MAP_SIZE_METERS = args.mapSize ? parseFloat(args.mapSize) : 200;
var STEPS = args.steps ? parseInt(args.steps, 10) : 260;
var SEED = args.seed ? parseInt(args.seed, 10) : 1234;
var TIME_STEP = 1 / 60;
// A step's pair count is "plateaued" (settled phase) once it stays within this fraction of the max
// pair count seen so far for PLATEAU_HOLD consecutive steps.
var PLATEAU_FRACTION = 0.98;
var PLATEAU_HOLD = 15;

function mulberry32(seed) {
	return function () {
		seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
		var t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
		t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}
var rand = mulberry32(SEED);

function buildMapMesh(targetTris) {
	var quads = Math.max(1, Math.round(targetTris / 2));
	var res = Math.max(2, Math.round(Math.sqrt(quads)));
	var halfExtent = MAP_SIZE_METERS / 2;
	var verts = [], faces = [];
	var step = (halfExtent * 2) / res;
	for (var z = 0; z <= res; z++) {
		for (var x = 0; x <= res; x++) {
			var wx = -halfExtent + x * step;
			var wz = -halfExtent + z * step;
			var h = Math.sin(wx * 0.05) * 0.6 + Math.cos(wz * 0.05) * 0.6;
			if (rand() < 0.01) h += 1.5 + rand() * 2;
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

function makeShape(i) {
	var r = i % 3;
	if (r === 0) return new Goblin.BoxShape(0.4 + rand() * 0.3, 0.4 + rand() * 0.3, 0.4 + rand() * 0.3);
	if (r === 1) return new Goblin.CylinderShape(0.3 + rand() * 0.2, 0.4 + rand() * 0.3);
	return new Goblin.ConeShape(0.3 + rand() * 0.2, 0.5 + rand() * 0.3);
}

console.log('=== Phase-bucketed breakdown (falling / impact / settled) ===');
console.log('bodies=' + NUM_BODIES + ' tris=' + TARGET_TRIS + ' mapSize=' + MAP_SIZE_METERS + 'm steps=' + STEPS);

var mapData = buildMapMesh(TARGET_TRIS);
var mapShape = new Goblin.MeshShape(mapData.verts, mapData.faces);
var world = new Goblin.World(new Goblin.SAPBroadphase(), new Goblin.NarrowPhase(), new Goblin.IterativeSolver());
world.gravity.set(0, -9.8, 0);
var mapBody = new Goblin.RigidBody(mapShape, Infinity);
world.addRigidBody(mapBody);

var halfExtent = MAP_SIZE_METERS / 2;
for (var i = 0; i < NUM_BODIES; i++) {
	var shape = makeShape(i);
	var body = new Goblin.RigidBody(shape, 5 + rand() * 10);
	body.position.set((rand() * 2 - 1) * halfExtent * 0.9, 2 + rand() * 6, (rand() * 2 - 1) * halfExtent * 0.9);
	body.rotation.set(rand() - 0.5, rand() - 0.5, rand() - 0.5, 1);
	body.rotation.normalize();
	world.addRigidBody(body);
}

// --- Instrumentation: per-step deltas, captured fresh each step, attributed to that step's phase ---
function newAcc() {
	return {
		steps: 0, totalMs: 0,
		aabbIntersects: 0, aabbIntersectsMs: 0,
		gjkCalls: 0, gjkMs: 0,
		epaCalls: 0, epaMs: 0, epaIterationsTotal: 0,
		addContactCalls: 0, addContactMs: 0,
		manifoldUpdateMs: 0,
		broadphaseMs: 0,
		solverProcessMs: 0, solverPrepareMs: 0, solverResolveMs: 0, solverSolveMs: 0, solverApplyMs: 0,
	};
}
var phases = { falling: newAcc(), impact: newAcc(), settled: newAcc() };
var cur; // the accumulator for whatever step is currently running

var origIntersects = Goblin.AABB.prototype.intersects;
Goblin.AABB.prototype.intersects = function (o) {
	cur.aabbIntersects++;
	var t0 = process.hrtime.bigint();
	var r = origIntersects.call(this, o);
	cur.aabbIntersectsMs += Number(process.hrtime.bigint() - t0) / 1e6;
	return r;
};

var origBroadphaseUpdate = world.broadphase.update.bind(world.broadphase);
world.broadphase.update = function () {
	var t0 = process.hrtime.bigint();
	origBroadphaseUpdate();
	cur.broadphaseMs += Number(process.hrtime.bigint() - t0) / 1e6;
};

var origGJK = Goblin.GjkEpa.GJK;
Goblin.GjkEpa.GJK = function () {
	cur.gjkCalls++;
	var t0 = process.hrtime.bigint();
	var r = origGJK.apply(this, arguments);
	cur.gjkMs += Number(process.hrtime.bigint() - t0) / 1e6;
	return r;
};
var epaIterThisCall = 0;
var origAddVertex = Goblin.GjkEpa.Polyhedron.prototype.addVertex;
Goblin.GjkEpa.Polyhedron.prototype.addVertex = function () { epaIterThisCall++; return origAddVertex.apply(this, arguments); };
var origEPA = Goblin.GjkEpa.EPA;
Goblin.GjkEpa.EPA = function () {
	cur.epaCalls++;
	epaIterThisCall = 0;
	var t0 = process.hrtime.bigint();
	var r = origEPA.apply(this, arguments);
	cur.epaMs += Number(process.hrtime.bigint() - t0) / 1e6;
	cur.epaIterationsTotal += epaIterThisCall;
	return r;
};

var origAddContact = Goblin.NarrowPhase.prototype.addContact;
Goblin.NarrowPhase.prototype.addContact = function () {
	cur.addContactCalls++;
	var t0 = process.hrtime.bigint();
	var r = origAddContact.apply(this, arguments);
	cur.addContactMs += Number(process.hrtime.bigint() - t0) / 1e6;
	return r;
};

var origManifoldUpdate = Goblin.ContactManifold.prototype.update;
Goblin.ContactManifold.prototype.update = function () {
	var t0 = process.hrtime.bigint();
	var r = origManifoldUpdate.apply(this, arguments);
	cur.manifoldUpdateMs += Number(process.hrtime.bigint() - t0) / 1e6;
	return r;
};

var solver = world.solver;
function wrapSolverStage(name, accKey) {
	var orig = solver[name].bind(solver);
	solver[name] = function () {
		var t0 = process.hrtime.bigint();
		var r = orig.apply(null, arguments);
		cur[accKey] += Number(process.hrtime.bigint() - t0) / 1e6;
		return r;
	};
}
wrapSolverStage('processContactManifolds', 'solverProcessMs');
wrapSolverStage('prepareConstraints', 'solverPrepareMs');
wrapSolverStage('resolveContacts', 'solverResolveMs');
wrapSolverStage('solveConstraints', 'solverSolveMs');
wrapSolverStage('applyConstraints', 'solverApplyMs');

function mergeInto(dst, src) {
	dst.steps++;
	Object.keys(src).forEach(function (k) {
		if (k === 'steps') return;
		dst[k] += src[k];
	});
}

var maxPairsSeen = 0, plateauStreak = 0, seenFirstContact = false;
var phaseLog = [];

for (var s = 0; s < STEPS; s++) {
	cur = newAcc();
	var t0 = process.hrtime.bigint();
	world.step(TIME_STEP);
	cur.totalMs = Number(process.hrtime.bigint() - t0) / 1e6;

	var pairs = world.broadphase.collision_pairs.length;
	if (pairs > maxPairsSeen) maxPairsSeen = pairs;

	var phase;
	if (!seenFirstContact) {
		if (pairs > 0) seenFirstContact = true;
		phase = pairs > 0 ? 'impact' : 'falling';
	} else {
		var nearMax = maxPairsSeen > 0 && pairs >= maxPairsSeen * PLATEAU_FRACTION;
		if (nearMax) plateauStreak++; else plateauStreak = 0;
		phase = plateauStreak >= PLATEAU_HOLD ? 'settled' : 'impact';
	}

	mergeInto(phases[phase], cur);
	if (s < 5 || s % 20 === 0) phaseLog.push({ step: s, phase: phase, ms: cur.totalMs.toFixed(1), pairs: pairs });
}

console.log('');
console.log('phase transitions (sampled):');
phaseLog.forEach(function (e) { console.log('  step=' + e.step + ' phase=' + e.phase + ' step_ms=' + e.ms + ' pairs=' + e.pairs); });

function report(name, acc) {
	if (acc.steps === 0) {
		console.log('\n--- ' + name + ' phase: 0 steps (never occurred in this run) ---');
		return;
	}
	var n = acc.steps;
	console.log('\n--- ' + name + ' phase: ' + n + ' steps, mean=' + (acc.totalMs / n).toFixed(2) + 'ms/step ---');
	var total = acc.totalMs;
	function line(label, ms, extra) {
		console.log('  ' + label.padEnd(26) + (ms / n).toFixed(3) + 'ms/step  (' + (100 * ms / total).toFixed(1) + '%)' + (extra ? '  ' + extra : ''));
	}
	line('broadphase.update()', acc.broadphaseMs);
	line('aabb.intersects()', acc.aabbIntersectsMs, acc.aabbIntersects + ' calls, ' + (acc.aabbIntersects / n).toFixed(0) + '/step');
	line('GJK', acc.gjkMs, acc.gjkCalls + ' calls, ' + (acc.gjkCalls / n).toFixed(0) + '/step');
	line('EPA', acc.epaMs, acc.epaCalls + ' calls, ' + (acc.epaCalls / n).toFixed(1) + '/step, avg ' + (acc.epaCalls ? (acc.epaIterationsTotal / acc.epaCalls).toFixed(1) : '0') + ' iterations/call');
	line('addContact', acc.addContactMs, acc.addContactCalls + ' calls, ' + (acc.addContactCalls / n).toFixed(0) + '/step');
	line('manifold.update()', acc.manifoldUpdateMs);
	line('solver.processContactManifolds', acc.solverProcessMs);
	line('solver.prepareConstraints', acc.solverPrepareMs);
	line('solver.resolveContacts', acc.solverResolveMs);
	line('solver.solveConstraints', acc.solverSolveMs);
	line('solver.applyConstraints', acc.solverApplyMs);
	var accountedMs = acc.gjkMs + acc.epaMs + acc.addContactMs + acc.manifoldUpdateMs + acc.broadphaseMs + acc.aabbIntersectsMs +
		acc.solverProcessMs + acc.solverPrepareMs + acc.solverResolveMs + acc.solverSolveMs + acc.solverApplyMs;
	console.log('  ' + 'accounted'.padEnd(26) + (accountedMs / n).toFixed(3) + 'ms/step  (' + (100 * accountedMs / total).toFixed(1) + '%)');
	console.log('  ' + 'unaccounted'.padEnd(26) + ((total - accountedMs) / n).toFixed(3) + 'ms/step  (' + (100 * (total - accountedMs) / total).toFixed(1) + '%)');
}

report('FALLING', phases.falling);
report('IMPACT', phases.impact);
report('SETTLED', phases.settled);
