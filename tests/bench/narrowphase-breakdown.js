/**
 * Fine-grained wall-clock breakdown of narrowphase cost at true steady-state, more precise than
 * --prof sampling. Instruments: BVH traversal (node.aabb.intersects calls), GJK-only time, EPA-only
 * time, contact/manifold bookkeeping, and solver substages, each as an isolated accumulator per step,
 * averaged over many settled steps.
 *
 * Usage:
 *   node tests/bench/narrowphase-breakdown.js
 *   node tests/bench/narrowphase-breakdown.js --bodies=500 --tris=1000000 --mapSize=200 --steps=60
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
var STEPS = args.steps ? parseInt(args.steps, 10) : 60;
var WARMUP_STEPS = args.warmup ? parseInt(args.warmup, 10) : 150;
var SEED = args.seed ? parseInt(args.seed, 10) : 1234;
var TIME_STEP = 1 / 60;

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

console.log('=== Narrowphase fine-grained breakdown ===');
console.log('bodies=' + NUM_BODIES + ' tris=' + TARGET_TRIS + ' mapSize=' + MAP_SIZE_METERS + 'm steps=' + STEPS + ' warmup=' + WARMUP_STEPS);

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

// --- Instrumentation ---
var acc = {
	aabbIntersects: 0, aabbIntersectsMs: 0,
	gjkCalls: 0, gjkMs: 0,
	epaCalls: 0, epaMs: 0,
	triangleConvexCalls: 0,
	addContactCalls: 0, addContactMs: 0,
	manifoldUpdateMs: 0,
	broadphaseMs: 0,
	solverProcessMs: 0, solverPrepareMs: 0, solverResolveMs: 0, solverSolveMs: 0, solverApplyMs: 0,
};

var origIntersects = Goblin.AABB.prototype.intersects;
Goblin.AABB.prototype.intersects = function (o) {
	acc.aabbIntersects++;
	var t0 = process.hrtime.bigint();
	var r = origIntersects.call(this, o);
	acc.aabbIntersectsMs += Number(process.hrtime.bigint() - t0) / 1e6;
	return r;
};

var origBroadphaseUpdate = world.broadphase.update.bind(world.broadphase);
world.broadphase.update = function () {
	var t0 = process.hrtime.bigint();
	origBroadphaseUpdate();
	acc.broadphaseMs += Number(process.hrtime.bigint() - t0) / 1e6;
};

var origGJK = Goblin.GjkEpa.GJK;
Goblin.GjkEpa.GJK = function () {
	acc.gjkCalls++;
	var t0 = process.hrtime.bigint();
	var r = origGJK.apply(this, arguments);
	acc.gjkMs += Number(process.hrtime.bigint() - t0) / 1e6;
	return r;
};
var origEPA = Goblin.GjkEpa.EPA;
Goblin.GjkEpa.EPA = function () {
	acc.epaCalls++;
	var t0 = process.hrtime.bigint();
	var r = origEPA.apply(this, arguments);
	acc.epaMs += Number(process.hrtime.bigint() - t0) / 1e6;
	return r;
};

var origAddContact = Goblin.NarrowPhase.prototype.addContact;
Goblin.NarrowPhase.prototype.addContact = function () {
	acc.addContactCalls++;
	var t0 = process.hrtime.bigint();
	var r = origAddContact.apply(this, arguments);
	acc.addContactMs += Number(process.hrtime.bigint() - t0) / 1e6;
	return r;
};

var origManifoldUpdate = Goblin.ContactManifold.prototype.update;
Goblin.ContactManifold.prototype.update = function () {
	var t0 = process.hrtime.bigint();
	var r = origManifoldUpdate.apply(this, arguments);
	acc.manifoldUpdateMs += Number(process.hrtime.bigint() - t0) / 1e6;
	return r;
};

var solver = world.solver;
function wrapSolverStage(name, accKey) {
	var orig = solver[name].bind(solver);
	solver[name] = function () {
		var t0 = process.hrtime.bigint();
		var r = orig.apply(null, arguments);
		acc[accKey] += Number(process.hrtime.bigint() - t0) / 1e6;
		return r;
	};
}
wrapSolverStage('processContactManifolds', 'solverProcessMs');
wrapSolverStage('prepareConstraints', 'solverPrepareMs');
wrapSolverStage('resolveContacts', 'solverResolveMs');
wrapSolverStage('solveConstraints', 'solverSolveMs');
wrapSolverStage('applyConstraints', 'solverApplyMs');

// Warmup (unmeasured settling)
for (var w = 0; w < WARMUP_STEPS; w++) {
	world.step(TIME_STEP);
}

// Reset accumulators, then measure.
Object.keys(acc).forEach(function (k) { acc[k] = 0; });
var t0 = process.hrtime.bigint();
for (var s = 0; s < STEPS; s++) {
	world.step(TIME_STEP);
}
var totalMs = Number(process.hrtime.bigint() - t0) / 1e6;

function per(ms) { return (ms / STEPS).toFixed(3) + 'ms/step (' + (100 * ms / totalMs).toFixed(1) + '%)'; }
function perCount(n) { return (n / STEPS).toFixed(1) + '/step'; }

console.log('');
console.log('total: ' + (totalMs / STEPS).toFixed(3) + 'ms/step over ' + STEPS + ' steps');
console.log('');
console.log('--- narrowphase ---');
console.log('broadphase.update()     time=' + per(acc.broadphaseMs));
console.log('aabb.intersects() calls: ' + perCount(acc.aabbIntersects) + '  time=' + per(acc.aabbIntersectsMs) + '  (BVH traversal node visits, includes instrumentation overhead)');
console.log('GJK   calls=' + perCount(acc.gjkCalls) + '  time=' + per(acc.gjkMs));
console.log('EPA   calls=' + perCount(acc.epaCalls) + '  time=' + per(acc.epaMs));
console.log('addContact calls=' + perCount(acc.addContactCalls) + '  time=' + per(acc.addContactMs));
console.log('manifold.update() time=' + per(acc.manifoldUpdateMs));
console.log('');
console.log('--- solver ---');
console.log('processContactManifolds time=' + per(acc.solverProcessMs));
console.log('prepareConstraints      time=' + per(acc.solverPrepareMs));
console.log('resolveContacts         time=' + per(acc.solverResolveMs));
console.log('solveConstraints        time=' + per(acc.solverSolveMs));
console.log('applyConstraints        time=' + per(acc.solverApplyMs));
console.log('');
var accountedMs = acc.gjkMs + acc.epaMs + acc.addContactMs + acc.manifoldUpdateMs + acc.broadphaseMs + acc.aabbIntersectsMs +
	acc.solverProcessMs + acc.solverPrepareMs + acc.solverResolveMs + acc.solverSolveMs + acc.solverApplyMs;
console.log('accounted: ' + (accountedMs / STEPS).toFixed(3) + 'ms/step (' + (100 * accountedMs / totalMs).toFixed(1) + '% of total)');
console.log('unaccounted (BVH walk, broadphase, integrate, other): ' + ((totalMs - accountedMs) / STEPS).toFixed(3) + 'ms/step (' + (100 * (totalMs - accountedMs) / totalMs).toFixed(1) + '%)');
