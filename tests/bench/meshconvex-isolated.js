/**
 * Isolated cost of ONE convex-vs-mesh contact test (Goblin.NarrowPhase.prototype.getContact for a
 * mesh/cylinder pair), independent of broadphase and the other 499 bodies. This reproduces the
 * original complaint directly: 9 cylinders resting against a mesh feels slow — how much does a single
 * mesh/cylinder contact test cost?
 *
 * Map footprint is pinned at MAP_SIZE_METERS (matching the real scene: a 200m x 200m x 200m map).
 * The --tris sweep changes triangle DENSITY on that fixed footprint (i.e. level-of-detail), not map
 * size — that answers "does perf hold up as world-geo detail increases on the actual map" rather than
 * the unrelated (and unrealistic) question of testing ever-larger maps at fixed density.
 *
 * Usage:
 *   node tests/bench/meshconvex-isolated.js
 *   node tests/bench/meshconvex-isolated.js --tris=1000000 --n=9 --calls=500
 *   node tests/bench/meshconvex-isolated.js --mapSize=500 --tris=1000000
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
var TRIS_LIST = args.tris ? [parseInt(args.tris, 10)] : [2500, 25000, 100000, 1000000];
var N_SHAPES = args.n ? parseInt(args.n, 10) : 9;
var CALLS = args.calls ? parseInt(args.calls, 10) : 200;
var MAP_SIZE_METERS = args.mapSize ? parseFloat(args.mapSize) : 200;

function buildMapMesh(targetTris, halfExtent) {
	var quads = Math.max(1, Math.round(targetTris / 2));
	var res = Math.max(2, Math.round(Math.sqrt(quads)));
	var verts = [], faces = [];
	var step = (halfExtent * 2) / res;
	for (var z = 0; z <= res; z++) {
		for (var x = 0; x <= res; x++) {
			var wx = -halfExtent + x * step, wz = -halfExtent + z * step;
			var h = Math.sin(wx * 0.05) * 0.6 + Math.cos(wz * 0.05) * 0.6;
			verts.push(new Goblin.Vector3(wx, h, wz));
		}
	}
	function idx(x, z) { return z * (res + 1) + x; }
	for (z = 0; z < res; z++) {
		for (x = 0; x < res; x++) {
			var a = idx(x, z), b = idx(x + 1, z), c = idx(x + 1, z + 1), d = idx(x, z + 1);
			faces.push(a, b, c); faces.push(a, c, d);
		}
	}
	return { verts: verts, faces: faces };
}

function fmt(n) { return n.toFixed(4); }

console.log('=== Isolated mesh-vs-convex contact cost ===');
console.log('shapes=' + N_SHAPES + ' calls per config=' + CALLS + ' mapSize=' + MAP_SIZE_METERS + 'm');
console.log('');

TRIS_LIST.forEach(function (targetTris) {
	var halfExtent = MAP_SIZE_METERS / 2;
	var mapData = buildMapMesh(targetTris, halfExtent);
	var tBuild0 = Date.now();
	var mapShape = new Goblin.MeshShape(mapData.verts, mapData.faces);
	var buildMs = Date.now() - tBuild0;
	var mapBody = new Goblin.RigidBody(mapShape, Infinity);
	mapBody.updateDerived();

	var narrowphase = new Goblin.NarrowPhase();

	// N cylinders resting at the mesh surface (y ~ 0, matching the flat-ish mesh near origin), spread
	// out slightly so they don't all query the exact same leaf.
	var cylinders = [];
	for (var i = 0; i < N_SHAPES; i++) {
		var body = new Goblin.RigidBody(new Goblin.CylinderShape(0.35, 0.5), 5);
		body.position.set((i % 3 - 1) * 1.5, 0.5, (Math.floor(i / 3) - 1) * 1.5);
		body.updateDerived();
		cylinders.push(body);
	}

	// Warm up (JIT, pool allocation) before timing.
	for (var w = 0; w < 10; w++) {
		for (i = 0; i < cylinders.length; i++) {
			narrowphase.getContact(mapBody, cylinders[i]);
		}
	}

	var t0 = process.hrtime.bigint();
	for (var c = 0; c < CALLS; c++) {
		for (i = 0; i < cylinders.length; i++) {
			narrowphase.getContact(mapBody, cylinders[i]);
		}
	}
	var totalMs = Number(process.hrtime.bigint() - t0) / 1e6;
	var perBatchMs = totalMs / CALLS;
	var perPairMs = perBatchMs / N_SHAPES;

	console.log('tris=' + mapData.faces.length / 3 + ' (BVH build ' + buildMs + 'ms)');
	console.log('  ' + N_SHAPES + '-shape batch: mean=' + fmt(perBatchMs) + 'ms   per-pair mean=' + fmt(perPairMs) + 'ms');
	console.log('  implied max Hz if THIS were the whole frame: ' + (1000 / perBatchMs).toFixed(1));
	console.log('');
});
