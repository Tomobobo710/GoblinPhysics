// Measures World.shapeIntersect against a many-child CompoundShape ground, mirroring the character
// controller's Collision.js/Probes.js query pattern (they call world.shapeIntersect/rayIntersect, not
// world.step's broadphase-pair loop) — confirms whether midPhase's new AABB filter benefits that path
// too, since shapeIntersect calls narrowphase.getContact directly (see World.js:363).
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
var NUM_CHILDREN = args.children ? parseInt(args.children, 10) : 3071;
var MAP_SIZE_METERS = args.mapSize ? parseFloat(args.mapSize) : 200;
var TRIALS = args.trials ? parseInt(args.trials, 10) : 500;

function buildCompoundGround() {
	var perSide = Math.max(1, Math.round(Math.sqrt(NUM_CHILDREN)));
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
			var v = [
				new Goblin.Vector3(cx - th, 0, cz - th),
				new Goblin.Vector3(cx + th, 0, cz - th),
				new Goblin.Vector3(cx + th, 0, cz + th),
				new Goblin.Vector3(cx - th, 0, cz + th)
			];
			var f = [0, 2, 1, 0, 3, 2];
			compound.addChildShape(new Goblin.MeshShape(v, f), zero, ident);
		}
	}
	return compound;
}

var world = new Goblin.World(new Goblin.SAPBroadphase(), new Goblin.NarrowPhase(), new Goblin.IterativeSolver());
world.gravity.set(0, -9.8, 0);
var groundBody = new Goblin.RigidBody(buildCompoundGround(), Infinity);
world.addRigidBody(groundBody);

// A handful of props too, matching a real scene (character queries happen alongside props existing).
function mulberry32(seed) {
	return function () {
		seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
		var t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
		t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}
var rand = mulberry32(99);
for (var i = 0; i < 565; i++) {
	var body = new Goblin.RigidBody(new Goblin.BoxShape(0.4, 0.4, 0.4), 5);
	body.position.set((rand() * 2 - 1) * 90, 0.5, (rand() * 2 - 1) * 90);
	world.addRigidBody(body);
}

var boxShape = new Goblin.BoxShape(0.3, 0.9, 0.3); // character capsule-ish probe box

var t0 = process.hrtime.bigint();
for (var t = 0; t < TRIALS; t++) {
	var x = (rand() * 2 - 1) * 90, z = (rand() * 2 - 1) * 90;
	var start = new Goblin.Vector3(x, 3, z);
	var end = new Goblin.Vector3(x, -1, z);
	world.shapeIntersect(boxShape, start, end);
}
var elapsed = Number(process.hrtime.bigint() - t0) / 1e6;

console.log('children=' + NUM_CHILDREN + ' trials=' + TRIALS);
console.log('total=' + elapsed.toFixed(2) + 'ms  per-call=' + (elapsed / TRIALS).toFixed(4) + 'ms');
