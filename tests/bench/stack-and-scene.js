/**
 * The two reference scenes, measured the same way every time. This is the ONLY source for perf
 * numbers - anything quoted from an ad-hoc script is not comparable.
 *
 *   node tests/bench/stack-and-scene.js            both scenes, 5 runs each
 *   node tests/bench/stack-and-scene.js scene      just the 500/3000 compound scene
 *   node tests/bench/stack-and-scene.js stack      just the 385-box pyramid
 *   node tests/bench/stack-and-scene.js --solver=pbd  measure XPBD instead of PGS
 *   node tests/bench/stack-and-scene.js --engine=<path>   measure a different build
 *
 * Protocol: build the scene, run SETTLE ticks untimed, then time MEASURE ticks and report the
 * median plus min-max across runs. Fixed window on a settled scene, so the number does not depend
 * on catching the cost curve at a particular moment.
 *
 * Budget: the compound scene must stay at or under 15ms/step.
 */
var path = require('path');

var engineArg = process.argv.slice(2).filter(function (a) { return a.indexOf('--engine=') === 0; })[0];
var ENGINE = engineArg ? engineArg.slice(9) : path.join(__dirname, '..', '..', 'build', 'goblin.js');
var Goblin = require(ENGINE);

var which = process.argv.slice(2).filter(function (a) { return a.indexOf('--') !== 0; })[0] || 'both';
var RUNS = 5, SETTLE = 150, MEASURE = 200;
var SCENE_BUDGET_MS = 15;

// Deterministic PRNG so every run builds the identical scene.
function mulberry32(a) {
	return function () {
		a |= 0; a = (a + 0x6D2B79F5) | 0;
		var t = Math.imul(a ^ (a >>> 15), 1 | a);
		t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}

// --solver=pbd measures XPBD instead of the default PGS.
var SOLVER = ( process.argv.slice( 2 ).filter( function ( a ) { return a.indexOf( '--solver=' ) === 0; } )[0] || '' ).slice( 9 ) || 'pgs';

function newWorld() {
	var solver = SOLVER === 'pbd' ? new Goblin.PBDSolver() : new Goblin.IterativeSolver();
	var w = new Goblin.World( new Goblin.SAPBroadphase(), new Goblin.NarrowPhase(), solver );
	w.gravity = new Goblin.Vector3( 0, -9.8, 0 );
	return w;
}

// 500 mixed props on a 3025-child CompoundShape ground, matching perf-settle-scene-compound.js.
function buildScene() {
	var rand = mulberry32( 1234 );
	var w = newWorld();
	var perSide = 55, half = 100, tile = ( half * 2 ) / perSide;
	var comp = new Goblin.CompoundShape(),
		ident = new Goblin.Quaternion( 0, 0, 0, 1 ),
		zero = new Goblin.Vector3( 0, 0, 0 );

	for ( var gz = 0; gz < perSide; gz++ ) {
		for ( var gx = 0; gx < perSide; gx++ ) {
			var cx = -half + tile / 2 + gx * tile, cz = -half + tile / 2 + gz * tile, th = tile / 2;
			var h0 = Math.sin( cx * 0.05 ) * 0.6 + Math.cos( cz * 0.05 ) * 0.6;
			comp.addChildShape( new Goblin.MeshShape( [
				new Goblin.Vector3( cx - th, h0, cz - th ), new Goblin.Vector3( cx + th, h0, cz - th ),
				new Goblin.Vector3( cx + th, h0, cz + th ), new Goblin.Vector3( cx - th, h0, cz + th )
			], [ 0, 2, 1, 0, 3, 2 ] ), zero, ident );
		}
	}
	w.addRigidBody( new Goblin.RigidBody( comp, Infinity ) );

	var bodies = [];
	for ( var i = 0; i < 500; i++ ) {
		var kind = i % 3, b;
		var x = ( rand() * 2 - 1 ) * half * 0.9, z = ( rand() * 2 - 1 ) * half * 0.9, y = 2 + rand() * 6;
		if ( kind === 0 ) b = new Goblin.RigidBody( new Goblin.BoxShape( 0.4 + rand() * 0.3, 0.4 + rand() * 0.3, 0.4 + rand() * 0.3 ), 5 + rand() * 10 );
		else if ( kind === 1 ) b = new Goblin.RigidBody( new Goblin.CylinderShape( 0.3 + rand() * 0.2, 0.4 + rand() * 0.3 ), 5 + rand() * 10 );
		else b = new Goblin.RigidBody( new Goblin.ConeShape( 0.3 + rand() * 0.2, 0.5 + rand() * 0.3 ), 5 + rand() * 10 );
		b.position.set( x, y, z );
		b.rotation = new Goblin.Quaternion( rand() - 0.5, rand() - 0.5, rand() - 0.5, 1 );
		w.addRigidBody( b );
		bodies.push( b );
	}
	return { world: w, bodies: bodies };
}

// The 385-box pyramid from tests/js/chandler/stack.js.
function buildStack() {
	var w = newWorld();
	w.addRigidBody( new Goblin.RigidBody( new Goblin.PlaneShape( 1, 20, 20 ), 0 ) );
	var SIZE = 10, bodies = [];
	for ( var i = 0; i < SIZE; i++ ) {
		for ( var j = 0; j < SIZE - i; j++ ) {
			for ( var k = 0; k < SIZE - i; k++ ) {
				var b = new Goblin.RigidBody( new Goblin.BoxShape( 1, 1, 1 ), 1 );
				b.position.set( 2 * j * 1.3 - SIZE + i * 1.2, i * 2.2 + 1, 2 * k * 1.3 - SIZE + i * 1.2 );
				b.friction = 2.5;
				w.addRigidBody( b );
				bodies.push( b );
			}
		}
	}
	return { world: w, bodies: bodies };
}

function measure( build ) {
	var medians = [], stats = null;
	for ( var r = 0; r < RUNS; r++ ) {
		var scene = build();
		var w = scene.world, s;
		for ( s = 0; s < SETTLE; s++ ) w.step( 1 / 60 );
		var times = [];
		for ( s = 0; s < MEASURE; s++ ) {
			var t0 = process.hrtime.bigint();
			w.step( 1 / 60 );
			times.push( Number( process.hrtime.bigint() - t0 ) / 1e6 );
		}
		times.sort( function ( a, b ) { return a - b; } );
		medians.push( times[ Math.floor( times.length / 2 ) ] );

		if ( r === 0 ) {
			var manifolds = 0, m = w.narrowphase.contact_manifolds.first;
			while ( m ) { manifolds++; m = m.next_manifold; }
			var moving = 0;
			scene.bodies.forEach( function ( b ) {
				var v = b.linear_velocity;
				if ( Math.sqrt( v.x * v.x + v.y * v.y + v.z * v.z ) > 0.1 ) moving++;
			} );
			stats = { pairs: w.broadphase.collision_pairs.length, manifolds: manifolds, moving: moving, total: scene.bodies.length };
		}
	}
	medians.sort( function ( a, b ) { return a - b; } );
	return {
		median: medians[ Math.floor( medians.length / 2 ) ],
		min: medians[0], max: medians[ medians.length - 1 ], stats: stats
	};
}

function report( name, r, budget ) {
	var line = name.padEnd( 22 ) + r.median.toFixed( 2 ) + 'ms  (range ' + r.min.toFixed( 2 ) + '-' + r.max.toFixed( 2 ) + ')' +
		'  pairs=' + r.stats.pairs + ' manifolds=' + r.stats.manifolds + ' stillMoving=' + r.stats.moving + '/' + r.stats.total;
	if ( budget ) {
		line += r.median <= budget ? '   [OK, budget ' + budget + 'ms]' : '   [OVER BUDGET ' + budget + 'ms]';
	}
	console.log( line );
}

console.log( 'engine: ' + ENGINE );
console.log( 'solver: ' + SOLVER );
console.log( RUNS + ' runs, ' + SETTLE + ' settle ticks, median of ' + MEASURE + ' timed ticks\n' );
if ( which === 'both' || which === 'scene' ) report( '500 props / 3000ch', measure( buildScene ), SCENE_BUDGET_MS );
if ( which === 'both' || which === 'stack' ) report( '385-box pyramid', measure( buildStack ), null );
