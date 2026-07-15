// Tom's Suite — FPSCharacterController helpers. Build a Goblin.World + Goblin.FPSCharacterController
// directly; the renderer (render.js) draws every world body by shape/_color, exactly like everything
// else in this suite.
//
// A controller test does:
//   1. build a Goblin.World + FPSCharacterController + any object bodies,
//   2. register those + the controller for the viewer via PBF.renderables(t, controller, extras),
//   3. drive the controller each tick via PBF.drive(t, controller, cmdFor) (installs a tick hook),
//   4. call t.simulate(world, ticks) and assert with t.expect(...) live — same as every other test here.
(function (root, factory) {
	var Goblin = (typeof module !== 'undefined' && module.exports)
		? require('../../../build/goblin.js')   // js/tom/_util_fps.js -> ../../../build/goblin.js
		: root.Goblin;
	var GoblinRunner = (typeof module !== 'undefined' && module.exports)
		? require('../runner.js')
		: root.GoblinRunner;
	var mod = factory(Goblin, GoblinRunner);
	if (typeof module !== 'undefined' && module.exports) module.exports = mod;
	else root.PBF = mod;
})(typeof window !== 'undefined' ? window : this, function (Goblin, GoblinRunner) {

	var DT = 1 / 60;

	// Default controller options: bare physics core, no cosmetic identity needed for a headless/visual
	// physics test. FPSCharacterController has no weapon/combat/grab/model/view/input components at
	// all — there's nothing to disable here.
	var CONTROLLER_COLOR = '#cc4444';

	// Shared material defaults applied to every body built by this file unless the caller passes its
	// own — friction/restitution/damping meaningfully change contact behavior (a bare-default floor
	// vs. this one changes how a pushed box actually settles), so every helper here goes through one
	// place rather than letting Goblin's own bare RigidBody defaults (friction:0.5, restitution:0.1,
	// zero damping) apply inconsistently per test.
	var MATERIAL_DEFAULTS = { friction: 3.0, restitution: 0.33, linearDamping: 0.1, angularDamping: 0.9 };
	function applyMat(b, mat) {
		var m = mat || {};
		b.friction = m.friction !== undefined ? m.friction : MATERIAL_DEFAULTS.friction;
		b.restitution = m.restitution !== undefined ? m.restitution : MATERIAL_DEFAULTS.restitution;
		b.linear_damping = m.linearDamping !== undefined ? m.linearDamping : MATERIAL_DEFAULTS.linearDamping;
		b.angular_damping = m.angularDamping !== undefined ? m.angularDamping : MATERIAL_DEFAULTS.angularDamping;
	}

	function makeWorld() {
		var w = new Goblin.World(new Goblin.SAPBroadphase(), new Goblin.NarrowPhase(), new Goblin.IterativeSolver());
		w.gravity = new Goblin.Vector3(0, -9.81, 0);
		return w;
	}

	// A static floor slab, top face at y=0 — the standard scene every controller test shares.
	function flatWorld() {
		var w = makeWorld();
		var floor = new Goblin.RigidBody(new Goblin.BoxShape(60, 0.5, 60), Infinity);
		floor.position.set(0, -0.5, 0);
		applyMat(floor, null);
		floor._color = '#333333';
		w.addRigidBody(floor);
		w._floor = floor;
		return w;
	}

	// A visible dynamic object (box). side/mass in world units; pos is a Goblin.Vector3-shaped {x,y,z}.
	// mat overrides friction/restitution; objects default to {friction:0.4, restitution:0.1} —
	// distinct from the engine MATERIAL_DEFAULTS above, which is the fallback for static geometry.
	function object(w, side, mass, pos, color, mat) {
		var b = new Goblin.RigidBody(new Goblin.BoxShape(side / 2, side / 2, side / 2), mass);
		b.position.set(pos.x, pos.y, pos.z);
		applyMat(b, Object.assign({ friction: 0.4, restitution: 0.1 }, mat || {}));
		b._color = color || '#cc4444';
		w.addRigidBody(b);
		return b;
	}

	// Static (mass=Infinity) box geometry — walls, platforms, ledges, ceilings, ramps. half-extents
	// hx/hy/hz; pos {x,y,z}; rot an optional Goblin.Quaternion (see axisAngleQuat below).
	function staticBox(hx, hy, hz, pos, color, mat, rot) {
		var b = new Goblin.RigidBody(new Goblin.BoxShape(hx, hy, hz), Infinity);
		b.position.set(pos.x, pos.y, pos.z);
		if (rot) b.rotation.set(rot.x, rot.y, rot.z, rot.w);
		b.updateDerived();
		applyMat(b, mat);
		b._color = color || '#888888';
		return b;
	}

	// A static ladder volume — a solid box tagged isLadder so FPSCharacterController's mount probe
	// recognizes it. half-extents hx/hy/hz; pos {x,y,z}.
	function ladder(w, hx, hy, hz, pos, color) {
		var b = new Goblin.RigidBody(new Goblin.BoxShape(hx, hy, hz), Infinity);
		b.position.set(pos.x, pos.y, pos.z);
		b.updateDerived();
		applyMat(b, { friction: 0, restitution: 0 });
		b._color = color || '#8a5a2b';
		b.isLadder = true;
		w.addRigidBody(b);
		return b;
	}

	// Spawn a controller. extra merges over the base options (e.g. {scale:0.5, climbSteepSlopes:true}).
	function spawn(w, pos, extra) {
		var opts = Object.assign({ position: pos, color: CONTROLLER_COLOR }, extra || {});
		var p = new Goblin.FPSCharacterController(w, opts);
		return p;
	}

	// Register the renderable bodies for the viewer. render.js's captureSetup snapshots ctx.bodies ONCE
	// at t=0 (see runner.js: it's a plain array, not re-scanned per frame) — so the only correct source
	// of "everything visible" is the WORLD's actual contents at setup time: the floor, every wall/ramp/
	// object a test built, and the controller's own body + ghost. Pulling straight from
	// `world.rigid_bodies` means a test never has to remember to list its own scene explicitly, and can
	// never miss a body it built.
	//
	// A controller that REBUILDS its own body mid-run (crouch/setScale) swaps `controller.body`/
	// `controller._ghost` for fresh Goblin.RigidBody instances added into the SAME world — those are
	// still caught by this scan, and the OLD body/ghost (removed from the world on rebuild) drop out of
	// render.js's live mesh diff rather than lingering as a stale mesh (see render.js's per-frame
	// world.rigid_bodies reconciliation).
	function renderables(t, controller, extras) {
		var bodies = controller.world.rigid_bodies.slice();
		if (extras) extras.forEach(function (b) { if (bodies.indexOf(b) === -1) bodies.push(b); });
		t.bodies = bodies;
		return bodies;
	}

	// Install a tick hook that drives beginStep/world.step/endStep each tick. `cmdFor(tick)` returns
	// the input command for that tick (e.g. {forward:1, sprint:true}); default is no input. The
	// controller brackets a single physics step exactly like a real game loop would.
	//
	// NOTE: this suite's runner (js/runner.js) already owns `world.step()` inside t.simulate()/the
	// render loop — onTick fires BEFORE that per-tick step (see runner.js's simulate()/render.js's
	// loop()), so beginStep goes in onTick and endStep must run immediately after. Since neither
	// runner exposes a "post-step, pre-eval" hook, endStep is called from an evalTick-time wrapper:
	// PBF.drive wraps t.expect to call endStep exactly once per tick, the first time any expectation
	// is evaluated that tick.
	function drive(t, controller, cmdFor) {
		var lastEndTick = 0;
		function ensureEnded(tick) {
			if (tick > lastEndTick) { controller.endStep(DT); lastEndTick = tick; }
		}
		t.onTick(function (world, tick) {
			var cmd = (cmdFor ? cmdFor(tick) : null) || {};
			controller.beginStep(cmd, DT);
		});
		// Patch evalTick so endStep runs right after world.step(), before any predicate reads state —
		// evalTick is called every tick by both the headless simulate() loop and the live render loop,
		// always immediately after world.step() and before expectations are read (see runner.js).
		var realEvalTick = t.evalTick;
		t.evalTick = function (world, tick) {
			ensureEnded(tick);
			return realEvalTick(world, tick);
		};
	}

	// ---- Sequential multi-block scenes -----------------------------------------------------------------
	// Some tests run N variants of a scene IN TURN inside ONE reused world — wipe + rebuild at the start
	// of each block, watch it play out, move to the next (e.g. sweeping a ramp test across several
	// angles, or a push test across several object masses, one after another in the same viewer run).
	// This is built purely from onTick (fires before world.step(), so a block boundary just rebuilds the
	// world's rigid bodies there) + the same endStep-in-evalTick trick drive() uses, generalized to a
	// CURRENT controller that changes across block boundaries.
	//
	// blocks: array of block descriptors. blockTicks: ticks per block (shared by all blocks).
	// build(world, blockIndex, blockDescriptor) must wipe the world's existing rigid bodies and
	// construct the new block's scene, returning the controller to drive for this block.
	// onBlockTick(local, blockIndex, blockDescriptor, controller, world) runs once per tick, AFTER this
	// tick's beginStep/world.step/endStep have completed, with `local` the 1-based tick index WITHIN the
	// current block — this is where a test records its measurements.
	function sequentialBlocks(t, world, blocks, blockTicks, build, onBlockTick, cmdFor) {
		var cur = null, pbTick = 0;
		function rebuild(idx) {
			world.rigid_bodies.slice().forEach(function (b) { world.removeRigidBody(b); });
			cur = build(world, idx, blocks[idx]);
		}
		rebuild(0);
		var lastEndTick = 0;
		t.onTick(function (w, tick) {
			pbTick = tick;
			var idx = Math.floor((tick - 1) / blockTicks);
			var local = ((tick - 1) % blockTicks) + 1;
			if (idx < blocks.length && local === 1 && tick > 1) rebuild(idx);
			var cmd = (cmdFor && idx < blocks.length) ? (cmdFor(local, idx, blocks[idx]) || {}) : {};
			cur.beginStep(cmd, DT);
		});
		var realEvalTick = t.evalTick;
		t.evalTick = function (w, tick) {
			if (tick > lastEndTick) {
				cur.endStep(DT);
				lastEndTick = tick;
				var idx = Math.floor((tick - 1) / blockTicks);
				var local = ((tick - 1) % blockTicks) + 1;
				if (idx < blocks.length) onBlockTick(local, idx, blocks[idx], cur, w);
			}
			return realEvalTick(w, tick);
		};
		return { tickOf: function () { return pbTick; }, controllerOf: function () { return cur; } };
	}

	// ---- SCALE-PARAMETRIC support ---------------------------------------------------------------------
	// Every controller test runs at three character scales. The WORLD is NOT scaled (ramps stay the
	// same angle, walls stay put); only the CHARACTER scales, and the test's own instruments
	// (proportional objects, head-relative drops) + world-unit thresholds scale with it.
	var SCALES = [0.5, 1.0, 2.0];

	// Per-scale helper bundle handed to a scaleTest build fn.
	function scaleHelpers(SC) {
		return {
			SC: SC,
			// world-unit threshold that scales with the character (a speed, a distance the body covers)
			sc: function (x) { return x * SC; },
			flat: flatWorld,
			// scale-PROPORTIONAL object: side and mass scale so its size+mass RATIO to the character is
			// constant (a fixed 1x1x1 object is a step at 2x / an immovable wall at 0.5x). base values
			// are scale-1.
			scaledObject: function (w, baseSide, baseMass, z, color, mat) {
				var s = baseSide * SC, m = baseMass * SC * SC * SC;
				return object(w, s, m, { x: 0, y: s / 2, z: z * SC }, color, mat);
			},
			// A square pillar ladder (1.2x6x1.2 pre-scale, climbable on every face) centered at the
			// world origin, base on the floor.
			pillarLadder: function (w, color) {
				var wd = 0.6 * SC, h = 6 * SC;
				return ladder(w, wd, h / 2, wd, { x: 0, y: h / 2, z: 0 }, color);
			},
			// drop a box onto the character's ACTUAL head from `above` units over it (head-relative, so
			// impact speed is the same at every scale). side/mass default to 0.8/2.
			dropOnHead: function (w, p, above, side, mass, mat) {
				var s = side !== undefined ? side : 0.8;
				var headTop = p.body.position.y + p.height / 2;
				var b = new Goblin.RigidBody(new Goblin.BoxShape(s / 2, s / 2, s / 2), mass !== undefined ? mass : 2);
				b.position.set(0, headTop + above, 0);
				applyMat(b, Object.assign({ friction: 0.4, restitution: 0 }, mat || {}));
				b._color = '#0f0';
				w.addRigidBody(b);
				return b;
			},
			// spawn with this scale baked in; extra merges (e.g. climbSteepSlopes:true)
			spawn: function (w, pos, extra) { return spawn(w, pos, Object.assign({ scale: SC }, extra || {})); },
			// spawn feet-planted at the floor (removes the fall transient a fixed y=1 spawn causes off-scale)
			feetSpawn: function (w, x, z, extra) { return spawn(w, { x: x, y: 0.9 * SC + 0.001, z: z }, Object.assign({ scale: SC }, extra || {})); }
		};
	}

	// Register ONE logical test at all three scales as separate watchable rows: "<name> @0.5" etc.
	// build(t, S) receives the test ctx and the per-scale helper bundle S (S.SC, S.sc, S.scaledObject, S.spawn, ...).
	function scaleTest(group, id, name, build, meta) {
		meta = meta || {};
		SCALES.forEach(function (SC) {
			var S = scaleHelpers(SC);
			GoblinRunner.test(group, id + ' ' + name + ' @' + SC, function (t) {
				build(t, S);
			}, {
				visual: true,
				steps: meta.steps || 0,
				page: meta.page || group,
				description: (meta.description || '') + '  [scale ' + SC + ']'
			});
		});
	}

	function hsp(p) { var v = p.body.linear_velocity; return Math.hypot(v.x, v.z); }

	// Goblin's own Quaternion has no fromAxisAngle helper; construct one directly from the standard
	// axis-angle formula. axis need not be pre-normalized for a UNIT axis like (1,0,0) — every ramp
	// test here rotates about a single cardinal axis.
	function axisAngleQuat(x, y, z, rad) {
		var s = Math.sin(rad / 2);
		return new Goblin.Quaternion(x * s, y * s, z * s, Math.cos(rad / 2));
	}

	return {
		DT: DT, SCALES: SCALES,
		flatWorld: flatWorld, makeWorld: makeWorld, object: object, staticBox: staticBox, ladder: ladder, spawn: spawn,
		renderables: renderables, drive: drive, sequentialBlocks: sequentialBlocks,
		scaleHelpers: scaleHelpers, scaleTest: scaleTest,
		axisAngleQuat: axisAngleQuat,
		hsp: hsp
	};
});
