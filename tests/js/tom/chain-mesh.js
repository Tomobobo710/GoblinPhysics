(function (Runner, Goblin) {
	Runner.suite('tom');

	// Real chainlink model geometry, shared by node and browser (see chainlink-data.js). Mirrors the
	// mesh-mesh.html example: MeshShape links dropped onto a MeshShape ground as interconnected chains.
	var CHAIN = (typeof module !== 'undefined' && module.exports) ? require('./chainlink-data.js') : window.GOBLIN_CHAINLINK;

	// Bounding box of the raw model (unscaled), used to space chain links like the example does.
	var BB = (function () {
		var min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity], i;
		for (i = 0; i < CHAIN.v.length; i += 3) {
			for (var j = 0; j < 3; j++) {
				if (CHAIN.v[i + j] < min[j]) min[j] = CHAIN.v[i + j];
				if (CHAIN.v[i + j] > max[j]) max[j] = CHAIN.v[i + j];
			}
		}
		return { min: min, max: max };
	})();

	function makeLinkShape(scale) {
		var verts = [], i;
		for (i = 0; i < CHAIN.v.length; i += 3)
			verts.push(new Goblin.Vector3(CHAIN.v[i] * scale, CHAIN.v[i + 1] * scale, CHAIN.v[i + 2] * scale));
		return new Goblin.MeshShape(verts, CHAIN.f);
	}

	// A flat static MeshShape ground plane (2 triangles, +Y outward) at y=0 - the same "ground is a mesh"
	// choice the example makes.
	function makeGround(w, half) {
		var verts = [
			new Goblin.Vector3(-half, 0, -half),
			new Goblin.Vector3(half, 0, -half),
			new Goblin.Vector3(half, 0, half),
			new Goblin.Vector3(-half, 0, half)
		];
		var faces = [0, 2, 1, 0, 3, 2];
		var ground = new Goblin.RigidBody(new Goblin.MeshShape(verts, faces), Infinity);
		ground.position.set(0, 0, 0);
		ground.updateDerived();
		w.addRigidBody(ground);
		return ground;
	}

	// Faithful reproduction of the example's spawnChain: links spaced vertically by link_height*0.7 and
	// each successive link rotated a fixed amount from the previous one (0.4 about y, normalized), so the
	// chain hangs twisted instead of sitting flat. Returns the list of links (so the caller can track the
	// ADJACENT pairs that must stay interlocked) and that chain's link_height (a separation yardstick).
	function spawnChain(w, x, link_count, starting_height, scale) {
		var link_height = (BB.max[1] - BB.min[1]) * scale,
			rot = new Goblin.Quaternion(0, 0.4, 0, 1);
		rot.normalize();
		var chain = [];
		for (var i = 0; i < link_count; i++) {
			var link = new Goblin.RigidBody(makeLinkShape(scale), 1);
			link.position.x = x;
			link.position.y = starting_height - i * link_height * 0.7;
			if (i > 0) {
				link.rotation.multiplyQuaternions(chain[i - 1].rotation, rot);
				link.rotation.normalize();
			}
			link.updateDerived();
			w.addRigidBody(link);
			chain.push(link);
		}
		return { links: chain, link_height: link_height };
	}

	Runner.test('chain', 'chain links settle on a mesh floor (the mesh-mesh.html scene)', function (t) {
		// The example spawns three chains (big/medium/small) of twisted, interconnected MeshShape links
		// onto a MeshShape ground. This reproduces the scene headless. The failure we're guarding against:
		// links FALL THROUGH / explode through the mesh floor (the reported regression).
		var w = t.makeWorld({ gravity: -9.8 });
		w.solver.relaxation = 0.1;   // the example sets this to relax penetration solving / avoid jitter
		makeGround(w, 25);

		var chains = [];
		chains.push(spawnChain(w, -10, 7, 10, 0.6));      // big chain (example: spawnChain(-10, 7, 10))
		chains.push(spawnChain(w, 0, 12, 10, 0.36));      // medium chain (spawnChain(0, 12, 10))
		chains.push(spawnChain(w, 10, 20, 10, 0.216));    // small chain (spawnChain(10, 20, 10))

		// Flat list (for the minY check) + per-adjacent-pair interlock tracking. Each adjacent pair in
		// spawn order is recorded once; we measure the biggest center-distance it ever reaches, in units
		// of that chain's link_height. Interlocked links stay close (< ~1.5 lh); a link that slips out of
		// its neighbor's loop goes farther.
		var links = [];
		var maxSep = [];   // { lh, n } per adjacent pair
		chains.forEach(function (c) {
			for (var i = 0; i < c.links.length; i++) links.push(c.links[i]);
			for (var i = 0; i < c.links.length - 1; i++) maxSep.push({ lh: c.link_height, n: 0 });
		});

		var ticks = 0, minY = Infinity, exploded = false;
		t.onTick(function (world, tick) {
			ticks = tick;
			for (var i = 0; i < links.length; i++) {
				var y = links[i].position.y;
				if (!isFinite(y)) { exploded = true; continue; }
				if (y < minY) minY = y;
			}
			// interlock: update each adjacent pair's max center distance (in link-height units)
			var p = 0;
			chains.forEach(function (c) {
				for (var i = 0; i < c.links.length - 1; i++) {
					var a = c.links[i].position, b = c.links[i + 1].position;
					var d = Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)) / c.link_height;
					if (d > maxSep[p].n) maxSep[p].n = d;
					p++;
				}
			});
		});

		t.log('Drop the example\'s three chains (7/12/20 twisted MeshShape links) onto a MeshShape ground. They must settle on the floor — nothing falls through or explodes — and the links must STAY IN each other\'s loops (adjacent links must not separate).');

		t.expect('no link falls through the mesh floor or explodes (min y > -0.5, all finite)', function (world) {
			if (ticks < 600) return false;
			return {
				ok: !exploded && minY > -0.5,
				detail: 'minY=' + (minY === Infinity ? 'n/a' : minY.toFixed(3)) + ' exploded=' + exploded
			};
		});

		// Interlock assertion: a link that stays threaded through its neighbor's loop keeps its center
		// within ~1.5 link-heights of that neighbor. Exceeding that means the link has come OUT of the
		// loop — a real, user-visible failure (links "fall out of each other's loops").
		t.expect('adjacent links stay interlocked (never separate more than 1.5 link-heights)', function (world) {
			if (ticks < 600) return false;
			var worst = 0, worstPair = -1;
			for (var i = 0; i < maxSep.length; i++) if (maxSep[i].n > worst) { worst = maxSep[i].n; worstPair = i; }
			return {
				ok: worst < 1.5,
				detail: 'worstAdjacentSeparation=' + worst.toFixed(2) + ' link-heights (pair #' + worstPair + ', threshold 1.5)'
			};
		});

		t.simulate(w, 600);
	}, { page: 'mesh', steps: 600, description: 'Reproduces the mesh-mesh.html chain scene: three chains of twisted MeshShape links (7/12/20) dropped onto a static MeshShape ground. Guards against links falling through/exploding the mesh floor AND against links separating out of each other\'s loops.' });
})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner,
	typeof module !== 'undefined' && module.exports ? require('../../../build/goblin.js') : window.Goblin);
