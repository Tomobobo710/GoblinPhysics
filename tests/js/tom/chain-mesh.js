(function (Runner, Goblin) {
	Runner.suite('tom');

	var CHAIN = (typeof module !== 'undefined' && module.exports) ? require('./chainlink-data.js') : window.GOBLIN_CHAINLINK;

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

	// Ring lies flat in local X-Y, hole axis is local Z, built from 14 cross-section rings of 6 verts.
	var LINK_CENTROID = (function () {
		var n = CHAIN.v.length / 3, cx = 0, cy = 0;
		for (var i = 0; i < n; i++) { cx += CHAIN.v[i * 3]; cy += CHAIN.v[i * 3 + 1]; }
		return { x: cx / n, y: cy / n };
	})();
	var LINK_HOLE_RADIUS = (function () {
		var n = CHAIN.v.length / 3, minR = Infinity;
		for (var i = 0; i < n; i++) {
			var dx = CHAIN.v[i * 3] - LINK_CENTROID.x, dy = CHAIN.v[i * 3 + 1] - LINK_CENTROID.y;
			var r = Math.sqrt(dx * dx + dy * dy);
			if (r < minR) minR = r;
		}
		return minR;
	})();

	var LINK_CENTERLINE = (function () {
		var rings = 14, perRing = CHAIN.v.length / 3 / rings;
		var pts = [];
		for (var i = 0; i < rings; i++) {
			var cx = 0, cy = 0, cz = 0;
			for (var j = 0; j < perRing; j++) {
				var idx = (i * perRing + j) * 3;
				cx += CHAIN.v[idx]; cy += CHAIN.v[idx + 1]; cz += CHAIN.v[idx + 2];
			}
			pts.push({ x: cx / perRing, y: cy / perRing, z: cz / perRing });
		}
		return pts;
	})();

	// True if b's centerline path actually crosses a's hole plane within a's hole radius.
	function isThreaded(a, b, scale) {
		var world_to_a = new Goblin.Matrix4();
		world_to_a.copy(a.transform_inverse);
		var p0 = new Goblin.Vector3(), p1 = new Goblin.Vector3();
		var r2 = (LINK_HOLE_RADIUS * scale) * (LINK_HOLE_RADIUS * scale);
		var cx = LINK_CENTROID.x * scale, cy = LINK_CENTROID.y * scale;
		var n = LINK_CENTERLINE.length;
		for (var i = 0; i < n; i++) {
			var c0 = LINK_CENTERLINE[i], c1 = LINK_CENTERLINE[(i + 1) % n];
			p0.set(c0.x * scale, c0.y * scale, c0.z * scale);
			p1.set(c1.x * scale, c1.y * scale, c1.z * scale);
			b.transform.transformVector3(p0);
			b.transform.transformVector3(p1);
			world_to_a.transformVector3(p0);
			world_to_a.transformVector3(p1);

			if ((p0.z >= 0 && p1.z <= 0) || (p0.z <= 0 && p1.z >= 0)) {
				var dz = p1.z - p0.z;
				var t = Math.abs(dz) < 1e-9 ? 0 : (0 - p0.z) / dz;
				if (t < 0) t = 0; else if (t > 1) t = 1;
				var ix = p0.x + (p1.x - p0.x) * t, iy = p0.y + (p1.y - p0.y) * t;
				var dx = ix - cx, dy = iy - cy;
				if (dx * dx + dy * dy < r2) return true;
			}
		}
		return false;
	}

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

	// tiltX90: tip the whole chain 90deg about X around its own center so it falls flat.
	function spawnChain(w, x, link_count, starting_height, scale, tiltX90) {
		var link_height = (BB.max[1] - BB.min[1]) * scale,
			rot = new Goblin.Quaternion(0, 0.4, 0, 1);
		rot.normalize();

		var y0 = starting_height, y1 = starting_height - (link_count - 1) * link_height * 0.7;
		var centerY = (y0 + y1) / 2;

		var tilt = new Goblin.Quaternion(Math.SQRT1_2, 0, 0, Math.SQRT1_2);

		var chain = [];
		var untiltedRotations = [];
		for (var i = 0; i < link_count; i++) {
			var link = new Goblin.RigidBody(makeLinkShape(scale), 1);
			var py = starting_height - i * link_height * 0.7;

			if (tiltX90) {
				var dy = py - centerY;
				link.position.x = x;
				link.position.y = centerY;
				link.position.z = dy;
			} else {
				link.position.x = x;
				link.position.y = py;
			}

			var ownRotation = new Goblin.Quaternion(0, 0, 0, 1);
			if (i > 0) {
				ownRotation.multiplyQuaternions(untiltedRotations[i - 1], rot);
				ownRotation.normalize();
			}
			untiltedRotations.push(ownRotation);

			if (tiltX90) {
				link.rotation.multiplyQuaternions(tilt, ownRotation);
				link.rotation.normalize();
			} else {
				link.rotation.set(ownRotation.x, ownRotation.y, ownRotation.z, ownRotation.w);
			}

			link.updateDerived();
			link._color = '#4af';
			w.addRigidBody(link);
			chain.push(link);
		}
		return { links: chain, link_height: link_height, scale: scale };
	}

	Runner.test('chain', 'chain links settle on a mesh floor (the mesh-mesh.html scene)', function (t) {
		var w = t.makeWorld({ gravity: -9.8 });
		w.solver.relaxation = 0.1;
		makeGround(w, 25);

		var chains = [];
		chains.push(spawnChain(w, -10, 12, 10, 0.36));
		chains.push(spawnChain(w, 10, 12, 10, 0.36, true));

		var links = [];
		chains.forEach(function (c) {
			for (var i = 0; i < c.links.length; i++) {
				var neighbors = [];
				if (i > 0) neighbors.push(c.links[i - 1]);
				if (i < c.links.length - 1) neighbors.push(c.links[i + 1]);
				links.push({ body: c.links[i], neighbors: neighbors, scale: c.scale });
			}
		});
		var everFullyUnthreaded = false;

		var ticks = 0, minY = Infinity, exploded = false;
		var worstVertY = Infinity, tmpV = new Goblin.Vector3();
		t.onTick(function (world, tick) {
			ticks = tick;
			for (var i = 0; i < links.length; i++) {
				var y = links[i].body.position.y;
				if (!isFinite(y)) { exploded = true; continue; }
				if (y < minY) minY = y;

				var verts = links[i].body.shape.vertices;
				for (var k = 0; k < verts.length; k++) {
					tmpV.copy(verts[k]);
					links[i].body.transform.transformVector3(tmpV);
					if (tmpV.y < worstVertY) worstVertY = tmpV.y;
				}
			}
			for (var i = 0; i < links.length; i++) {
				var L = links[i];
				var connected = 0;
				for (var j = 0; j < L.neighbors.length; j++) {
					var n = L.neighbors[j];
					if (isThreaded(L.body, n, L.scale) || isThreaded(n, L.body, L.scale)) connected++;
				}
				if (connected === L.neighbors.length) {
					L.body._color = '#4af';
				} else if (connected > 0) {
					L.body._color = '#ffcc00';
				} else {
					L.body._color = '#ff0000';
					everFullyUnthreaded = true;
				}
			}
		});

		t.log('Drop two medium chains (12 links each) onto a MeshShape ground - one hanging normally, one tipped 90deg about X so it falls flat. Links are tinted live by how many of their OWN starting neighbors they are still threaded to: blue = all, yellow = lost one, red = lost every neighbor.');

		t.expect('no link falls through the mesh floor or explodes (min y > -0.5, all finite)', function (world) {
			if (ticks < 600) return false;
			return {
				ok: !exploded && minY > -0.5,
				detail: 'minY=' + (minY === Infinity ? 'n/a' : minY.toFixed(3)) + ' exploded=' + exploded
			};
		});

		t.expect('at rest no chain link vertex sits below the ground plane (y >= -0.1)', function (world) {
			if (ticks < 600) return false;
			// Check the FINAL state, not the all-time minimum. A vertex that dips below on impact
			// but recovers is fine; only a vertex still below at rest is a real floor penetration.
			var finalWorstY = Infinity, fv = new Goblin.Vector3();
			for (var i = 0; i < links.length; i++) {
				var verts = links[i].body.shape.vertices;
				for (var k = 0; k < verts.length; k++) {
					fv.copy(verts[k]);
					links[i].body.transform.transformVector3(fv);
					if (fv.y < finalWorstY) finalWorstY = fv.y;
				}
			}
			return {
				ok: finalWorstY >= -0.1,
				detail: 'finalWorstY=' + (finalWorstY === Infinity ? 'n/a' : finalWorstY.toFixed(4)) +
					' (all-time worstVertY=' + (worstVertY === Infinity ? 'n/a' : worstVertY.toFixed(4)) + ')'
			};
		});

		t.expect('no link ever fully unthreads from all of its starting neighbors', function (world) {
			if (ticks < 600) return false;
			return {
				ok: !everFullyUnthreaded,
				detail: everFullyUnthreaded ? 'at least one link lost all starting neighbors at some point' : 'every link kept at least one neighbor the whole run'
			};
		});

		t.simulate(w, 600);
	}, { page: 'mesh', steps: 600, description: 'Reproduces the mesh-mesh.html chain scene: two chains of twisted MeshShape links (12 each) dropped onto a static MeshShape ground. Guards against links falling through/exploding the mesh floor AND against links separating out of each other\'s loops.' });
})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner,
	typeof module !== 'undefined' && module.exports ? require('../../../build/goblin.js') : window.Goblin);
