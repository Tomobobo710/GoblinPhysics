/**
 * Browser-only visualization + live console for the Goblin test suite. Loaded after runner.js + the
 * test files. Exposes window.GoblinRender:
 *   run(test, opts)  — run one test live: stream its console, draw its 3D (if visual), resolve verdict.
 *   clear()          — clear the viewport + console.
 *
 * HONESTY CONTRACT: the picture, the console, and the verdict all come from the SAME run of the SAME
 * test. run(test) calls Runner.runOne(test) once — that returns { ok, error, ctx, logs } — and we show
 * exactly those logs, draw exactly that ctx, and report exactly that ok. No cross-run caching.
 */
(function () {
	var Runner = window.GoblinRunner, Goblin = window.Goblin;

	var R = null; // three.js state
	var anim = { world: null, meshes: [], steps: 0, tick: 0, stepping: false };
	var consoleEl = function () { return document.getElementById('console'); };
	var hudEl = function () { return document.getElementById('hud'); };
	var viewport3d = function () { return document.getElementById('viewport3d'); };

	// Self-contained UP-LOCKED orbit controller (no external file). Drag to orbit around `target`; the
	// world up (+Y) stays up, so the horizon is always level and you can't roll/get disoriented like
	// TrackballControls let you. Wheel zooms. Same .update()/.target interface the loop expects.
	function makeOrbit(camera, dom) {
		var target = new THREE.Vector3(0, 0, 0);
		// current spherical position of the camera relative to target
		var off = camera.position.clone().sub(target);
		var radius = off.length();
		var yaw = Math.atan2(off.x, off.z);        // around Y
		var pitch = Math.asin(Math.max(-1, Math.min(1, off.y / radius)));  // up/down
		var dragging = false, lx = 0, ly = 0;
		var PITCH_LIMIT = Math.PI / 2 - 0.05;      // don't flip over the poles

		dom.addEventListener('mousedown', function (e) { dragging = true; lx = e.clientX; ly = e.clientY; });
		window.addEventListener('mouseup', function () { dragging = false; });
		window.addEventListener('mousemove', function (e) {
			if (!dragging) return;
			var dx = e.clientX - lx, dy = e.clientY - ly; lx = e.clientX; ly = e.clientY;
			yaw -= dx * 0.005;
			pitch = Math.max(-PITCH_LIMIT, Math.min(PITCH_LIMIT, pitch + dy * 0.005));
		});
		dom.addEventListener('wheel', function (e) {
			e.preventDefault();
			radius *= (1 + (e.deltaY > 0 ? 0.1 : -0.1));
			radius = Math.max(2, Math.min(200, radius));
		}, { passive: false });

		return {
			target: target,
			update: function () {
				var cp = Math.cos(pitch);
				camera.position.set(
					target.x + radius * cp * Math.sin(yaw),
					target.y + radius * Math.sin(pitch),
					target.z + radius * cp * Math.cos(yaw)
				);
				camera.up.set(0, 1, 0);            // up stays up — always level
				camera.lookAt(target);
			}
		};
	}

	function init3D() {
		if (R) return R;
		var vp = viewport3d();
		var renderer = new THREE.WebGLRenderer({ antialias: true });
		renderer.setClearColor(0x05070a);
		renderer.setSize(vp.clientWidth, vp.clientHeight);
		vp.appendChild(renderer.domElement);
		var scene = new THREE.Scene();
		var camera = new THREE.PerspectiveCamera(45, vp.clientWidth / vp.clientHeight, 0.1, 2000);
		camera.position.set(9, 7, 16);
		var controls = makeOrbit(camera, renderer.domElement);
		scene.add(new THREE.AmbientLight(0x999999));
		var dir = new THREE.DirectionalLight(0xffffff, 0.7); dir.position.set(6, 12, 8); scene.add(dir);
		scene.add(new THREE.GridHelper(40, 40, 0x223344, 0x151f28));
		window.addEventListener('resize', resize3D);
		R = { renderer: renderer, scene: scene, camera: camera, controls: controls, meshes: [], extras: [] };
		loop();
		return R;
	}
	function resize3D() {
		if (!R) return; var vp = viewport3d();
		if (!vp.clientWidth) return;
		R.renderer.setSize(vp.clientWidth, vp.clientHeight);
		R.camera.aspect = vp.clientWidth / vp.clientHeight; R.camera.updateProjectionMatrix();
	}

	// geometry for one primitive shape. Every Goblin shape the tests use is handled here.
	function geoForShape(s) {
		if (s instanceof Goblin.SphereShape) return new THREE.SphereGeometry(s.radius, 24, 16);
		if (s instanceof Goblin.BoxShape) return new THREE.BoxGeometry(s.half_width * 2, s.half_height * 2, s.half_depth * 2);
		if (s instanceof Goblin.CylinderShape) return new THREE.CylinderGeometry(s.radius, s.radius, s.half_height * 2, 24);
		if (s instanceof Goblin.ConeShape) return new THREE.CylinderGeometry(0, s.radius, s.half_height * 2, 24);
		// PlaneShape stores _half_width/_half_height/_half_depth with the flat axis = 0 (per orientation);
		// a thin slab on that axis renders it correctly. Give the flat axis a small thickness so it's visible.
		if (s instanceof Goblin.PlaneShape) return new THREE.BoxGeometry(s._half_width * 2 || 0.04, s._half_height * 2 || 0.04, s._half_depth * 2 || 0.04);
		if (s instanceof Goblin.ConvexShape) {
			var verts = s.vertices.map(function (v) { return new THREE.Vector3(v.x, v.y, v.z); });
			return new THREE.ConvexGeometry(verts);
		}
		// MeshShape: build the geometry straight from the collider's own triangles — exactly what the
		// physics sees (each triangle carries local-space a/b/c corners).
		if (s instanceof Goblin.MeshShape) {
			var g = new THREE.Geometry();
			s.triangles.forEach(function (tri) {
				var base = g.vertices.length;
				g.vertices.push(new THREE.Vector3(tri.a.x, tri.a.y, tri.a.z),
					new THREE.Vector3(tri.b.x, tri.b.y, tri.b.z),
					new THREE.Vector3(tri.c.x, tri.c.y, tri.c.z));
				g.faces.push(new THREE.Face3(base, base + 1, base + 2));
			});
			g.computeFaceNormals();
			return g;
		}
	}
	function litMesh(geo, color) {
		var mesh = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ color: color, transparent: true, opacity: 0.82 }));
		mesh.add(new THREE.Mesh(geo.clone(), new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true, transparent: true, opacity: 0.12 })));
		return mesh;
	}
	function meshForBody(b) {
		var s = b.shape, color = b._color ? parseInt(b._color.replace('#', '0x')) : 0x4488ff;
		// CompoundShape: draw each child at its local offset/rotation inside a group; the group then
		// follows the body's transform (syncMesh sets that).
		if (s instanceof Goblin.CompoundShape) {
			var grp = new THREE.Group();
			s.child_shapes.forEach(function (child) {
				var cm = litMesh(geoForShape(child.shape), color);
				cm.position.set(child.position.x, child.position.y, child.position.z);
				cm.quaternion.set(child.rotation.x, child.rotation.y, child.rotation.z, child.rotation.w);
				grp.add(cm);
			});
			return grp;
		}
		// CapsuleShape has no single THREE geometry: draw it as a group — a cylinder barrel plus a
		// hemisphere cap at each end, at ±cylinder_half_height along the local Y axis (capsule's long axis).
		if (s instanceof Goblin.CapsuleShape) {
			var cap = new THREE.Group();
			cap.add(litMesh(new THREE.CylinderGeometry(s.radius, s.radius, s.cylinder_height, 24), color));
			// Only the OUTER hemisphere at each end, so it doesn't overlap the barrel: top cap is the upper
			// half (theta 0..π/2), bottom cap the lower half (theta π/2..π). SphereGeometry(r, wSeg, hSeg,
			// phiStart, phiLength, thetaStart, thetaLength).
			var half = Math.PI / 2;
			var topCap = litMesh(new THREE.SphereGeometry(s.radius, 24, 12, 0, Math.PI * 2, 0, half), color);
			topCap.position.y = s.cylinder_half_height; cap.add(topCap);
			var botCap = litMesh(new THREE.SphereGeometry(s.radius, 24, 12, 0, Math.PI * 2, half, half), color);
			botCap.position.y = -s.cylinder_half_height; cap.add(botCap);
			return cap;
		}
		return litMesh(geoForShape(s), color);
	}
	function syncMesh(m, b) { m.position.set(b.position.x, b.position.y, b.position.z); m.quaternion.set(b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w); }
	function clearScene() { if (!R) return; R.meshes.forEach(function (m) { R.scene.remove(m); if (m._b) m._b._drawable = null; }); R.meshes = []; R.extras.forEach(function (m) { R.scene.remove(m); }); R.extras = []; }
	function drawBodies(bodies) { bodies.forEach(function (b) { var m = meshForBody(b); m._b = b; b._drawable = m; R.meshes.push(m); R.scene.add(m); syncMesh(m, b); }); }
	function drawRay(ray) {
		var g = new THREE.Geometry();
		g.vertices.push(new THREE.Vector3(ray.start[0], ray.start[1], ray.start[2]), new THREE.Vector3(ray.stop[0], ray.stop[1], ray.stop[2]));
		var line = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x58a6ff })); R.extras.push(line); R.scene.add(line);
		if (ray.hit) { var hm = new THREE.Mesh(new THREE.SphereGeometry(0.18, 12, 8), new THREE.MeshBasicMaterial({ color: 0xff5555 })); hm.position.set(ray.hit[0], ray.hit[1], ray.hit[2]); R.extras.push(hm); R.scene.add(hm); }
	}
	// sweptshape tests: draw the base shape at the START and END of the sweep (faint), plus a line along
	// the sweep axis, so the swept volume is visible (the base shape smeared from start to end).
	function midpoint(a, b) { return { x: (a[0] + b[0]) / 2, y: (a[1] + b[1]) / 2, z: (a[2] + b[2]) / 2 }; }
	function drawSwept(sw) {
		[sw.start, sw.end].forEach(function (pt) {
			var m = litMesh(geoForShape(sw.shape), 0x45b7d1);
			m.material.opacity = 0.4;
			m.position.set(pt[0], pt[1], pt[2]);
			R.extras.push(m); R.scene.add(m);
		});
		var g = new THREE.Geometry();
		g.vertices.push(new THREE.Vector3(sw.start[0], sw.start[1], sw.start[2]), new THREE.Vector3(sw.end[0], sw.end[1], sw.end[2]));
		var line = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x9fe7ff }));
		R.extras.push(line); R.scene.add(line);
	}
	// support-point tests: a green arrow showing the query DIRECTION, and a red marker on the shape at
	// the found support point (the farthest point of the shape in that direction).
	function drawSupport(sup) {
		var p = new THREE.Vector3(sup.point[0], sup.point[1], sup.point[2]);
		var dir = new THREE.Vector3(sup.dir[0], sup.dir[1], sup.dir[2]).normalize();
		// arrow of fixed length pointing along the query direction, anchored at the support point
		var arrow = new THREE.ArrowHelper(dir, p.clone().sub(dir.clone().multiplyScalar(3)), 3, 0x3fb950, 0.6, 0.35);
		R.extras.push(arrow); R.scene.add(arrow);
		var m = new THREE.Mesh(new THREE.SphereGeometry(0.18, 12, 8), new THREE.MeshBasicMaterial({ color: 0xff5555 }));
		m.position.copy(p); R.extras.push(m); R.scene.add(m);
	}
	function frameCamera(bodies) {
		if (!R || !bodies.length) return;
		var cx = 0, cy = 0, cz = 0; bodies.forEach(function (b) { cx += b.position.x; cy += b.position.y; cz += b.position.z; });
		R.controls.target.set(cx / bodies.length, cy / bodies.length, cz / bodies.length);
	}

	// ---- console (criteria-first checklist) ----
	// Paint the whole timeline immediately: narration lines shown, criteria shown as PENDING (grey ○).
	// Then, on a stagger, reveal narration in order and flip each criterion to ✓/✗ in place — so you see
	// the full list of what's being checked first, then watch results land (like mocha).
	function clearConsole() { var c = consoleEl(); if (c) c.innerHTML = ''; }

	function paintTimeline(timeline) {
		var c = consoleEl(); if (!c) return [];
		c.innerHTML = '';
		var nodes = [];
		timeline.forEach(function (e) {
			var d = document.createElement('div');
			if (e.type === 'criterion') {
				d.className = 'cl crit pending';
				d.textContent = '○ ' + e.label;
			} else {
				d.className = 'cl info hidden';
				d.textContent = '  ' + e.msg;
			}
			c.appendChild(d);
			nodes.push({ el: d, entry: e });
		});
		return nodes;
	}
	// Reveal/flip entries one at a time. Narration lines fade in; criteria flip pending -> pass/fail.
	function revealTimeline(nodes, fast, onDone) {
		var c = consoleEl();
		var i = 0;
		(function step() {
			if (i >= nodes.length) { if (onDone) onDone(); return; }
			var n = nodes[i++];
			if (n.entry.type === 'criterion') {
				var ok = n.entry.status === 'pass';
				n.el.className = 'cl crit ' + (ok ? 'pass' : 'fail');
				n.el.textContent = (ok ? '✓ ' : '✗ ') + n.entry.label + (n.entry.detail ? '   — ' + n.entry.detail : '');
			} else {
				n.el.className = 'cl info';
			}
			if (c) c.scrollTop = c.scrollHeight;
			setTimeout(step, fast ? 0 : (n.entry.type === 'criterion' ? 120 : 45));
		})();
	}

	// ---- the public run ----
	// Two kinds of test:
	//  • SCHEDULED dynamics (ctx.schedule non-empty): we drive the live sim ourselves and fire the test's
	//    t.at(tick, …) checks at their ticks against the on-screen world. Each ✓/✗ lands the instant the
	//    physics reaches the moment it measures — one world, one clock, nothing replayed.
	//  • everything else (geometry / math / un-scheduled): computed instantly; we replay the finished
	//    timeline on a stagger just for readability. (These have no time dimension to be "live" against.)
	function run(test, opts) {
		opts = opts || {};
		init3D();
		document.getElementById('empty').style.display = 'none';
		anim.live = false; anim.world = null; anim.meshes = [];
		clearScene(); clearConsole();

		var hud = hudEl();
		hud.style.display = 'block';
		function setHud(state) {
			hud.innerHTML = '<div class="vt">' + test.page + ' — ' + test.name + '</div>'
				+ (state === 'run' ? '<div class="vv" style="color:#8b949e">running…</div>'
					: '<div class="vv ' + (state ? 'pass' : 'fail') + '">' + (state ? 'PASS' : 'FAIL: ' + lastResult.error) + '</div>')
				+ (test.steps > 0 ? '<div class="vv" id="tickline"></div>' : '');
		}
		var lastResult;
		setHud('run');

		// Capture the test's setup WITHOUT running its simulate() — this leaves the world at t=0, the
		// criteria all pending, and ctx.schedule populated. We then own the stepping.
		var cap = captureSetup(test);
		lastResult = cap.result;
		var nodes = paintTimeline(cap.result.timeline);         // criteria render as pending ○
		var entryToNode = {}; nodes.forEach(function (n) { entryToNode[nodeKey(n.entry)] = n; });

		// reveal the narration lines immediately (they're context, not results)
		nodes.forEach(function (n) { if (n.entry.type === 'log') n.el.className = 'cl info'; });

		if (cap.scheduled && cap.world) {
			// LIVE: draw the world at t=0, then step it in the render loop, firing scheduled checks at
			// their ticks and flipping their console line + the 3D in lockstep.
			drawBodies(cap.bodies); frameCamera(cap.bodies);
			anim.ctx = cap.ctx; anim.world = cap.world; anim.meshes = R.meshes.slice();
			anim.totalTicks = cap.simTicks;   // the tick count the test passed to t.simulate()
			anim.tick = 0; anim.live = true; anim.fast = !!opts.fast; anim._finished = false;
			anim._accum = 0; anim._last = null;   // reset the real-time accumulator for this run
			// reflect any criterion whose status changed onto its console line
			anim.reflect = function () {
				cap.result.timeline.forEach(function (e) { if (e.type === 'criterion') updateNode(entryToNode[nodeKey(e)], e); });
			};
			anim.onFinish = function () {
				anim.ctx.failUnmet();   // any predicate never satisfied by the end -> red, live
				anim.reflect();
				var ok = true, err = '';
				cap.result.timeline.forEach(function (e) { if (e.type === 'criterion' && e.status === 'fail') { ok = false; err = (err ? err + '; ' : '') + e.label; } });
				lastResult = { suite: test.suite, group: test.group, name: test.name, ok: ok, error: err, ctx: cap.ctx };
				setHud(ok);
				if (opts.onComplete) opts.onComplete(lastResult);
			};
			return lastResult;
		}

		// NON-LIVE (geometry / math): result is already computed; replay the checklist on a stagger for
		// readability, draw the static state / ray.
		if (cap.ctx.ray) drawRay(cap.ctx.ray);
		if (cap.ctx.support) drawSupport(cap.ctx.support);
		if (cap.ctx.swept) { drawSwept(cap.ctx.swept); frameCamera([{ position: midpoint(cap.ctx.swept.start, cap.ctx.swept.end) }]); }
		if (cap.bodies && cap.bodies.length) { drawBodies(cap.bodies); frameCamera(cap.bodies); }
		revealTimeline(nodes, opts.fast, function () {
			setHud(cap.result.ok);
			if (opts.onComplete) opts.onComplete(cap.result);
		});
		return cap.result;
	}

	// Run the test's fn but intercept simulate() so the world is left at t=0 with expectations declared
	// (all pending) and nothing evaluated yet — the renderer will step the world and evaluate live.
	function captureSetup(test) {
		var captured = { world: null, simTicks: 0, isLive: false };
		var clone = {
			suite: test.suite, group: test.group, name: test.name, page: test.page, visual: test.visual, steps: test.steps, description: test.description,
			fn: function (ctx) {
				var realSim = ctx.simulate;
				ctx.simulate = function (world, totalTicks) { captured.world = world; captured.simTicks = totalTicks; captured.isLive = true; /* do NOT step or evaluate */ };
				// let any error propagate to runOne, which records it as the test's failure; finally just
				// restores simulate so the intercept doesn't leak.
				try { test.fn(ctx); } finally { ctx.simulate = realSim; }
			}
		};
		var result = Runner.runOne(clone);
		return {
			result: result, ctx: result.ctx, world: captured.world, bodies: result.ctx.bodies,
			scheduled: captured.isLive && result.ctx.expectations && result.ctx.expectations.length > 0,
			simTicks: captured.simTicks
		};
	}
	function nodeKey(entry) { return entry.type + ':' + (entry.label || entry.msg); }

	// render a criterion's console line for its CURRENT state: pending shows the live value (grey ○),
	// pass/fail shows the resolved ✓/✗ with detail.
	function updateNode(n, entry) {
		if (!n) return;
		if (entry.status === 'pending') {
			n.el.className = 'cl crit pending';
			n.el.textContent = '○ ' + entry.label + (entry.detail ? '   — ' + entry.detail : '');
		} else {
			var ok = entry.status === 'pass';
			n.el.className = 'cl crit ' + (ok ? 'pass' : 'fail');
			n.el.textContent = (ok ? '✓ ' : '✗ ') + entry.label + (entry.detail ? '   — ' + entry.detail : '');
		}
	}

	var DT_MS = 1000 / 60;   // one physics tick is 1/60 s of wall-clock

	function loop(now) {
		requestAnimationFrame(loop);
		if (anim.live && anim.world && anim.ctx) {
			// Fixed-timestep with a real-time accumulator: advance one 1/60 tick per 16.67 ms of ACTUAL
			// elapsed wall-clock, independent of the monitor's refresh rate — so a 120-tick test always
			// visibly takes ~2 s on any display. Test Now runs at a speed multiplier so batches don't crawl.
			if (anim._last == null) anim._last = now;
			var speed = anim.fast ? 6 : 1;
			anim._accum += Math.min(250, (now - anim._last)) * speed;   // clamp huge gaps (tab was hidden)
			anim._last = now;
			// Suppress the early-out when the test scripts mid-sim events (a shove/jiggle), so a two-phase
			// scene always plays its full tick budget — matching headless simulate().
			var canEarlyOut = !(anim.ctx.tickHooks && anim.ctx.tickHooks.length);
			var done = false, guard = 0;
			while (anim._accum >= DT_MS && anim.tick < anim.totalTicks && !(done && canEarlyOut) && guard++ < 600) {
				anim._accum -= DT_MS;
				anim.ctx.runTickHooks(anim.world, anim.tick + 1);   // fire scripted events before the step
				anim.world.step(1 / 60); anim.tick++;
				// evaluate EVERY live expectation against the world right now — any that just became true
				// flips to pass this tick; the rest update their live value. This is the real check.
				done = anim.ctx.evalTick(anim.world, anim.tick);
			}
			anim.reflect();                                   // paint pending values + any flips
			// Reconcile the drawn mesh set with the world's ACTUAL live rigid bodies. anim.meshes started
			// as a fixed snapshot from t=0 (drawBodies(cap.bodies) in run()), but a test can spawn bodies
			// mid-run from inside its own tick callback (a thrown crate, a box dropped on the head at
			// tick 20/30) — those don't exist yet when the snapshot was taken, so without this diff they
			// silently never appear. Add a mesh for any world body we haven't drawn yet; drop the mesh for
			// any body no longer in the world (removeRigidBody, e.g. a controller rebuilding its collider
			// on crouch/scale). Cheap: rigid_bodies lists are small in these tests, and this only runs once
			// per animation frame, not per physics substep.
			var live = anim.world.rigid_bodies;
			for (var li = 0; li < live.length; li++) {
				var lb = live[li];
				if (!lb._drawable) {
					var lm = meshForBody(lb); lm._b = lb; lb._drawable = lm;
					R.meshes.push(lm); R.scene.add(lm); syncMesh(lm, lb);
					anim.meshes.push(lm);
				}
			}
			for (var mi = anim.meshes.length - 1; mi >= 0; mi--) {
				var mb = anim.meshes[mi]._b;
				if (mb && live.indexOf(mb) === -1) {
					R.scene.remove(anim.meshes[mi]);
					var ri = R.meshes.indexOf(anim.meshes[mi]); if (ri !== -1) R.meshes.splice(ri, 1);
					mb._drawable = null;
					anim.meshes.splice(mi, 1);
				}
			}
			anim.meshes.forEach(function (m) { if (m._b) syncMesh(m, m._b); });
			var tl = document.getElementById('tickline'); if (tl) tl.textContent = 'tick ' + anim.tick + ' / ' + anim.totalTicks;
			if ((anim.tick >= anim.totalTicks || (done && canEarlyOut)) && !anim._finished) { anim._finished = true; anim.onFinish(); }
		}
		if (R) { R.controls.update(); R.renderer.render(R.scene, R.camera); }
	}

	function clear() {
		clearScene(); clearConsole();
		var hud = hudEl(); if (hud) hud.style.display = 'none';
		document.getElementById('empty').style.display = '';
		anim.live = false;
	}

	window.GoblinRender = { run: run, clear: clear, resize: resize3D };
})();
