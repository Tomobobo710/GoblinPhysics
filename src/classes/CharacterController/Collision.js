// Kinematic wall/step collision: the character is excluded from the solver's own contact resolution
// (collision_mask 1), so this file is what actually stops the character at walls, lets it climb steps,
// and depenetrates it out of geometry it sank into. Shared by both the character body and its ghost
// via _sweptCollideAndSlide.
var proto = Goblin.FPSCharacterController.prototype;
var FPSC = Goblin.FPSCharacterController.FPSC;

/**
 * Kinematic collide-and-slide. The character is excluded from the solver, so we stop
 * ourselves at walls and slide along them here. For the current horizontal velocity
 * we cast a fan of rays (across the footprint width, at a few heights) in the move
 * direction; if a vertical wall is within the box's reach this step we remove the
 * into-wall velocity component and re-test, so corners stop on both walls. Floors and
 * ramps (normal.y >= 0.5) are ignored — those are handled by the ground clamp.
 *
 * @method _collideAndSlide
 * @private
 * @param {Number} vx - incoming horizontal velocity, x.
 * @param {Number} vz - incoming horizontal velocity, z.
 * @param {Number} dt
 * @return {Object} result
 * @return {Number} result.x - clipped horizontal velocity, x.
 * @return {Number} result.z - clipped horizontal velocity, z.
 */
proto._collideAndSlide = function(vx, vz, dt) {
    var res = this._sweptCollideAndSlide({
        position: this.body.position,
        width: this.width, depth: this.depth, height: this.height,
        skin: this._skin, mass: this.mass, stepHeight: this.stepHeight,
        selfBody: this.body, otherSelfBody: this._ghost || null,
        // A SLIDE is exempt from the too-steep-can't-move-up block (the slide IS the climb — momentum,
        // not input, is what carries it up). This must hold while AIRBORNE-sliding too: an airborne
        // slide sweeping into a steep face otherwise wall-clips to zero speed mid-air, which kills the
        // slide before it ever lands on the surface. _climbableSlopeAhead inside still tells real
        // slopes from vertical walls, so walls keep stopping a slide. this._moveState is already
        // MOVE_SLIDE on the true first-contact tick too — endStep decides movement state (including
        // slide entry) BEFORE this function runs later in the same beginStep, so there's no
        // "one tick behind" gap here to patch around.
        climbSteepSlopes: this.climbSteepSlopes || this._moveState === FPSC.MOVE_SLIDE,
        vx: vx, vz: vz, dt: dt,
    });
    // Depenetration is a horizontal position correction out of a wall, separate from the velocity move.
    if (res.depenX !== 0 || res.depenZ !== 0) {
        var bp = this.body.position;
        this.body.position.set(bp.x + res.depenX, bp.y, bp.z + res.depenZ);
        this.body.updateDerived();
    }
    return { x: res.x, z: res.z };
};

/**
 * Sweeps an inset box along a horizontal velocity and clips it against blocking contacts,
 * sub-stepped so long sweeps can't return a wrong-axis normal. Shared by the character body
 * and its ghost so both get identical wall/mass-yield behavior from one implementation.
 *
 * @method _sweptCollideAndSlide
 * @private
 * @param {Object} opts
 * @param {Goblin.Vector3} opts.position - Sweep origin (box center).
 * @param {Number} opts.width - Box width (x), pre-inset.
 * @param {Number} opts.depth - Box depth (z), pre-inset.
 * @param {Number} opts.height - Box height (y), pre-inset.
 * @param {Number} opts.skin - Contact/sweep tolerance subtracted from each half-extent.
 * @param {Number} opts.mass - Sweeping body's mass, used for the push mass-yield ratio.
 * @param {Number} [opts.stepHeight=0] - Step-up height; 0 disables step-up entirely.
 * @param {Goblin.RigidBody} opts.selfBody - Body to exclude from its own sweep hits.
 * @param {Goblin.RigidBody} [opts.otherSelfBody] - A second body to exclude (e.g. the character
 *   excludes its ghost, and vice versa).
 * @param {Boolean} [opts.climbSteepSlopes=false] - Exempt too-steep floor-like faces that have a
 *   climbable slope ahead from the wall-block rule.
 * @param {Number} opts.vx - Incoming horizontal velocity, x.
 * @param {Number} opts.vz - Incoming horizontal velocity, z.
 * @param {Number} opts.dt - Tick duration in seconds.
 * @return {Object} result
 * @return {Number} result.x - Clipped horizontal velocity, x.
 * @return {Number} result.z - Clipped horizontal velocity, z.
 * @return {Number} result.depenX - Position correction out of a penetrated wall, x (0 if none).
 * @return {Number} result.depenZ - Position correction out of a penetrated wall, z (0 if none).
 */
proto._sweptCollideAndSlide = function(opts) {
    var vx = opts.vx, vz = opts.vz;
    var position = opts.position, width = opts.width, depth = opts.depth, height = opts.height,
        skin = opts.skin, mass = opts.mass, dt = opts.dt, selfBody = opts.selfBody, otherSelfBody = opts.otherSelfBody;
    var stepHeight = opts.stepHeight || 0;
    var climbSteepSlopes = !!opts.climbSteepSlopes;
    var world = this.world;
    if (!world || typeof world.shapeIntersect !== "function") { return { x: vx, z: vz, depenX: 0, depenZ: 0 }; }

    // Original move heading, before any clipping this tick — used by the climb-slope-ahead probe
    // so a mid-loop velocity clip doesn't collapse the probe direction.
    var moveLen0 = Math.sqrt(vx * vx + vz * vz);
    var mdx0 = moveLen0 > FPSC.EPS_DIR ? vx / moveLen0 : 0;
    var mdz0 = moveLen0 > FPSC.EPS_DIR ? vz / moveLen0 : 0;

    // Swept-box collide-and-slide: sweep an inset box along the move each tick and clip velocity
    // against the real contact plane.
    var p = position;
    var halfW = width / 2 - skin;
    var halfD = depth / 2 - skin;
    // Lift the swept box a small amount off the feet so it doesn't graze the floor slab's top
    // edge (which returns a degenerate near-vertical normal and fakes a wall), while staying low
    // enough to still catch a steep ramp's toe.
    var lift = skin * 2;
    var halfH = Math.max(0.05, height / 2 - lift / 2);
    var yOffset = lift / 2;
    // Cache the swept probe box per caller (different callers may have different dimensions).
    var cacheKey = selfBody === this.body ? "_sweepBox" : "_altSweepBox";
    if (!this[cacheKey] || this[cacheKey + "W"] !== halfW || this[cacheKey + "H"] !== halfH || this[cacheKey + "D"] !== halfD) {
        this[cacheKey] = new Goblin.BoxShape(halfW, halfH, halfD);
        this[cacheKey + "W"] = halfW; this[cacheKey + "H"] = halfH; this[cacheKey + "D"] = halfD;
    }
    var boxShape = this[cacheKey];
    var minStandableNy = this._minStandableNormalY;

    // Sub-step so each swept chunk stays well under the smallest half-extent (a long sweep can
    // return a wrong-axis normal from EPA).
    var chunkLen = Math.min(halfW, halfD) * FPSC.SUBSTEP_FRAC;
    var full = Math.sqrt(vx * vx + vz * vz) * dt;
    var nSub = Math.max(1, Math.ceil(full / Math.max(chunkLen, FPSC.EPS_LEN)));
    var sdt = dt / nSub;

    // shapeIntersect's contact normal points FROM the swept box TOWARD the other object (into the
    // wall). "Heading into this face" is v.n > 0; pushing out of penetration moves along -n.
    //
    // Nearest valid blocking contact for a sweep, or null. { n, pen, keep }.
    var self_ = this;
    function findBlock(start, end) {
        var hits = world.shapeIntersect(boxShape, start, end);
        for (var hi = 0; hi < hits.length; hi++) {
            var h = hits[hi];
            var b0 = h.object;
            if (b0 === selfBody || b0 === otherSelfBody || (b0 && b0.isKinematicCharacter)) { continue; }
            var n = h.normal;
            if (!n || !isFinite(n.x) || !isFinite(n.y) || !isFinite(n.z)) { continue; }
            var nlen = Math.sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
            if (nlen < FPSC.N_DEGENERATE) { continue; }
            if (Math.abs(n.y) >= minStandableNy) { continue; }
            // Vertical wall: normal horizontal, points character->object; heading in is v.n > 0.
            // Too-steep floor-like face (0.1 < n.y < cutoff): normal tilts up-and-back, so heading
            // in is v.(n.x,n.z) < 0 — sign flipped below.
            var floorLike = n.y > FPSC.NY_FLOORLIKE;
            // A floor-like too-steep face is only a legitimate "slope ahead" block near the feet
            // (walking into a ramp's toe). The same face type contacted up near head height is an
            // OVERHANG (ramp underside above a wedged character), not a slope to stop forward
            // progress on — treating it as a wall-slide clip can zero velocity in every direction,
            // including retreat, trapping the character. Overhead clearance is the headroom gate's
            // job; skip it here so a sideways/backward escape isn't blocked by the same contact.
            if (floorLike && h.point && (h.point.y - (p.y - height / 2)) > height * FPSC.TOE_BAND_FRAC) { continue; }
            if (climbSteepSlopes && self_._climbableSlopeAhead(start, mdx0, mdz0)) { continue; }
            var into = floorLike ? -(vx * n.x + vz * n.z) : (vx * n.x + vz * n.z);
            if (into <= 0) { continue; }
            var keep = 0;
            var b = h.object;
            // Platforms never yield like a pushable object — they're scripted geometry.
            if (b && !b.isPlatform && b._mass !== Infinity && b._mass > 0 && isFinite(b._mass) &&
                b._mass <= self_._pushMassLimit && !b.isKinematicCharacter) {
                keep = mass / (mass + b._mass);
            }
            return { n: n, pen: h.penetration || 0, keep: keep };
        }
        return null;
    }

    // Contact test with no directional gate (unlike findBlock). Used by the recovery back-probe,
    // since after velocity is clipped the body is no longer "moving into" the wall.
    function contactAt(x, y, z) {
        var hits = world.shapeIntersect(boxShape, new Goblin.Vector3(x, y, z), new Goblin.Vector3(x, y, z));
        for (var hi = 0; hi < hits.length; hi++) {
            var h = hits[hi];
            var b0 = h.object;
            if (b0 === selfBody || b0 === otherSelfBody || (b0 && b0.isKinematicCharacter)) { continue; }
            var n = h.normal;
            if (!n || !isFinite(n.x) || !isFinite(n.y) || !isFinite(n.z)) { continue; }
            if (Math.sqrt(n.x * n.x + n.y * n.y + n.z * n.z) < FPSC.N_DEGENERATE) { continue; }
            if (Math.abs(n.y) >= minStandableNy) { continue; } // walkable ground/ramp — not a wall
            return true;
        }
        return false;
    }

    var sy = p.y + yOffset;

    // CLIP velocity against walls the move would hit this tick, and DEPENETRATE out of any wall
    // sunk into (push along -n by overlap+skin so it rests just clear — the swept cast is
    // penetration-based, so without this a fast move ends up inside and sticks). Velocity-only for
    // the move; position correction only for depenetration. Sub-stepped for reliable normals.
    var cx = p.x, cz = p.z;
    var depenX = 0, depenZ = 0;
    for (var s = 0; s < nSub; s++) {
        for (var iter = 0; iter < 4; iter++) {
            var speed = Math.sqrt(vx * vx + vz * vz);
            if (speed < FPSC.EPS_DIR) { break; }
            var start = new Goblin.Vector3(cx, sy, cz);
            var end = new Goblin.Vector3(cx + vx * sdt, sy, cz + vz * sdt);
            var blk = findBlock(start, end);
            if (!blk) { break; }
            // Step-up: before walling a near-vertical, non-yielding face, test if it's clear when
            // swept raised by stepHeight — if so it's steppable, let the move through.
            if (blk.keep < FPSC.KEEP_BLOCKED && Math.abs(blk.n.y) < FPSC.NY_NEAR_VERTICAL && stepHeight > 0) {
                var upStart = new Goblin.Vector3(cx, sy + stepHeight, cz);
                var upEnd = new Goblin.Vector3(cx + vx * sdt, sy + stepHeight, cz + vz * sdt);
                if (!findBlock(upStart, upEnd)) { break; }
            }
            var n = blk.n, keep = blk.keep;
            // Clip the into-face velocity using the horizontal blocking direction (never inject
            // vertical; the ground clamp owns y).
            var floorLike = n.y > FPSC.NY_FLOORLIKE;
            var bx = floorLike ? -n.x : n.x, bz = floorLike ? -n.z : n.z;
            var blen = Math.sqrt(bx * bx + bz * bz);
            if (blen < FPSC.EPS_SPD) { break; }
            bx /= blen; bz /= blen;
            var dot = vx * bx + vz * bz;
            if (dot > 0) {
                vx -= dot * bx * (1 - keep);
                vz -= dot * bz * (1 - keep);
            }
            // Depenetration is recovery-only: detect BURIED vs GRAZING with a back-probe (sweep one
            // fixed small step along -n; if still in contact there, nudge out by that step), rather
            // than trusting the reported penetration depth directly. Vertical walls only; a
            // floor-like too-steep toe is owned by the clamp / steep-slope path.
            if (!floorLike && keep < FPSC.KEEP_BLOCKED && blk.pen > 0) {
                var step = Math.min(width, depth) * FPSC.BACKPROBE_WIDTH_FRAC;
                if (contactAt(cx - n.x * step, sy, cz - n.z * step)) {
                    depenX -= n.x * step; depenZ -= n.z * step;
                    cx -= n.x * step; cz -= n.z * step;
                }
            }
            if (keep > FPSC.KEEP_BLOCKED) { break; }
        }
        cx += vx * sdt;
        cz += vz * sdt;
    }
    return { x: vx, z: vz, depenX: depenX, depenZ: depenZ };
};
