// Mantle: a two-phase kinematic arc that pulls the character up and over a ledge it can't step onto
// but can reach by grabbing. Runs as a pre-dispatch hook in beginStep (exactly like _updateLadder) —
// returns true while owning the tick so normal movement is suppressed.
//
// PHASES
//   lift  — character rises vertically against the ledge face until feet clear the ledge top.
//   vault — character advances forward (into the surface) to land on top; then endStep's normal
//           ground clamp snaps it cleanly onto the surface and transitions to MOVE_WALK.
//
// DETECTION (fires only on cmd.mantle tap; probe direction is the look yaw)
//   1. Cast a ray forward at HEAD height to find a near-vertical ledge face.
//   2. Cast a ray straight DOWN from just above the hit point to find the ledge top surface.
//   3. Accept if:
//      - The top is floor-like (normal.y >= NY_FLOORLIKE).
//      - The top surface is above the step-up limit (otherwise step-up handles it).
//      - The rise from current feet to the ledge top is <= mantleHeight (reachable by arms).
//      - A ledge above standHeight/2 requires the character to already be airborne (jumped) —
//        can't grab-and-pull onto something that tall from a flat-footed standstill.
//      - There is enough headroom to stand at the ledge top (_ceilingClearanceAt).
//   No move-input or static-geometry restriction — player decides what to grab.
//
// ANIMATION
//   The arc duration is fixed at mantleDuration seconds, split liftFrac / (1-liftFrac).
//   During lift: gravity is zeroed, gb.y is driven upward to bring feet to ledge-top height.
//   During vault: gravity stays zeroed, gb.x/z drive the character forward onto the surface.
//   On completion: gravity is restored, _moveState → MOVE_AIRBORNE; endStep's ground clamp
//   lands it correctly the same tick or next.

var proto = Goblin.FPSCharacterController.prototype;
var FPSC = Goblin.FPSCharacterController.FPSC;
var raycast = Goblin.FPSCharacterController._raycast;

/**
 * Probe forward at head height and, on hitting a near-vertical surface, probe downward from just
 * above the hit to find the ledge top. Returns { faceNormal, topPoint, topNormal } or null.
 *
 * @method _probeLedgeAhead
 * @private
 * @param {Number} dx  - horizontal wish direction x (need not be unit)
 * @param {Number} dz  - horizontal wish direction z
 * @return {Object|null}
 */
proto._probeLedgeAhead = function(dx, dz) {
    var dlen = Math.sqrt(dx * dx + dz * dz);
    if (dlen < FPSC.EPS_LEN) { return null; }
    dx /= dlen; dz /= dlen;

    var p = this.body.position;
    var feetY = p.y - this.height / 2;
    var headY = p.y + this.height / 2;

    // Horizontal reach: half-width + a small extra so the ray hits geometry we're touching, not
    // our own collider skin. Probe at several heights spanning the grabbable window so a ledge is
    // found whether the character is level with its face, jumping above it, or approaching from below.
    var reach = this.width / 2 + this.mantleReach;
    var probeHeights = [
        feetY + this.mantleHeight, // top of the reachable grab window (catches ledges when airborne above them)
        p.y,                       // body centre
        feetY + this.stepHeight    // just above step-up floor, catches low ledge faces
    ];

    for (var hi = 0; hi < probeHeights.length; hi++) {
        var probeY = probeHeights[hi];
        var fwdHit = raycast(
            this.world,
            new Goblin.Vector3(p.x, probeY, p.z),
            new Goblin.Vector3(p.x + dx * reach, probeY, p.z + dz * reach),
            this._ignoreSelf
        );
        if (!fwdHit) { continue; }
        // Must be a near-vertical surface (a ledge face, not the floor or a ceiling).
        if (Math.abs(fwdHit.normal.y) > FPSC.NY_GROUNDISH) { continue; }

        // Down-probe from above the hit face to find the ledge's top surface. Scans the full
        // reachable window (feetY+mantleHeight down to feetY+stepHeight). A single nearest-hit
        // raycast isn't enough here: if some OTHER object (e.g. a ceiling slab) sits in this same
        // vertical column above the actual ledge, its underside is the nearest hit and would be
        // misread as "the ledge top" — so this walks every hit along the ray, in order, looking
        // specifically for a floor-like surface belonging to the SAME object we grabbed the face
        // from (fwdHit.object). Anything belonging to a different object is a separate, possibly
        // disconnected surface and is skipped rather than accepted.
        var topProbeX = fwdHit.point.x + dx * this._skin;
        var topProbeZ = fwdHit.point.z + dz * this._skin;
        var scanTop = feetY + this.mantleHeight + this._skin;
        var scanBot = feetY + this.stepHeight + this._skin;
        if (scanTop <= scanBot) { continue; }

        var downHits = this.world.rayIntersect(
            new Goblin.Vector3(topProbeX, scanTop, topProbeZ),
            new Goblin.Vector3(topProbeX, scanBot, topProbeZ)
        );
        var downHit = null;
        if (downHits) {
            for (var di = 0; di < downHits.length; di++) {
                var dh = downHits[di];
                if (dh.object === fwdHit.object && dh.normal.y >= FPSC.NY_FLOORLIKE) { downHit = dh; break; }
            }
        }
        if (!downHit) { continue; }

        return {
            faceNormal: fwdHit.normal,
            topPoint: downHit.point,
            topNormal: downHit.normal,
            probeDx: dx,
            probeDz: dz
        };
    }
    return null;
};

/**
 * Mantle state machine — mirrors _updateLadder's contract exactly. Called once per beginStep,
 * before the main dispatch. Returns true while the mantle arc is active (owns the tick).
 *
 * @method _updateMantle
 * @private
 * @param {Object} cmd
 * @param {Number} moveYaw
 * @param {Number} dt
 * @return {Boolean} true if this tick's velocity is fully owned by the mantle
 */
proto._updateMantle = function(cmd, moveYaw, dt) {
    var gb = this.body.linear_velocity;
    var p = this.body.position;

    // ---- Active arc ----
    // The arc drives the body's POSITION directly (like the ladder's descent-block clamp), not
    // velocity through _collideAndSlide — the whole point of a mantle is to cross a surface
    // (the ledge face) that collide-and-slide correctly treats as a blocking wall for ordinary
    // movement. Interpolating position from the stored start/end anchors sidesteps that fight.
    if (this._mantleActive) {
        this._mantleTimer += dt;
        var total = this.mantleDuration;
        var liftEnd = total * this.mantleLiftFrac;
        var frac = Math.min(1, this._mantleTimer / total);

        var newX, newY, newZ;
        if (this._mantleTimer <= liftEnd) {
            // LIFT phase: rise straight up in place (XZ held at the grab point) until feet clear
            // the ledge top height.
            var liftFrac = Math.min(1, this._mantleTimer / Math.max(liftEnd, FPSC.EPS_LEN));
            newX = this._mantleStartX;
            newZ = this._mantleStartZ;
            newY = this._mantleStartY + (this._mantleTopBodyY - this._mantleStartY) * liftFrac;
        } else {
            // VAULT phase: body is already at ledge-top height (held from the end of lift);
            // advance XZ from the grab point to the stored landing point (past the ledge edge,
            // fully over the standable surface — see the landing-point derivation below).
            var vaultFrac = Math.min(1, (this._mantleTimer - liftEnd) / Math.max(total - liftEnd, FPSC.EPS_LEN));
            newX = this._mantleStartX + (this._mantleLandX - this._mantleStartX) * vaultFrac;
            newZ = this._mantleStartZ + (this._mantleLandZ - this._mantleStartZ) * vaultFrac;
            newY = this._mantleTopBodyY;
        }

        gb.x = 0; gb.y = 0; gb.z = 0;
        this.body.position.set(newX, newY, newZ);
        this.body.updateDerived();

        var done = frac >= 1;
        if (done) {
            this._mantleActive = false;
            this._mantleTimer = 0;
            this._moveState = FPSC.MOVE_AIRBORNE;
            // Restore gravity — let the solver's ground contact + endStep's clamp handle landing.
            this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
            gb.x = 0; gb.y = 0; gb.z = 0;
            return false; // release back to normal dispatch this same tick so endStep can land us
        }

        return true; // still active
    }

    // ---- Detection ----
    // Only probe on an explicit mantle tap. The look direction (yaw) steers the probe; no move
    // input required — the player may be stationary against a surface when they tap.
    if (!cmd.mantle) { return false; }

    var fwd = this.getForwardHorizontal(moveYaw);
    var ledge = this._probeLedgeAhead(fwd.x, fwd.z);
    if (!ledge) { return false; }

    var feetY = p.y - this.height / 2;
    var rise = ledge.topPoint.y - feetY;

    // Below step-up threshold: step-up handles it.
    if (rise <= this.stepHeight + this._skin) { return false; }
    // Above mantle reach: can't grab it.
    if (rise > this.mantleHeight) { return false; }
    // A ledge above chest height (~77% of standing height) needs a running jump first — you can't
    // reach up and pull yourself onto something that tall from a flat-footed standstill. Below this
    // line, a grounded tap is enough (a low grab-and-pull); above it, the tap must land airborne.
    var chestHeight = this.standHeight * FPSC.MANTLE_CHEST_HEIGHT_FRAC;
    if (rise > chestHeight && this.grounded) { return false; }

    // Landing point: advance from directly-above-the-grab-point forward past the face by the
    // character's own depth (enough for the whole footprint to clear the edge, not just the
    // leading point), then verify THAT spot is actually standable — walking the landing point
    // back toward the grab point in steps if the far probe comes up short (a shallow ledge, e.g.
    // a windowsill, doesn't have a full depth of standing room past the lip).
    var dx = ledge.probeDx, dz = ledge.probeDz;
    var topBodyY = ledge.topPoint.y + this.height / 2;
    var desiredAdvance = this.depth; // clear the whole footprint past the face, not just the nose
    var landX = p.x, landZ = p.z, landFound = false;
    var steps = 4;
    for (var s = steps; s >= 1; s--) {
        var advance = desiredAdvance * (s / steps);
        var tryX = ledge.topPoint.x + dx * advance;
        var tryZ = ledge.topPoint.z + dz * advance;
        // Confirm solid ground under this candidate landing point at ledge-top height.
        var landHit = raycast(
            this.world,
            new Goblin.Vector3(tryX, topBodyY, tryZ),
            new Goblin.Vector3(tryX, ledge.topPoint.y - this._skin - this.mantleHeight, tryZ),
            this._ignoreSelf
        );
        if (landHit && landHit.normal.y >= FPSC.NY_FLOORLIKE &&
            Math.abs(landHit.point.y - ledge.topPoint.y) < this.stepHeight + this._skin) {
            landX = tryX; landZ = tryZ; landFound = true;
            break;
        }
    }
    if (!landFound) { return false; } // no safe footing past the edge — refuse the mantle

    // Headroom check: the arc drives the body's POSITION directly (bypassing collide-and-slide,
    // see the active-arc branch above), so nothing else will catch a low ceiling along the way —
    // this is the only check standing between a clean mantle and clipping straight through
    // overhead geometry. Verify standing clearance at BOTH the grab point (where the lift phase
    // rises to) and the landing point (where the vault phase ends), not just one — a ledge can
    // have room to grab but not to stand at, or vice versa.
    var clearanceAtGrab = this._ceilingClearanceAt(p.x, p.z, ledge.topPoint.y);
    if (clearanceAtGrab < this.standHeight - this._skin) { return false; }
    var clearanceAtLand = this._ceilingClearanceAt(landX, landZ, ledge.topPoint.y);
    if (clearanceAtLand < this.standHeight - this._skin) { return false; }

    // Commit to the mantle.
    this._mantleActive = true;
    this._mantleTimer = 0;
    this._mantleStartX = p.x;
    this._mantleStartY = p.y;
    this._mantleStartZ = p.z;
    this._mantleTopBodyY = topBodyY;
    this._mantleLandX = landX;
    this._mantleLandZ = landZ;

    this._moveState = FPSC.MOVE_MANTLE;
    this.grounded = false;
    this._groundSuppress = FPSC.GROUND_SUPPRESS_JUMP;
    this._jumpRising = true;
    // Zero gravity for the arc duration — restored on completion above.
    this.body.setGravity(0, 0, 0);
    gb.x = 0; gb.y = 0; gb.z = 0;

    return true;
};
