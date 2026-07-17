// Ladder climbing: a fourth movement state alongside grounded/slip/airborne/slide, resolved once per
// beginStep before the main movement dispatch runs (see _updateLadder's call site in Step.js).
var proto = Goblin.FPSCharacterController.prototype;
var FPSC = Goblin.FPSCharacterController.FPSC;

/**
 * Ladder state transitions + climb velocity. A fourth movement state alongside grounded /
 * noTraction / airborne, resolved once per beginStep before that branch runs. The ladder body is
 * never excluded from collision — _collideAndSlide still runs afterward on whatever velocity this
 * writes, so ordinary contact resolution is what holds the character against the face tick over tick.
 *
 * Mount requires movement intent toward the ladder (wishdir), not mere proximity — probing along
 * the current input direction rather than scanning all directions means jumping away from a ladder
 * and holding the opposite key back toward it, or passing a ladder mid-air, can't remount it a
 * frame later while disconnected from it.
 *
 * Forward/back and strafe contributions to climb velocity are summed independently, without
 * normalizing the combined wish vector — holding both diagonally into the face climbs strictly
 * faster than either alone. Look pitch steers climb direction: the forward axis is the full
 * pitched look direction, not flattened, so holding forward while looking down descends.
 *
 * Jump dismounts with a purely horizontal shove away from the face — no vertical component.
 *
 * @method _updateLadder
 * @private
 * @param {Object} cmd
 * @param {Number} moveYaw
 * @param {Number} movePitch
 * @param {Number} dt
 * @return {Boolean} true if this tick's velocity is fully owned by the ladder branch
 */
proto._updateLadder = function(cmd, moveYaw, movePitch, dt) {
    var gb = this.body.linear_velocity;

    var hit;
    if (this._onLadder) {
        var probeDir = new Goblin.Vector3(-this._ladderNormal.x, 0, -this._ladderNormal.z);
        hit = this._findLadderAhead(probeDir);
    } else {
        var fwdH = this.getForwardHorizontal(moveYaw);
        var rgtH = this.getRightHorizontal(moveYaw);
        var cmdF0 = cmd.forward || 0;
        var cmdR0 = cmd.right || 0;
        var wishdir = new Goblin.Vector3(
            fwdH.x * cmdF0 + rgtH.x * cmdR0, 0, fwdH.z * cmdF0 + rgtH.z * cmdR0
        );
        hit = this._findLadderAhead(wishdir);
    }

    if (cmd.jumpPressed && this._onLadder) {
        var n0 = this._ladderNormal;
        gb.x = n0.x * this.ladderDismountPushSpeed;
        gb.z = n0.z * this.ladderDismountPushSpeed;
        gb.y = 0;
        this._onLadder = false;
        // The push flings the character off the ladder into the air — next tick's beginStep
        // dispatch (before endStep gets a chance to re-probe) must see AIRBORNE, not whatever
        // ground state was true before this ladder mount.
        this._moveState = FPSC.MOVE_AIRBORNE;
        this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
        return true;
    }

    if (!hit) {
        this._onLadder = false;
        this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
        return false;
    }

    var hasMoveInput = (cmd.forward || 0) !== 0 || (cmd.right || 0) !== 0;
    if (!this._onLadder && !hasMoveInput) { return false; }

    this._onLadder = true;
    this.grounded = false;
    // Mounting owns movement now — any grounded state carried in from the tick before must not read
    // as still active while climbing (or linger stale after a later dismount): beginStep's dispatch
    // only reads this._moveState when NOT on a ladder, so nothing else would ever clear this.
    this._moveState = FPSC.MOVE_LADDER;
    this._ladderNormal.set(hit.normal.x, 0, hit.normal.z);
    var nl = Math.sqrt(this._ladderNormal.x * this._ladderNormal.x + this._ladderNormal.z * this._ladderNormal.z);
    if (nl > FPSC.EPS_LEN) { this._ladderNormal.x /= nl; this._ladderNormal.z /= nl; }

    this.body.setGravity(0, 0, 0);
    if (!hasMoveInput) { gb.x = 0; gb.y = 0; gb.z = 0; return true; }

    var cp = Math.cos(movePitch);
    var fwd = new Goblin.Vector3(Math.sin(moveYaw) * cp, Math.sin(movePitch), Math.cos(moveYaw) * cp);
    var rgt = this.getRightHorizontal(moveYaw);
    var cmdF = cmd.forward || 0;
    var cmdR = cmd.right || 0;

    var velX = fwd.x * cmdF * this.ladderClimbSpeed + rgt.x * cmdR * this.ladderStrafeSpeed;
    var velY = fwd.y * cmdF * this.ladderClimbSpeed;
    var velZ = fwd.z * cmdF * this.ladderClimbSpeed + rgt.z * cmdR * this.ladderStrafeSpeed;

    var n = this._ladderNormal;
    var out = velX * n.x + velZ * n.z;
    gb.x = velX - out * n.x;
    gb.z = velZ - out * n.z;
    gb.y = velY - out;

    // Descent is blocked against solid ground here (rather than in endStep's ground clamp, which is
    // skipped while mounted so it doesn't re-snap the character onto the floor near the ladder's base
    // even while climbing up). Uses the same _probeGroundCandidates primitive endStep itself uses.
    if (gb.y < 0) {
        var half = this.height / 2;
        var reach2 = -gb.y * dt + this._skin;
        var ground = this._probeGroundCandidates(reach2)[0];
        if (ground) {
            var feetGap = this.body.position.y - half - ground.point.y;
            if (feetGap <= reach2) {
                var clampedY = ground.point.y + half;
                if (!this._resimulating) { this._viewDisplacementY += clampedY - this.body.position.y; }
                this.body.position.set(this.body.position.x, clampedY, this.body.position.z);
                this.body.updateDerived();
                gb.y = 0;
                this.grounded = true;
                // Still LADDER for as long as _onLadder stays true this tick (movement is fully
                // owned above) — this only matters for the tick AFTER dismounting, so beginStep's
                // dispatch sees WALK rather than a stale pre-mount state.
                this._moveState = FPSC.MOVE_WALK;
                this.groundNormal.set(ground.normal.x, ground.normal.y, ground.normal.z);
            }
        }
    }
    return true;
};
