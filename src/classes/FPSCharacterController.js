/**
 * Engine-agnostic, reusable first-person character controller built directly on Goblin
 * (NOT `Goblin.CharacterController` — that's a separate, spring-based capsule controller; this
 * one is a kinematic box mover with its own ground/wall/slope/ghost handling). Uses a BOX
 * collider that is angular-locked so it can never tip. Grounding, slopes, walls and resting are
 * handled by hand-written raycast/sweep probes each tick, not by the physics solver — the
 * controller does NOT hard-teleport the body to the ground every frame (that fights the solver
 * and jitters). It only:
 *   - sets HORIZONTAL velocity from input each step (snappy, no momentum fighting),
 *   - projects that velocity along the ground plane (no sliding on slopes) and off walls
 *     (smooth move-and-slide, so we never ram the solver), and
 *   - applies targeted raycast assists for STEP-UP and STEP-DOWN, which the solver can't
 *     do with a box collider.
 * Vertical motion (gravity, landing) is left to the solver; only jump / jetpack thrust write
 * the vertical velocity directly.
 *
 * Also handles two further movement states parallel to ground/air: climbing a body tagged
 * isLadder (see _updateLadder), and riding a body tagged isPlatform via base-velocity inheritance
 * (see _baseVelocity in the constructor, and beginStep/endStep/_updateVertical) — jumping off a
 * rising platform adds its velocity into the jump.
 *
 * DESIGN SEAMS:
 *   The controller never reads input directly. Gameplay samples an input command (pure data, so
 *   any caller can run remote characters' commands through the exact same path) and feeds it in,
 *   bracketing a single Goblin world step:
 *       const cmd = mySampleInput(input);       // input mapping is policy, lives outside this class
 *       controller.beginStep(cmd, dt);           // pre-physics: velocity + assists
 *       world.step(dt);                          // ONE world step (all bodies)
 *       controller.endStep(dt);                  // post-physics: grounded + step-down
 *
 * EXTENSIBILITY:
 *   This base IS the default "kit" (instantiate it directly). A game adds an alternate kit by
 *   subclassing and overriding `_updateVertical` (jump/gravity) and/or `_getMoveSpeed` without
 *   touching ground/step/wall logic.
 *
 * Units: METERS (gravity -9.81 by default); defaults are in meters (a ~1.8m human ≈ 1.8 units
 * tall). Use `scale` to resize the whole character.
 *
 * @class FPSCharacterController
 * @constructor
 * @param {Goblin.World} world - The physics world this controller's body/ghost live in.
 * @param {Object} [options] - See FPS_CONTROLLER_DEFAULTS (FPSControllerConstants.js) for every
 *   tunable default and its meaning; each `options.X` below overrides that default per-instance.
 * @param {Goblin.Vector3} [options.position] - Spawn position (body center). Default (0,20,0).
 * @param {Number} [options.scale=1] - Uniform size multiplier for the whole character.
 * @param {Number} [options.width] - Collider width (x) before scale.
 * @param {Number} [options.depth] - Collider depth (z) before scale.
 * @param {Number} [options.height] - Collider height (y) before scale.
 * @param {Number} [options.mass] - Body mass before scale.
 * @param {Number} [options.eyeHeight] - Eye offset above body CENTER before scale.
 * @param {Number} [options.walkSpeed] - Held-walk gait speed before scale (slower than run).
 * @param {Number} [options.moveSpeed] - RUN speed (the default no-modifier gait) before scale.
 * @param {Number} [options.sprintSpeed] - Sprint move speed before scale.
 * @param {Number} [options.crouchSpeedMult] - Multiplier on the active gait while crouched.
 * @param {Number} [options.sprintDecay] - Rate (units/sec) the sprint boost fades after release.
 * @param {Number} [options.groundStopDecel] - Deceleration (units/sec) on releasing all move keys.
 * @param {Number} [options.airControl] - 0..1 horizontal steering authority per step while airborne.
 * @param {Number} [options.jumpSpeed] - Jump velocity before scale.
 * @param {Number} [options.friction] - Body friction (0 keeps wall-slides clean; kinematic
 *   grounding holds slopes without relying on solver friction).
 * @param {Number} [options.stepHeight] - Max step-UP height before scale.
 * @param {Number} [options.stepDownDist] - Max step-DOWN snap before scale.
 * @param {Number} [options.coyoteTime] - Seconds after leaving a ledge you can still jump (0=off).
 * @param {Number} [options.jumpBuffer] - Seconds before landing a jump press is remembered (0=off).
 * @param {Boolean} [options.slideEnabled=true] - Enable crouch-at-speed sliding.
 * @param {Boolean} [options.slideRequiresMoveInput=true] - Require a movement key held to START a slide (exit never requires it).
 * @param {Boolean} [options.slideAllowLandingWithoutInput=true] - Waive the movement-key requirement on the landing frame, so an impact-slide can start from crouch + speed alone.
 * @param {Number} [options.slideMinSpeed] - Min along-ground speed (pre-scale) to start a slide.
 * @param {Number} [options.slideEndSpeed] - Flat slide ends below this speed (pre-scale).
 * @param {Number} [options.slideFriction] - Speed bled per second on flat ground (pre-scale).
 * @param {Number} [options.slideBoost] - Launch speed multiplier at slide entry.
 * @param {Number} [options.slideControl] - 0..1 carve authority while sliding (speed-preserving).
 * @param {Number} [options.slideSlopeAccel] - Gravity-along-slope multiplier while sliding.
 * @param {Number} [options.slideSlopeMin] - Min slope (sin of angle) that sustains a slide via gravity.
 * @param {Number} [options.slideSlopeFriction] - Cross-slope bleed per second on a sustaining slope.
 * @param {Number} [options.slideReversalBrakeMult] - Multiplier on slideSlopeFriction for how hard a
 *   deliberate on-slope reversal (wish opposing current slide direction) brakes before the carve
 *   steering picks the new heading back up.
 * @param {Boolean} [options.receivePush=true] - Enable object-to-character knockback via the ghost body.
 * @param {Number} [options.receiveMaxSpeed] - Cap on how fast a single object hit can knock the character.
 * @param {Number} [options.receiveKnockbackFraction] - Fraction of the ghost's contact velocity transferred.
 * @param {Number} [options.ghostMaxSpeed] - Cap on the ghost's follow/shove speed (units/sec).
 * @param {Number} [options.ghostDamping] - Fraction of the ghost's current velocity damped each tick (0..1).
 * @param {Number} [options.maxSlopeAngle] - Max standable slope in degrees (90+ disables the limit).
 * @param {Boolean} [options.visible=false] - Whether a consumer should treat the collider as drawable
 *   (this controller does no rendering itself — see `object.isVisible`).
 * @param {String} [options.color] - Cosmetic color tag, opaque to this class.
 * @param {Number} [options.skin] - Contact/sweep tolerance override (see FPSC.SKIN).
 */
Goblin.FPSCharacterController = (function() {

// Internal algorithm constants — the thresholds/epsilons/factors baked into the controller's collision,
// grounding, slope and ghost math. These are NOT caller-facing feel knobs (those live in
// FPS_CONTROLLER_DEFAULTS); they are implementation tolerances kept named here so nothing is a bare literal
// at a use site. Changing them changes solver behavior — treat as internals, not tuning.
var FPSC = {
    // Contact tolerances (metres, multiplied by the character scale where used).
    SKIN: 0.01,               // sweep/contact skin width
    GROUND_TOL: 0.1,          // how close the feet must be to a surface to count as grounded
    GHOST_GROUND_INSET: 0.25, // fraction of height the ghost's bottom is lifted above the feet

    // "Effectively zero" epsilons — vector/speed magnitudes below which we treat a quantity as null.
    EPS_LEN: 1e-4,            // general length/normal guard
    EPS_DIR: 1e-5,            // direction-normalisation guard
    EPS_SPD: 1e-6,            // speed-normalisation guard
    EPS_INPUT2: 1e-10,        // squared move-input threshold (has-input test)
    EPS_SPEED_MARGIN: 1e-3,   // speed must exceed a target by this to count as "above" it

    // Ground-normal.y classifiers (a surface normal's up-component; 1 = flat floor, 0 = vertical wall).
    NY_CEILING: -0.4,         // normal.y ABOVE this is not a ceiling (must face downward to be one)
    NY_STEEP_MIN: 0.3,        // steep-but-not-wall floor-like face lower bound
    NY_GROUNDISH: 0.5,        // normal.y at/above this is walkable-ish ground (skip as a wall/step)
    NY_FLOORLIKE: 0.1,        // normal.y above this tilts up (floor-like), below is a vertical wall
    N_DEGENERATE: 0.5,        // reject a contact normal whose length is below this (bad EPA result)
    TOE_BAND_FRAC: 0.6,       // a too-steep floor-like contact only blocks as a slope-toe within this
                              // fraction of body height above the feet; higher is an overhang (headroom
                              // gate's job), not a wall to clip horizontal velocity against

    // Slide reversal (see _updateSlide's onSlope steering). Below this dot product between wish and
    // current slide direction, wish counts as a deliberate reversal (brake) rather than a carve.
    SLIDE_REVERSAL_DOT: -0.5,

    // MOVEMENT STATE — one flat enum, mutually exclusive, decided ONCE per tick by endStep (the only
    // place with a fresh ground probe) and read everywhere else (beginStep dispatches on it verbatim;
    // nothing re-derives it from other flags). See the "Movement state machine" comment above endStep
    // for the full design and why it replaced the old grounded+sliding+wishSlide flag soup.
    //   LADDER   = mounted on a ladder; _updateLadder owns velocity fully.
    //   AIRBORNE = no ground contact; gravity + air control own velocity.
    //   WALK     = grounded, standable surface, not sliding: ordinary input-driven movement.
    //   SLIP     = grounded, too-steep surface, not sliding: gravity-fed slip, weak air-control.
    //   SLIDE    = grounded, crouch-at-speed slide: _updateSlide's surface-tracking model owns velocity.
    MOVE_LADDER: 'ladder',
    MOVE_AIRBORNE: 'airborne',
    MOVE_WALK: 'walk',
    MOVE_SLIP: 'slip',
    MOVE_SLIDE: 'slide',

    // Knockback gating (see _readGhostKnockback).
    KB_CLOSING_MIN: 0.5,      // object must close on the character faster than this (units/s) to knock back
    KB_MIN: 0.05,             // ignore a computed knockback smaller than this

    // Ground-suppress frame counts — ticks the ground clamp is held off after an event.
    GROUND_SUPPRESS_KB: 5,    // after taking knockback
    GROUND_SUPPRESS_JUMP: 8,  // after jumping

    // Sweep sub-stepping + wall interaction.
    SUBSTEP_FRAC: 0.5,        // sub-step length as a fraction of the smallest half-extent
    NEAR_CENTER_FRAC: 0.4,    // "hit near my centre" band as a fraction of width
    PUSH_INTO_MIN: 0.5,       // dot(vel, toHit) above this = actively moving into a contact

    // Wall clip / step-up / depenetration.
    KEEP_BLOCKED: 0.01,       // keep-fraction below this = a non-yielding wall (fully blocks / triggers step-up)
    NY_NEAR_VERTICAL: 0.2,    // |normal.y| below this = a near-vertical face (steppable candidate)
    // Depenetration back-probe step, as a fraction of the character's own half-width — independent of
    // skin (skin is a contact/tunneling tolerance, not a "how fast should a buried body recover" rate;
    // coupling the two meant shrinking skin for tunneling reasons silently crippled buried-recovery speed).
    BACKPROBE_WIDTH_FRAC: 0.1,
    // Climbable-slope look-ahead sample points, as multiples of the character's DEPTH past the footprint
    // edge (so the probe reaches the same RELATIVE forward zone at any scale — a fixed-meter reach would
    // under-reach a big character and over-reach a small one, breaking steep-slope walk off default scale).
    CLIMB_PROBE_DEPTH_MULTS: [0, 0.5, 1.0, 1.67],

    // Render.
    VIEW_DISP_SNAP: 0.01,     // pending view-displacement above which eye interpolation snaps (crouch/step)
};

/**
 * Cast a ray from start to end in the physics world, kept private since nothing outside this file
 * needs it. Returns the first valid hit (nearest first, skipping anything named in
 * `ignoreObjects`), or null.
 *
 * @method raycast
 * @private
 * @param {Goblin.World} world
 * @param {Goblin.Vector3} start
 * @param {Goblin.Vector3} end
 * @param {String[]} [ignoreObjects] - body `.name` values to skip
 * @return {Object|null} { object, point:Vector3, normal:Vector3, t:Number } or null
 */
function raycast(world, start, end, ignoreObjects) {
    var hits = world.rayIntersect(start, end);
    if (!hits || hits.length === 0) { return null; }
    for (var i = 0; i < hits.length; i++) {
        var hit = hits[i];
        if (hit.object && hit.object.name && ignoreObjects &&
            ignoreObjects.indexOf(hit.object.name) !== -1) { continue; }
        return hit;
    }
    return null;
}

function FPSCharacterController(world, options) {
    this.world = world;
    var o = options || {};

    var D = Goblin.FPS_CONTROLLER_DEFAULTS;
    var dim = D.dimensions, mv = D.movement, jmp = D.jump, slp = D.slopes, sld = D.slide,
        gh = D.ghost, kb = D.knockback, net = D.netcode, vw = D.view, rnd = D.render, msc = D.misc,
        lad = D.ladder;

    // Base (pre-scale) values.
    this._baseWidth = o.width !== undefined ? o.width : dim.width;
    this._baseDepth = o.depth !== undefined ? o.depth : dim.depth;
    this._baseHeight = o.height !== undefined ? o.height : dim.height;
    this._baseMass = o.mass !== undefined ? o.mass : dim.mass;
    this._baseEyeHeight = o.eyeHeight !== undefined ? o.eyeHeight : this._baseHeight * dim.eyeHeightRatio;

    this._baseWalkSpeed = o.walkSpeed !== undefined ? o.walkSpeed : mv.walkSpeed;
    this._baseMoveSpeed = o.moveSpeed !== undefined ? o.moveSpeed : mv.moveSpeed;
    this._baseSprintSpeed = o.sprintSpeed !== undefined ? o.sprintSpeed : mv.sprintSpeed;
    this.crouchSpeedMult = o.crouchSpeedMult !== undefined ? o.crouchSpeedMult : mv.crouchSpeedMult;
    this._baseSprintDecay = o.sprintDecay !== undefined ? o.sprintDecay : mv.sprintDecay;
    this._baseGroundStopDecel = o.groundStopDecel !== undefined ? o.groundStopDecel : mv.groundStopDecel;
    this._baseJumpSpeed = o.jumpSpeed !== undefined ? o.jumpSpeed : jmp.jumpSpeed;
    this._baseStepHeight = o.stepHeight !== undefined ? o.stepHeight : jmp.stepHeight;
    this._baseStepDownDist = o.stepDownDist !== undefined ? o.stepDownDist : jmp.stepDownDist;
    // Contact/sweep tolerance. A per-instance override (not just FPSC.SKIN) lets a project tune this
    // for a specific character without touching the shared engine default.
    this._baseSkin = o.skin !== undefined ? o.skin : FPSC.SKIN;

    // Object interaction (push and be pushed) runs through the ghost body (see _buildGhost / _readGhostKnockback).
    this._receivePush = o.receivePush !== undefined ? o.receivePush !== false : kb.receivePush;
    // Speed-like (a velocity cap), so it must scale with character size the same way sprintSpeed
    // does — stored as a BASE here and scaled in _applyScale, not a fixed literal, so a 2x
    // character's (faster, harder-hitting) knockback is judged against a 2x cap, not the 1x default.
    this._baseReceiveMaxSpeed = o.receiveMaxSpeed !== undefined ? o.receiveMaxSpeed : kb.maxSpeed;
    this._receiveKnockbackFraction = o.receiveKnockbackFraction !== undefined ? o.receiveKnockbackFraction : kb.knockbackFraction;
    this._receiveSelfPush = o.receiveSelfPush !== undefined ? o.receiveSelfPush === true : kb.selfPush;
    // Ghost follow-drive (see _syncGhost). maxSpeed / maxDampSpeed default to a multiple of sprint
    // speed, and — like sprintSpeed itself — must scale with character size: a 2x character's
    // sprint (and the momentum it can impart to an object) is 2x faster, but a ghost capped at the
    // UNSCALED base speed can't keep pace at that scale, falls behind, and stops being physically
    // present to block a fast-moving object from passing straight through the (solver-invisible)
    // character. Explicit overrides are taken as literal (caller asked for that exact number, not a
    // scale-derived one); the multiplier-derived defaults are re-derived in _applyScale so they
    // track scale both at construction and on any later setScale().
    this._ghostMaxSpeedOverride = o.ghostMaxSpeed;
    this._ghostMaxDampSpeedOverride = o.ghostMaxDampSpeed;
    this._ghostMaxSpeedMult = gh.maxSpeedSprintMult;
    this._ghostMaxDampSpeedMult = gh.maxDampSpeedSprintMult;
    // Ghost body's physics material (not the chase-behavior tuning above) — read once here so
    // _buildGhost (called on every rebuild: crouch, setScale, respawn) doesn't need its own access
    // to Goblin.FPS_CONTROLLER_DEFAULTS.
    this._ghostMaterial = o.ghostMaterial || gh.material;
    this._ghostDamping = o.ghostDamping !== undefined ? o.ghostDamping : gh.damping;
    this._ghostStiffness = o.ghostStiffness !== undefined ? o.ghostStiffness : gh.stiffness;
    this._driveGhostDuringResim = o.driveGhostDuringResim !== undefined ? o.driveGhostDuringResim !== false : net.driveGhostDuringResim;
    this._hardsnapGhostOnReconcile = o.hardsnapGhostOnReconcile !== undefined ? o.hardsnapGhostOnReconcile !== false : net.hardsnapGhostOnReconcile;
    this._pushMassLimitOverride = o.pushMassLimit;
    this._pushMassBaseMult = gh.pushMassBaseMult;

    this.airControl = o.airControl !== undefined ? o.airControl : mv.airControl;
    this.friction = o.friction !== undefined ? o.friction : mv.friction;

    this.coyoteTime = o.coyoteTime !== undefined ? o.coyoteTime : jmp.coyoteTime;
    this.jumpBuffer = o.jumpBuffer !== undefined ? o.jumpBuffer : jmp.jumpBuffer;
    this._coyoteTimer = 0;
    this._jumpBufferTimer = 0;

    // Max standable slope, in degrees. Stored as the cosine (_minStandableNormalY) since that's
    // what the per-tick ground-normal check compares against. 90 (or more) disables the limit.
    this.maxSlopeAngle = o.maxSlopeAngle !== undefined ? o.maxSlopeAngle : slp.maxSlopeAngle;
    this._minStandableNormalY = Math.cos(Math.min(90, this.maxSlopeAngle) * Math.PI / 180);
    this.climbSteepSlopes = o.climbSteepSlopes !== undefined ? o.climbSteepSlopes === true : slp.climbSteepSlopes;

    // Slide (crouch-at-speed). slide* tuning values only take effect once sliding.
    this.slideEnabled = o.slideEnabled !== undefined ? o.slideEnabled !== false : sld.enabled;
    this.slideRequiresMoveInput = o.slideRequiresMoveInput !== undefined ? !!o.slideRequiresMoveInput : sld.requiresMoveInput;
    this.slideAllowLandingWithoutInput = o.slideAllowLandingWithoutInput !== undefined ? !!o.slideAllowLandingWithoutInput : sld.allowLandingWithoutInput;
    this._baseSlideMinSpeed = o.slideMinSpeed !== undefined ? o.slideMinSpeed : sld.minSpeed;
    this._baseSlideEndSpeed = o.slideEndSpeed !== undefined ? o.slideEndSpeed : sld.endSpeed;
    this._baseSlideFriction = o.slideFriction !== undefined ? o.slideFriction : sld.friction;
    this.slideBoost = o.slideBoost !== undefined ? o.slideBoost : sld.boost;
    this.slideControl = o.slideControl !== undefined ? o.slideControl : sld.control;
    this.slideSlopeAccel = o.slideSlopeAccel !== undefined ? o.slideSlopeAccel : sld.slopeAccel;
    this.slideSlopeMin = o.slideSlopeMin !== undefined ? o.slideSlopeMin : sld.slopeMin;
    this._baseSlideSlopeFriction = o.slideSlopeFriction !== undefined ? o.slideSlopeFriction : sld.slopeFriction;
    // Reversal brake rate, as a multiplier on slideSlopeFriction — how hard a deliberate reversal
    // (wish opposing current slide direction, see FPSC.SLIDE_REVERSAL_DOT) bleeds speed before the
    // ordinary carve blend picks the new heading back up.
    this.slideReversalBrakeMult = o.slideReversalBrakeMult !== undefined ? o.slideReversalBrakeMult : sld.reversalBrakeMult;
    // Authoritative movement state — see the "Movement state machine" comment above endStep. Starts
    // AIRBORNE; the first tick's endStep probe corrects it (e.g. to WALK if spawned on the ground).
    this._moveState = FPSC.MOVE_AIRBORNE;
    this._slipJustEntered = false; // gates the SLIP branch's one-time velocity projection; set by endStep
    this._wantCrouch = false; // this tick's crouch intent, stashed by beginStep for endStep to read
    this._hasMoveInput = false; // this tick's movement input, stashed by beginStep for endStep to read
    this._prevCrouch = false;

    // Ladders (see _updateLadder). base* values scale with the character like every other speed.
    this._baseLadderClimbSpeed = o.ladderClimbSpeed !== undefined ? o.ladderClimbSpeed : lad.climbSpeed;
    this._baseLadderStrafeSpeed = o.ladderStrafeSpeed !== undefined ? o.ladderStrafeSpeed : lad.strafeSpeed;
    this._baseLadderMountReach = o.ladderMountReach !== undefined ? o.ladderMountReach : lad.mountReach;
    this._baseLadderDismountPushSpeed = o.ladderDismountPushSpeed !== undefined ? o.ladderDismountPushSpeed : lad.dismountPushSpeed;
    this._onLadder = false;
    this._ladderNormal = new Goblin.Vector3(0, 0, 1); // points OUT of the ladder face, toward the character

    // Moving platforms (see endStep's acquire + beginStep's apply). A body tagged isPlatform=true,
    // when it's what the ground probe is currently resting on, has its linear_velocity read into
    // this vector once per endStep. beginStep adds it into the horizontal move so collide-and-slide
    // carries the rider through real swept collision; it stays baked into gb.x/z afterward (position
    // integrates from gb on a LATER, separate world step, so subtracting it back out first would
    // discard the ride). _ownVelocityX/Z tracks the character's OWN horizontal velocity separately, so
    // endStep's groundStopDecel (and the sprint-decay branch) decay the character's momentum without
    // also decaying the platform's contribution. The vertical component is folded into a jump's
    // velocity ASSIGNMENT additively (not overwritten) in _updateVertical.
    this._baseVelocity = new Goblin.Vector3(0, 0, 0);
    this._ownVelocityX = 0;
    this._ownVelocityZ = 0;

    var g = world.gravity || { y: -9.81 };
    this._gravityVec = new Goblin.Vector3(0, g.y, 0);
    this._groundSuppress = 0;
    this._prevTopCandidateY = null; // last tick's highest ground candidate — see the slide-launch gate in endStep
    this._slideLaunched = false; // latched true the tick a slide apex launch fires; see endStep

    this._color = o.color || msc.color;
    this._visible = o.visible !== undefined ? o.visible === true : msc.visible;
    this._bodyName = o.bodyName || msc.bodyName;

    this.yaw = o.yaw !== undefined ? o.yaw : vw.yaw;
    this.pitch = o.pitch !== undefined ? o.pitch : vw.pitch;
    this.maxPitch = o.maxPitch !== undefined ? o.maxPitch : vw.maxPitch;

    // Live, render-only aim set per frame via aim(). Separate from yaw/pitch (the commanded,
    // networked, fixed-tick facing) so the view can update every frame without touching the
    // simulation. Falls back to yaw/pitch until aim() is called. See getLiveAimDirection().
    this._liveYaw = this.yaw;
    this._livePitch = this.pitch;
    this._liveAimSet = false;

    // Render interpolation: the body steps at the fixed tick but the screen draws at display
    // refresh. captureRenderState() stashes the last two fixed-tick eyes; renderEye(alpha) lerps
    // them for the draw. _renderSnapDist2 is the squared per-tick eye jump above which the
    // interpolation snaps instead of sliding (teleport/respawn).
    this._prevEye = null;
    this._currEye = null;
    // Base (scale-1) interp snap distance. The SQUARED, scale-adjusted value used at the compare site
    // is (re)derived in _applyScale — a scaled character legitimately moves the eye N× farther per tick,
    // so a fixed 1× threshold would read normal motion as a teleport and snap every tick (killing the
    // sub-tick smoothing → jitter at high scale).
    this._baseRenderSnapDist = o.renderSnapDist !== undefined ? o.renderSnapDist : rnd.snapDist;
    this._renderSnapDist2 = this._baseRenderSnapDist * this._baseRenderSnapDist;
    this._renderProxy = null;

    this.grounded = false;
    this.groundNormal = new Goblin.Vector3(0, 1, 0);
    this.velocityY = 0;
    // Vertical eye displacement this controller applied via the ground-clamp/crouch/scale snaps
    // (not from velocity integration). Render-only; a camera consumes it to smooth those snaps.
    this._viewDisplacementY = 0;
    // True while the caller is resimulating already-run commands (see beginResim/endResim).
    // View-displacement is suppressed during resim so re-derived state doesn't double-count.
    this._resimulating = false;

    // Crouch is an instant collider-height swap. crouchRatio is the fraction of standing
    // height when crouched.
    this.crouchRatio = o.crouchRatio !== undefined ? o.crouchRatio : dim.crouchRatio;
    this.crouching = false;

    // Opaque consumer payload; the controller never reads inside it. Rides the same
    // command->state->snapshot path as crouch/scale.
    this.userData = null;

    this.scale = 1;
    var spawnOpt = o.position;
    var spawn = spawnOpt ? new Goblin.Vector3(spawnOpt.x, spawnOpt.y, spawnOpt.z)
        : new Goblin.Vector3(msc.spawn.x, msc.spawn.y, msc.spawn.z);
    this._applyScale(o.scale !== undefined ? o.scale : msc.scale);
    this._buildBody(spawn);
}

var proto = FPSCharacterController.prototype;

// Resolve scaled dimensions/speeds from the base values.
proto._applyScale = function(scale) {
    this.scale = scale;
    this.width = this._baseWidth * scale;
    this.depth = this._baseDepth * scale;
    // Standing dimensions, then the active height/eye reflect the crouch state.
    this.standHeight = this._baseHeight * scale;
    this.standEye = this._baseEyeHeight * scale;
    this.height = this.crouching ? this.standHeight * this.crouchRatio : this.standHeight;
    this.eyeHeight = this.crouching ? this.standEye * this.crouchRatio : this.standEye;
    this.mass = this._baseMass * scale * scale * scale; // volume scaling
    this.walkSpeed = this._baseWalkSpeed * scale;
    this.moveSpeed = this._baseMoveSpeed * scale;
    this.sprintSpeed = this._baseSprintSpeed * scale;
    this.sprintDecay = this._baseSprintDecay * scale; // excess-speed bleed rate (Infinity = instant)
    this.groundStopDecel = this._baseGroundStopDecel * scale; // idle ground stop rate (Infinity = instant hard-stop)
    this.slideMinSpeed = this._baseSlideMinSpeed * scale;
    this.slideEndSpeed = this._baseSlideEndSpeed * scale;
    this.slideFriction = this._baseSlideFriction * scale;
    this.slideSlopeFriction = this._baseSlideSlopeFriction * scale;
    this.jumpSpeed = this._baseJumpSpeed * Math.sqrt(scale); // jump height scales ~linearly
    this.stepHeight = this._baseStepHeight * scale;
    this.stepDownDist = this._baseStepDownDist * scale;
    this.ladderClimbSpeed = this._baseLadderClimbSpeed * scale;
    this.ladderStrafeSpeed = this._baseLadderStrafeSpeed * scale;
    this.ladderMountReach = this._baseLadderMountReach * scale;
    this.ladderDismountPushSpeed = this._baseLadderDismountPushSpeed * scale;
    this._skin = this._baseSkin * scale; // contact tolerance
    this._groundTol = FPSC.GROUND_TOL * scale; // how close feet must be to count as grounded
    // Terminal fall speed. Also keeps per-step fall distance < ground-probe reach so
    // the raycast ground clamp can't be tunneled through on big drops.
    this._maxFall = 22 * scale;
    // Render interp snap threshold scales with the body: a 4x character sprints ~4x faster, so its eye
    // legitimately jumps ~4x farther per tick. Without this, that normal motion trips the teleport-snap
    // and the sub-tick smoother snaps every tick instead of easing — the high-scale render jitter.
    var rs = (this._baseRenderSnapDist || 0.8) * scale;
    this._renderSnapDist2 = rs * rs;
    // Ghost chase speed and the push-mass eligibility limit must scale with the character, same as
    // sprintSpeed/mass do just above — see the comment where these overrides are read in the
    // constructor. Speed-like (linear); mass-like (volume, scale^3) — matching sprintSpeed/mass.
    this._ghostMaxSpeed = this._ghostMaxSpeedOverride !== undefined ?
        this._ghostMaxSpeedOverride : this._baseSprintSpeed * scale * this._ghostMaxSpeedMult;
    this._ghostMaxDampSpeed = this._ghostMaxDampSpeedOverride !== undefined ?
        this._ghostMaxDampSpeedOverride : this._baseSprintSpeed * scale * this._ghostMaxDampSpeedMult;
    this._pushMassLimit = this._pushMassLimitOverride !== undefined ?
        this._pushMassLimitOverride : this._baseMass * scale * scale * scale * this._pushMassBaseMult;
    this._receiveMaxSpeed = this._baseReceiveMaxSpeed * scale;
};

// Apply this controller's material/behavior defaults + explicit overrides to a freshly created body.
function applyMaterial(body, opts) {
    opts = opts || {};
    body.friction = opts.friction !== undefined ? opts.friction : 3.0;
    body.restitution = opts.restitution !== undefined ? opts.restitution : 0.33;
    body.linear_damping = opts.linearDamping !== undefined ? opts.linearDamping : 0.1;
    body.angular_damping = opts.angularDamping !== undefined ? opts.angularDamping : 0.9;
    if (opts.gravity) { body.setGravity(opts.gravity.x, opts.gravity.y, opts.gravity.z); }
}

// (Re)create the box body at a position, preserving look + velocity where possible.
proto._buildBody = function(position) {
    var carriedVel = null;
    if (this.body) {
        var v = this.body.linear_velocity;
        carriedVel = { x: v.x, y: v.y, z: v.z };
        this.world.removeRigidBody(this.body);
    }
    this._destroyGhost();

    var shape = new Goblin.BoxShape(this.width / 2, this.height / 2, this.depth / 2);
    this.body = new Goblin.RigidBody(shape, this.mass);
    applyMaterial(this.body, {});
    this.body.position.copy(position);
    this.body.updateDerived();
    // `object` is the lightweight cosmetic handle a consumer (renderer) can use to decide whether/how
    // to draw the collider. This controller never renders anything itself.
    this.object = { body: this.body, isVisible: this._visible };

    // Never tip; resting/slopes/walls handled by the solver (gravity + friction).
    this.body.angular_factor.set(0, 0, 0);
    this.body.friction = this.friction;
    this.body.restitution = 0;
    this.body.linear_damping = 0;
    this.body.angular_damping = 0;

    // Tag our physics body so raycasts can ignore ourselves. Also ignore the ghost's name so the
    // kinematic body's own probing raycasts never treat its own trailing ghost as a wall.
    this.body.name = this._bodyName;
    this._ignoreSelf = [this._bodyName, this._bodyName + "_ghost"];
    // Mark this as a kinematic character body so OTHER characters' receive-push pass skips it
    // (character-vs-character is already handled by collide-and-slide treating each other as walls;
    // the body-push coupling is only meant for free dynamic objects).
    this.body.isKinematicCharacter = true;

    // Exclude the character from ALL solver contacts (mask bit 1 = only collide with bodies sharing a
    // matching group; world geometry is group 0). The body still integrates and is still
    // raycast-queryable. Collision is done entirely via raycasts (ground clamp + collide-and-slide),
    // so the solver can never fight the control loop.
    this.body.collision_mask = 1;

    if (carriedVel) { this.body.linear_velocity.set(carriedVel.x, carriedVel.y, carriedVel.z); }

    this.world.addRigidBody(this.body);
    this._buildGhost(position, carriedVel);
};

/**
 * GHOST: a solver-participating dynamic body that trails the kinematic character. The character's own
 * body is excluded from solver contacts (collision_mask 1); the ghost is its stand-in for object
 * contact. Control is one-way: character position -> ghost target. The ghost's position never writes
 * back to the character; only contact-derived knockback flows back (_syncGhost), as a velocity nudge.
 *
 * @method _buildGhost
 * @private
 */
proto._buildGhost = function(position, carriedVel) {
    // Ghost bottom is inset above the character's feet so it doesn't overlap a surface the character is
    // standing on (that's _probeGround's job). Top is unchanged, so head-height contact is unaffected.
    var groundInset = this.height * FPSC.GHOST_GROUND_INSET;
    var ghostHeight = this.height - groundInset;
    var ghostPos = new Goblin.Vector3(position.x, position.y + groundInset / 2, position.z);
    var ghostShape = new Goblin.BoxShape(this.width / 2, ghostHeight / 2, this.depth / 2);
    this._ghost = new Goblin.RigidBody(ghostShape, this.mass);
    applyMaterial(this._ghost, {
        friction: this._ghostMaterial.friction,
        restitution: this._ghostMaterial.restitution,
        linearDamping: this._ghostMaterial.linearDamping,
        angularDamping: this._ghostMaterial.angularDamping,
        gravity: new Goblin.Vector3(0, 0, 0)
    });
    this._ghost.position.copy(ghostPos);
    this._ghost.updateDerived();
    this._ghostObject = { body: this._ghost, isVisible: false };
    this._ghostCommandedVel = null;
    this._ghost.name = this._bodyName + "_ghost";
    this._ghost.angular_factor.set(0, 0, 0);
    this._ghost.isKinematicCharacter = true;
    this._ghostGroundInset = groundInset;
    if (carriedVel) { this._ghost.linear_velocity.set(carriedVel.x, carriedVel.y, carriedVel.z); }
    this.world.addRigidBody(this._ghost);
};

proto._destroyGhost = function() {
    if (this._ghostObject) {
        this.world.removeRigidBody(this._ghost);
        this._ghostObject = null;
        this._ghost = null;
    }
};

/**
 * Drive the ghost toward the character each tick and read back contact-driven knockback. Called once
 * per endStep, after the character's position is settled.
 *
 * @method _syncGhost
 * @private
 */
proto._syncGhost = function(dt) {
    if (!this._ghost) { return; }
    var p = this.body.position;
    var gp = this._ghost.position;
    var targetY = p.y + (this._ghostGroundInset || 0) / 2;
    var dx = p.x - gp.x, dy = targetY - gp.y, dz = p.z - gp.z;
    var gap = Math.sqrt(dx * dx + dy * dy + dz * dz);
    var gv = this._ghost.linear_velocity;

    // A gap this large is a rebuild/respawn/teleport: beam the ghost to the character instead of chasing.
    var teleportDist = Math.max(this.width, this.height) * 2;
    if (gap > teleportDist) {
        this._ghost.position.set(p.x, targetY, p.z);
        gv.set(0, 0, 0);
        this._ghostCommandedVel = { x: 0, y: 0, z: 0 };
        return;
    }

    // Knockback signal = (ghost's actual velocity) - (velocity the drive commanded last tick). This
    // runs during resim too: an authority that never resims applies knockback in its own live step, so
    // skipping it here while resimulating would reconcile the character's velocity to a value that
    // permanently disagrees with authority by the knockback amount. It only needs to be deterministic
    // run-to-run (it is — the read is a pure function of the current contact state).
    this._readGhostKnockback();

    // Blend the ghost's velocity toward the velocity that closes the gap this tick (want = gap/dt).
    //   want = gap/dt; gv = gv*(1-k) + want*k.
    var k = this._ghostStiffness;
    var wx = dx / dt, wy = dy / dt, wz = dz / dt;
    var wLen = Math.sqrt(wx * wx + wy * wy + wz * wz);
    var maxSpeed = this._ghostMaxSpeed;
    if (wLen > maxSpeed) { var s = maxSpeed / wLen; wx *= s; wy *= s; wz *= s; }
    gv.x = gv.x * (1 - k) + wx * k;
    gv.y = gv.y * (1 - k) + wy * k;
    gv.z = gv.z * (1 - k) + wz * k;

    // Clip the ghost's horizontal velocity through the same swept collide-and-slide the character uses.
    var clip = this._sweptCollideAndSlide({
        position: new Goblin.Vector3(gp.x, gp.y, gp.z),
        width: this.width, depth: this.depth, height: this.height - (this._ghostGroundInset || 0),
        skin: this._skin, mass: this.mass, stepHeight: 0,
        selfBody: this._ghost, otherSelfBody: this.body,
        climbSteepSlopes: false,
        vx: gv.x, vz: gv.z, dt: dt,
    });
    gv.x = clip.x; gv.z = clip.z;
    if (clip.depenX !== 0 || clip.depenZ !== 0) {
        this._ghost.position.set(gp.x + clip.depenX, gp.y, gp.z + clip.depenZ);
    }

    this._ghostCommandedVel = { x: gv.x, y: gv.y, z: gv.z }; // baseline for next tick's (actual - commanded) knockback read
};

/**
 * Knockback speed = mass ratio (objectMass/(objectMass+playerMass)) x the object's closing speed
 * onto the character, gated to only apply when the object is moving into the character above a small
 * momentum floor. Horizontal only; never moves position, only velocity.
 *
 * @method _readGhostKnockback
 * @private
 */
proto._readGhostKnockback = function() {
    if (!this._receivePush) { return; }
    var world = this.world;
    if (!world || !world.narrowphase) { return; }
    var ghostBody = this._ghost;
    var pb = this.body.linear_velocity;
    var mP = this.mass;

    var manifold = world.narrowphase.contact_manifolds.first;
    while (manifold) {
        var other =
            manifold.object_a === ghostBody ? manifold.object_b :
            manifold.object_b === ghostBody ? manifold.object_a : null;
        if (other && other._mass !== Infinity && other._mass > 0 && !other.isKinematicCharacter) {
            var mB = other._mass;
            var ov = other.linear_velocity;
            var nx = this._ghost.position.x - other.position.x;
            var nz = this._ghost.position.z - other.position.z;
            var nlen = Math.sqrt(nx * nx + nz * nz);
            if (nlen > FPSC.EPS_LEN) { nx /= nlen; nz /= nlen; } else { nx = 0; nz = 0; }
            // n points box->character. The knockback should trigger on how fast the BOX is coming at you
            // (ov.n), NOT the relative closing speed (ov-pb).n. Using the relative speed folds in YOUR
            // OWN approach velocity (-pb.n > 0 when you walk into the box), so pushing a box knocked you
            // backward every tick — you push, it shoves you back, you re-approach: a limit cycle that
            // renders as the box micro-oscillating toward/away from you at close range. Gating on the
            // box's own inbound speed means a box only knocks you when IT carries momentum at you
            // (someone else shoved it, an explosion) — your own push no longer bounces back. Opt-out via
            // receiveSelfPush to restore the old relative-speed behavior.
            var closing = this._receiveSelfPush ?
                (ov.x - pb.x) * nx + (ov.z - pb.z) * nz :   // legacy: relative closing (self-push included)
                ov.x * nx + ov.z * nz;                      // box's own inbound speed only
            if (closing > FPSC.KB_CLOSING_MIN) {
                var massRatio = mB / (mB + mP);
                var kbv = massRatio * closing;
                if (kbv > this._receiveMaxSpeed) { kbv = this._receiveMaxSpeed; }
                kbv *= this._receiveKnockbackFraction;
                // Cap the RESULTING along-n speed, not just this tick's increment: clamping only kb
                // bounds each tick's contribution but not the running total, so sustained contact (a
                // heavy object pressed against the character for many ticks) adds another kb-worth of speed
                // every tick and blows straight past receiveMaxSpeed. Clamp what the character's velocity
                // ALONG n would become after this tick's push to receiveMaxSpeed instead — a fresh hit
                // (little/no existing along-n speed) still gets up to the full kb, but once already at
                // the cap from prior contact, further ticks add nothing more.
                var alongN = pb.x * nx + pb.z * nz;
                var room = this._receiveMaxSpeed - alongN;
                if (room > 0) { kbv = Math.min(kbv, room); } else { kbv = 0; }
                if (kbv > FPSC.KB_MIN) {
                    pb.x += nx * kbv;
                    pb.z += nz * kbv;
                    this.grounded = false;
                    // This runs mid-tick, inside beginStep's ghost sync — the movement-state dispatch
                    // for THIS tick already ran (it's earlier in beginStep), so this can't retroactively
                    // change what velocity model owned this tick's motion. It CAN and must fix what the
                    // NEXT tick sees: without this, next tick's dispatch would read the stale grounded
                    // sub-state (WALK) and immediately re-clamp the character back onto the ground via
                    // WALK's kinematic model, killing the knockback before it ever got airborne.
                    this._moveState = FPSC.MOVE_AIRBORNE;
                    if (this._groundSuppress < FPSC.GROUND_SUPPRESS_KB) { this._groundSuppress = FPSC.GROUND_SUPPRESS_KB; }
                }
            }
            break;
        }
        manifold = manifold.next_manifold;
    }
};

/**
 * Resize the whole character at runtime (rebuilds the collider, feet planted).
 * @method setScale
 * @param {Number} scale
 */
proto.setScale = function(scale) {
    var p = this.body.position;
    var eyeBefore = p.y + this.eyeHeight;
    var feetY = p.y - this.height / 2;
    this._applyScale(scale);
    this._buildBody(new Goblin.Vector3(p.x, feetY + this.height / 2, p.z));
    if (!this._resimulating) { this._viewDisplacementY += this.body.position.y + this.eyeHeight - eyeBefore; } // eye jump from the resize
};

// Instantly enter/leave crouch by rebuilding the collider at the new height, feet planted.
proto._setCrouch = function(want) {
    if (want === this.crouching) { return; }
    var p = this.body.position;
    var eyeBefore = p.y + this.eyeHeight;
    var feetY = p.y - this.height / 2;
    this.crouching = want;
    this._applyScale(this.scale); // recompute height/eye for the new crouch state
    this._buildBody(new Goblin.Vector3(p.x, feetY + this.height / 2, p.z));
    if (!this._resimulating) { this._viewDisplacementY += this.body.position.y + this.eyeHeight - eyeBefore; } // eye jump from the crouch swap
};

/**
 * Multi-ray UP probe across the footprint. Returns the LOWEST ceiling (down-facing
 * surface) within `reachAboveFeet` of the feet, or null. Mirror of _probeGround; covers
 * sloped overhead geometry (e.g. a ramp underside) that forward rays can't see.
 *
 * @method _probeCeiling
 * @private
 */
proto._probeCeiling = function(reachAboveFeet) {
    var p = this.body.position;
    var feetY = p.y - this.height / 2;
    var startY = feetY + this._skin;
    var endY = feetY + reachAboveFeet + this._skin;
    var ix = this.width / 2 - this._skin;
    var iz = this.depth / 2 - this._skin;
    var offsets = [[0, 0], [ix, 0], [-ix, 0], [0, iz], [0, -iz]];
    var best = null;
    for (var i = 0; i < offsets.length; i++) {
        var ox = offsets[i][0];
        var oz = offsets[i][1];
        var hit = raycast(this.world,
            new Goblin.Vector3(p.x + ox, startY, p.z + oz),
            new Goblin.Vector3(p.x + ox, endY, p.z + oz),
            this._ignoreSelf);
        if (!hit || hit.normal.y > FPSC.NY_CEILING) { continue; } // not a ceiling (must face downward)
        if (!best || hit.point.y < best.point.y) { best = hit; }
    }
    return best;
};

/**
 * Deflect velocity along an overhead surface we're about to contact, instead of capping the
 * rise to zero — a hard cap leaves no velocity to escape and glues us to ceilings, flat AND
 * sloped. Projects out the into-surface component using the ceiling's own normal: v -= (v.n) n.
 * A flat underside (n straight down) zeroes only the vertical, so horizontal motion survives; a
 * sloped underside redirects the upward motion down-and-along the slope, sliding us out. Only
 * acts when actually rising toward a ceiling within this tick's reach.
 *
 * @method _ceilingSlide
 * @private
 */
proto._ceilingSlide = function(vx, vy, vz, dt) {
    if (vy <= 0) { return { vx: vx, vy: vy, vz: vz }; } // not rising -> nothing overhead to resolve
    var reach = this.height + vy * dt + this._skin;
    var ceil = this._probeCeiling(reach);
    if (!ceil) { return { vx: vx, vy: vy, vz: vz }; }
    var gap = ceil.point.y - (this.body.position.y + this.height / 2);
    if (gap > vy * dt + this._skin) { return { vx: vx, vy: vy, vz: vz }; } // won't reach it this tick
    var n = ceil.normal; // down-facing (n.y < 0)
    var dot = vx * n.x + vy * n.y + vz * n.z;
    if (dot < 0) {
        // Heading into the surface: remove that component, leaving motion tangent to it.
        vx -= dot * n.x;
        vy -= dot * n.y;
        vz -= dot * n.z;
    }
    return { vx: vx, vy: vy, vz: vz };
};

/**
 * Is there room to stand up? (No ceiling within standHeight of the feet.)
 * @method _canStand
 * @private
 */
proto._canStand = function() {
    var feetY = this.body.position.y - this.height / 2;
    var ceil = this._probeCeiling(this.standHeight);
    return !ceil || ceil.point.y - feetY >= this.standHeight - this._skin;
};

/**
 * Single ray probe for a ladder ahead, along `dir` (horizontal, need not be unit length). Placed
 * halfway between the feet and stepHeight above them rather than the body center. Returns the raw
 * hit `{object, point, normal, t}` with the normal flipped to point OUT of the face (toward the
 * caller), or null.
 *
 * @method _findLadderAhead
 * @private
 * @param {Goblin.Vector3} dir
 * @return {Object|null}
 */
proto._findLadderAhead = function(dir) {
    var p = this.body.position;
    var dlen = Math.sqrt(dir.x * dir.x + dir.z * dir.z);
    if (dlen < FPSC.EPS_LEN) { return null; }
    var dx = dir.x / dlen, dz = dir.z / dlen;
    var reach = this.width / 2 + this.ladderMountReach;
    var feetY = p.y - this.height / 2;
    var probeY = feetY + this.stepHeight / 2;
    var hit = raycast(this.world,
        new Goblin.Vector3(p.x, probeY, p.z),
        new Goblin.Vector3(p.x + dx * reach, probeY, p.z + dz * reach),
        this._ignoreSelf);
    if (!hit || !hit.object || !hit.object.isLadder) { return null; }
    return hit;
};

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

// ---- Look --------------------------------------------------------------

proto.look = function(deltaYaw, deltaPitch) {
    this.yaw += deltaYaw;
    this.pitch += deltaPitch;
    if (this.pitch > this.maxPitch) { this.pitch = this.maxPitch; }
    if (this.pitch < -this.maxPitch) { this.pitch = -this.maxPitch; }
};

proto.setLook = function(yaw, pitch) {
    this.yaw = yaw;
    this.pitch = pitch;
};

/**
 * Full 3D look direction (includes pitch).
 * @method getLookDirection
 */
proto.getLookDirection = function() {
    var cp = Math.cos(this.pitch);
    return new Goblin.Vector3(Math.sin(this.yaw) * cp, Math.sin(this.pitch), Math.cos(this.yaw) * cp);
};

/**
 * Set the LIVE, caller-owned aim — call once per render frame from your mouse-look. Render-only:
 * this NEVER enters the simulation (it doesn't touch yaw/pitch, the command, or movement), it just
 * keeps a viewmodel/camera glued to the present view instead of the 60Hz sim yaw — fixing the
 * between-tick "dangle" in every mode.
 *
 * @method aim
 */
proto.aim = function(yaw, pitch) {
    this._liveYaw = yaw;
    this._livePitch = pitch;
    this._liveAimSet = true;
};

/**
 * The live aim's full 3D direction (render-only; from aim()). Falls back to the sim look.
 * @method getLiveAimDirection
 */
proto.getLiveAimDirection = function() {
    if (!this._liveAimSet) { return this.getLookDirection(); }
    var cp = Math.cos(this._livePitch);
    return new Goblin.Vector3(Math.sin(this._liveYaw) * cp, Math.sin(this._livePitch), Math.cos(this._liveYaw) * cp);
};

/**
 * Horizontal forward for a given yaw (defaults to current facing).
 * @method getForwardHorizontal
 */
proto.getForwardHorizontal = function(yaw) {
    if (yaw === undefined) { yaw = this.yaw; }
    return new Goblin.Vector3(Math.sin(yaw), 0, Math.cos(yaw));
};

/**
 * Horizontal right for a given yaw (defaults to current facing). Negated to match a
 * left-handed view convention so DirRight strafes to the character's visual right.
 * @method getRightHorizontal
 */
proto.getRightHorizontal = function(yaw) {
    if (yaw === undefined) { yaw = this.yaw; }
    return new Goblin.Vector3(-Math.cos(yaw), 0, Math.sin(yaw));
};

/**
 * World-space eye position (camera goes here).
 * @method getEyePosition
 */
proto.getEyePosition = function() {
    var p = this.body.position;
    return new Goblin.Vector3(p.x, p.y + this.eyeHeight, p.z);
};

/**
 * Return the artificial vertical eye displacement accumulated since the last call (step/landing
 * snaps + crouch/scale swaps) and reset it. A camera folds this into a decaying offset so it
 * eases over those discontinuities. Call once per render frame. Render-only — does not affect sim.
 * @method consumeViewDisplacementY
 */
proto.consumeViewDisplacementY = function() {
    var d = this._viewDisplacementY;
    this._viewDisplacementY = 0;
    return d;
};

/**
 * Peek at the pending vertical eye displacement WITHOUT consuming it. A render-side smoother
 * consumes (consumeViewDisplacementY); a caller that only wants to DETECT a discontinuity this
 * frame (e.g. to snap interpolation instead of sliding the eye) reads this and leaves the value
 * for the smoother. Read-only — never mutates sim or render state.
 * @method peekViewDisplacementY
 */
proto.peekViewDisplacementY = function() {
    return this._viewDisplacementY;
};

/**
 * Stash this fixed tick's eye for sub-tick render interpolation. Call ONCE per REAL fixed step,
 * right after the step settles. A teleport-sized jump (respawn / kill-plane / hard resync) or an
 * artificial step/crouch eye snap snaps the interpolation — prev := curr — so the eye doesn't
 * smear across the discontinuity.
 *
 * @method captureRenderState
 */
proto.captureRenderState = function() {
    var e = this.getEyePosition();
    var snap = !this._currEye;
    if (this._currEye) {
        var dx = e.x - this._currEye.x, dy = e.y - this._currEye.y, dz = e.z - this._currEye.z;
        if (dx * dx + dy * dy + dz * dz > this._renderSnapDist2) { snap = true; } // teleport-sized
    }
    if (Math.abs(this.peekViewDisplacementY()) > FPSC.VIEW_DISP_SNAP) { snap = true; }
    this._prevEye = snap ? e : this._currEye;
    this._currEye = e;
};

/**
 * The render-only eye position: the last two captured fixed-tick eyes lerped by the sub-tick factor
 * `alpha` (0..1, the fraction into the current fixed step the renderer hands the draw call). Falls
 * back to the live physics eye until two ticks have been captured.
 *
 * @method renderEye
 */
proto.renderEye = function(alpha) {
    if (!this._prevEye || !this._currEye) { return this.getEyePosition(); }
    var a = alpha < 0 ? 0 : alpha > 1 ? 1 : alpha;
    var ex = this._prevEye.x + (this._currEye.x - this._prevEye.x) * a;
    var ey = this._prevEye.y + (this._currEye.y - this._prevEye.y) * a;
    var ez = this._prevEye.z + (this._currEye.z - this._prevEye.z) * a;
    return new Goblin.Vector3(ex, ey, ez);
};

/**
 * True while a slide is active this tick. Reads the single authoritative _moveState field that
 * endStep sets — see the "Movement state machine" comment above endStep.
 * @property sliding
 * @type {Boolean}
 * @readOnly
 */
Object.defineProperty(proto, 'sliding', { get: function() { return this._moveState === FPSC.MOVE_SLIDE; } });

/**
 * This tick's movement state: one of FPSC.MOVE_LADDER / MOVE_AIRBORNE / MOVE_WALK / MOVE_SLIP /
 * MOVE_SLIDE. Set exactly once per tick, by endStep, from a fresh ground probe — beginStep (which
 * runs BEFORE endStep, on the state endStep decided last tick) only ever READS this, never
 * re-derives it. See the "Movement state machine" comment above endStep for the full design.
 * @property moveState
 * @type {String}
 * @readOnly
 */
Object.defineProperty(proto, 'moveState', { get: function() { return this._moveState; } });

/**
 * The "too steep to stand on" rule — a floor whose normal tilts below the standable limit gives no
 * footing (MOVE_SLIP). climbSteepSlopes opts out.
 * @method _isSlipSurface
 * @param {Object} normal - a surface normal (uses .y)
 * @return {Boolean}
 * @private
 */
proto._isSlipSurface = function(normal) {
    return !this.climbSteepSlopes && normal.y < this._minStandableNormalY;
};

/**
 * This controller's physics-body name (the value raycasts exclude to avoid self-hits).
 * @property bodyId
 * @type {String}
 * @readOnly
 */
Object.defineProperty(proto, 'bodyId', { get: function() { return this._bodyName; } });

/**
 * The body-name list this controller's own probes ignore — pass to a game's own raycasts
 * (weapons, line-of-sight) so a shooter's cast doesn't hit itself.
 * @property raycastIgnore
 * @type {String[]}
 * @readOnly
 */
Object.defineProperty(proto, 'raycastIgnore', { get: function() { return this._ignoreSelf; } });

/**
 * Reconciliation hooks (opt-in, called by the caller around a ROLLBACK-AND-RESIM of already-run
 * commands — distinct from a game "replay"). During resim the controller re-derives already-
 * perceived state, so its step/crouch snaps must NOT feed a render smoother (that double-counts
 * every step until the resim catches back up). Live ticks are unaffected.
 * @method beginResim
 */
proto.beginResim = function() { this._resimulating = true; };
/**
 * @method endResim
 */
proto.endResim = function() { this._resimulating = false; };

// ---- Simulation (bracketed around one physics world step) --------------

/**
 * PRE-physics: set this tick's horizontal velocity (slope/wall projected) + assists.
 *
 * Aim/sim separation: the movement basis comes from the COMMAND's yaw (`cmd.yaw`) — a per-tick
 * input — not from any persistent "live aim". The live aim belongs to the caller (a camera reads
 * it, never the sim), so replaying commands during reconciliation can't drag the view backward.
 * We record the commanded yaw/pitch as this entity's facing (for getState/avatars) only when the
 * command carries them; a caller that never sets yaw keeps driving facing via look() instead.
 *
 * Also applies platform base velocity into the horizontal move (see the constructor's
 * _baseVelocity comment) immediately before collide-and-slide, so a rider is carried through real
 * swept motion rather than a position teleport.
 *
 * @method beginStep
 * @param {Object} command - pure-data input command struct; any field may be absent
 * @param {Number} dt
 */
proto.beginStep = function(command, dt) {
    var cmd = command || {};

    if (this._jumpBufferTimer > 0) { this._jumpBufferTimer = Math.max(0, this._jumpBufferTimer - dt); }

    if (cmd.scale !== undefined && Math.abs(cmd.scale - this.scale) > FPSC.EPS_LEN) { this.setScale(cmd.scale); }
    // Steep-slope walk intent from the command. Applying it immediately lets local prediction climb
    // right away; if an authority later overrules it, setState corrects the flag from the snapshot.
    // Read live per-tick, so a plain assignment is enough.
    if (cmd.climb !== undefined) { this.climbSteepSlopes = !!cmd.climb; }
    var wantCrouch = !!cmd.crouch || (this.crouching && !this._canStand());
    if (wantCrouch !== this.crouching) { this._setCrouch(wantCrouch); }

    if (cmd.userData !== undefined) { this.userData = cmd.userData; }

    var gb = this.body.linear_velocity;

    this._prevY = this.body.position.y;

    var moveYaw = cmd.yaw !== undefined ? cmd.yaw : this.yaw;
    var movePitch = cmd.pitch !== undefined ? cmd.pitch : this.pitch;
    if (cmd.yaw !== undefined) { this.yaw = cmd.yaw; }
    if (cmd.pitch !== undefined) { this.pitch = cmd.pitch; }

    var fwd = this.getForwardHorizontal(moveYaw);
    var rgt = this.getRightHorizontal(moveYaw);
    var cmdF = cmd.forward || 0;
    var cmdR = cmd.right || 0;
    var dirX = fwd.x * cmdF + rgt.x * cmdR;
    var dirZ = fwd.z * cmdF + rgt.z * cmdR;
    var dirLen = Math.sqrt(dirX * dirX + dirZ * dirZ);
    var hasInput = dirLen > FPSC.EPS_DIR;
    this._cmdIdle = !hasInput;
    var speed = this._getMoveSpeed(cmd);
    var wishX = 0;
    var wishZ = 0;
    if (hasInput) {
        wishX = (dirX / dirLen) * speed;
        wishZ = (dirZ / dirLen) * speed;
    }

    // Stashed for endStep (this same tick, after world.step) to use when it decides this tick's
    // movement state from the fresh ground probe — see the "MOVEMENT STATE DECISION" block there.
    this._wantCrouch = wantCrouch;
    this._hasMoveInput = hasInput;

    // _updateLadder mounts/dismounts and, while mounted, owns velocity fully — checked first since
    // it can override every other state this tick (a ladder grab works even mid-air or mid-slide).
    var onLadderThisTick = this._updateLadder(cmd, moveYaw, movePitch, dt);

    var vx, vz;
    if (onLadderThisTick) {
        // LADDER: _updateLadder already wrote gb.x/gb.z; velocity is fully its.
        vx = gb.x;
        vz = gb.z;
    } else {
        // A jump flips grounded→airborne HERE, before the dispatch below reads this._moveState —
        // _updateVertical updates this._moveState directly on a jump so the same-tick dispatch
        // correctly takes the AIRBORNE branch instead of the stale GROUNDED one.
        this._updateVertical(cmd, dt);

        // ================================================================================
        // MOVEMENT STATE DISPATCH — reads this._moveState, set authoritatively by LAST tick's
        // endStep (or by _updateVertical just above, on a jump this tick). Never re-derives the
        // state from other flags; each branch below is a fully self-contained velocity model for
        // that one state, duplicated rather than shared, so there is exactly one thing to read
        // (this._moveState) to know which branch is live and exactly one place per state that
        // decides its velocity. See the "Movement state machine" comment above endStep.
        // ================================================================================
        if (this._moveState === FPSC.MOVE_SLIDE && this.grounded) {
            // SLIDE, GROUNDED: crouch-at-speed, owns velocity via _updateSlide's surface-tracking
            // model. _updateSlide is a pure per-tick evolver here — it does NOT decide entry/exit
            // anymore (endStep already decided this tick IS a slide); it only advances the
            // slide's velocity one tick (slope accel, friction, steering) from gb, which endStep
            // already set to the correct tangential speed for this tick.
            var slideResult = this._updateSlide(cmd, wishX, wishZ, dt);
            vx = slideResult.vx;
            vz = slideResult.vz;
            gb.y = slideResult.vy;
        } else if (this._moveState === FPSC.MOVE_SLIDE && !this.grounded) {
            // SLIDE, AIRBORNE: a slide that left the ground (ramp lip, drop-off) — see endStep's
            // "genuinely airborne" branch for the condition that keeps this state through the
            // launch. Carried ballistically (gravity, no air-control degradation, no slope model —
            // there's no surface under the character to track) until it lands or slows below
            // slideEndSpeed, at which point endStep drops it to AIRBORNE.
            this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
            if (gb.y < -this._maxFall) { gb.y = -this._maxFall; }
            vx = gb.x;
            vz = gb.z;
        } else if (this._moveState === FPSC.MOVE_SLIP) {
            // SLIP: too-steep surface, gravity-fed, weak air-control.
            this.body.setGravity(0, 0, 0);
            var n = this.groundNormal;
            var slopeMag = Math.sqrt(n.x * n.x + n.z * n.z);
            var dxu = slopeMag > FPSC.EPS_LEN ? n.x / slopeMag : 0;
            var dzu = slopeMag > FPSC.EPS_LEN ? n.z / slopeMag : 0;
            var g = -this._gravityVec.y;
            // Project the incoming 3D velocity onto the plane ONLY on the tick contact is new
            // (endStep left gb.y raw, non-zero, from the fall/toss, on that one tick — see the
            // "MOVEMENT STATE DECISION" comment in endStep). On every later slip tick, endStep
            // zeroes gb.y (the kinematic model owns vertical here, not the solver), so gb.x/gb.z
            // are ALREADY the correctly-accumulated tangential speed from the previous tick's
            // formula below — re-projecting again would read that zeroed gb.y as "no vertical
            // motion yet" and subtract a spurious correction, fighting the accumulation into a
            // false plateau instead of letting speed build tick over tick.
            var gbx = gb.x, gbz = gb.z;
            if (this._slipJustEntered) {
                var dot0 = gb.x * n.x + gb.y * n.y + gb.z * n.z;
                gbx = gb.x - dot0 * n.x;
                gbz = gb.z - dot0 * n.z;
                this._slipJustEntered = false;
            }
            vx = gbx + dxu * g * slopeMag * dt;
            vz = gbz + dzu * g * slopeMag * dt;
            if (hasInput) {
                var twx = wishX, twz = wishZ;
                var up = -(twx * dxu + twz * dzu);
                if (up > 0) { twx += dxu * up; twz += dzu * up; }
                vx += (twx - vx) * this.airControl;
                vz += (twz - vz) * this.airControl;
                var along2 = vx * dxu + vz * dzu;
                if (along2 < 0) { vx -= dxu * along2; vz -= dzu * along2; }
            }
            var alongOut = vx * dxu + vz * dzu;
            gb.y = -alongOut * slopeMag / Math.max(n.y, 0.1);
        } else if (this._moveState === FPSC.MOVE_WALK) {
            // WALK: ordinary input-driven ground movement, projected tangent to groundNormal.
            // KINEMATIC GROUND: gravity off; endStep clamps the feet to the surface. Fully
            // deterministic, doesn't rely on the solver to hold us on a slope (which jittered).
            this.body.setGravity(0, 0, 0);
            var n2 = this.groundNormal;
            var mx, mz;
            if (hasInput) {
                // When slowing while still moving, bleed excess speed at sprintDecay instead of
                // snapping to the lower target speed. _ownVelocityX/Z, not gb.x/z — gb may
                // already carry a platform's base velocity baked in (see the constructor
                // comment); reading it here would re-seed "current speed" with the platform's
                // own speed already added, which then gets base velocity added AGAIN below
                // every tick instead of decaying.
                var cvx = this._ownVelocityX;
                var cvz = this._ownVelocityZ;
                var curSp = Math.sqrt(cvx * cvx + cvz * cvz);
                var wishSp = Math.sqrt(wishX * wishX + wishZ * wishZ);
                if (curSp > wishSp + FPSC.EPS_LEN) {
                    var target = Math.max(wishSp, curSp - this.sprintDecay * dt);
                    var kf = curSp > FPSC.EPS_DIR ? target / curSp : 0;
                    mx = cvx * kf;
                    mz = cvz * kf;
                } else {
                    mx = wishX;
                    mz = wishZ;
                }
            } else {
                // Carry current ground velocity; endStep's groundStopDecel is the sole stop
                // authority. _ownVelocityX/Z, NOT gb.x/z — same reasoning as above.
                mx = this._ownVelocityX;
                mz = this._ownVelocityZ;
            }
            var dot = mx * n2.x + mz * n2.z;
            vx = mx - dot * n2.x;
            vz = mz - dot * n2.z;
            gb.y = -dot * n2.y;
        } else {
            // AIRBORNE: gravity + air control own velocity.
            this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
            var cur = gb;
            if (cur.y < -this._maxFall) { gb.y = -this._maxFall; }
            var curSp2 = Math.sqrt(cur.x * cur.x + cur.z * cur.z);
            var wishSp2 = Math.sqrt(wishX * wishX + wishZ * wishZ);
            if (hasInput) {
                if (wishSp2 >= curSp2) {
                    vx = cur.x + (wishX - cur.x) * this.airControl;
                    vz = cur.z + (wishZ - cur.z) * this.airControl;
                } else {
                    // Steer heading toward wish at the same magnitude, without bleeding speed.
                    var wl = wishSp2 || 1;
                    var tx = (wishX / wl) * curSp2;
                    var tz = (wishZ / wl) * curSp2;
                    vx = cur.x + (tx - cur.x) * this.airControl;
                    vz = cur.z + (tz - cur.z) * this.airControl;
                }
            } else {
                vx = cur.x;
                vz = cur.z;
            }
        }
    }

    if (!onLadderThisTick) {
        var cs = this._ceilingSlide(vx, gb.y, vz, dt);
        vx = cs.vx;
        vz = cs.vz;
        gb.y = cs.vy;
    }

    // Headroom gate: stop us advancing into an overhang too low to fit under (a ramp
    // underside closing onto the floor). A near-horizontal overhang has almost no
    // horizontal surface normal, so collide-and-slide can't see it — we gate on
    // ceiling CLEARANCE instead. Runs before collide-and-slide so walls act on the
    // already-gated velocity.
    var gated = this._headroomGate(vx, vz, dt);

    // Platform base velocity: added in immediately before the swept move so a rider is carried
    // through the SAME collide-and-slide every other velocity goes through (real swept motion, not
    // a position teleport). Stays in gb.x/z afterward — see the constructor's comment for why.
    var bvx = onLadderThisTick ? 0 : this._baseVelocity.x;
    var bvz = onLadderThisTick ? 0 : this._baseVelocity.z;

    // Step-up/step-down are emergent: collide-and-slide ignores anything shorter than
    // stepHeight, and the ground clamp in endStep raises/lowers us onto it. _collideAndSlide reads
    // this._moveState itself (see its own comment) to exempt an active slide from the too-steep
    // wall rule.
    var slid = this._collideAndSlide(gated.x + bvx, gated.z + bvz, dt);
    gb.x = slid.x;
    gb.z = slid.z;
    this._ownVelocityX = slid.x - bvx;
    this._ownVelocityZ = slid.z - bvz;

    this._prevCrouch = !!cmd.crouch;
};

/**
 * POST-physics: decide grounded and clamp the feet to the ground surface. Also acquires this
 * tick's platform base velocity (see the constructor's _baseVelocity comment) from whatever
 * isPlatform-tagged body the ground probe lands on, read fresh every tick.
 * @method endStep
 * @param {Number} dt
 */
proto.endStep = function(dt) {
    var gb = this.body.linear_velocity;

    // While mounted, _updateLadder owns vertical motion (including its own descent-blocks-on-ground
    // check) — the clamp below would otherwise re-snap the character onto the floor near the ladder's
    // base every tick even while actively climbing up, since this.grounded still reads whatever it
    // was at mount time.
    if (this._onLadder) {
        this.velocityY = gb.y;
        if (!this._resimulating || this._driveGhostDuringResim) { this._syncGhost(dt); }
        return;
    }

    if (this._groundSuppress > 0) { this._groundSuppress--; }
    // Only suppress grounding while rising (just jumped/thrust); while falling the ground
    // catch must stay live or the body tunnels through the floor.
    var suppressed = this._groundSuppress > 0 && gb.y > 1;

    var half = this.height / 2;
    var maxStick = this.grounded ? this.stepDownDist + this._skin : this._groundTol;

    // Walk candidates highest-first and take the first that ISN'T too tall to step onto (relative
    // to current feet, only while already grounded — see tooHighToStep below). Falling through to
    // a lower, valid candidate keeps grounding honest when a taller obstacle (e.g. a box shoved
    // against the footprint) is also in reach.
    var candidates = this._probeGroundCandidates(this.stepDownDist);
    // Slide launch off a ramp apex: only while SLIDING and genuinely rising (gb.y > 0, tangent to the
    // surface being ridden). Climbing a slope, the highest ground candidate rises every tick with the
    // character. At the uphill edge the forward probe rays overshoot into air, so the highest hit stops
    // rising and RECEDES (the rear rays win) — the tick that happens is the real apex. Clamping to that
    // receded surface would hug the character down a fraction (a one-tick dip) before the edge finally
    // leaves probe reach; instead skip the clamp so the slide launches ballistically off the true edge.
    // Slide launch off a ramp apex — only while SLIDING and rising (walking off the same edge just
    // follows the ground down). ANGLE-BLIND: a slide treats every slope identically regardless of
    // steepness, so this gate never asks "is this too steep" — only "is this still the surface I'm
    // riding." Two ways the true edge shows up in the probe, both handled here:
    //   1. The highest surface RECEDES: the ramp face we were climbing runs out ahead, so the highest
    //      remaining ramp hit drops vs last tick. Clamping to it would hug us down a one-tick dip.
    //   2. A MISMATCHED face (e.g. the ramp's own end-cap) becomes the highest candidate: taller than
    //      the ramp face but not the surface we're riding (normal meaningfully off groundNormal). It can
    //      mask signal #1 by sitting on top, so we test it independently — riding a ramp, the candidate
    //      still ON that same face keeps matching every tick and never trips this; only a genuinely
    //      different face (the real edge) does.
    var topCandidate = candidates.length > 0 ? candidates[0] : null;
    var topCandidateY = topCandidate ? topCandidate.point.y : null;
    var wasSliding = this._moveState === FPSC.MOVE_SLIDE;
    if (this.grounded && wasSliding && gb.y > FPSC.EPS_LEN && topCandidate !== null) {
        var receded = this._prevTopCandidateY !== null && topCandidateY < this._prevTopCandidateY - FPSC.EPS_LEN;
        var normalDot = topCandidate.normal.x * this.groundNormal.x +
            topCandidate.normal.y * this.groundNormal.y +
            topCandidate.normal.z * this.groundNormal.z;
        var mismatched = normalDot < this._minStandableNormalY;
        if (receded || mismatched) { candidates = []; this._slideLaunched = true; }
    }
    // A slide apex launch is latched, not a one-tick decision: the tick it fires, grounded flips false
    // immediately, so the gate above (which requires it true) can never re-arm to catch a second graze
    // later in the same arc. Without this latch, a low/shallow launch that skims just above the ramp's
    // tail gets ground-clamped straight back down the very next tick the probe happens to reach it — a
    // one-tick "dip" mid-arc. Sliding off an apex must NEVER re-hug the geometry, full stop, so once
    // latched we force every candidate away regardless of what the probe finds, for as long as the arc
    // is still rising. The latch clears once gb.y stops climbing (the arc has peaked and started to
    // fall) — from that point a real landing is legitimate and ground detection must resume normally.
    if (this._slideLaunched) {
        if (gb.y > FPSC.EPS_LEN) { candidates = []; }
        else { this._slideLaunched = false; }
    }
    this._prevTopCandidateY = topCandidateY;
    var probe = null, tooHighToStep = false;
    for (var ci = 0; ci < candidates.length; ci++) {
        var c = candidates[ci];
        var rise = (c.point.y + half) - this.body.position.y;
        var tooHigh = this.grounded && rise > this.stepHeight + this._skin;
        if (!tooHigh) { probe = c; tooHighToStep = false; break; }
        if (!probe) { probe = c; tooHighToStep = true; } // keep the highest as a fallback reference
    }

    // feetGap > 0 = feet above ground; < 0 = penetrating (always clamp back out).
    var feetGap = probe ? this.body.position.y - half - probe.point.y : Infinity;

    if (!suppressed && probe && feetGap <= maxStick && !tooHighToStep) {
        var p = this.body.position;
        var clampedY = probe.point.y + half;
        if (!this._resimulating) { this._viewDisplacementY += clampedY - p.y; }
        this.body.position.set(p.x, clampedY, p.z);
        this.body.updateDerived();

        // Acquire base velocity from whatever we're now resting on BEFORE splitting gb into
        // own-vs-base components below — every own-velocity computation this tick (tangentX/Z,
        // _ownVelocityX/Z) must subtract THIS tick's platform speed, not last tick's. Read fresh
        // every tick so stepping off a platform onto solid ground (or vice versa) updates
        // immediately, and so landing on a moving platform doesn't misread one tick of its speed
        // as newly-acquired character momentum.
        var standingOn = probe.object;
        if (standingOn && standingOn.isPlatform) {
            var pv = standingOn.linear_velocity;
            this._baseVelocity.set(pv.x, pv.y, pv.z);
        } else {
            this._baseVelocity.set(0, 0, 0);
        }

        // ================================================================================
        // MOVEMENT STATE DECISION — the ONE place per tick this is decided, from the ONE real
        // ground probe this tick has. beginStep (next tick) only ever reads this._moveState; it
        // never re-derives sliding/slipping/walking from other flags.
        // ================================================================================
        var pn = probe.normal;
        var probeSlope = Math.sqrt(pn.x * pn.x + pn.z * pn.z);

        // Project the incoming 3D velocity onto the surface plane ONCE, here, on every grounding
        // tick — not just the first-contact tick. (v -= (v·n)n): removes the into-surface
        // component, keeps the along-surface (tangential) component. On a tick where the body was
        // already resting on this same surface last tick too, this is a no-op (gb is already
        // tangent), so it's safe to always run — no separate "first contact only" special case.
        var vdotn = gb.x * pn.x + gb.y * pn.y + gb.z * pn.z;
        var tangentX = gb.x - vdotn * pn.x;
        var tangentZ = gb.z - vdotn * pn.z;
        var horizTangentSpeed = Math.sqrt(tangentX * tangentX + tangentZ * tangentZ);

        // TRUE along-the-ground speed, for the slide entry/sustain SPEED test only (tangentX/Z above
        // is what actually gets written to gb — always the horizontal projection). On a steep slope,
        // riding fast downhill puts most of the character's speed into the VERTICAL component (the
        // kinematic ground clamp keeps gb.y at 0 between ticks, so a slip's own steady-state horizontal
        // speed converges to a value bounded near moveSpeed by its air-control blend toward wish — it
        // can never actually EXCEED moveSpeed on its own, and never would cross the slide-entry
        // threshold below if measured on the horizontal component alone). Reconstruct the true 3D
        // along-surface speed the same way a raw fall's vertical energy converts to horizontal on a
        // slide: split gb into along-slope (dxu,dzu) and cross-slope, then divide the along-slope part
        // by ny to recover the steeper true speed a shallow horizontal reading was hiding.
        var slopeMag0 = probeSlope;
        var ny0 = Math.max(pn.y, 0.1);
        var groundSp;
        if (slopeMag0 > FPSC.EPS_LEN) {
            var dxu0 = pn.x / slopeMag0, dzu0 = pn.z / slopeMag0;
            var alongH = tangentX * dxu0 + tangentZ * dzu0;
            var crossSq = Math.max(0, horizTangentSpeed * horizTangentSpeed - alongH * alongH);
            var surfFall = alongH / ny0;
            groundSp = Math.sqrt(surfFall * surfFall + crossSq);
        } else {
            groundSp = horizTangentSpeed;
        }
        var tangentSpeed = groundSp;

        var isSlipSurface = this._isSlipSurface(pn);
        // Slide ENTRY/SUSTAIN uses the SAME rule regardless of whether this is the first contact
        // tick or the 500th tick of an already-active slide: crouch held, and (on a slope, ride
        // until crouch releases; on flat, need speed above slideEndSpeed to keep going / above
        // moveSpeed to start). This mirrors _updateSlide's old entry/sustain split, but evaluated
        // ONCE, with this tick's own fresh probe normal and true tangential speed — not the
        // previous tick's groundNormal, not a landing-only special case.
        var slopeSlideEligible = probeSlope >= this.slideSlopeMin;
        var hasMoveInputThisTick = this._hasMoveInput;
        var slideInputOk = !this.slideRequiresMoveInput || hasMoveInputThisTick ||
            (this.slideAllowLandingWithoutInput && !this.grounded);
        var slideSustainOk = slopeSlideEligible || tangentSpeed >= this.slideEndSpeed;
        var slideEntryOk = slideInputOk && tangentSpeed > this.moveSpeed + FPSC.EPS_SPEED_MARGIN;
        var wantsSlide = !!this._wantCrouch && (wasSliding ? slideSustainOk : slideEntryOk);

        if (wantsSlide) {
            this._moveState = FPSC.MOVE_SLIDE;
            var enteringSlide = !wasSliding;
            // slideBoost applied HERE, on the exact entry tick, directly to the velocity endStep is
            // about to commit — not inside _updateSlide (which only runs the FOLLOWING tick, in
            // beginStep). Applying it there would show the boost one tick later than the state
            // transition itself, which is observably wrong (a caller reading "just started
            // sliding" this tick would see un-boosted speed).
            var boostedX = tangentX, boostedZ = tangentZ;
            if (enteringSlide && this.slideBoost !== 1) {
                boostedX *= this.slideBoost;
                boostedZ *= this.slideBoost;
            }
            gb.x = boostedX;
            gb.z = boostedZ;
            // gb.y is left for _updateSlide's onSlope solve to derive from the tangential speed
            // above — writing a raw projected vertical here overshoots the surface-follow value
            // and skips the character off the ramp for a tick (a bounce).
            gb.y = 0;
        } else if (isSlipSurface) {
            // Entry edge: this tick starts a NEW slip iff last tick wasn't already one. beginStep's
            // SLIP branch only re-projects gb onto
            // groundNormal on that one entry tick (see its own comment) — every later tick, gb.y
            // is already 0 (set below) and gb.x/gb.z already hold the correctly-accumulated
            // tangential speed from beginStep's own per-tick formula, so re-projecting again would
            // corrupt that accumulation into a false plateau.
            var enteringSlip = this._moveState !== FPSC.MOVE_SLIP;
            this._slipJustEntered = enteringSlip;
            this._moveState = FPSC.MOVE_SLIP;
            // Keep the RAW incoming gb.x/gb.z/gb.y (NOT the tangential projection) on the entry
            // tick — beginStep's SLIP branch does its own plane projection from this.groundNormal
            // next tick, gated to _slipJustEntered, and needs gb.y to still be the real incoming
            // fall speed to project. From the SECOND slip tick on, gb.y is zeroed here as usual —
            // beginStep's per-tick formula derives its own vy from there on, and leaving a stale
            // gb.y would double-count it.
            if (!enteringSlip) { gb.y = 0; }
        } else {
            this._moveState = FPSC.MOVE_WALK;
            gb.x = tangentX;
            gb.z = tangentZ;
            gb.y = 0;
        }
        this._ownVelocityX = gb.x - this._baseVelocity.x;
        this._ownVelocityZ = gb.z - this._baseVelocity.z;

        // Idle ground-stop: WALK only. Bleeds horizontal speed toward zero at groundStopDecel.
        // Reads/writes _ownVelocityX/Z (the character's OWN component), NOT gb.x/z directly — gb
        // may already carry a platform's base velocity baked in, and decaying THAT would fight
        // the ride. The decayed own-component is added back onto base velocity so gb ends up
        // carrying: decayed own motion + full undecayed platform motion.
        if (this._cmdIdle && this._moveState === FPSC.MOVE_WALK) {
            var cvx = this._ownVelocityX || 0;
            var cvz = this._ownVelocityZ || 0;
            var sp = Math.sqrt(cvx * cvx + cvz * cvz);
            var target = Math.max(0, sp - this.groundStopDecel * dt);
            var kf = sp > FPSC.EPS_SPD ? target / sp : 0;
            this._ownVelocityX = cvx * kf;
            this._ownVelocityZ = cvz * kf;
            gb.x = this._ownVelocityX + this._baseVelocity.x;
            gb.z = this._ownVelocityZ + this._baseVelocity.z;
        }

        this.grounded = true;
        this.groundNormal.set(probe.normal.x, probe.normal.y, probe.normal.z);
    } else if (tooHighToStep) {
        // Refusing to climb something too tall (e.g. a box shoved into the footprint) must NOT be
        // treated as leaving the ground: the feet haven't moved, there's no gap, no fall — the
        // character is exactly where it was a moment ago, still resting on whatever it was resting
        // on. Staying grounded on rejection keeps the height-limit check
        // (this.grounded && rise > stepHeight) honest on the next tick too.
        gb.y = 0;
        // Movement state is UNCHANGED here on purpose: the character is exactly where it was,
        // still resting on whatever it was resting on, so whatever state that was is still true.
    } else {
        this.grounded = false;
        // A slide that leaves the ground (ramp lip, drop-off) stays MOVE_SLIDE through the airborne
        // arc — carried mostly ballistically rather than air-controlled — as long as horizontal
        // speed is still above slideEndSpeed (the same floor flat sliding itself uses to decide
        // "still going") and crouch is still held. beginStep's SLIDE branch has its own airborne vs.
        // grounded sub-cases for exactly this reason. Landing re-enters the ordinary MOVEMENT STATE
        // DECISION above on the fresh probe normal, so it naturally continues sliding (onto a ramp)
        // or drops to WALK/SLIP there — no separate landing special-case needed here.
        var wasSlideBeforeLoss = this._moveState === FPSC.MOVE_SLIDE;
        var stillFastEnough = Math.sqrt(gb.x * gb.x + gb.z * gb.z) >= this.slideEndSpeed;
        if (wasSlideBeforeLoss && this._wantCrouch && stillFastEnough) {
            this._moveState = FPSC.MOVE_SLIDE;
        } else {
            this._moveState = FPSC.MOVE_AIRBORNE;
        }
        // Genuinely airborne — no ground entity to inherit velocity from. A jump already captured
        // baseVelocity.y additively the tick it fired (_updateVertical); clearing here only stops
        // FUTURE ticks from reading a stale platform velocity while falling free.
        this._baseVelocity.set(0, 0, 0);
    }

    // Coyote window: refill while grounded, bleed down once airborne. No-op when coyoteTime=0.
    if (this.grounded) { this._coyoteTimer = this.coyoteTime; }
    else if (this._coyoteTimer > 0) { this._coyoteTimer = Math.max(0, this._coyoteTimer - dt); }

    this.velocityY = gb.y;

    // Drive the ghost every tick, INCLUDING during resim: the ghost is how the character pushes objects,
    // and object pushes must be reproduced when already-run commands get rolled back and resimulated
    // (otherwise a pushed object is predicted live but snaps back every snapshot — rubber-banding). The
    // ghost drive is deterministic given the character's state. What must NOT run during resim is the
    // knockback READBACK from the ghost into the character (see _syncGhost / _readGhostKnockback): feeding
    // a solver body's contact velocity back into the character mid-rollback is what injects non-determinism
    // into the reconciled character path. That readback is gated inside _syncGhost.
    // Opt-out (driveGhostDuringResim=false): freeze the ghost during resim (older behavior).
    if (!this._resimulating || this._driveGhostDuringResim) { this._syncGhost(dt); }
};

// ---- Overridable kit hooks --------------------------------------------

proto._getMoveSpeed = function(cmd) {
    // Gait priority: sprint > walk > run. Crouch scales the chosen gait.
    var gait = cmd.sprint ? this.sprintSpeed : cmd.walk ? this.walkSpeed : this.moveSpeed;
    return cmd.crouch ? gait * this.crouchSpeedMult : gait;
};

/**
 * Slide velocity EVOLVER — advances one tick of the slide's surface-tracking model (slope accel,
 * friction, steering). Pure: only called from beginStep's MOVE_SLIDE branch, which is only reached
 * when endStep has ALREADY decided this tick is a slide (see the "MOVEMENT STATE DECISION" block
 * in endStep) and has already written the correct starting tangential velocity into gb — including
 * the one-time entry boost (slideBoost), applied there rather than here so it lands on the exact
 * tick the state transition itself is observable, not one tick later. This function does not
 * decide whether to slide — it has no entry gate, no exit gate, no stored flag. It reads gb (this
 * tick's starting velocity, already tangent to groundNormal), advances it one tick, and returns
 * the result.
 *
 * @method _updateSlide
 * @private
 * @return {Object} {vx, vy, vz} — this tick's slide velocity, always 3D.
 */
proto._updateSlide = function(cmd, wishX, wishZ, dt) {
    // _ownVelocityX/Z, NOT gb.x/z — gb carries the platform's base velocity baked in (see the
    // constructor's _baseVelocity comment). Evolving the raw gb value would re-seed the slide's own
    // momentum with the platform's speed already added, which then compounds every tick instead of
    // properly decaying (the platform reads as if its own speed were the character's own build-up —
    // a "boost pad" while sliding on a moving platform).
    var vx = this._ownVelocityX;
    var vz = this._ownVelocityZ;
    var sp = Math.sqrt(vx * vx + vz * vz);

    var n = this.groundNormal;
    var slopeMag = Math.sqrt(n.x * n.x + n.z * n.z);
    var gy = this._gravityVec.y;
    var onSlope = slopeMag >= this.slideSlopeMin;
    // Downhill fall-line unit vector, used both by the slope-accel step below and by the reversal
    // brake's uphill test further down. Only meaningful when onSlope; 0 otherwise (unused there).
    var dx = onSlope ? n.x / slopeMag : 0;
    var dz = onSlope ? n.z / slopeMag : 0;

    if (onSlope) {
        // Gravity accelerates the fall-line (downhill) component; the cross-slope (sideways)
        // part bleeds lightly. Returned as full 3D so the grounded branch doesn't re-project it.
        var along = vx * dx + vz * dz;
        var crossX = vx - along * dx;
        var crossZ = vz - along * dz;
        // Along-slope gravitational accel is g*sin(theta) — slopeMag alone (sin of the tilt from
        // horizontal). The extra n.y (cos theta) factor here was wrong: sin(theta)*cos(theta) PEAKS at
        // 45° and falls back off toward vertical, so a 55°+ face decelerated barely harder than a 20°
        // one, and a near-vertical wall almost not at all — backwards from real physics, where steeper
        // always means more deceleration, up to g at 90°.
        along += -gy * slopeMag * this.slideSlopeAccel * dt;
        var cs = Math.sqrt(crossX * crossX + crossZ * crossZ);
        var cn = Math.max(0, cs - this.slideSlopeFriction * dt);
        var cf = cs > FPSC.EPS_DIR ? cn / cs : 0;
        crossX *= cf;
        crossZ *= cf;
        vx = along * dx + crossX;
        vz = along * dz + crossZ;
        sp = Math.sqrt(vx * vx + vz * vz);
    } else {
        var next = Math.max(0, sp - this.slideFriction * dt);
        var f = sp > FPSC.EPS_DIR ? next / sp : 0;
        vx *= f;
        vz *= f;
        sp = next;
    }

    // Rotate the slide heading toward input without adding speed (renormalize to sp).
    var wl = Math.sqrt(wishX * wishX + wishZ * wishZ);
    if (this.slideControl > 0 && wl > FPSC.EPS_DIR && sp > FPSC.EPS_DIR) {
        var wnx = wishX / wl, wnz = wishZ / wl;
        // Wish opposing current motion (e.g. holding backward mid-slide) is a deliberate reversal,
        // not a carve — the ordinary partial blend below would slowly rotate the heading through an
        // arc instead of braking straight back. Detect that case (wish nearly opposite current
        // velocity) and brake toward zero along the CURRENT heading instead of blending toward
        // wish; once speed has bled down, the same blend below is what picks the (now-reversed)
        // heading back up, so the reversal itself still ends up sliding in the wish direction — it
        // just brakes-then-goes instead of curving through it. Applies on flat ground too: without
        // this, flat sliding's own friction decay bled speed down to the slideEndSpeed exit
        // threshold WHILE the un-braked blend was arcing the heading toward wish, so a backward
        // hold curved through a U-turn on its way out instead of braking straight.
        var brakeRate = onSlope ? this.slideSlopeFriction * this.slideReversalBrakeMult
            : this.slideFriction * this.slideReversalBrakeMult;
        var vnx = vx / sp, vnz = vz / sp;
        var facing = wnx * vnx + wnz * vnz; // 1 = same direction, -1 = dead opposite
        // ANGLE-BLIND: on ANY slope, gravity always wins the fall-line — you can't carve a slide uphill
        // against it, only brake. A wish with any uphill component (against the downhill fall-line
        // dx/dz) must BRAKE toward a stop, not carve; otherwise the carve below redirects the blocked
        // uphill momentum into a cross-slope skid off the side. On flat there's no fall-line to fight,
        // so only a near-opposite wish counts as a reversal there (unchanged).
        var uphillOnSlope = onSlope && (wnx * dx + wnz * dz) < 0;
        if (uphillOnSlope || facing < FPSC.SLIDE_REVERSAL_DOT) {
            var braked = Math.max(0, sp - brakeRate * dt);
            var bf = sp > FPSC.EPS_DIR ? braked / sp : 0;
            vx *= bf;
            vz *= bf;
        } else {
            var tx = vx + (wnx * sp - vx) * this.slideControl;
            var tz = vz + (wnz * sp - vz) * this.slideControl;
            var tl = Math.sqrt(tx * tx + tz * tz) || 1;
            vx = (tx / tl) * sp;
            vz = (tz / tl) * sp;
        }
    }

    var vy = 0;
    if (onSlope) {
        var inv2 = 1 / slopeMag;
        var alongOut = vx * (n.x * inv2) + vz * (n.z * inv2);
        vy = -alongOut * slopeMag / Math.max(n.y, 0.1);
        // The velocity returned here is already tangent to the surface — including on a TOO-STEEP slope.
        // The too-steep-can't-move-up rules don't re-clip it: an active slide is exempt everywhere they
        // apply (see _collideAndSlide's climbSteepSlopes opt) — the slide IS the climb.
    }
    // Flat ground (!onSlope): groundNormal.y is ~1, so gb.y should stay ~0 — the caller (beginStep's
    // ground clamp path, same as WALK) doesn't need a nonzero vy to track a surface that's already
    // level. vy=0 here is that "no vertical correction needed" case, not a special flat-only shape.
    return { vx: vx, vy: vy, vz: vz };
};

/**
 * Vertical hook. Base = grounded jump only (gravity/landing handled by the solver). A jump adds
 * platform base velocity's Y component additively, not an overwrite — jumping off a rising
 * platform flings the character higher than jumpSpeed alone would.
 * @method _updateVertical
 * @protected
 */
proto._updateVertical = function(cmd, dt) {
    var canJump = this.grounded || this._coyoteTimer > 0;
    var wantJump = cmd.jumpPressed || this._jumpBufferTimer > 0;
    if (canJump && wantJump) {
        // Additive, not a bare overwrite: jumping off a platform that's currently rising carries
        // its vertical base velocity into the jump (a "fling"), on top of whatever base velocity
        // the character already had that tick.
        this.body.linear_velocity.y = this.jumpSpeed + this._baseVelocity.y;
        this.grounded = false;
        // beginStep's movement-state dispatch runs right after this call, on the SAME tick — must
        // see AIRBORNE now, not whatever grounded sub-state was true a moment ago.
        this._moveState = FPSC.MOVE_AIRBORNE;
        this._groundSuppress = FPSC.GROUND_SUPPRESS_JUMP;
        this._coyoteTimer = 0;
        this._jumpBufferTimer = 0;
    } else if (cmd.jumpPressed) {
        this._jumpBufferTimer = this.jumpBuffer;
    }
};

// ---- Internal helpers --------------------------------------------------

/**
 * Is there a too-steep-but-climbable slope surface rising just ahead of the move? Used only when
 * climbSteepSlopes is on. Casts down-rays a short distance ahead and looks for an upward-tilted,
 * too-steep-to-stand hit that is still a real slope (not flat floor, not a vertical wall).
 * @method _climbableSlopeAhead
 * @private
 */
proto._climbableSlopeAhead = function(start, dx, dz) {
    if (dx === 0 && dz === 0) { return false; }
    var feetY = this.body.position.y - this.height / 2;
    var base = this.depth / 2 + this._skin; // footprint edge (both scale with the character)
    for (var mi = 0; mi < FPSC.CLIMB_PROBE_DEPTH_MULTS.length; mi++) {
        var m = FPSC.CLIMB_PROBE_DEPTH_MULTS[mi];
        var ahead = base + m * this.depth; // reach past the footprint in units of depth (scale-invariant)
        var ax = start.x + dx * ahead;
        var az = start.z + dz * ahead;
        var hit = raycast(this.world,
            new Goblin.Vector3(ax, feetY + this.stepHeight + this._skin, az),
            new Goblin.Vector3(ax, feetY - this.stepHeight, az),
            this._ignoreSelf);
        if (hit && hit.normal.y > FPSC.NY_STEEP_MIN && hit.normal.y < this._minStandableNormalY) { return true; }
    }
    return false;
};

/**
 * Multi-point ground probe (center + four edge midpoints). Returns ALL floor-like hits, highest
 * first — NOT collapsed to a single "best" here, because the caller needs to fall back to a
 * lower (but valid) hit when the highest one is rejected as too tall to step onto (e.g. one edge
 * ray grazing a box pushed against the footprint, while the other four rays are still squarely
 * over real floor). Collapsing to one hit here would throw the floor away before the caller ever
 * gets a chance to prefer it.
 * @method _probeGroundCandidates
 * @private
 */
proto._probeGroundCandidates = function(maxSnap) {
    var half = this.height / 2;
    var p = this.body.position;
    // Cast from the higher of this tick's start position and the current position, so a fast
    // descent that penetrated the floor this tick doesn't miss it.
    var topY = Math.max(this._prevY !== undefined ? this._prevY : p.y, p.y) + this._skin;
    var bottomY = p.y - (half + maxSnap + this._skin);
    var ix = this.width / 2 - this._skin;
    var iz = this.depth / 2 - this._skin;
    var offsets = [[0, 0], [ix, 0], [-ix, 0], [0, iz], [0, -iz]];

    var candidates = [];
    for (var i = 0; i < offsets.length; i++) {
        var ox = offsets[i][0];
        var oz = offsets[i][1];
        var start = new Goblin.Vector3(p.x + ox, topY, p.z + oz);
        var end = new Goblin.Vector3(p.x + ox, bottomY, p.z + oz);
        var hit = raycast(this.world, start, end, this._ignoreSelf);
        if (!hit || hit.normal.y < FPSC.NY_FLOORLIKE) { continue; }
        // Exclude a pushable object as ground only when walking INTO its side (pushing it), not
        // when it's roughly under our own center (standing on it).
        var gm = hit.object && hit.object._mass;
        var isPushable = gm !== undefined && gm !== Infinity && gm > 0 && isFinite(gm) && gm <= this._pushMassLimit;
        if (isPushable) {
            var gv = this.body.linear_velocity;
            var toHitX = hit.point.x - p.x, toHitZ = hit.point.z - p.z;
            var towardLen = Math.sqrt(toHitX * toHitX + toHitZ * toHitZ);
            var movingIntoIt = towardLen > FPSC.EPS_LEN &&
                (gv.x * toHitX + gv.z * toHitZ) / towardLen > FPSC.PUSH_INTO_MIN;
            var nearCenter = towardLen < this.width * FPSC.NEAR_CENTER_FRAC;
            if (movingIntoIt && !nearCenter) { continue; }
        }
        candidates.push(hit);
    }
    candidates.sort(function(a, b) { return b.point.y - a.point.y; });
    return candidates;
};

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
 * Core swept-move collide-and-slide, parameterized so any caller (character body, ghost body) can
 * apply the same wall/mass-yield rule.
 *
 * `opts`: { position, width, depth, height, skin, mass, stepHeight, selfBody, otherSelfBody,
 *           climbSteepSlopes, vx, vz, dt }. `otherSelfBody` is excluded from hits. Step-up/climb
 * are opt-in via stepHeight/climbSteepSlopes; pass stepHeight=0 and climbSteepSlopes=false to skip
 * that logic. Returns { x, z, depenX, depenZ }.
 *
 * @method _sweptCollideAndSlide
 * @private
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

/**
 * Lowest ceiling clearance over the footprint centered at (cx,cz). Infinity if nothing overhead.
 * @method _ceilingClearanceAt
 * @private
 */
proto._ceilingClearanceAt = function(cx, cz, feetY) {
    // Start above step-up height so a steppable obstacle (stair/low box) doesn't register as a
    // low ceiling; anything below feet+stepHeight is the ground clamp's job, not the gate's.
    var startY = feetY + this.stepHeight + this._skin;
    var endY = feetY + this.standHeight + this._skin;
    var ix = this.width / 2 - this._skin;
    var iz = this.depth / 2 - this._skin;
    var offsets = [[0, 0], [ix, 0], [-ix, 0], [0, iz], [0, -iz]];
    var lowest = Infinity;
    for (var i = 0; i < offsets.length; i++) {
        var ox = offsets[i][0];
        var oz = offsets[i][1];
        var hit = raycast(this.world,
            new Goblin.Vector3(cx + ox, startY, cz + oz),
            new Goblin.Vector3(cx + ox, endY, cz + oz),
            this._ignoreSelf);
        if (!hit || hit.normal.y > FPSC.NY_CEILING) { continue; } // not a ceiling (must face downward)
        // A dynamic/pushable object is never a "ceiling" — it's something the swept mover + push handle,
        // not the headroom gate. Without this, an object being actively shoved forward can wobble a few
        // degrees off-axis from contact torque, and its top face intermittently pokes above the
        // stepHeight cutoff below on some ticks but not others — a real, observed source of push
        // oscillation (the gate flickering the character's forward velocity to zero and back as the
        // box jitters). Only STATIC geometry (mass===Infinity) counts as an overhang.
        if (hit.object && hit.object._mass !== Infinity) { continue; }
        var clr = hit.point.y - feetY;
        // A "ceiling" clearance at or below step height is NOT an overhang — it's a low obstacle at
        // shin/waist level that the swept mover + push handle, not the headroom gate. Without this, a
        // low stepHeight drops the ray start (feetY+stepHeight) INTO a waist-high object ahead, and the
        // ray reports a bogus ~stepHeight-clearance "ceiling", so the gate walls the character in open
        // space in front of a pushable box (worse the lower stepHeight is). Only count genuine overhangs
        // — clearance meaningfully above the step line — as ceilings.
        if (clr <= this.stepHeight + this._skin) { continue; }
        if (clr < lowest) { lowest = clr; }
    }
    return lowest;
};

/**
 * Treat insufficient headroom as a virtual wall: gate on ceiling clearance ahead (rather than
 * surface normal, which a near-horizontal ramp underside can't provide) and slide along the
 * horizontal gradient of increasing clearance.
 * @method _headroomGate
 * @private
 */
proto._headroomGate = function(vx, vz, dt) {
    var speed = Math.sqrt(vx * vx + vz * vz);
    if (speed < FPSC.EPS_DIR) { return { x: vx, z: vz }; }

    if (this.climbSteepSlopes && this._climbableSlopeAhead(this.body.position, vx / speed, vz / speed)) {
        return { x: vx, z: vz };
    }

    var p = this.body.position;
    var feetY = p.y - this.height / 2;
    var need = this.height + this._skin;
    var halfDiag = Math.sqrt((this.width / 2) * (this.width / 2) + (this.depth / 2) * (this.depth / 2));

    // Check clearance centered at the CURRENT position, not a forward-projected point — _ceilingClearanceAt
    // already samples +-(width/2-skin) / +-(depth/2-skin) around its center argument, which is the box's own
    // full footprint including its leading edge. Projecting a "reach" forward on top of that double-counts.
    // The footprint offsets ARE the reach.
    if (this._ceilingClearanceAt(p.x, p.z, feetY) >= need) { return { x: vx, z: vz }; }

    var eps = halfDiag + this._skin;
    var cR = this._ceilingClearanceAt(p.x + eps, p.z, feetY);
    var cL = this._ceilingClearanceAt(p.x - eps, p.z, feetY);
    var cF = this._ceilingClearanceAt(p.x, p.z + eps, feetY);
    var cB = this._ceilingClearanceAt(p.x, p.z - eps, feetY);

    var cap = this.standHeight + this._skin;
    function fin(c) { return isFinite(c) ? c : cap; }
    var gx = fin(cR) - fin(cL);
    var gz = fin(cF) - fin(cB);
    var glen = Math.sqrt(gx * gx + gz * gz);
    if (glen < FPSC.EPS_DIR) {
        // Grounded: stop (forces a crouch). Airborne: let horizontal flow, ceiling slide owns vertical.
        return this.grounded ? { x: 0, z: 0 } : { x: vx, z: vz };
    }
    gx /= glen;
    gz /= glen;

    var into = vx * gx + vz * gz;
    if (into < 0) {
        vx -= into * gx;
        vz -= into * gz;
    }
    return { x: vx, z: vz };
};

// ---- Entity interface (authoritative snapshots / reconciliation) ----
// beginStep/endStep are above; getState/setState complete the duck-typed entity contract
// {beginStep, endStep, getState, setState} an external framework can drive.

/**
 * Snapshot this controller's authoritative state for the network.
 * @method getState
 */
proto.getState = function() {
    var p = this.body.position;
    var v = this.body.linear_velocity;
    return {
        x: p.x, y: p.y, z: p.z,
        vx: v.x, vy: v.y, vz: v.z,
        yaw: this.yaw, pitch: this.pitch,
        grounded: this.grounded,
        w: this.width, h: this.height,
        // The full authoritative movement state (see the "Movement state machine" comment above
        // endStep) — not just a sliding boolean, so resim re-adopts WALK/SLIP/SLIDE exactly, not a
        // flag it has to re-derive (and could re-derive differently than live prediction did).
        moveState: this._moveState,
        // Plain boolean, for snapshot consumers that only care about this one bit (e.g. a body
        // model tilting itself while sliding) and shouldn't need to know the full state enum above.
        sliding: this._moveState === FPSC.MOVE_SLIDE,
        // Jump/air transition timers; resim of an airborne phase must start from these or it
        // re-grounds / re-times the jump differently than the live prediction did.
        gs: this._groundSuppress,
        ct: this._coyoteTimer,
        jb: this._jumpBufferTimer,
        gnx: this.groundNormal.x, gny: this.groundNormal.y, gnz: this.groundNormal.z,
        // Steep-slope walk allowance can be granted/refused by an authority outside this controller.
        // Serialized so prediction + resim read the authoritative value, not a locally-flipped one
        // that rubber-bands.
        climb: this.climbSteepSlopes,
        // Ladder state: which branch beginStep takes next tick depends on this, so resim of a ladder
        // sequence must start from the authoritative on/off flag and face normal, not a locally
        // re-detected one.
        onLadder: this._onLadder,
        lnx: this._ladderNormal.x, lnz: this._ladderNormal.z,
        // NB: the ghost (the body that pushes objects) is deliberately NOT serialized. It's a local
        // follow-the-character construct; setState re-derives it locally by snapping it to the
        // authoritative character. Serializing it added bandwidth for identical results.
        userData: this.userData
    };
};

/**
 * Apply an authoritative state (from a snapshot). Sets position, velocity and grounded; does not
 * touch yaw/pitch. Used for reconciliation before replaying already-run commands.
 * @method setState
 */
proto.setState = function(s) {
    // Rebuild the collider at the authoritative center/height before adopting position, so the
    // geometry matches the snapshot's before replay (a height mismatch would re-plant crouch from
    // the wrong baseline every snapshot).
    if (s.h !== undefined && Math.abs(s.h - this.height) > FPSC.EPS_SPEED_MARGIN) {
        this.crouching = s.h < this.standHeight - FPSC.EPS_SPEED_MARGIN;
        this.height = s.h;
        this.eyeHeight = this.crouching ? this.standEye * this.crouchRatio : this.standEye;
        this._buildBody(new Goblin.Vector3(s.x, s.y, s.z));
    }
    this.body.position.set(s.x, s.y, s.z);
    this.body.updateDerived();
    var v = this.body.linear_velocity;
    v.x = s.vx;
    v.y = s.vy;
    v.z = s.vz;
    this.velocityY = s.vy;
    // _ownVelocityX/Z aren't snapshot fields — re-derive them from gb so they don't go stale (see
    // constructor comment).
    this._ownVelocityX = v.x - this._baseVelocity.x;
    this._ownVelocityZ = v.z - this._baseVelocity.z;
    if (s.grounded !== undefined) { this.grounded = s.grounded; }
    // Adopt the authoritative movement state directly — resim then starts from exactly the state
    // live prediction was in (WALK/SLIP/SLIDE/AIRBORNE/LADDER), not a locally re-derived guess.
    if (s.moveState !== undefined) { this._moveState = s.moveState; }
    if (s.gs !== undefined) { this._groundSuppress = s.gs; }
    if (s.ct !== undefined) { this._coyoteTimer = s.ct; }
    if (s.jb !== undefined) { this._jumpBufferTimer = s.jb; }
    if (s.gnx !== undefined) { this.groundNormal.set(s.gnx, s.gny, s.gnz); }
    // Adopt the authoritative steep-slope allowance. This is the ONLY place the live flag is written
    // from outside — a command only sets INTENT, an authority grants/refuses it, and the truth comes
    // back here. Read live each tick by the mover/grounding, so no rebuild is needed.
    if (s.climb !== undefined) { this.climbSteepSlopes = s.climb; }
    if (s.onLadder !== undefined) { this._onLadder = s.onLadder; }
    if (s.lnx !== undefined) { this._ladderNormal.set(s.lnx, 0, s.lnz); }
    // Re-baseline the ghost LOCALLY (not from the snapshot — the ghost isn't serialized). Snap it onto
    // the just-adopted authoritative character, moving at the character's velocity, so every resim starts
    // from the same consistent ghost state and re-pushes objects identically each time.
    // Opt-out (hardsnapGhostOnReconcile=false): leave the ghost drifted.
    if (this._ghost && this._hardsnapGhostOnReconcile) {
        var bp = this.body.position, pv = this.body.linear_velocity;
        this._ghost.position.set(bp.x, bp.y + (this._ghostGroundInset || 0) / 2, bp.z);
        this._ghost.linear_velocity.set(pv.x, pv.y, pv.z);
        this._ghostCommandedVel = { x: pv.x, y: pv.y, z: pv.z };
    }
    this._prevCrouch = this.crouching;
    if (s.userData !== undefined) { this.userData = s.userData; }
};

/**
 * Add a velocity impulse and force the character airborne (explosions / knockback / rocket-jumping).
 * @method applyKnockback
 */
proto.applyKnockback = function(vx, vy, vz) {
    var gb = this.body.linear_velocity;
    gb.x += vx;
    gb.y += vy;
    gb.z += vz;
    this.grounded = false;
    this._moveState = FPSC.MOVE_AIRBORNE;
    this._groundSuppress = 10;
    this.velocityY = gb.y;
};

// ---- Lifecycle ---------------------------------------------------------

/**
 * @method setPosition
 * @param {Goblin.Vector3} pos
 */
proto.setPosition = function(pos) {
    this.body.position.set(pos.x, pos.y, pos.z);
    this.body.updateDerived();
    this.body.linear_velocity.set(0, 0, 0);
    this.grounded = false;
    this._moveState = FPSC.MOVE_AIRBORNE;
    if (this._ghost) {
        var inset = this._ghostGroundInset || 0;
        this._ghost.position.set(pos.x, pos.y + inset / 2, pos.z);
        this._ghost.linear_velocity.set(0, 0, 0);
    }
};

/**
 * @method destroy
 */
proto.destroy = function() {
    this.world.removeRigidBody(this.body);
    this._destroyGhost();
};

return FPSCharacterController;

})();
