/**
 * Every tunable default for FPSCharacterController, in ONE place, grouped by subsystem. The
 * controller reads each default from here (constructor: `o.walkSpeed !== undefined ?
 * o.walkSpeed : Goblin.FPS_CONTROLLER_DEFAULTS.movement.walkSpeed`), so a caller can still
 * override any single value per-instance via the options object — this is only the fallback.
 *
 * What is NOT here (on purpose): algorithm-internal epsilons/thresholds inside the collision +
 * slope math (1e-4 guards, normal.y classifications, sub-step fractions) — those are
 * implementation details, not feel knobs, and stay at their use site inside
 * FPSCharacterController.js (the FPSC object).
 *
 * @class FPS_CONTROLLER_DEFAULTS
 * @static
 */
Goblin.FPS_CONTROLLER_DEFAULTS = {
    // ---- Collider dimensions + mass (pre-scale "base" values; _applyScale multiplies at runtime) ----
    dimensions: {
        width: 0.6,
        depth: 0.6,
        height: 1.8,
        mass: 10,
        eyeHeightRatio: 0.42, // eyeHeight default = height * this (overridable directly via o.eyeHeight)
        crouchRatio: 0.55,    // crouched collider height as a fraction of standing height
    },

    // ---- Ground movement. Three gaits: walk (held modifier) < move/run (default) < sprint. ----
    movement: {
        walkSpeed: 3.8,        // deliberate slow gait (held walk modifier)
        moveSpeed: 7,          // RUN speed — the no-modifier default
        sprintSpeed: 11.5,     // top gait
        crouchSpeedMult: 0.5,  // multiplies whichever gait is active while crouched (unitless)
        sprintDecay: 10,       // units/sec bleed of excess speed after releasing sprint (Infinity = instant)
        groundStopDecel: 80,   // units/sec decel when all move keys released (idle stop)
        airControl: 0.12,      // steering authority while airborne (0..1)
        friction: 0,           // body friction (0 keeps wall-slides clean; kinematic grounding holds slopes)
    },

    // ---- Jump + forgiveness windows ----
    jump: {
        jumpSpeed: 4.6,
        stepHeight: 0.4,       // max ledge height the mover steps up onto (base/1x; scales linearly with player scale)
        stepDownDist: 0.5,     // max drop the mover snaps down to keep grounded
        coyoteTime: 0.1,       // sec after leaving a ledge a jump still registers
        jumpBuffer: 0.12,      // sec before landing a jump press is remembered and fires on touchdown
    },

    // ---- Slopes ----
    slopes: {
        maxSlopeAngle: 45.57,  // max standable slope, degrees (>=90 disables the limit)
        climbSteepSlopes: false, // can the player ascend a too-steep slope by walking into it
    },

    // ---- Slide (crouch-at-speed) ----
    slide: {
        enabled: true,
        requiresMoveInput: false, // also require a movement key held (not crouch alone)
        minSpeed: 7.8,         // speed at/above which a crouch launches a slide
        endSpeed: 1,           // slide ends when speed bleeds below this
        friction: 6,           // flat-ground slide friction
        boost: 1.3,            // launch speed multiplier
        control: 0.14,         // steering authority while sliding (0..1)
        slopeAccel: 1.5,       // downhill acceleration factor while sliding
        slopeMin: 0.2,         // sin(angle) at/above which the slide is gravity-governed (Infinity disables)
        slopeFriction: 1.5,    // cross-slope friction while gravity-sliding
        coyoteFrames: 5,       // frames after dropping below slide speed a crouch still launches a slide
    },

    // ---- Ghost: the solver body that trails the player and pushes objects (see _syncGhost) ----
    // maxSpeed / maxDampSpeed default to sprintSpeed * this multiplier (kept as a ratio so they scale with
    // the character's speed tuning); override with an absolute units/sec value via options if desired.
    ghost: {
        maxSpeedSprintMult: 1.3,     // ghost chase-speed cap = sprintSpeed * this
        maxDampSpeedSprintMult: 1.3, // damping-term cap = sprintSpeed * this
        damping: 1.0,          // fraction of the ghost's velocity opposed each tick (0..1)
        stiffness: 0.9,        // 0..1 blend toward the gap-closing velocity each tick
        pushMassBaseMult: 35,  // objects heavier than mass * this block like a wall; lighter yield proportionally
    },

    // ---- Knockback: how the player RECEIVES a push from an object (see _readGhostKnockback) ----
    knockback: {
        receivePush: true,        // gate the whole knockback path
        maxSpeed: 16,             // cap on received knockback speed
        knockbackFraction: 1.0,   // scale received knockback
        selfPush: false,          // false = only an object with its OWN inbound momentum knocks you (no self-push
                                  //         oscillation); true = legacy relative-closing gate (oscillates)
    },

    // ---- Netcode / prediction behavior for the ghost (both default ON; false reverts to older behavior) ----
    netcode: {
        driveGhostDuringResim: true,    // run the ghost drive during rollback resim (off = objects rubber-band)
        hardsnapGhostOnReconcile: true, // snap ghost onto authority on setState (off = objects oscillate)
    },

    // ---- View / aim ----
    view: {
        yaw: 0,
        pitch: 0,
        maxPitch: 1.5,         // clamp, radians
    },

    // ---- Render (sub-tick eye interpolation) ----
    render: {
        snapDist: 0.8,         // per-tick eye jump (units) above which the interp snaps instead of sliding
    },

    // ---- Misc identity defaults (not feel knobs, but kept here so nothing is scattered) ----
    misc: {
        color: "#cc4444",
        visible: false,        // the collider body is invisible by default (a model/render layer draws the player)
        bodyName: "fpsControllerBody",
        scale: 1,              // 1 = no scaling
        spawn: { x: 0, y: 2, z: 0 }, // fallback spawn when no position is passed
    },
};
