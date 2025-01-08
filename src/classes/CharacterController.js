/**
 * Physics-based character controller with advanced slope handling and movement projection.
 *
 * @class CharacterController 
 * @constructor
 * @param {Goblin.World} world - Physics world instance
 * @param {Object} [options] - Configuration options
 * @param {Number} [options.width=4] - Character width in meters
 * @param {Number} [options.height=6] - Character height in meters
 * @param {Number} [options.depth=4] - Character depth in meters
 * @param {Number} [options.mass=1] - Character mass in kg
 * @param {Number} [options.moveSpeed=0.5] - Base movement speed in meters/second
 * @param {Number} [options.maxSpeed=50] - Maximum movement speed in meters/second
 * @param {Number} [options.stopFactor=0.9] - Deceleration factor when stopping (0-1)
 * @param {Number} [options.stoppingThreshold=0.1] - Speed below which movement stops completely
 * @param {Number} [options.rideHeight=4] - Desired height above ground in meters
 * @param {Number} [options.rayLength] - Length of ground detection ray (defaults to shape half-height)
 * @param {Number} [options.springStrength=1] - Ground spring force multiplier
 * @param {Number} [options.springDamping=0.3] - Ground spring damping factor
 * @param {Boolean} [options.allowYRotation=false] - Whether character can rotate around Y axis
 */
Goblin.CharacterController = function(world, options) {
    if (!world.broadphase) {
        throw new Error("CharacterController requires a GoblinPhysics world with a broadphase!");
    }

    /**
     * Reference to the physics world
     * 
     * @property world
     * @type {Goblin.World}
     */
    this.world = world;
    options = options || {};

    /**
     * Box shape representing the character's collision volume
     *
     * @property shape
     * @type {Goblin.BoxShape}
     */
    var half_width = (options.width || 4) * 0.5;
    var half_height = (options.height || 6) * 0.5;
    var half_depth = (options.depth || 4) * 0.5;
    this.shape = new Goblin.BoxShape(half_width, half_height, half_depth);

    /**
     * Rigid body for physics simulation
     *
     * @property body
     * @type {Goblin.RigidBody}
     */
    this.body = new Goblin.RigidBody(this.shape, options.mass || 1);

    // Configure rotation constraints
    if (options.allowYRotation === true) {
        this.body.angular_factor = new Goblin.Vector3(0, 1, 0);
    } else {
        this.body.angular_factor = new Goblin.Vector3(0, 0, 0);
    }

    /**
     * Base movement speed in meters/second
     *
     * @property moveSpeed
     * @type {Number}
     * @default 0.5
     */
    this.moveSpeed = options.moveSpeed || 0.5;

    /**
     * Maximum movement speed in meters/second
     *
     * @property maxSpeed
     * @type {Number}
     * @default 50
     */
    this.maxSpeed = options.maxSpeed || 50;

    /**
     * Deceleration factor applied when stopping (0-1)
     *
     * @property stopFactor
     * @type {Number}
     * @default 0.9
     */
    this.stopFactor = options.stopFactor || 0.9;

    /**
     * Speed threshold below which movement stops completely
     *
     * @property stoppingThreshold
     * @type {Number}
     * @default 0.1
     */
    this.stoppingThreshold = options.stoppingThreshold || 0.1;

    /**
     * Current ground contact normal
     *
     * @property contactNormal
     * @type {Goblin.Vector3}
     * @default [0, 1, 0]
     */
    this.contactNormal = new Goblin.Vector3(0, 1, 0);

    /**
     * Temporary vector used for calculations
     *
     * @property tempVector
     * @type {Goblin.Vector3}
     * @private
     */
    this.tempVector = new Goblin.Vector3();

    /**
     * Raw movement vector before projection
     *
     * @property moveVector
     * @type {Goblin.Vector3}
     * @private
     */
    this.moveVector = new Goblin.Vector3();

    /**
     * Movement vector after projection onto ground plane
     *
     * @property projectedMove
     * @type {Goblin.Vector3}
     * @private
     */
    this.projectedMove = new Goblin.Vector3();

    /**
     * Desired height above ground in meters
     *
     * @property rideHeight
     * @type {Number}
     * @default 4
     */
    this.rideHeight = options.rideHeight || 4;

    /**
     * Length of ground detection ray
     *
     * @property rayLength
     * @type {Number}
     */
    this.rayLength = options.rayLength || this.body.shape.half_height;

    /**
     * Ground spring force multiplier
     *
     * @property springStrength
     * @type {Number}
     * @default 1
     */
    this.springStrength = options.springStrength || 1;

    /**
     * Ground spring damping factor
     *
     * @property springDamping
     * @type {Number}
     * @default 0.3
     */
    this.springDamping = options.springDamping || 0.3;

    /**
     * Last ground ray hit result for debugging
     *
     * @property _lastGroundHit
     * @type {RayIntersection|null}
     * @private
     */
    this._lastGroundHit = null;

    /**
     * Last height error for debugging
     *
     * @property _lastHeightError
     * @type {Number|null}
     * @private
     */
    this._lastHeightError = null;

    /**
     * Last applied spring force for debugging
     *
     * @property _lastSpringForce
     * @type {Number|null}
     * @private
     */
    this._lastSpringForce = null;

    /**
     * Last raw movement delta for debugging
     *
     * @property _lastMoveDelta
     * @type {Goblin.Vector3}
     * @private
     */
    this._lastMoveDelta = new Goblin.Vector3();

    /**
     * Last projected movement for debugging
     *
     * @property _lastProjectedMove
     * @type {Goblin.Vector3}
     * @private
     */
    this._lastProjectedMove = new Goblin.Vector3();
};

/**
 * Updates ground spring forces to maintain desired ride height.
 * Casts a ray downward to detect ground and applies spring forces based on height error.
 * Stores debug information about spring calculations and ground contact.
 *
 * @method updateGroundSpring
 * @private
 */
Goblin.CharacterController.prototype.updateGroundSpring = function() {
    /**
     * Ray start position, slightly below character's bottom center
     * Small offset prevents numerical issues with ground detection
     */
    var rayStart = new Goblin.Vector3(
        this.body.position.x,
        this.body.position.y - this.body.shape.half_height - 0.00001,
        this.body.position.z
    );

    /**
     * Ray end position at maximum detection distance
     */
    var rayEnd = new Goblin.Vector3(
        rayStart.x, 
        rayStart.y - this.rayLength,
        rayStart.z
    );

    // Perform ground detection ray cast
    var hits = this.world.broadphase.rayIntersect(rayStart, rayEnd);

    if (hits.length > 0) {
        // Store closest hit for ground contact
        var hit = hits[0];
        this._lastGroundHit = hit;
        
        // Update contact normal for slope calculations
        this.contactNormal.copy(hit.normal);

        // Calculate spring force based on height error and velocity
        var heightError = this.rideHeight - hit.t;
        var verticalVelocity = this.body.linear_velocity.y;
        
        var springForce = (heightError * this.springStrength) - 
                         (verticalVelocity * this.springDamping);

        // Store debug information
        this._lastHeightError = heightError;
        this._lastSpringForce = springForce;

        // Apply vertical spring force
        this.body.applyForce(new Goblin.Vector3(0, springForce, 0));

        // Return ray intersections to object pool
        for (var i = 0; i < hits.length; i++) {
            Goblin.ObjectPool.freeObject("RayIntersection", hits[i]);
        }
    } else {
        // No ground contact - reset debug info and use default up vector
        this._lastGroundHit = null;
        this._lastHeightError = null;
        this._lastSpringForce = null;
        this.contactNormal.set(0, 1, 0);
    }
};

/**
 * Projects and applies movement forces to the character, handling slopes and speed limits.
 * Movement is projected onto the ground contact plane to handle slopes naturally.
 * Stores debug information about movement calculations and applied forces.
 *
 * @method move
 * @param {Goblin.Vector3} direction - Normalized input movement direction vector
 * @param {Number} deltaTime - Time step in seconds
 */
Goblin.CharacterController.prototype.move = function(direction, deltaTime) {
    /**
     * Store raw input direction for debugging
     */
    this._lastInputDirection = new Goblin.Vector3();
    if (direction) {
        this._lastInputDirection.copy(direction);
    }

    // Update ground contact and spring forces
    this.updateGroundSpring();

    // Handle stopping when no input is provided
    if (!direction || direction.lengthSquared() === 0) {
        // Calculate current horizontal speed
        var currentHorizontalSpeed = Math.sqrt(
            this.body.linear_velocity.x * this.body.linear_velocity.x +
            this.body.linear_velocity.z * this.body.linear_velocity.z
        );

        // Apply deceleration or complete stop
        if (currentHorizontalSpeed > this.stoppingThreshold) {
            // Gradually reduce velocity using stop factor
            this.body.linear_velocity.x *= this.stopFactor;
            this.body.linear_velocity.z *= this.stopFactor;
        } else {
            // Below threshold - complete stop
            this.body.linear_velocity.x = 0;
            this.body.linear_velocity.z = 0;
        }
        return;
    }

    /**
     * Convert input direction to movement force vector
     */
    this.moveVector.copy(direction);
    this.moveVector.scale(this.moveSpeed);
    this._lastMoveDelta.copy(this.moveVector);

    /**
     * Project movement onto ground contact plane
     * This handles slope movement by removing the surface normal component
     */
    var dot = this.moveVector.dot(this.contactNormal);
    this.projectedMove.copy(this.moveVector);
    
    // Remove normal component to get movement parallel to surface
    this.tempVector.copy(this.contactNormal);
    this.tempVector.scale(dot);
    this.projectedMove.subtract(this.tempVector);
    
    // Store projected movement for debugging
    this._lastProjectedMove.copy(this.projectedMove);

    /**
     * Store final applied force for debugging
     */
    this._lastAppliedForce = new Goblin.Vector3();
    this._lastAppliedForce.copy(this.projectedMove);

    /**
     * Apply speed limiting if necessary
     */
    var currentSpeed = this.body.linear_velocity.length();
    if (currentSpeed > this.maxSpeed) {
        var scale = this.maxSpeed / currentSpeed;
        this.body.linear_velocity.scale(scale);
    }

    /**
     * Apply final projected movement force
     */
    this.body.applyForce(this.projectedMove);
};

/**
 * Resets the character's position and clears all physics state.
 * Zeros out all velocities, forces, and movement tracking data.
 * Used when teleporting or respawning the character.
 *
 * @method setPosition
 * @param {Goblin.Vector3} position - New world position to place the character
 */
Goblin.CharacterController.prototype.setPosition = function(position) {
    /**
     * Copy new position to rigid body
     */
    this.body.position.copy(position);

    /**
     * Reset all physics velocities to zero
     */
    this.body.linear_velocity.set(0, 0, 0);
    this.body.angular_velocity.set(0, 0, 0);

    /**
     * Clear all accumulated forces
     */
    this.body.force.set(0, 0, 0);
    this.body.accumulated_force.set(0, 0, 0);

    /**
     * Reset contact normal to default up vector
     */
    this.contactNormal.set(0, 1, 0);

    /**
     * Clear all debug tracking data
     */
    this._lastGroundHit = null;
    this._lastHeightError = null;
    this._lastSpringForce = null;
    this._lastMoveDelta.set(0, 0, 0);
    this._lastProjectedMove.set(0, 0, 0);
};

/**
 * Returns detailed debug information about the character's current state.
 * Includes physics state, movement calculations, spring behavior, and ground contact.
 * Useful for debugging physics behavior and movement issues.
 *
 * @method getDebugInfo
 * @return {Object} Debug state information object
 * @return {Object} .physics - Physics body state
 * @return {Object} .physics.position - Current world position
 * @return {Object} .physics.velocity - Current linear velocity
 * @return {Object} .movement - Movement calculation data
 * @return {Object} .movement.input_direction - Last raw input direction
 * @return {Object} .movement.raw_move - Last movement vector before projection
 * @return {Object} .movement.projected_move - Last movement vector after slope projection
 * @return {Object} .movement.applied_force - Last force actually applied to body
 * @return {Object} .spring - Ground spring calculations
 * @return {Number} .spring.hit_distance - Distance to ground from last ray cast
 * @return {Number} .spring.height_error - Difference between current and desired height
 * @return {Number} .spring.spring_force - Last calculated spring force
 * @return {Number} .spring.target_height - Desired ride height
 * @return {Number} .spring.spring_strength - Current spring strength setting
 * @return {Number} .spring.spring_damping - Current spring damping setting
 * @return {Object} .spring.ray_start - Ground detection ray start position
 * @return {Object} .spring.ray_end - Ground detection ray end position
 * @return {Object} .contact - Ground contact information
 * @return {Object} .contact.normal - Current ground contact normal
 * @return {Object} .contact.hit - Detailed ground hit information if available
 */
Goblin.CharacterController.prototype.getDebugInfo = function() {
    /**
     * Gather physics body state
     */
    var physics = {
        position: {
            x: this.body.position.x,
            y: this.body.position.y,
            z: this.body.position.z
        },
        velocity: {
            x: this.body.linear_velocity.x,
            y: this.body.linear_velocity.y,
            z: this.body.linear_velocity.z
        }
    };

    /**
     * Gather movement calculation data
     */
    var movement = {
        input_direction: this._lastInputDirection ? {
            x: this._lastInputDirection.x,
            y: this._lastInputDirection.y,
            z: this._lastInputDirection.z
        } : null,
        raw_move: {
            x: this._lastMoveDelta.x,
            y: this._lastMoveDelta.y,
            z: this._lastMoveDelta.z
        },
        projected_move: {
            x: this._lastProjectedMove.x,
            y: this._lastProjectedMove.y,
            z: this._lastProjectedMove.z
        },
        applied_force: this._lastAppliedForce ? {
            x: this._lastAppliedForce.x,
            y: this._lastAppliedForce.y,
            z: this._lastAppliedForce.z
        } : null
    };

    /**
     * Gather spring system state
     */
    var spring = {
        hit_distance: this._lastGroundHit ? this._lastGroundHit.t : null,
        height_error: this._lastHeightError,
        spring_force: this._lastSpringForce,
        target_height: this.rideHeight,
        spring_strength: this.springStrength,
        spring_damping: this.springDamping,
        ray_start: {
            x: this.body.position.x,
            y: this.body.position.y - this.body.shape.half_height,
            z: this.body.position.z
        },
        ray_end: {
            x: this.body.position.x,
            y: (this.body.position.y - this.body.shape.half_height) - this.rayLength,
            z: this.body.position.z
        }
    };

    /**
     * Gather ground contact data
     */
    var contact = {
        normal: {
            x: this.contactNormal.x,
            y: this.contactNormal.y,
            z: this.contactNormal.z
        },
        hit: this._lastGroundHit ? {
            point: {
                x: this._lastGroundHit.point.x,
                y: this._lastGroundHit.point.y,
                z: this._lastGroundHit.point.z
            },
            normal: {
                x: this._lastGroundHit.normal.x,
                y: this._lastGroundHit.normal.y,
                z: this._lastGroundHit.normal.z
            },
            distance: this._lastGroundHit.t
        } : null
    };

    return {
        physics: physics,
        movement: movement,
        spring: spring,
        contact: contact
    };
};

Goblin.EventEmitter.apply(Goblin.CharacterController);