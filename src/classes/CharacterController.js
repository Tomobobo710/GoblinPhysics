/**
 * Physics-based character controller with advanced slope handling and movement projection.
 * Provides smooth movement along surfaces while respecting physics constraints.
 *
 * @class CharacterController
 * @constructor 
 * @param {Goblin.World} world - Physics world instance
 * @param {Object} [options] - Configuration options
 * @param {Number} [options.width=1] - Character width in meters
 * @param {Number} [options.height=2] - Character height in meters
 * @param {Number} [options.depth=1] - Character depth in meters 
 * @param {Number} [options.mass=5] - Character mass in kg
 * @param {Number} [options.moveSpeed=5] - Base movement speed
 * @param {Number} [options.jumpForce=300] - Jump impulse force
 * @param {Number} [options.linearDamping=0.9] - How quickly linear velocity decreases (0-1)
 * @param {Number} [options.friction=0.1] - Surface friction coefficient
 * @param {Number} [options.restitution=0] - Bounciness coefficient (0-1)
 * @param {Boolean} [options.allowYRotation=true] - Whether character can rotate around Y axis
 * @param {Number} [options.dimensionScale=0.5] - Scale factor applied to width/height/depth for collision shape
 * @param {Number} [options.horizontalJumpFactor=0.1] - Multiplier for horizontal component of jump force
 * @param {Object} [options.defaultVectors] - Default vector configurations
 * @param {Object} [options.defaultVectors.up] - Default up vector
 * @param {Number} [options.defaultVectors.up.x=0] - X component of up vector
 * @param {Number} [options.defaultVectors.up.y=1] - Y component of up vector
 * @param {Number} [options.defaultVectors.up.z=0] - Z component of up vector
 * @param {Object} [options.defaultVectors.zero] - Default zero vector
 * @param {Number} [options.defaultVectors.zero.x=0] - X component of zero vector
 * @param {Number} [options.defaultVectors.zero.y=0] - Y component of zero vector
 * @param {Number} [options.defaultVectors.zero.z=0] - Z component of zero vector
 */
Goblin.CharacterController = function(world, options) {
    if (!world) {
        throw new Error('CharacterController requires a physics world');
    }
    if (!world.broadphase) {
        throw new Error('CharacterController requires a world with a broadphase');
    }

    // Store world reference
    this.world = world;
    // Store broadphase reference
    this.broadphase = world.broadphase;
    
    options = options || {};
    
    // Initialize default vectors for consistent zero and up directions
    options.defaultVectors = options.defaultVectors || {};
    options.defaultVectors.up = options.defaultVectors.up || { x: 0, y: 1, z: 0 };
    options.defaultVectors.zero = options.defaultVectors.zero || { x: 0, y: 0, z: 0 };

    // Physics shape dimensions - scaled by dimensionScale for collision shape creation
    var dimensionScale = options.dimensionScale || 0.5;
    var radius = ((options.width || 1) * dimensionScale) / 2; // Use width for radius
    var half_height = (options.height || 2) * dimensionScale;

    /**
     * Cylinder shape for character collision
     * @property shape
     * @type {Goblin.CylinderShape}
     * @private
     */
    this.shape = new Goblin.CylinderShape(radius, half_height);

    /**
     * Physics body for character
     * @property body
     * @type {Goblin.RigidBody} 
     */
    this.body = new Goblin.RigidBody(this.shape, options.mass || 5);

    // Configure physics body properties using provided options or defaults
    this.body.linear_damping = options.linearDamping || 0.9;
    this.body.friction = options.friction || 0.1;
    this.body.restitution = options.restitution || 0;
	// Configure rotation constraints using the default vectors
	if (options.allowYRotation === true) {  // Changed from !== false
		// Allow only Y-axis rotation for natural character turning
		this.body.angular_factor.set(
			options.defaultVectors.zero.x,
			1, // Y-axis rotation enabled
			options.defaultVectors.zero.z
		);
	} else {
		// Lock all rotation axes for more rigid control
		this.body.angular_factor.set(
			options.defaultVectors.zero.x,
			options.defaultVectors.zero.y,
			options.defaultVectors.zero.z
		);
	}

    // Initialize force vectors if not present
    if (!this.body.force) {
        this.body.force = new Goblin.Vector3(
            options.defaultVectors.zero.x,
            options.defaultVectors.zero.y,
            options.defaultVectors.zero.z
        );
    }
    if (!this.body.accumulated_force) {
        this.body.accumulated_force = new Goblin.Vector3(
            options.defaultVectors.zero.x,
            options.defaultVectors.zero.y,
            options.defaultVectors.zero.z
        );
    }

    /**
     * Movement properties
     * @property move_speed
     * @type {Number}
     */
    this.move_speed = options.moveSpeed || 5;

   /**
     * Jump-related properties
     */
    this.jump_force = options.jumpForce || 30;
    this.jump_cooldown = options.jumpCooldown || 0.3; // seconds before spring reactivates
    this.jump_velocity_threshold = options.jumpVelocityThreshold || -2; // vertical velocity at which we consider falling
    this._is_jumping = false;
    this._jump_timer = 0;
    
    // Store the original downward strength to restore it later
    this._original_downward_strength = this.downward_strength;

    /**
     * Current contact surface normal
     * @property contact_normal
     * @type {Goblin.Vector3}
     * @private
     */
    this.contact_normal = new Goblin.Vector3(
        options.defaultVectors.up.x,
        options.defaultVectors.up.y,
        options.defaultVectors.up.z
    );

    // Movement state tracking
    this.contact_count = 0;

    // Initialize temporary vectors for physics calculations using default zeros
    this._temp_point = new Goblin.Vector3(
        options.defaultVectors.zero.x,
        options.defaultVectors.zero.y,
        options.defaultVectors.zero.z
    );
    this._temp_normal = new Goblin.Vector3(
        options.defaultVectors.zero.x,
        options.defaultVectors.zero.y,
        options.defaultVectors.zero.z
    );
    this._movement_delta = new Goblin.Vector3(
        options.defaultVectors.zero.x,
        options.defaultVectors.zero.y,
        options.defaultVectors.zero.z
    );
    this._projected_movement = new Goblin.Vector3(
        options.defaultVectors.zero.x,
        options.defaultVectors.zero.y,
        options.defaultVectors.zero.z
    );

    // Set up contact event listeners
    this.body.addListener('contact', this._handleContact.bind(this));
    this.body.addListener('endContact', this._handleEndContact.bind(this));
    this.body.addListener('endAllContact', this._handleEndAllContact.bind(this));

    // Add body to physics world
    world.addRigidBody(this.body);

    // Initialize event listener container
    this.listeners = {};
	/**
     * Ground following configuration
     * @private
     */
    this.ride_height = options.rideHeight || 0.5;
    this.ray_length = options.rayLength || this.body.shape.half_height; // Ray checks from bottom of character
    
    // Separate spring strengths for up/down forces
    this.upward_strength = options.upwardStrength || 0.1;
    this.downward_strength = options.downwardStrength || 0.01; // Gentler downward force
    
    // Spring damping and smoothing
    this.spring_damper = options.springDamper || 1;  // Reduces oscillation
    this.force_smoothing = options.forceSmoothing || 0.9; // 0-1, higher = smoother
    
    // Keep track of spring state
    this._last_spring_force = 0;
    this._accumulated_force = new Goblin.Vector3(0, 0, 0);
	
	/**
     * Maximum speed the character can move
     * @property max_speed
     * @type {Number}
     */
    this.max_speed = options.maxSpeed || 200; // Default max speed of 10
	
	/**
     * How quickly the character stops when no input is received
     * 0 = instant stop, 0-1 = smooth deceleration
     * @property stopLerp
     * @type {Number}
     */
    this.stopLerp = options.stopLerp || 0; // 0 = instant stop

    /**
     * Minimum speed threshold before coming to a complete stop
     * @property stopThreshold
     * @type {Number}
     */
    this.stopThreshold = options.stopThreshold || 0.1;
};

/**
 * Updates ground spring forces to maintain ride height
 * Should be called each physics step
 * 
 * @method updateGroundSpring
 * @private
 */
Goblin.CharacterController.prototype.updateGroundSpring = function() {
    // Cast ray from bottom of character
    var rayStart = new Goblin.Vector3(
        this.body.position.x,
        this.body.position.y - this.body.shape.half_height,
        this.body.position.z
    );
    
    var rayEnd = new Goblin.Vector3(
        rayStart.x,
        rayStart.y - this.ray_length,
        rayStart.z
    );

    var intersections = this.world.broadphase.rayIntersect(rayStart, rayEnd);
    
    if (intersections.length > 0) {
		
		
        var hit = intersections[0];

        // Get velocities for damping
        var characterVel = this.body.linear_velocity;
        var groundVel = hit.object ? hit.object.linear_velocity : new Goblin.Vector3(0, 0, 0);
        
        // Relative vertical velocity for damping
        var relVel = characterVel.y - groundVel.y;

        // Calculate height error from desired ride height
        var heightError = this.ride_height - hit.t;
        
        // Choose strength based on direction needed
        var strength = heightError > 0 ? this.upward_strength : this.downward_strength;
        
        // Calculate raw spring force with damping
        var rawSpringForce = (heightError * strength) - (relVel * this.spring_damper);
        
        // Smooth the force transition using lerp
        var smoothedForce = this._last_spring_force + 
            (rawSpringForce - this._last_spring_force) * (1 - this.force_smoothing);
            
        // Store for next frame's smoothing
        this._last_spring_force = smoothedForce;

        // Apply the smoothed force
        var force = new Goblin.Vector3(0, smoothedForce, 0);
        this.body.applyForce(force);
        
        // Apply equal and opposite force to ground object if it's dynamic
        if (hit.object && hit.object._mass !== Infinity) {
			hit.object.applyForceAtWorldPoint(
				new Goblin.Vector3(0, -smoothedForce, 0),
				hit.point
			);
		}
		
        // Store debug values
        this._lastHitDistance = hit.t;
        this._lastHeightError = heightError;
        this._lastSpringForce = smoothedForce;

        // Clean up ray intersections
        for (var i = 0; i < intersections.length; i++) {
            Goblin.ObjectPool.freeObject('RayIntersection', intersections[i]);
        }
    }
};


/**
 * Projects a movement vector onto a contact plane for slope handling
 *
 * @method projectOnPlane
 * @param {Goblin.Vector3} vector - Vector to project
 * @param {Goblin.Vector3} normal - Normal of the plane to project onto
 * @return {Goblin.Vector3} The projected vector
 * @private
 */
Goblin.CharacterController.prototype.projectOnPlane = function(vector, normal) {
    // Calculate dot product for projection
    var dot = vector.dot(normal);
    
    // Return projected vector components
    return new Goblin.Vector3(
        vector.x - normal.x * dot,
        vector.y - normal.y * dot,
        vector.z - normal.z * dot
    );
};

/**
 * Projects and applies movement forces to the character, handling slopes
 *
 * @method move 
 * @param {Goblin.Vector3} direction - Normalized input direction (typically from keyboard/gamepad)
 * @param {Number} deltaTime - Time step in seconds
 */
Goblin.CharacterController.prototype.move = function(direction, deltaTime) {
    // Update ground spring forces
    //this.updateGroundSpring();
    
    // If no input, handle stopping
    if (!direction || direction.lengthSquared() === 0) {
        if (this.stopLerp === 0) {
            // Instant stop - zero out horizontal velocity
            this.body.linear_velocity.x = 0;
            this.body.linear_velocity.z = 0;
        } else {
            // Smooth deceleration
            var currentStoppingSpeed = Math.sqrt(
                this.body.linear_velocity.x * this.body.linear_velocity.x + 
                this.body.linear_velocity.z * this.body.linear_velocity.z
            );
            
            if (currentStoppingSpeed > this.stopThreshold) {
                var stopFactor = Math.pow(1 - this.stopLerp, deltaTime);
                this.body.linear_velocity.x *= stopFactor;
                this.body.linear_velocity.z *= stopFactor;
            } else {
                // Below threshold, come to complete stop
                this.body.linear_velocity.x = 0;
                this.body.linear_velocity.z = 0;
            }
        }
        return;
    }
    
    // Only process movement if we have actual input
    if (direction.lengthSquared() > 0) {
        // Convert normalized input direction into movement force
        this._movement_delta.x = direction.x * this.move_speed;
        this._movement_delta.y = this._temp_point.y; // Zero out vertical component
        this._movement_delta.z = direction.z * this.move_speed;

        if (this.contact_count > 0) {
            // Project movement onto contact surface for slope handling
            var dot = this._movement_delta.dot(this.contact_normal);
            
            this._projected_movement.x = this._movement_delta.x - this.contact_normal.x * dot;
            this._projected_movement.y = this._movement_delta.y - this.contact_normal.y * dot;
            this._projected_movement.z = this._movement_delta.z - this.contact_normal.z * dot;
        } else {
            // Use raw movement when not in contact
            this._projected_movement.copy(this._movement_delta);
        }

        // Apply speed limit before applying force
        var currentSpeed = this.body.linear_velocity.length();
        if (currentSpeed > this.max_speed) {
            // Scale down velocity to max speed
            var scale = this.max_speed / currentSpeed;
            this.body.linear_velocity.scale(scale);
        }

        // Apply final movement force
        this.body.applyForce(this._projected_movement);
    }
};

Goblin.EventEmitter.apply(Goblin.CharacterController);

/**
* Projects a movement vector onto a contact plane
*
* @method projectOnPlane
* @param {Goblin.Vector3} vector - Vector to project
* @param {Goblin.Vector3} normal - Normal of the plane to project onto
* @return {Goblin.Vector3} The projected vector
* @private
*/
Goblin.CharacterController.prototype.projectOnPlane = function(vector, normal) {
   var dot = vector.dot(normal);
   return new Goblin.Vector3(
       vector.x - normal.x * dot,
       vector.y - normal.y * dot,
       vector.z - normal.z * dot
   );
};

/**
 * Performs a jump if character is in a valid jumping state
 * Applies jump force modified by surface normal for realistic slope jumps
 * 
 * @method jump
 * @return {Boolean} Whether jump was performed
 */
Goblin.CharacterController.prototype.jump = function() {
    // Check if we're close enough to the ground to jump
    if (this._lastHitDistance && this._lastHitDistance < this.ride_height * 1.5) {
        // Disable downward spring force
        this.downward_strength = 0;
        
        // Apply jump impulse
        var jump_vec = new Goblin.Vector3(0, this.jump_force, 0);
        this.body.applyImpulse(jump_vec);
        
        this._is_jumping = true;
        this._jump_timer = this.jump_cooldown;
        
        this.emit('jump');
        return true;
    }
    return false;
};


Goblin.CharacterController.prototype.update = function(deltaTime) {
    if (this._is_jumping) {
        this._jump_timer -= deltaTime;
        
        // Check if we should end jump state
        if (this._jump_timer <= 0 || this.body.linear_velocity.y < this.jump_velocity_threshold) {
            this._is_jumping = false;
            this.downward_strength = this._original_downward_strength;
        }
    }
    
    // The regular spring update can continue
    this.updateGroundSpring();
};

/**
 * Handles new contacts between character and other objects
 * Updates contact normal for movement projection and slope handling
 *
 * @method _handleContact
 * @private
 * @param {Goblin.RigidBody} other_body - The body we're contacting
 * @param {Goblin.ContactDetails} contact - Details about the contact point
 */
Goblin.CharacterController.prototype._handleContact = function(other_body, contact) {
    this.contact_count++;
    this.contact_normal.copy(contact.contact_normal);
    this.can_jump = true;
    this.emit('grounded', true);
};

/**
 * Handles losing a single contact point
 * Updates state when character loses contact with a surface
 *
 * @method _handleEndContact
 * @private
 */
Goblin.CharacterController.prototype._handleEndContact = function(other_body) {
    this.contact_count--;
    
    if (this.contact_count <= 0) {
        // Reset to default up vector when losing all contacts
        this.contact_normal.copy(this._temp_normal);
        this.steepest_angle = 0;
        this.can_jump = false;
        this.is_sliding = false;
        this.emit('grounded', false);
    }
};

/**
 * Handles losing all contacts simultaneously
 * Resets character state to default airborne configuration
 *
 * @method _handleEndAllContact
 * @private
 */
Goblin.CharacterController.prototype._handleEndAllContact = function() {
    this.contact_count = 0;
    this.contact_normal.copy(this._temp_normal);
    this.steepest_angle = 0;
    this.can_jump = false;
    this.is_sliding = false;
    this.emit('grounded', false);
};

/**
 * Gets current character state for debugging or state management
 *
 * @method getState
 * @return {Object} Current state object containing physics and gameplay properties
 */
Goblin.CharacterController.prototype.getState = function() {
    return {
        position: this.body.position,
        velocity: this.body.linear_velocity,
        rotation: this.body.rotation,
        grounded: this.can_jump,
        sliding: this.is_sliding,
        contact_normal: this.contact_normal,
        contact_count: this.contact_count
    };
};

/**
 * Resets character to a specified position
 * Clears all physics state and movement data
 *
 * @method resetPosition 
 * @param {Goblin.Vector3} position - Position to reset to
 */
Goblin.CharacterController.prototype.resetPosition = function(position) {
    this.body.position.copy(position);
    
    // Reset all vectors to their default states
    this.body.linear_velocity.copy(this._temp_point);
    this.body.angular_velocity.copy(this._temp_point);
    this.body.force.copy(this._temp_point);
    this.body.accumulated_force.copy(this._temp_point);
    this.contact_normal.copy(this._temp_normal);
    
    // Reset state flags
    this.can_jump = false;
    this.is_sliding = false;
    this.contact_count = 0;
    this.steepest_angle = 0;
};

/**
 * Gets comprehensive debug information about current movement state
 * Useful for debugging physics behavior and movement issues
 *
 * @method getDebugInfo
 * @return {Object} Debug state information including forces and contacts
 */
Goblin.CharacterController.prototype.getDebugInfo = function() {
    return {
        movement: {
            raw_force: this.body.force ? {
                x: this.body.force.x,
                y: this.body.force.y,
                z: this.body.force.z
            } : null,
            projected_movement: this._projected_movement ? {
                x: this._projected_movement.x,
                y: this._projected_movement.y,
                z: this._projected_movement.z
            } : null,
            movement_delta: this._movement_delta ? {
                x: this._movement_delta.x,
                y: this._movement_delta.y,
                z: this._movement_delta.z
            } : null
        },
        spring: {
            hit_distance: this._lastHitDistance || null,
            height_error: this._lastHeightError || null,
            spring_force: this._lastSpringForce || null,
            target_height: this.ride_height,
            spring_strength: this.spring_strength,
            spring_damper: this.spring_damper
        },
        contact: {
            normal: {
                x: this.contact_normal.x,
                y: this.contact_normal.y,
                z: this.contact_normal.z
            },
            can_jump: this.can_jump,
            is_sliding: this.is_sliding,
            contact_count: this.contact_count,
            steepest_angle: this.steepest_angle
        },
        physics: {
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
        }
    };
};