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
 * @param {Number} [options.mass=1] - Character mass in kg
 * @param {Number} [options.moveSpeed=5] - Base movement speed
 * @param {Boolean} [options.allowYRotation=true] - Whether character can rotate around Y axis
 */
Goblin.CharacterController = function(world, options) {
    if (!world.broadphase) {
        throw new Error('CharacterController requires a GoblinPhysics world with a broadphase!');
    }

    this.world = world;
    options = options || {};
    
    // Create box shape
    var half_width = (options.width || 4) * 0.5;
    var half_height = (options.height || 6) * 0.5;
    var half_depth = (options.depth || 4) * 0.5;
    this.shape = new Goblin.BoxShape(half_width, half_height, half_depth);
    this.body = new Goblin.RigidBody(this.shape, options.mass || 1);

    // Configure rotation 
    if (options.allowYRotation === true) {
        this.body.angular_factor = new Goblin.Vector3(0, 1, 0);
    } else {
        this.body.angular_factor = new Goblin.Vector3(0, 0, 0);
    }

    this.move_speed = options.moveSpeed || 0.5;

    // Initialize all vectors
    this.contact_normal = new Goblin.Vector3(0, 1, 0);
    this._temp_point = new Goblin.Vector3(0, 0, 0);
    this._temp_normal = new Goblin.Vector3(0, 0, 0);
    this._movement_delta = new Goblin.Vector3(0, 0, 0);
    this._projected_movement = new Goblin.Vector3(0, 0, 0);

    // Ground spring config
    this.ride_height = options.rideHeight || 4;
    this.ray_length = options.rayLength || this.body.shape.half_height;
    this.upward_strength = options.upwardStrength || 10;
    this.downward_strength = options.downwardStrength || 10;
    this.spring_damper = options.springDamper || 0.5;
    this.force_smoothing = options.forceSmoothing || 0.1;
    this._last_spring_force = 0;

    // Movement limits
    this.max_speed = options.maxSpeed || 200;
    this.stopLerp = options.stopLerp || 0;
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
		this.contact_normal.copy(hit.normal);
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
    } else {
		// No ground detected
		this.contact_normal.set(0, 1, 0); // Reset to up vector
	}
	
};

/**
 * Projects and applies movement forces to the character, handling slopes
 *
 * @method move 
 * @param {Goblin.Vector3} direction - Normalized input direction (typically from keyboard/gamepad)
 * @param {Number} deltaTime - Time step in seconds
 */
Goblin.CharacterController.prototype.move = function(direction, deltaTime) {
    this.updateGroundSpring();
    
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

        // Project movement onto contact surface for slope handling
        var dot = this._movement_delta.dot(this.contact_normal);
            
        this._projected_movement.x = this._movement_delta.x - this.contact_normal.x * dot;
        this._projected_movement.y = this._movement_delta.y - this.contact_normal.y * dot;
        this._projected_movement.z = this._movement_delta.z - this.contact_normal.z * dot;

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

Goblin.CharacterController.prototype.update = function(deltaTime) {
    //this.updateGroundSpring();
};

/**
 * Sets character to a specified position
 * Clears all physics state and movement data
 *
 * @method setPosition 
 * @param {Goblin.Vector3} position - Position to reset to
 */
Goblin.CharacterController.prototype.setPosition = function(position) {
    this.body.position.copy(position);
    
    // Reset all vectors to their default states
    this.body.linear_velocity.copy(this._temp_point);
    this.body.angular_velocity.copy(this._temp_point);
    this.body.force.copy(this._temp_point);
    this.body.accumulated_force.copy(this._temp_point);
    this.contact_normal.copy(this._temp_normal);
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

Goblin.EventEmitter.apply(Goblin.CharacterController);