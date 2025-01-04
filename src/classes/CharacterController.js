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

    options = options || {};
    
    // Initialize default vectors for consistent zero and up directions
    options.defaultVectors = options.defaultVectors || {};
    options.defaultVectors.up = options.defaultVectors.up || { x: 0, y: 1, z: 0 };
    options.defaultVectors.zero = options.defaultVectors.zero || { x: 0, y: 0, z: 0 };

    // Physics shape dimensions - scaled by dimensionScale for collision shape creation
    // dimensionScale allows fine-tuning of the collision shape relative to visual size
    var dimensionScale = options.dimensionScale || 0.5;
    var half_width = (options.width || 1) * dimensionScale;
    var half_height = (options.height || 2) * dimensionScale; 
    var half_depth = (options.depth || 1) * dimensionScale;

    /**
     * Box shape for character collision
     * @property shape
     * @type {Goblin.BoxShape}
     * @private
     */
    this.shape = new Goblin.BoxShape(half_width, half_height, half_depth);

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
    if (options.allowYRotation !== false) {
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
     * Jump force magnitude
     * @property jump_force
     * @type {Number}
     */
    this.jump_force = options.jumpForce || 300;

    /**
     * Factor applied to horizontal components during jumps
     * @property horizontal_jump_factor
     * @type {Number}
     * @private
     */
    this.horizontal_jump_factor = options.horizontalJumpFactor || 0.1;

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
    this.can_jump = false;
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
    // Default to zero vector if no direction provided using stored defaults
    if (!direction) {
        direction = new Goblin.Vector3(
            this._temp_point.x,
            this._temp_point.y,
            this._temp_point.z
        );
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
    if (!this.can_jump) {
        return false;
    }

    // Calculate jump vector using contact normal and configured forces
    var jump_vec = new Goblin.Vector3(
        this.contact_normal.x * this.jump_force * this.horizontal_jump_factor,
        this.jump_force,
        this.contact_normal.z * this.jump_force * this.horizontal_jump_factor
    );

    this.body.applyImpulse(jump_vec);
    this.can_jump = false;
    this.emit('jump');
    return true;
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