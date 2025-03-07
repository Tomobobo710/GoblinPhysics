/**
 * Physics-based character controller with advanced slope handling, movement projection, and state management.
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
 * @param {Number} [options.airAccleration=0.3] - How quickly to acclerate horizontally in air
 * * @param {Number} [options.groundAccleration=0.3] - How quickly to acclerate horizontally on ground
 * @param {Boolean} [options.allowYRotation=true] - Whether character can rotate around Y axis
 */
Goblin.CharacterController = function(world, options) {
    if (!world.broadphase) {
        throw new Error("CharacterController requires a GoblinPhysics world with a broadphase!");
    }

    this.world = world;
    options = options || {};

    // Create capsule shape
    var radius = Math.min((options.width || 4) * 0.5, (options.depth || 4) * 0.5);
    var totalHeight = options.height || 6;
    this.shape = new Goblin.CapsuleShape(radius, totalHeight);
    this.body = new Goblin.RigidBody(this.shape, options.mass || 1);

    // Configure rotation
    if (options.allowYRotation === true) {
        this.body.angular_factor = new Goblin.Vector3(0, 1, 0);
    } else {
        this.body.angular_factor = new Goblin.Vector3(0, 0, 0);
    }

    // Movement configuration
    this.moveSpeed = options.moveSpeed || 0.5;
    this.maxSpeed = options.maxSpeed || 50;
    this.stopFactor = options.stopFactor || 0.9;
    this.stoppingThreshold = options.stoppingThreshold || 0.1;
    this.jumpForce = options.jumpForce || 10;
    this.airAcceleration = options.airAccleration || 0.3;
    this.groundAcceleration = options.groundAccleration || 0.3;
    // Input handling
    this._inputDirection = new Goblin.Vector3();
    this._hasInputThisFrame = false;

    // Initialize working vectors
    this.contactNormal = new Goblin.Vector3(0, 1, 0);
    this.tempVector = new Goblin.Vector3();
    this.moveVector = new Goblin.Vector3();
    this.projectedMove = new Goblin.Vector3();

    // Ground spring config
    this.rideHeight = options.rideHeight || 4;
    this.rayLength = options.rayLength || this.body.shape.half_height;
    this.springStrength = options.springStrength || 1;
    this.springDamping = options.springDamping || 0.3;
    
    // State management
    this.states = {};
    this.currentState = null;
    this._lastStateChange = {
        from: null,
        to: null,
        time: Date.now()
    };

    // Debug tracking
    this._lastGroundHit = null;
    this._lastHeightError = null;
    this._lastSpringForce = null;
    this._lastMoveDelta = new Goblin.Vector3();
    this._lastProjectedMove = new Goblin.Vector3();

    // Initialize states and start in falling
    this._initializeStates();
    this.changeState('falling');
};

/**
 * Initializes all available character states and their behaviors
 *
 * @method _initializeStates
 * @private
 */
Goblin.CharacterController.prototype._initializeStates = function() {
    var self = this;  // Store reference to controller

    // Falling state
    this.states.falling = {
        name: 'falling',
        
        enter: function() {
            // Currently no special falling entry behavior
        }.bind(this),
        
        update: function(deltaTime) {
            this.updateGroundSpring();
            
            if (this._hasInputThisFrame) {
                this.move(this._inputDirection, deltaTime);
            }
            
            if (this._lastGroundHit) {
                return 'grounded';
            }
        }.bind(this),
        
        exit: function() {
            // Currently no special falling exit behavior
        }.bind(this)
    };

    this.states.grounded = {
        name: 'grounded',
        
        enter: function() {
            // No special entry behavior
        }.bind(this),
        
        update: function(deltaTime) {
            this.updateGroundSpring();
            
            // Handle jump input here - you'll need to add this input to your input system
            if (this._jumpRequested) {
                this._jumpRequested = false;  // Clear the request
                return 'jumping';
            }
            
            if (this._hasInputThisFrame) {
                this.move(this._inputDirection, deltaTime);
            } else {
                var currentHorizontalSpeed = Math.sqrt(
                    this.body.linear_velocity.x * this.body.linear_velocity.x +
                    this.body.linear_velocity.z * this.body.linear_velocity.z
                );

                if (currentHorizontalSpeed > this.stoppingThreshold) {
                    this.body.linear_velocity.x *= this.stopFactor;
                    this.body.linear_velocity.z *= this.stopFactor;
                } else {
                    this.body.linear_velocity.x = 0;
                    this.body.linear_velocity.z = 0;
                }
            }
            
            if (!this._lastGroundHit) {
                return 'falling';
            }
        }.bind(this),
        
        exit: function() {
            // No special exit behavior
        }.bind(this)
    };
    
    this.states.jumping = {
        name: 'jumping',
        
        enter: function() {
            // Zero out vertical velocity before jumping
            this.body.linear_velocity.y = 0;
            // Apply initial jump force
            this.body.linear_velocity.y = this.jumpForce;
        }.bind(this),
        
        update: function(deltaTime) {
            // Handle movement while jumping
            if (this._hasInputThisFrame) {
                this.move(this._inputDirection, deltaTime);
            }
            
            // Check for transition to falling
            if (this.body.linear_velocity.y <= 0) {
                return 'falling';
            }
        }.bind(this),
        
        exit: function() {
            // No special cleanup needed
        }.bind(this)
    };

};

/**
 * Requests a jump to be processed on next update
 *
 * @method wishJump
 */
Goblin.CharacterController.prototype.wishJump = function() {
    if (this.currentState && this.currentState.name === 'grounded') {
        this._jumpRequested = true;
    }
};

/**
 * Changes the current state of the character
 *
 * @method changeState
 * @param {String} newStateName - Name of state to transition to
 */
Goblin.CharacterController.prototype.changeState = function(newStateName) {
    var newState = this.states[newStateName];
    
    if (!newState) {
        throw new Error('Invalid state: ' + newStateName);
    }
    
    // Exit current state
    if (this.currentState) {
        this.currentState.exit.call(this);
    }
    
    // Record state change
    this._lastStateChange = {
        from: this.currentState ? this.currentState.name : null,
        to: newState.name,
        time: Date.now()
    };
    
    // Enter new state
    this.currentState = newState;
    this.currentState.enter.call(this);
};

/**
 * Stores input direction for processing during update
 *
 * @method handleInput
 * @param {Goblin.Vector3} direction - Normalized input direction
 */
Goblin.CharacterController.prototype.handleInput = function(direction) {
    if (direction && direction.lengthSquared() > 0) {
        this._inputDirection.copy(direction);
        this._hasInputThisFrame = true;
    } else {
        this._inputDirection.set(0, 0, 0);
        this._hasInputThisFrame = false;
    }
};

/**
 * Updates character physics and state
 * Should be called each frame after handleInput
 *
 * @method update
 * @param {Number} deltaTime - Time step in seconds
 */
Goblin.CharacterController.prototype.update = function(deltaTime) {
    if (this.currentState) {
        var nextState = this.currentState.update.call(this, deltaTime);
        if (nextState && nextState !== this.currentState.name) {
            this.changeState(nextState);
        }
    }
    
    // Clear input after update
    this._hasInputThisFrame = false;
};

/**
 * Updates ground spring forces to maintain ride height
 * Should be called each physics step while in FALLING or GROUNDED states
 *
 * @method updateGroundSpring
 * @private
 */
Goblin.CharacterController.prototype.updateGroundSpring = function() {
    var rayStart = new Goblin.Vector3(
        this.body.position.x,
        this.body.position.y - this.body.shape.half_height - 0.00001,
        this.body.position.z
    );

    var rayEnd = new Goblin.Vector3(
        rayStart.x, 
        rayStart.y - this.rayLength,
        rayStart.z
    );

    var hits = this.world.broadphase.rayIntersect(rayStart, rayEnd);

    if (hits.length > 0) {
        var hit = hits[0];
        this._lastGroundHit = hit;
        this.contactNormal.copy(hit.normal);

        var heightError = this.rideHeight - hit.t;
        var verticalVelocity = this.body.linear_velocity.y;
        
        var springForce = (heightError * this.springStrength) - 
                         (verticalVelocity * this.springDamping);

        this._lastHeightError = heightError;
        this._lastSpringForce = springForce;

        this.body.applyForce(new Goblin.Vector3(0, springForce, 0));

        for (var i = 0; i < hits.length; i++) {
            Goblin.ObjectPool.freeObject("RayIntersection", hits[i]);
        }
    } else {
        this._lastGroundHit = null;
        this._lastHeightError = null;
        this._lastSpringForce = null;
        this.contactNormal.set(0, 1, 0);
    }
};

/**
 * Projects and applies movement forces to the character, handling slopes
 *
 * @method move
 * @param {Goblin.Vector3} direction - Normalized input direction
 * @param {Number} deltaTime - Time step in seconds
 * @private
 */
Goblin.CharacterController.prototype.move = function(direction, deltaTime) {
    // Convert input to desired velocity
    // Convert input to desired velocity
    this.moveVector.copy(direction);
    this.moveVector.scale(this.moveSpeed);
    this._lastMoveDelta.copy(this.moveVector);

    if (this.currentState.name === 'falling' || this.currentState.name === 'jumping') {
        // In air - only affect horizontal velocity
        var currentY = this.body.linear_velocity.y;
        
        // Interpolate to target velocity
        this.body.linear_velocity.x += (this.moveVector.x - this.body.linear_velocity.x) * this.airAcceleration;
        this.body.linear_velocity.z += (this.moveVector.z - this.body.linear_velocity.z) * this.airAcceleration;
        this.body.linear_velocity.y = currentY;
    } else {
        // On ground - project along surface
        var dot = this.moveVector.dot(this.contactNormal);
        this.projectedMove.copy(this.moveVector);
        this.tempVector.copy(this.contactNormal);
        this.tempVector.scale(dot);
        this.projectedMove.subtract(this.tempVector);
        this._lastProjectedMove.copy(this.projectedMove);
        
        // Interpolate to target velocity
        this.body.linear_velocity.x += (this.projectedMove.x - this.body.linear_velocity.x) * this.groundAcceleration;
        this.body.linear_velocity.z += (this.projectedMove.z - this.body.linear_velocity.z) * this.groundAcceleration;
    }
    
    // Speed limit check as before
    var currentSpeed = Math.sqrt(
        this.body.linear_velocity.x * this.body.linear_velocity.x + 
        this.body.linear_velocity.z * this.body.linear_velocity.z
    );
    if (currentSpeed > this.maxSpeed) {
        var scale = this.maxSpeed / currentSpeed;
        this.body.linear_velocity.x *= scale;
        this.body.linear_velocity.z *= scale;
    }
};

/**
 * Gets comprehensive debug information about current movement state
 *
 * @method getDebugInfo
 * @return {Object} Debug state information including forces and contacts
 */
Goblin.CharacterController.prototype.getDebugInfo = function() {
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

    var movement = {
        input_direction: this._hasInputThisFrame ? {
            x: this._inputDirection.x,
            y: this._inputDirection.y,
            z: this._inputDirection.z
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
    
    var contact = {
        normal: {
            x: this.contactNormal.x,
            y: this.contactNormal.y,
            z: this.contactNormal.z
        },
        hit: this._lastGroundHit ? {
            point: this._lastGroundHit.point,
            normal: this._lastGroundHit.normal,
            distance: this._lastGroundHit.t
        } : null
    };

    var state = {
        current: this.currentState ? this.currentState.name : null,
        lastTransition: this._lastStateChange
    };

    return {
        physics: physics,
        movement: movement,
        spring: spring,
        state: state,
        contact: contact
    };
};

Goblin.EventEmitter.apply(Goblin.CharacterController);