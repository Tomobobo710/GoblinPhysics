/**
* Physics-based character controller with advanced slope handling and movement projection
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
* @param {Number} [options.slopeLimit=0.7] - Maximum slope normal.y value that can be climbed (0.7 ~= 45 degrees)
*/
Goblin.CharacterController = function(world, options) {
   options = options || {};

   var half_width = (options.width || 1) * 0.5;
   var half_height = (options.height || 2) * 0.5; 
   var half_depth = (options.depth || 1) * 0.5;

   /**
    * Box shape for character collision
    * @property shape
    * @type {Goblin.BoxShape}
    */
   this.shape = new Goblin.BoxShape(half_width, half_height, half_depth);

   /**
    * Physics body for character
    * @property body
    * @type {Goblin.RigidBody} 
    */
   this.body = new Goblin.RigidBody(this.shape, options.mass || 5);

   // Configure physics body
   this.body.linear_damping = 0.9;
   this.body.friction = 0.1;
   this.body.restitution = 0.0;
   this.body.angular_factor.set(0, 1, 0); // Only allow Y rotation
   
   if (!this.body.force) {
       this.body.force = new Goblin.Vector3();
   }
   if (!this.body.accumulated_force) {
       this.body.accumulated_force = new Goblin.Vector3();
   }

   /**
    * Movement properties
    */
   this.move_speed = options.moveSpeed || 5;
   this.jump_force = options.jumpForce || 300;
   this.slope_limit = options.slopeLimit || 0.7;
   this.contact_normal = new Goblin.Vector3(0, 1, 0);

   /**
    * Contact state tracking
    */
   this.can_jump = false;
   this.is_sliding = false;
   this.current_contacts = [];
   this.steepest_angle = 0;
   this.contact_count = 0;

   // Contact point tracking
   this._temp_point = new Goblin.Vector3();
   this._temp_normal = new Goblin.Vector3();
   this._movement_delta = new Goblin.Vector3();
   this._projected_movement = new Goblin.Vector3();

   // Contact tracking
   this.body.addListener('contact', this._handleContact.bind(this));
   this.body.addListener('endContact', this._handleEndContact.bind(this));
   this.body.addListener('endAllContact', this._handleEndAllContact.bind(this));

   world.addRigidBody(this.body);

   this.listeners = {};
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
* Updates movement state and handles physics
*
* @method move
* @param {Goblin.Vector3} direction - Normalized direction vector
* @param {Number} deltaTime - Time step in seconds
*/
Goblin.CharacterController.prototype.move = function(direction, deltaTime) {
    if (!direction) {
        direction = new Goblin.Vector3(0, 0, 0);
    }
    
    if (direction.lengthSquared() > 0) {
        // Create raw movement vector
        this._movement_delta.x = direction.x * this.move_speed;
        this._movement_delta.y = 0; 
        this._movement_delta.z = direction.z * this.move_speed;

        if (this.contact_count > 0) {
            // Simple vector projection
            var dot = this._movement_delta.dot(this.contact_normal);
            this._projected_movement.x = this._movement_delta.x - this.contact_normal.x * dot;
            this._projected_movement.y = this._movement_delta.y - this.contact_normal.y * dot;
            this._projected_movement.z = this._movement_delta.z - this.contact_normal.z * dot;
        } else {
            this._projected_movement.copy(this._movement_delta);
        }

        this.body.applyForce(this._projected_movement);
    }
};

/**
* Perform a jump if possible
* 
* @method jump
* @return {Boolean} Whether jump was performed
*/
Goblin.CharacterController.prototype.jump = function() {
   if (!this.can_jump) {
       return false;
   }

   // Calculate jump vector using contact normal for slope jumps
   var jump_vec = new Goblin.Vector3(
       this.contact_normal.x * this.jump_force * 0.1,
       this.jump_force,
       this.contact_normal.z * this.jump_force * 0.1
   );

   this.body.applyImpulse(jump_vec);
   this.can_jump = false;
   this.emit('jump');
   return true;
};

/**
* Handles new contacts
*
* @method _handleContact
* @private
* @param {Goblin.RigidBody} other_body - Contacted body
* @param {Goblin.ContactDetails} contact - Contact information
*/
Goblin.CharacterController.prototype._handleContact = function(other_body, contact) {
    this.contact_count++;
    this.contact_normal.copy(contact.contact_normal);
    this.can_jump = true;
    this.emit('grounded', true);
};

/**
* Handles losing a contact
*
* @method _handleEndContact
* @private
*/
Goblin.CharacterController.prototype._handleEndContact = function(other_body) {
   this.contact_count--;
   
   if (this.contact_count <= 0) {
       this.contact_normal.set(0, 1, 0);
       this.steepest_angle = 0;
       this.can_jump = false;
       this.is_sliding = false;
       this.emit('grounded', false);
   }
};

/**
* Handles losing all contacts
*
* @method _handleEndAllContact
* @private
*/
Goblin.CharacterController.prototype._handleEndAllContact = function() {
   this.contact_count = 0;
   this.contact_normal.set(0, 1, 0);
   this.steepest_angle = 0;
   this.can_jump = false;
   this.is_sliding = false;
   this.emit('grounded', false);
};

/**
* Gets current character state
*
* @method getState
* @return {Object} Current state object
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
* Resets character to a position
*
* @method resetPosition 
* @param {Goblin.Vector3} position - Position to reset to
*/
Goblin.CharacterController.prototype.resetPosition = function(position) {
   this.body.position.copy(position);
   this.body.linear_velocity.set(0, 0, 0);
   this.body.angular_velocity.set(0, 0, 0);
   this.body.force.set(0, 0, 0);
   this.body.accumulated_force.set(0, 0, 0);
   this.contact_normal.set(0, 1, 0);
   this.can_jump = false;
   this.is_sliding = false;
   this.contact_count = 0;
   this.steepest_angle = 0;
};

/**
 * Gets debug information about current movement state
 *
 * @method getDebugInfo
 * @return {Object} Debug state information
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