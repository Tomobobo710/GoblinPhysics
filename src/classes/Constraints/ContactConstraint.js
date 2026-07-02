/**
 * The non-penetration half of a contact: a single-row, one-sided ([0, Infinity]) constraint along
 * the contact normal that prevents two bodies from interpenetrating, with restitution folded into
 * its bias. Always built and solved alongside a FrictionConstraint for the same ContactDetails -
 * see IterativeSolver.processContactManifolds.
 *
 * @class ContactConstraint
 * @constructor
 */
Goblin.ContactConstraint = function() {
	Goblin.Constraint.call( this );

	this.contact = null;
};
Goblin.ContactConstraint.prototype = Object.create( Goblin.Constraint.prototype );

/**
 * Initializes this constraint from a ContactDetails: sets object_a/object_b, wires a listener so
 * the constraint deactivates itself if the contact is destroyed, and builds the initial row.
 *
 * @method buildFromContact
 * @param contact {ContactDetails} the contact this constraint enforces
 */
Goblin.ContactConstraint.prototype.buildFromContact = function( contact ) {
	this.object_a = contact.object_a;
	this.object_b = contact.object_b;
	this.contact = contact;

	var self = this;
	var onDestroy = function() {
		this.removeListener( 'destroy', onDestroy );
		self.deactivate();
	};
	this.contact.addListener( 'destroy', onDestroy );

	var row = this.rows[0] || Goblin.ObjectPool.getObject( 'ConstraintRow' );
	row.lower_limit = 0;
	row.upper_limit = Infinity;
	this.rows[0] = row;

	this.update();
};

/**
 * Recomputes the constraint's single row from current body/contact state: the normal-direction
 * jacobian for both bodies, the restitution bias from relative velocity at the contact point, and
 * the perpendicular-lever correction (see `_bleedPerpLever`) that removes rounding-scale phantom
 * torque on a near-centered contact.
 *
 * @method update
 */

Goblin.ContactConstraint.prototype.update = function() {
	var row = this.rows[0];

	if ( this.object_a == null || this.object_a._mass === Infinity ) {
		row.jacobian[0] = row.jacobian[1] = row.jacobian[2] = 0;
		row.jacobian[3] = row.jacobian[4] = row.jacobian[5] = 0;
	} else {
		row.jacobian[0] = -this.contact.contact_normal.x;
		row.jacobian[1] = -this.contact.contact_normal.y;
		row.jacobian[2] = -this.contact.contact_normal.z;

		_tmp_vec3_1.subtractVectors( this.contact.contact_point, this.contact.object_a.position );
		_tmp_vec3_1.cross( this.contact.contact_normal );
		row.jacobian[3] = -_tmp_vec3_1.x;
		row.jacobian[4] = -_tmp_vec3_1.y;
		row.jacobian[5] = -_tmp_vec3_1.z;
	}

	if ( this.object_b == null || this.object_b._mass === Infinity ) {
		row.jacobian[6] = row.jacobian[7] = row.jacobian[8] = 0;
		row.jacobian[9] = row.jacobian[10] = row.jacobian[11] = 0;
	} else {
		row.jacobian[6] = this.contact.contact_normal.x;
		row.jacobian[7] = this.contact.contact_normal.y;
		row.jacobian[8] = this.contact.contact_normal.z;

		_tmp_vec3_1.subtractVectors( this.contact.contact_point, this.contact.object_b.position );
		_tmp_vec3_1.cross( this.contact.contact_normal );
		row.jacobian[9] = _tmp_vec3_1.x;
		row.jacobian[10] = _tmp_vec3_1.y;
		row.jacobian[11] = _tmp_vec3_1.z;
	}

	// Pre-calc error
	row.bias = 0;

	// Apply restitution, from each body's velocity at the world contact point. The lever arm is
	// world-space (contact_point - position), matching the angular jacobian above; a body-local
	// anchor is not a world lever once the body has rotated, and using one lets a rolling body's
	// spin leak into the normal velocity as phantom restitution.
	var velocity_along_normal = 0;
	if ( this.object_a._mass !== Infinity ) {
		_tmp_vec3_2.subtractVectors( this.contact.contact_point, this.object_a.position );
		_tmp_vec3_1.crossVectors( this.object_a.angular_velocity, _tmp_vec3_2 );
		_tmp_vec3_1.add( this.object_a.linear_velocity );
		velocity_along_normal += _tmp_vec3_1.dot( this.contact.contact_normal );
	}
	if ( this.object_b._mass !== Infinity ) {
		_tmp_vec3_2.subtractVectors( this.contact.contact_point, this.object_b.position );
		_tmp_vec3_1.crossVectors( this.object_b.angular_velocity, _tmp_vec3_2 );
		_tmp_vec3_1.add( this.object_b.linear_velocity );
		velocity_along_normal -= _tmp_vec3_1.dot( this.contact.contact_normal );
	}

	// Add restitution to bias
	row.bias += velocity_along_normal * this.contact.restitution;

	// Remove the normal impulse's phantom torque for a near-centered contact
	Goblin.ContactConstraint._bleedPerpLever( this.rows[0], 3, this.contact.contact_point, this.object_a, this.contact.contact_normal );
	Goblin.ContactConstraint._bleedPerpLever( this.rows[0], 9, this.contact.contact_point, this.object_b, this.contact.contact_normal );
};

/**
 * Perpendicular lever-arm length below which a contact is treated as centered under the body and its
 * normal-impulse torque is zeroed. Above it the contact is genuinely off-center and left untouched.
 *
 * @property PERP_GATE
 * @type {Number}
 * @static
 */
Goblin.ContactConstraint.PERP_GATE = 1e-4;

/**
 * Zeroes the phantom torque a normal impulse would apply through a near-centered contact's lever arm.
 * The normal row's angular jacobian is (contact_point - body.position) x contact_normal; its component
 * perpendicular to the normal is the torque lever. When that lever is within `PERP_GATE` the contact is
 * centered under the body and the lever is floating-point noise, so it is dropped (the jacobian becomes
 * the along-normal cross product, which is zero). Genuinely off-center contacts have a larger lever and
 * are left unchanged.
 *
 * @method _bleedPerpLever
 * @param row {ConstraintRow} the normal constraint row to correct
 * @param jbase {Number} angular jacobian offset for the body ( 3 for object_a, 9 for object_b )
 * @param contact_point {Vector3} world-space contact point
 * @param body {RigidBody} the body whose lever arm is measured
 * @param normal {Vector3} world-space contact normal
 * @static
 * @private
 */
Goblin.ContactConstraint._bleedPerpLever = (function(){
	var r = new Goblin.Vector3(), along_vec = new Goblin.Vector3(), lever = new Goblin.Vector3();
	return function( row, jbase, contact_point, body, normal ) {
		if ( body == null || body._mass === Infinity ) {
			return;
		}
		r.subtractVectors( contact_point, body.position );
		var along = r.x * normal.x + r.y * normal.y + r.z * normal.z;
		along_vec.scaleVector( normal, along );

		var px = r.x - along_vec.x, py = r.y - along_vec.y, pz = r.z - along_vec.z;
		var plen = Math.sqrt( px * px + py * py + pz * pz );
		if ( plen < 1e-12 || plen > Goblin.ContactConstraint.PERP_GATE ) {
			return;
		}

		lever.crossVectors( along_vec, normal );
		var sign = ( jbase === 3 ) ? -1 : 1;
		row.jacobian[ jbase ]     = sign * lever.x;
		row.jacobian[ jbase + 1 ] = sign * lever.y;
		row.jacobian[ jbase + 2 ] = sign * lever.z;
	};
})();