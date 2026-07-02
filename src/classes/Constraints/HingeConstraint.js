/**
 * Constrains two bodies (or one body and the world) to rotate about a shared axis and pivot point,
 * like a door hinge: 3 rows lock relative position at the pivot, 2 more lock rotation to the single
 * degree of freedom about the hinge axis. Optionally bounded by a ConstraintLimit (swing angle) and
 * driven by a ConstraintMotor (powered rotation).
 *
 * @class HingeConstraint
 * @constructor
 * @param object_a {RigidBody} first body
 * @param hinge_a {Vector3} hinge axis, in object_a's local space
 * @param point_a {Vector3} pivot point, in object_a's local space
 * @param object_b {RigidBody} second body, or null/undefined to hinge object_a to the world
 * @param point_b {Vector3} pivot point in object_b's local space (only used when object_b is set)
 */
Goblin.HingeConstraint = function( object_a, hinge_a, point_a, object_b, point_b ) {
	Goblin.Constraint.call( this );

	this.object_a = object_a;
	this.hinge_a = hinge_a;
	this.point_a = point_a;

	this.initial_quaternion = new Goblin.Quaternion();

	this.object_b = object_b || null;
	this.point_b = new Goblin.Vector3();
	this.hinge_b = new Goblin.Vector3();
	if ( this.object_b != null ) {
		this.object_a.rotation.transformVector3Into( this.hinge_a, this.hinge_b );
		_tmp_quat4_1.invertQuaternion( this.object_b.rotation );
		_tmp_quat4_1.transformVector3( this.hinge_b );

		this.point_b = point_b;

		this.initial_quaternion.multiplyQuaternions( _tmp_quat4_1, this.object_a.rotation );
	} else {
		this.object_a.updateDerived(); // Ensure the body's transform is correct
		this.object_a.rotation.transformVector3Into( this.hinge_a, this.hinge_b );
		this.object_a.transform.transformVector3Into( this.point_a, this.point_b );
		this.initial_quaternion.set( this.object_a.rotation.x, this.object_a.rotation.y, this.object_a.rotation.z, this.object_a.rotation.w );
	}

	this.erp = 0.1;

	// Create rows
	// rows 0,1,2 are the same as point constraint and constrain the objects' positions
	// rows 3,4 introduce the rotational constraints which constrains angular velocity orthogonal to the hinge axis
	for ( var i = 0; i < 5; i++ ) {
		this.rows[i] = Goblin.ConstraintRow.createConstraintRow();
	}
};
Goblin.HingeConstraint.prototype = Object.create( Goblin.Constraint.prototype );

function removeConstraintLimitRow( constraint ) {
	if ( constraint.limit.constraint_row != null ) {
		var row_idx = constraint.rows.indexOf(constraint.limit.constraint_row);
		constraint.rows.splice(row_idx, 1);
		constraint.limit.constraint_row = null;
	}
}

function removeConstraintMotorRow( constraint ) {
	if ( constraint.motor.constraint_row != null ) {
		var row_idx = constraint.rows.indexOf(constraint.motor.constraint_row);
		constraint.rows.splice(row_idx, 1);
		constraint.motor.constraint_row = null;
	}
}

/**
 * Adds or removes this hinge's limit row depending on whether the current swing angle about
 * `world_axis` violates `this.limit`. Lazily allocates the row on first violation and drops it once
 * the limit is no longer active, so an unlimited or currently-satisfied hinge costs nothing extra.
 *
 * @method updateLimits
 * @param world_axis {Vector3} the hinge axis, already transformed into world space
 * @param time_delta {Number} the step's time delta, in seconds
 */
Goblin.HingeConstraint.prototype.updateLimits = function( world_axis, time_delta ) {
	if ( this.limit.enabled === false ) {
		// remove existing `constraint_row` if it was previously set
		removeConstraintLimitRow( this );
		return;
	}

	var separating_angle, correction;

	if ( this.object_b == null ) {
		// this.initial_quaternion is the original rotation of object_a
		separating_angle = this.initial_quaternion.signedAngleBetween( this.object_a.rotation, world_axis );
	} else {
		// this.initial_quaternion is the original difference in rotation between object_a and object_b (A - B)
		_tmp_quat4_1.invertQuaternion( this.object_b.rotation );
		_tmp_quat4_1.multiply( this.object_a.rotation );

		separating_angle = this.initial_quaternion.signedAngleBetween( _tmp_quat4_1, world_axis );
	}

	if (
		( this.limit.limit_lower == null || this.limit.limit_lower < separating_angle ) &&
		( this.limit.limit_upper == null || this.limit.limit_upper > separating_angle )
	) {
		// there limit is not violated, ignore
		removeConstraintLimitRow( this );
		return;
	}

	if ( this.limit.limit_lower != null && separating_angle <= this.limit.limit_lower ) {
		if ( this.limit.constraint_row == null ) {
			this.limit.createConstraintRow();
			this.limit.constraint_row.upper_limit = 0;
			this.rows.push( this.limit.constraint_row );
		}
		this.limit.constraint_row.jacobian[3] = -world_axis.x;
		this.limit.constraint_row.jacobian[4] = -world_axis.y;
		this.limit.constraint_row.jacobian[5] = -world_axis.z;

		if ( this.object_b != null ) {
			this.limit.constraint_row.jacobian[9] = world_axis.x;
			this.limit.constraint_row.jacobian[10] = world_axis.y;
			this.limit.constraint_row.jacobian[11] = world_axis.z;
		}

		correction = separating_angle - this.limit.limit_lower;
		this.limit.constraint_row.bias = correction * this.limit.erp / time_delta;
	} else if ( this.limit.limit_upper != null && separating_angle >= this.limit.limit_upper ) {
		if ( this.limit.constraint_row == null ) {
			this.limit.createConstraintRow();
			this.limit.constraint_row.lower_limit = 0;
			this.rows.push( this.limit.constraint_row );
		}
		this.limit.constraint_row.jacobian[3] = -world_axis.x;
		this.limit.constraint_row.jacobian[4] = -world_axis.y;
		this.limit.constraint_row.jacobian[5] = -world_axis.z;

		if ( this.object_b != null ) {
			this.limit.constraint_row.jacobian[9] = world_axis.x;
			this.limit.constraint_row.jacobian[10] = world_axis.y;
			this.limit.constraint_row.jacobian[11] = world_axis.z;
		}

		correction = separating_angle - this.limit.limit_upper;
		this.limit.constraint_row.bias = correction * this.limit.erp / time_delta;
	}
};

/**
 * Adds or removes this hinge's motor row depending on whether `this.motor` is enabled, and (when
 * enabled) updates its target speed and torque limit for this step.
 *
 * @method updateMotor
 * @param world_axis {Vector3} the hinge axis, already transformed into world space
 */
Goblin.HingeConstraint.prototype.updateMotor = function( world_axis ) {
	if ( this.motor.enabled === false ) {
		removeConstraintMotorRow( this );
		return;
	}

	if ( this.motor.constraint_row == null ) {
		this.motor.createConstraintRow();
		this.rows.push( this.motor.constraint_row );
		this.motor.constraint_row.jacobian[3] = world_axis.x;
		this.motor.constraint_row.jacobian[4] = world_axis.y;
		this.motor.constraint_row.jacobian[5] = world_axis.z;

		if ( this.object_b != null ) {
			this.motor.constraint_row.jacobian[9] = -world_axis.x;
			this.motor.constraint_row.jacobian[10] = -world_axis.y;
			this.motor.constraint_row.jacobian[11] = -world_axis.z;
		}
	}

	this.motor.constraint_row.bias = this.motor.max_speed;
	if ( this.motor.max_speed >= 0 ) {
		this.motor.constraint_row.lower_limit = 0;
		this.motor.constraint_row.upper_limit = this.motor.torque;
	} else {
		this.motor.constraint_row.lower_limit = -this.motor.torque;
		this.motor.constraint_row.upper_limit = 0;
	}
};

/**
 * Recomputes the hinge's rows from current body state: the 3 positional rows and 2 rotational rows
 * described in the constructor, plus their bias terms for positional/angular error correction, and
 * then delegates to `updateLimits`/`updateMotor` for the optional limit/motor rows.
 *
 * @method update
 * @param time_delta {Number} the step's time delta, in seconds
 */
Goblin.HingeConstraint.prototype.update = (function(){
	var r1 = new Goblin.Vector3(),
		r2 = new Goblin.Vector3(),
		t1 = new Goblin.Vector3(),
		t2 = new Goblin.Vector3(),
		world_axis = new Goblin.Vector3();

	return function( time_delta ) {
		this.object_a.rotation.transformVector3Into( this.hinge_a, world_axis );

		this.object_a.transform.transformVector3Into( this.point_a, _tmp_vec3_1 );
		r1.subtractVectors( _tmp_vec3_1, this.object_a.position );

		// 0,1,2 are positional, same as PointConstraint
		this.rows[0].jacobian[0] = -1;
		this.rows[0].jacobian[1] = 0;
		this.rows[0].jacobian[2] = 0;
		this.rows[0].jacobian[3] = 0;
		this.rows[0].jacobian[4] = -r1.z;
		this.rows[0].jacobian[5] = r1.y;

		this.rows[1].jacobian[0] = 0;
		this.rows[1].jacobian[1] = -1;
		this.rows[1].jacobian[2] = 0;
		this.rows[1].jacobian[3] = r1.z;
		this.rows[1].jacobian[4] = 0;
		this.rows[1].jacobian[5] = -r1.x;

		this.rows[2].jacobian[0] = 0;
		this.rows[2].jacobian[1] = 0;
		this.rows[2].jacobian[2] = -1;
		this.rows[2].jacobian[3] = -r1.y;
		this.rows[2].jacobian[4] = r1.x;
		this.rows[2].jacobian[5] = 0;

		// 3,4 are rotational, constraining motion orthogonal to axis
		world_axis.findOrthogonal( t1, t2 );
		this.rows[3].jacobian[3] = -t1.x;
		this.rows[3].jacobian[4] = -t1.y;
		this.rows[3].jacobian[5] = -t1.z;

		this.rows[4].jacobian[3] = -t2.x;
		this.rows[4].jacobian[4] = -t2.y;
		this.rows[4].jacobian[5] = -t2.z;

		if ( this.object_b != null ) {
			this.object_b.transform.transformVector3Into( this.point_b, _tmp_vec3_2 );
			r2.subtractVectors( _tmp_vec3_2, this.object_b.position );

			// 0,1,2 are positional, same as PointConstraint
			this.rows[0].jacobian[6] = 1;
			this.rows[0].jacobian[7] = 0;
			this.rows[0].jacobian[8] = 0;
			this.rows[0].jacobian[9] = 0;
			this.rows[0].jacobian[10] = r2.z;
			this.rows[0].jacobian[11] = -r2.y;

			this.rows[1].jacobian[6] = 0;
			this.rows[1].jacobian[7] = 1;
			this.rows[1].jacobian[8] = 0;
			this.rows[1].jacobian[9] = -r2.z;
			this.rows[1].jacobian[10] = 0;
			this.rows[1].jacobian[11] = r2.x;

			this.rows[2].jacobian[6] = 0;
			this.rows[2].jacobian[7] = 0;
			this.rows[2].jacobian[8] = 1;
			this.rows[2].jacobian[9] = r2.y;
			this.rows[2].jacobian[10] = -r2.z;
			this.rows[2].jacobian[11] = 0;

			// 3,4 are rotational, constraining motion orthogonal to axis
			this.rows[3].jacobian[9] = t1.x;
			this.rows[3].jacobian[10] = t1.y;
			this.rows[3].jacobian[11] = t1.z;

			this.rows[4].jacobian[9] = t2.x;
			this.rows[4].jacobian[10] = t2.y;
			this.rows[4].jacobian[11] = t2.z;
		} else {
			_tmp_vec3_2.copy( this.point_b );
		}

		// Linear error correction
		_tmp_vec3_3.subtractVectors( _tmp_vec3_1, _tmp_vec3_2 );
		_tmp_vec3_3.scale( this.erp / time_delta );
		this.rows[0].bias = _tmp_vec3_3.x;
		this.rows[1].bias = _tmp_vec3_3.y;
		this.rows[2].bias = _tmp_vec3_3.z;

		// Angular error correction
		if (this.object_b != null) {
			this.object_a.rotation.transformVector3Into(this.hinge_a, _tmp_vec3_1);
			this.object_b.rotation.transformVector3Into(this.hinge_b, _tmp_vec3_2);
			_tmp_vec3_1.cross(_tmp_vec3_2);
			this.rows[3].bias = -_tmp_vec3_1.dot(t1) * this.erp / time_delta;
			this.rows[4].bias = -_tmp_vec3_1.dot(t2) * this.erp / time_delta;
		} else {
			this.rows[3].bias = this.rows[4].bias = 0;
		}

		// limits & motor
		this.updateLimits( world_axis, time_delta );
		this.updateMotor( world_axis );
	};
})( );