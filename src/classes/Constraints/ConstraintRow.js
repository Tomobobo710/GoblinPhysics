/**
 * One scalar row of a Constraint's velocity-level equation: `jacobian . v = bias`, solved by the
 * IterativeSolver as a 1D LCP bounded by `lower_limit`/`upper_limit`. `jacobian` packs both
 * bodies' linear and angular coefficients into 12 slots (object_a: [0..2] linear, [3..5] angular;
 * object_b: [6..8] linear, [9..11] angular). A Constraint may own several rows (e.g. a
 * HingeConstraint's 5 positional + rotational rows).
 *
 * @class ConstraintRow
 * @constructor
 */
Goblin.ConstraintRow = function() {
	this.jacobian = new Float64Array( 12 );
	this.B = new Float64Array( 12 ); // `B` is the jacobian multiplied by the objects' inverted mass & inertia tensors
	this.D = 0; // Length of the jacobian

	this.lower_limit = -Infinity;
	this.upper_limit = Infinity;

	this.bias = 0;
	this.multiplier = 0;
	this.multiplier_cached = 0;
	this.eta = 0;
	this.eta_row = new Float64Array( 12 );
};

/**
 * Fetches a ConstraintRow from the object pool and resets it to a fresh, unbounded, zero-jacobian
 * state, ready for a constraint to populate. Preferred over `new Goblin.ConstraintRow()` in the
 * per-step solve path to avoid churn.
 *
 * @method createConstraintRow
 * @return {ConstraintRow} a pooled row reset to defaults
 * @static
 */
Goblin.ConstraintRow.createConstraintRow = function() {
	var row =  Goblin.ObjectPool.getObject( 'ConstraintRow' );
	row.lower_limit = -Infinity;
	row.upper_limit = Infinity;
	row.bias = 0;

	row.jacobian[0] = row.jacobian[1] = row.jacobian[2] =
	row.jacobian[3] = row.jacobian[4] = row.jacobian[5] =
	row.jacobian[6] = row.jacobian[7] = row.jacobian[8] =
	row.jacobian[9] = row.jacobian[10] = row.jacobian[11] = 0;

	return row;
};

/**
 * Computes `B`, the jacobian pre-multiplied by each body's inverse mass and inverse inertia tensor
 * (and clamped by its linear/angular factor). `B` is the row's effective impulse-per-unit-lambda;
 * it's reused by `computeD` and by the solver's per-iteration impulse application.
 *
 * @method computeB
 * @param constraint {Constraint} the owning constraint, for its object_a/object_b
 */
Goblin.ConstraintRow.prototype.computeB = function( constraint ) {
	var invmass;

	if ( constraint.object_a != null && constraint.object_a._mass !== Infinity ) {
		invmass = constraint.object_a._mass_inverted;

		this.B[0] = invmass * this.jacobian[0] * constraint.object_a.linear_factor.x;
		this.B[1] = invmass * this.jacobian[1] * constraint.object_a.linear_factor.y;
		this.B[2] = invmass * this.jacobian[2] * constraint.object_a.linear_factor.z;

		_tmp_vec3_1.x = this.jacobian[3];
		_tmp_vec3_1.y = this.jacobian[4];
		_tmp_vec3_1.z = this.jacobian[5];
		constraint.object_a.inverseInertiaTensorWorldFrame.transformVector3( _tmp_vec3_1 );
		this.B[3] = _tmp_vec3_1.x * constraint.object_a.angular_factor.x;
		this.B[4] = _tmp_vec3_1.y * constraint.object_a.angular_factor.y;
		this.B[5] = _tmp_vec3_1.z * constraint.object_a.angular_factor.z;
	} else {
		this.B[0] = this.B[1] = this.B[2] = 0;
		this.B[3] = this.B[4] = this.B[5] = 0;
	}

	if ( constraint.object_b != null && constraint.object_b._mass !== Infinity ) {
		invmass = constraint.object_b._mass_inverted;
		this.B[6] = invmass * this.jacobian[6] * constraint.object_b.linear_factor.x;
		this.B[7] = invmass * this.jacobian[7] * constraint.object_b.linear_factor.y;
		this.B[8] = invmass * this.jacobian[8] * constraint.object_b.linear_factor.z;

		_tmp_vec3_1.x = this.jacobian[9];
		_tmp_vec3_1.y = this.jacobian[10];
		_tmp_vec3_1.z = this.jacobian[11];
		constraint.object_b.inverseInertiaTensorWorldFrame.transformVector3( _tmp_vec3_1 );
		this.B[9] = _tmp_vec3_1.x * constraint.object_b.linear_factor.x;
		this.B[10] = _tmp_vec3_1.y * constraint.object_b.linear_factor.y;
		this.B[11] = _tmp_vec3_1.z * constraint.object_b.linear_factor.z;
	} else {
		this.B[6] = this.B[7] = this.B[8] = 0;
		this.B[9] = this.B[10] = this.B[11] = 0;
	}
};

/**
 * Computes `D`, the effective mass of this row (`jacobian . B`) - the denominator used when
 * solving for the impulse `lambda` that satisfies this row's velocity constraint.
 *
 * @method computeD
 */
Goblin.ConstraintRow.prototype.computeD = function() {
	this.D = (
		this.jacobian[0] * this.B[0] +
		this.jacobian[1] * this.B[1] +
		this.jacobian[2] * this.B[2] +
		this.jacobian[3] * this.B[3] +
		this.jacobian[4] * this.B[4] +
		this.jacobian[5] * this.B[5] +
		this.jacobian[6] * this.B[6] +
		this.jacobian[7] * this.B[7] +
		this.jacobian[8] * this.B[8] +
		this.jacobian[9] * this.B[9] +
		this.jacobian[10] * this.B[10] +
		this.jacobian[11] * this.B[11]
	);
};

/**
 * Computes `eta`, the amount of work needed this step to satisfy the row's constraint: the
 * velocity implied by each body's current velocity plus its accumulated (unresolved) force/torque,
 * projected through the jacobian, offset by the row's position-error `bias`. This is the target
 * the solver drives `jacobian . v` toward.
 *
 * @method computeEta
 * @param constraint {Constraint} the owning constraint, for its object_a/object_b
 * @param time_delta {Number} the step's time delta, in seconds
 */
Goblin.ConstraintRow.prototype.computeEta = function( constraint, time_delta ) {
	var invmass,
		inverse_time_delta = 1 / time_delta;

	if ( constraint.object_a == null || constraint.object_a._mass === Infinity ) {
		this.eta_row[0] = this.eta_row[1] = this.eta_row[2] = this.eta_row[3] = this.eta_row[4] = this.eta_row[5] = 0;
	} else {
		invmass = constraint.object_a._mass_inverted;

		this.eta_row[0] = ( constraint.object_a.linear_velocity.x + ( invmass * constraint.object_a.accumulated_force.x ) ) * inverse_time_delta;
		this.eta_row[1] = ( constraint.object_a.linear_velocity.y + ( invmass * constraint.object_a.accumulated_force.y ) ) * inverse_time_delta;
		this.eta_row[2] = ( constraint.object_a.linear_velocity.z + ( invmass * constraint.object_a.accumulated_force.z ) ) * inverse_time_delta;

		_tmp_vec3_1.copy( constraint.object_a.accumulated_torque );
		constraint.object_a.inverseInertiaTensorWorldFrame.transformVector3( _tmp_vec3_1 );
		this.eta_row[3] = ( constraint.object_a.angular_velocity.x + _tmp_vec3_1.x ) * inverse_time_delta;
		this.eta_row[4] = ( constraint.object_a.angular_velocity.y + _tmp_vec3_1.y ) * inverse_time_delta;
		this.eta_row[5] = ( constraint.object_a.angular_velocity.z + _tmp_vec3_1.z ) * inverse_time_delta;
	}

	if ( constraint.object_b == null || constraint.object_b._mass === Infinity ) {
		this.eta_row[6] = this.eta_row[7] = this.eta_row[8] = this.eta_row[9] = this.eta_row[10] = this.eta_row[11] = 0;
	} else {
		invmass = constraint.object_b._mass_inverted;

		this.eta_row[6] = ( constraint.object_b.linear_velocity.x + ( invmass * constraint.object_b.accumulated_force.x ) ) * inverse_time_delta;
		this.eta_row[7] = ( constraint.object_b.linear_velocity.y + ( invmass * constraint.object_b.accumulated_force.y ) ) * inverse_time_delta;
		this.eta_row[8] = ( constraint.object_b.linear_velocity.z + ( invmass * constraint.object_b.accumulated_force.z ) ) * inverse_time_delta;

		_tmp_vec3_1.copy( constraint.object_b.accumulated_torque );
		constraint.object_b.inverseInertiaTensorWorldFrame.transformVector3( _tmp_vec3_1 );
		this.eta_row[9] = ( constraint.object_b.angular_velocity.x + _tmp_vec3_1.x ) * inverse_time_delta;
		this.eta_row[10] = ( constraint.object_b.angular_velocity.y + _tmp_vec3_1.y ) * inverse_time_delta;
		this.eta_row[11] = ( constraint.object_b.angular_velocity.z + _tmp_vec3_1.z ) * inverse_time_delta;
	}

	var jdotv = this.jacobian[0] * this.eta_row[0] +
		this.jacobian[1] * this.eta_row[1] +
		this.jacobian[2] * this.eta_row[2] +
		this.jacobian[3] * this.eta_row[3] +
		this.jacobian[4] * this.eta_row[4] +
		this.jacobian[5] * this.eta_row[5] +
		this.jacobian[6] * this.eta_row[6] +
		this.jacobian[7] * this.eta_row[7] +
		this.jacobian[8] * this.eta_row[8] +
		this.jacobian[9] * this.eta_row[9] +
		this.jacobian[10] * this.eta_row[10] +
		this.jacobian[11] * this.eta_row[11];

	this.eta = ( this.bias * inverse_time_delta ) - jdotv;
};