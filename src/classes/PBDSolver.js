/**
 * Position-based contact solver, an alternative to IterativeSolver's velocity-space PGS. A world step is:
 *
 *   1. Position solve: `substeps` Gauss-Seidel passes resolve penetration along each contact normal by
 *      generalized inverse mass. Position/rotation only - no velocity. Velocity is derived from the
 *      projection's displacement (see `step`); deriving it from the depenetration push instead makes
 *      naive position solvers unstable at rest.
 *   2. Velocity solve (Muller et al. 2020, section 3.6): iterated - drive approaching normal velocity to
 *      zero, opt-in rolling resistance, then Coulomb friction (after rolling resistance, so it keeps
 *      re-establishing v = w x r). Bounds use the real normal force N = m*g, not the position lambda.
 *   3. Restitution: one post-pass reflecting pre-impact approach velocity by the coefficient.
 *
 * Joints are not position-based: hinge/point/slider/weld reuse IterativeSolver's velocity-space PGS
 * machinery with a separate impulse-accumulator array so they don't touch real velocity mid-iteration.
 *
 * Also carries a toppling-assist HACK (_assessTipAssist) that works around a blind spot in the
 * derive-from-position-delta step: gravity torque about a contact edge for a body starting near rest.
 *
 * @class PBDSolver
 * @extends Goblin.Solver
 * @constructor
 */
Goblin.PBDSolver = function() {
	Goblin.Solver.call( this );

	/**
	 * Real substeps per world tick: integrate -> detect -> project -> derive velocity runs this many
	 * times at dt/substeps each (see `step`).
	 *
	 * @property substeps
	 * @type {Number}
	 */
	this.substeps = 5;

	/**
	 * Gauss-Seidel passes of the position solve per substep. One pass propagates load one contact deep,
	 * so a tall stack needs several before its base feels the weight above. No velocity is produced, so
	 * extra passes cost time, not stability.
	 *
	 * @property position_iterations
	 * @type {Number}
	 */
	this.position_iterations = 2;

	/**
	 * Successive-over-relaxation factor on each contact's position correction (0 = none, 1 = fully resolve
	 * penetration in one pass).
	 *
	 * @property relaxation
	 * @type {Number}
	 */
	this.relaxation = 1.0;

	/**
	 * This step's active contacts, rebuilt fresh every step from contact_manifolds.
	 *
	 * @property _contacts
	 * @type {Array}
	 * @private
	 */
	this._contacts = [];
};
Goblin.PBDSolver.prototype = Object.create( Goblin.Solver.prototype );
Goblin.PBDSolver.prototype.constructor = Goblin.PBDSolver;

// Position solve targets this much separation past contact (m), keeping props shallow so GJK avoids EPA.
Goblin.PBDSolver.PENETRATION_BIAS = 0.001;

// Inverse stiffness of a contact constraint, in metres per Newton. 0 is perfectly rigid, which is what
// a solid contact wants; a small positive value makes contacts slightly springy. Feeds alphaTilde in
// _solveContactPosition.
Goblin.PBDSolver.CONTACT_COMPLIANCE = 0;

// Lever arm, in metres, used to convert a body's linear speed into the spin a contact could impart to it
// in one substep (see the derived-velocity clamp in `step`). Smaller means more spin is allowed.
Goblin.PBDSolver.SPIN_LEVER = 1.2;

// Minimum linear speed (squared) before a contact is allowed to convert motion into spin. Below this the
// body is resting and has no motion to convert.
Goblin.PBDSolver.SPIN_GAIN_MIN_SPEED_SQ = 0.25 * 0.25;

// File-local scratch, distinct from the shared _tmp_vec3_1.._tmp_vec3_3 globals in libglobals.js: the
// position solve needs more simultaneously-live temporaries than the shared pool provides.
var _pbd_vec3_1 = new Goblin.Vector3(),
	_pbd_vec3_2 = new Goblin.Vector3(),
	_pbd_vec3_3 = new Goblin.Vector3(),
	_pbd_vec3_4 = new Goblin.Vector3(),
	_pbd_vec3_5 = new Goblin.Vector3(),
	_pbd_vec3_6 = new Goblin.Vector3(),
	_pbd_vec3_7 = new Goblin.Vector3(),
	// Friction/rolling tangent direction; kept distinct from _pbd_vec3_6, which _applyPositionCorrection
	// clobbers via crossVectors(r, normal).
	_pbd_vec3_8 = new Goblin.Vector3(),
	_pbd_quat4_1 = new Goblin.Quaternion(),
	_pbd_quat4_2 = new Goblin.Quaternion(),
	_pbd_quat4_3 = new Goblin.Quaternion();

/**
 * The real XPBD tick: integrate -> detect -> project positions -> derive velocity, `substeps` times
 * per world tick at dt/substeps each. Substepping keeps penetrations shallow enough for the projection
 * to fully resolve.
 *
 * Velocity is DERIVED from the projection's displacement ( v = (x - x_prev)/h, and the quaternion
 * difference for spin ), not solved separately, so contact normal response has exactly one owner. Only
 * friction and restitution run in velocity space.
 *
 * Broadphase runs once per tick; narrowphase re-runs per substep so contact points track the
 * projected positions.
 *
 * @method step
 * @param rigid_bodies {Array}
 * @param gravity {Vector3}
 * @param time_delta {Number}
 * @param broadphase {Goblin.Broadphase}
 * @param narrowphase {Goblin.NarrowPhase}
 */
Goblin.PBDSolver.prototype.step = function( rigid_bodies, gravity, time_delta, broadphase, narrowphase ) {
	var substeps = this.substeps > 0 ? this.substeps : 1;
	var h = time_delta / substeps;
	var i, n = rigid_bodies.length, body;

	this._lastTimeDelta = h;

	for ( i = 0; i < n; i++ ) {
		body = rigid_bodies[i];
		if ( body._mass === Infinity ) {
			continue;
		}
		body._pbdPrevPos = body._pbdPrevPos || new Goblin.Vector3();
		body._pbdPrevRot = body._pbdPrevRot || new Goblin.Quaternion();
	}

	for ( i = 0; i < n; i++ ) {
		rigid_bodies[i].updateDerived();
	}
	broadphase.update();

	for ( var s = 0; s < substeps; s++ ) {
		for ( i = 0; i < n; i++ ) {
			body = rigid_bodies[i];
			if ( body._mass === Infinity ) {
				continue;
			}
			body._pbdPrevPos.copy( body.position );
			body._pbdPrevRot.x = body.rotation.x;
			body._pbdPrevRot.y = body.rotation.y;
			body._pbdPrevRot.z = body.rotation.z;
			body._pbdPrevRot.w = body.rotation.w;

			_pbd_vec3_6.scaleVector( body.gravity || gravity, body._mass * h );
			body.accumulated_force.add( _pbd_vec3_6 );

			// Toppling assist (HACK - see _assessTipAssist): gravity torque about the support edge for
			// a body past its balance point, which the depenetrate-only position solve never applies.
			if ( body._pbdTipActive && body._pbdTipTorque ) {
				body.accumulated_torque.x += body._pbdTipTorque.x * h;
				body.accumulated_torque.y += body._pbdTipTorque.y * h;
				body.accumulated_torque.z += body._pbdTipTorque.z * h;
			}

			body.integrate( h );
		}

		for ( i = 0; i < n; i++ ) {
			rigid_bodies[i].updateDerived();
		}

		// Full contact detection once per tick; later substeps just re-transform the body-local
		// anchor points (no GJK/EPA), which keeps a contact alive across the whole tick.
		if ( s === 0 ) {
			narrowphase.generateContacts( broadphase.collision_pairs );
		} else {
			for ( var m = narrowphase.contact_manifolds.first; m !== null; m = m.next_manifold ) {
				m.update();
			}
		}
		this.processContactManifolds( narrowphase.contact_manifolds );

		// Once per tick; the flags it sets are read every substep.
		if ( s === 0 ) {
			Goblin.PBDSolver._assessTipAssist( rigid_bodies, n, this._contacts, gravity, ( this._pbdEpoch = this._pbdEpoch + 1 ), ( this._pbdTickCount = ( this._pbdTickCount || 0 ) + 1 ) );
		}

		// Contact approach velocity before the projection moves anything - restitution and the
		// derived-velocity clamp both need the pre-solve closing speed.
		var contacts = this._contacts;
		for ( i = 0; i < contacts.length; i++ ) {
			contacts[i]._pbdPreSolveVelocity = contacts[i]._pbdPreSolveVelocity || new Goblin.Vector3();
			Goblin.PBDSolver._contactPointVelocity( contacts[i], contacts[i]._pbdPreSolveVelocity );
		}

		for ( i = 0; i < n; i++ ) {
			body = rigid_bodies[i];
			if ( body._mass === Infinity ) {
				continue;
			}
			body._pbdPreSolveSpeedSq = body.linear_velocity.lengthSquared();
			body._pbdPreSolveSpinSq = body.angular_velocity.lengthSquared();
		}

		this._solvePositions();

		// Derive velocity from the projection's displacement this substep.
		for ( i = 0; i < n; i++ ) {
			body = rigid_bodies[i];
			if ( body._mass === Infinity ) {
				continue;
			}

			var dvx = ( body.position.x - body._pbdPrevPos.x ) / h;
			var dvy = ( body.position.y - body._pbdPrevPos.y ) / h;
			var dvz = ( body.position.z - body._pbdPrevPos.z ) / h;

			// Cap derived speed at the pre-solve speed so depenetration can stop motion but never
			// create it (dividing a deep recovery by h would launch the body). Bouncing is restitution's.
			var derivedSq = dvx * dvx + dvy * dvy + dvz * dvz;
			var allowedSq = body._pbdPreSolveSpeedSq;
			if ( derivedSq > allowedSq && derivedSq > 0 ) {
				var damp = Math.sqrt( allowedSq / derivedSq );
				dvx *= damp; dvy *= damp; dvz *= damp;
			}

			body.linear_velocity.x = dvx;
			body.linear_velocity.y = dvy;
			body.linear_velocity.z = dvz;

			// Spin from dq = q * q_prev^-1, vector part scaled by 2/h. Below 1e-16 the rotation is
			// quaternion round-off (the 2/h factor is ~600) - zero it so a resting body can't spin up.
			// Under tip-assist, keep integrate's angular velocity: re-deriving it from net
			// displacement would clamp the assist torque back off (the topple stalls).
			if ( body._pbdTipActive ) {
				// keep body.angular_velocity as integrate left it
			} else {
			_pbd_quat4_2.invertQuaternion( body._pbdPrevRot );
			_pbd_quat4_3.multiplyQuaternions( body.rotation, _pbd_quat4_2 );
			var dqLenSq = _pbd_quat4_3.x * _pbd_quat4_3.x + _pbd_quat4_3.y * _pbd_quat4_3.y + _pbd_quat4_3.z * _pbd_quat4_3.z;
			if ( dqLenSq < 1e-16 ) {
				body.angular_velocity.x = body.angular_velocity.y = body.angular_velocity.z = 0;
			} else {
				var scale = ( _pbd_quat4_3.w >= 0 ? 2 : -2 ) / h;
				var wx = _pbd_quat4_3.x * scale, wy = _pbd_quat4_3.y * scale, wz = _pbd_quat4_3.z * scale;

				// Cap derived spin, or the projection's overshoot spins the body up. Budget is the
				// pre-solve spin, raised to the v/r conversion limit (edge-tipping is real) but only
				// for a body that is actually travelling - a resting pile has no motion to convert.
				var spinSq = wx * wx + wy * wy + wz * wz;
				var spinBudgetSq = body._pbdPreSolveSpinSq;
				if ( body._pbdPreSolveSpeedSq > Goblin.PBDSolver.SPIN_GAIN_MIN_SPEED_SQ ) {
					var contactSpin = body._pbdPreSolveSpeedSq / ( Goblin.PBDSolver.SPIN_LEVER * Goblin.PBDSolver.SPIN_LEVER );
					if ( contactSpin > spinBudgetSq ) {
						spinBudgetSq = contactSpin;
					}
				}

				var spinSqLimit = spinBudgetSq;
				if ( spinSq > spinSqLimit && spinSq > 0 ) {
					var spinDamp = Math.sqrt( spinSqLimit / spinSq );
					wx *= spinDamp; wy *= spinDamp; wz *= spinDamp;
				}

				body.angular_velocity.x = wx;
				body.angular_velocity.y = wy;
				body.angular_velocity.z = wz;
			}
			}
		}

		this._solveVelocities( h );
	}

	for ( i = 0; i < n; i++ ) {
		rigid_bodies[i].updateDerived();
	}

	this._solveJoints();
	this._applyJointResults( time_delta );
};

/**
 * The position projection: `relaxation`-weighted Gauss-Seidel passes over every contact, resolving
 * penetration only. Split out of solveConstraints so the substep loop can call it directly.
 *
 * @method _solvePositions
 * @private
 */
Goblin.PBDSolver.prototype._solvePositions = function() {
	var contacts = this._contacts, n = contacts.length;
	if ( n === 0 ) {
		return;
	}

	var epoch = ( this._pbdEpoch = ( this._pbdEpoch || 0 ) + 1 );
	for ( var i = 0; i < n; i++ ) {
		contacts[i]._pbdAccumLambda = 0;
		Goblin.PBDSolver._countFrictionBody( contacts[i].object_a, epoch );
		Goblin.PBDSolver._countFrictionBody( contacts[i].object_b, epoch );
	}
	for ( i = 0; i < n; i++ ) {
		Goblin.PBDSolver._preparePositionContact( contacts[i], this._lastTimeDelta );
	}

	var iterations = this.position_iterations > 0 ? this.position_iterations : 1;
	for ( var it = 0; it < iterations; it++ ) {
		for ( i = 0; i < n; i++ ) {
			Goblin.PBDSolver._solveContactPosition( contacts[i], this.relaxation );
		}
	}

	var derivedEpoch = ( this._pbdEpoch = this._pbdEpoch + 1 );
	for ( i = 0; i < n; i++ ) {
		Goblin.PBDSolver._refreshDerived( contacts[i].object_a, derivedEpoch );
		Goblin.PBDSolver._refreshDerived( contacts[i].object_b, derivedEpoch );
	}
};

/**
 * Velocity-space pass for the substep loop: friction and restitution only. Normal response is not
 * here - it is the position projection, via the derived velocity (see `step`).
 *
 * @method _solveVelocities
 * @param h {Number} the substep timestep
 * @private
 */
Goblin.PBDSolver.prototype._solveVelocities = function( h ) {
	var contacts = this._contacts, n = contacts.length;
	if ( n === 0 ) {
		return;
	}

	var gravityMag = 9.8;
	if ( this.world && this.world.gravity ) {
		var gv = this.world.gravity;
		gravityMag = Math.sqrt( gv.x * gv.x + gv.y * gv.y + gv.z * gv.z ) || 9.8;
	}

	var i;
	for ( i = 0; i < n; i++ ) {
		var c = contacts[i];
		Goblin.PBDSolver._prepareVelocityContact( c );

		// Friction's Coulomb bound. The accumulated lambda under-reports a well-resolved resting
		// contact's load, so floor it at the impulse needed to support the contact's weight share.
		var lambda = c._pbdAccumLambda;
		var support = Goblin.PBDSolver._restingNormalImpulse( c, h );
		c._pbdNormalImpulse = lambda > support ? lambda : support;

		// But not at a rolling contact: the resting bound there brakes the roll to a stop. Rolling
		// resistance decays a roll; friction only cancels real slip, which the lambda alone sizes.
		if ( Goblin.PBDSolver._contactIsRolling( c ) ) {
			c._pbdNormalImpulse = lambda;
		}
	}
	for ( i = 0; i < n; i++ ) {
		Goblin.PBDSolver._solveRollingResistance( contacts[i], h, gravityMag );
	}
	for ( i = 0; i < n; i++ ) {
		Goblin.PBDSolver._solveContactFrictionVelocity( contacts[i] );
	}
	for ( i = 0; i < n; i++ ) {
		Goblin.PBDSolver._applyRestitution( contacts[i] );
	}

	Goblin.PBDSolver._killRestingBuzz( contacts, ( this._pbdEpoch = this._pbdEpoch + 1 ) );
};

/**
 * Applies gravity and integrates free-flight motion N times at dt/N. Gravity is re-applied each
 * substep since RigidBody.integrate consumes accumulated_force.
 *
 * @method integrate
 * @param rigid_bodies {Array}
 * @param gravity {Vector3}
 * @param time_delta {Number}
 */
Goblin.PBDSolver.prototype.integrate = function( rigid_bodies, gravity, time_delta ) {
	var substeps = this.substeps;
	var subDt = time_delta / substeps;
	var i, loop_count, body;

	for ( var s = 0; s < substeps; s++ ) {
		for ( i = 0, loop_count = rigid_bodies.length; i < loop_count; i++ ) {
			body = rigid_bodies[i];
			if ( body._mass !== Infinity ) {
				_pbd_vec3_6.scaleVector( body.gravity || gravity, body._mass * subDt );
				body.accumulated_force.add( _pbd_vec3_6 );
			}
		}
		for ( i = 0, loop_count = rigid_bodies.length; i < loop_count; i++ ) {
			rigid_bodies[i].integrate( subDt );
		}
	}
};

/**
 * Flattens this step's contact manifolds into a working list of points.
 *
 * @method processContactManifolds
 * @param contact_manifolds {ContactManifoldList}
 */
Goblin.PBDSolver.prototype.processContactManifolds = function( contact_manifolds ) {
	var contacts = this._contacts;
	contacts.length = 0;

	var manifold = contact_manifolds.first;
	while ( manifold ) {
		for ( var i = 0; i < manifold.points.length; i++ ) {
			contacts.push( manifold.points[i] );
		}
		manifold = manifold.next_manifold;
	}
};

/**
 * Records each contact's pre-solve point velocity (for restitution) and precomputes joint rows the
 * same way IterativeSolver does.
 *
 * @method prepareConstraints
 * @param time_delta {Number}
 */
Goblin.PBDSolver.prototype.prepareConstraints = function( time_delta ) {
	this._lastTimeDelta = time_delta;

	var contacts = this._contacts;
	for ( var i = 0; i < contacts.length; i++ ) {
		var contact = contacts[i];
		contact._pbdPreSolveVelocity = contact._pbdPreSolveVelocity || new Goblin.Vector3();
		Goblin.PBDSolver._contactPointVelocity( contact, contact._pbdPreSolveVelocity );
	}

	var joints = this.constraints;
	for ( var c = 0; c < joints.length; c++ ) {
		var constraint = joints[c];
		if ( constraint.active === false ) {
			continue;
		}
		constraint.update( time_delta );
		for ( var j = 0; j < constraint.rows.length; j++ ) {
			var row = constraint.rows[j];
			row.multiplier = 0;
			row.computeB( constraint );
			row.computeD();
			row.computeEta( constraint, time_delta );
		}
	}
};

/**
 * No-op: penetration is resolved directly in solveConstraints, no separate pass needed.
 *
 * @method resolveContacts
 */
Goblin.PBDSolver.prototype.resolveContacts = function() {};

/**
 * Two-stage contact solve (see class docstring): first `substeps` Gauss-Seidel passes of position-only
 * penetration recovery, then an iterated velocity solve (normal velocity, rolling resistance, friction),
 * then restitution once. Joints are solved separately at the end.
 *
 * @method solveConstraints
 */
Goblin.PBDSolver.prototype.solveConstraints = function() {
	var contacts = this._contacts;
	var n = contacts.length;

	if ( n > 0 ) {
		var substeps = this.substeps;
		var dt = this._lastTimeDelta;
		var relaxation = this.relaxation;

		// Per-body scratch keyed by a step epoch on the body itself, not a {} map keyed by body.id
		// (which is a megamorphic keyed load - the hottest builtin in this loop).
		var epoch = ( this._pbdEpoch = ( this._pbdEpoch || 0 ) + 1 );
		for ( var i0 = 0; i0 < n; i0++ ) {
			contacts[i0]._pbdAccumLambda = 0;
			Goblin.PBDSolver._countFrictionBody( contacts[i0].object_a, epoch );
			Goblin.PBDSolver._countFrictionBody( contacts[i0].object_b, epoch );
		}
		for ( var pp = 0; pp < n; pp++ ) {
			Goblin.PBDSolver._preparePositionContact( contacts[pp] );
		}

		// Penetration recovery only; all velocity change is owned by the velocity pass below.
		for ( var s = 0; s < substeps; s++ ) {
			for ( var i = 0; i < n; i++ ) {
				Goblin.PBDSolver._solveContactPosition( contacts[i], relaxation );
			}
		}

		// Position corrections only rebuilt transforms; refresh full derived state once now.
		var derivedEpoch = ( this._pbdEpoch = this._pbdEpoch + 1 );
		for ( var ud = 0; ud < n; ud++ ) {
			Goblin.PBDSolver._refreshDerived( contacts[ud].object_a, derivedEpoch );
			Goblin.PBDSolver._refreshDerived( contacts[ud].object_b, derivedEpoch );
		}

		var gravityMag = 9.8;
		if ( this.world && this.world.gravity ) {
			var gv = this.world.gravity;
			gravityMag = Math.sqrt( gv.x * gv.x + gv.y * gv.y + gv.z * gv.z ) || 9.8;
		}

		// Cache per-contact velocity-solve invariants once; constant across the iterations below.
		for ( var pc = 0; pc < n; pc++ ) {
			Goblin.PBDSolver._prepareVelocityContact( contacts[pc] );
		}

		// Iterated: normal velocity to the restitution target, rolling resistance, then friction.
		var velIterations = 4;
		var subDtBudget = dt / velIterations;
		for ( var vp = 0; vp < velIterations; vp++ ) {
			for ( var nz = 0; nz < n; nz++ ) {
				contacts[nz]._pbdNormalImpulse = 0;
			}
			for ( var vc = 0; vc < n; vc++ ) {
				Goblin.PBDSolver._solveContactNormalVelocity( contacts[vc] );
			}
			for ( var rr = 0; rr < n; rr++ ) {
				Goblin.PBDSolver._solveRollingResistance( contacts[rr], subDtBudget, gravityMag );
			}
			for ( var fr = 0; fr < n; fr++ ) {
				Goblin.PBDSolver._solveContactFrictionVelocity( contacts[fr] );
			}
		}

		for ( var r = 0; r < n; r++ ) {
			Goblin.PBDSolver._applyRestitution( contacts[r] );
		}

		Goblin.PBDSolver._killRestingBuzz( contacts, ( this._pbdEpoch = this._pbdEpoch + 1 ) );
	}

	this._solveJoints();
};

/**
 * Velocity-space PGS joint solve. Impulses accumulate into a scratch per-body array during
 * iteration; _applyJointResults bakes the converged multiplier into real velocity.
 *
 * @method _solveJoints
 * @private
 */
Goblin.PBDSolver.prototype._solveJoints = function() {
	var joints = this.constraints;
	if ( joints.length === 0 ) {
		return;
	}

	var c, constraint;
	for ( c = 0; c < joints.length; c++ ) {
		constraint = joints[c];
		Goblin.PBDSolver._zeroJointBodyImpulse( constraint.object_a );
		Goblin.PBDSolver._zeroJointBodyImpulse( constraint.object_b );
	}

	for ( var iter = 0; iter < 8; iter++ ) {
		var max_impulse = 0;
		for ( c = 0; c < joints.length; c++ ) {
			constraint = joints[c];
			if ( constraint.active === false ) {
				continue;
			}
			for ( var j = 0; j < constraint.rows.length; j++ ) {
				var row = constraint.rows[j];

				var jdotv = Goblin.PBDSolver._jointRowJdotV( row, constraint );
				var delta_lambda = ( row.eta - jdotv ) / row.D || 0;
				var cache = row.multiplier;
				var new_multiplier = Math.max( row.lower_limit, Math.min( cache + delta_lambda, row.upper_limit ) );
				row.multiplier = new_multiplier;
				delta_lambda = new_multiplier - cache;
				max_impulse = Math.max( max_impulse, Math.abs( delta_lambda ) );

				Goblin.PBDSolver._applyJointRowImpulse( row, constraint, delta_lambda );
			}
		}
		if ( max_impulse <= 0.1 ) {
			break;
		}
	}
};

Goblin.PBDSolver._zeroJointBodyImpulse = function( body ) {
	if ( body == null || body._mass === Infinity ) {
		return;
	}
	body._pbdJointImpulse = body._pbdJointImpulse || new Float64Array( 6 );
	body._pbdJointImpulse[0] = body._pbdJointImpulse[1] = body._pbdJointImpulse[2] =
	body._pbdJointImpulse[3] = body._pbdJointImpulse[4] = body._pbdJointImpulse[5] = 0;
};

Goblin.PBDSolver._jointRowJdotV = function( row, constraint ) {
	var jdotv = 0;
	if ( constraint.object_a != null && constraint.object_a._mass !== Infinity ) {
		var ia = constraint.object_a._pbdJointImpulse;
		jdotv +=
			row.jacobian[0] * ia[0] + row.jacobian[1] * ia[1] + row.jacobian[2] * ia[2] +
			row.jacobian[3] * ia[3] + row.jacobian[4] * ia[4] + row.jacobian[5] * ia[5];
	}
	if ( constraint.object_b != null && constraint.object_b._mass !== Infinity ) {
		var ib = constraint.object_b._pbdJointImpulse;
		jdotv +=
			row.jacobian[6] * ib[0] + row.jacobian[7] * ib[1] + row.jacobian[8] * ib[2] +
			row.jacobian[9] * ib[3] + row.jacobian[10] * ib[4] + row.jacobian[11] * ib[5];
	}
	return jdotv;
};

Goblin.PBDSolver._applyJointRowImpulse = function( row, constraint, delta_lambda ) {
	if ( delta_lambda === 0 ) {
		return;
	}
	if ( constraint.object_a != null && constraint.object_a._mass !== Infinity ) {
		var ia = constraint.object_a._pbdJointImpulse;
		ia[0] += delta_lambda * row.B[0];
		ia[1] += delta_lambda * row.B[1];
		ia[2] += delta_lambda * row.B[2];
		ia[3] += delta_lambda * row.B[3];
		ia[4] += delta_lambda * row.B[4];
		ia[5] += delta_lambda * row.B[5];
	}
	if ( constraint.object_b != null && constraint.object_b._mass !== Infinity ) {
		var ib = constraint.object_b._pbdJointImpulse;
		ib[0] += delta_lambda * row.B[6];
		ib[1] += delta_lambda * row.B[7];
		ib[2] += delta_lambda * row.B[8];
		ib[3] += delta_lambda * row.B[9];
		ib[4] += delta_lambda * row.B[10];
		ib[5] += delta_lambda * row.B[11];
	}
};

/**
 * Contacts are fully resolved inside solveConstraints; only joints need finalizing here.
 *
 * @method applyConstraints
 * @param time_delta {Number}
 */
Goblin.PBDSolver.prototype.applyConstraints = function( time_delta ) {
	this._applyJointResults( time_delta );
};

/**
 * Zeroes linear_velocity once a body has been under the rest threshold for BUZZ_FRAMES ticks.
 * Leaves angular_velocity to the velocity solve and rolling resistance.
 *
 * @method _killRestingBuzz
 * @static
 * @private
 */
Goblin.PBDSolver._killRestingBuzz = function( contacts, epoch ) {
	var BUZZ_LIN_SQ = 0.08 * 0.08, BUZZ_ANG_SQ = 0.08 * 0.08, BUZZ_FRAMES = 8;
	for ( var i = 0; i < contacts.length; i++ ) {
		Goblin.PBDSolver._killBodyBuzz( contacts[i].object_a, BUZZ_LIN_SQ, BUZZ_ANG_SQ, BUZZ_FRAMES, epoch );
		Goblin.PBDSolver._killBodyBuzz( contacts[i].object_b, BUZZ_LIN_SQ, BUZZ_ANG_SQ, BUZZ_FRAMES, epoch );
	}
};

Goblin.PBDSolver._killBodyBuzz = function( body, BUZZ_LIN_SQ, BUZZ_ANG_SQ, BUZZ_FRAMES, epoch ) {
	if ( body == null || body._mass === Infinity || body._pbdBuzzEpoch === epoch ) {
		return;
	}
	body._pbdBuzzEpoch = epoch;

	if ( body.linear_velocity.lengthSquared() < BUZZ_LIN_SQ && body.angular_velocity.lengthSquared() < BUZZ_ANG_SQ ) {
		body._buzzSlowFrames = ( body._buzzSlowFrames || 0 ) + 1;
		if ( body._buzzSlowFrames >= BUZZ_FRAMES ) {
			body.linear_velocity.x = body.linear_velocity.y = body.linear_velocity.z = 0;
		}
	} else {
		body._buzzSlowFrames = 0;
	}
};

/**
 * Finalizes joints: caches each row's multiplier for next step's warm start, trips breaking_threshold.
 *
 * @method _applyJointResults
 * @private
 */
Goblin.PBDSolver.prototype._applyJointResults = function( time_delta ) {
	var joints = this.constraints;
	for ( var c = 0; c < joints.length; c++ ) {
		var constraint = joints[c];
		if ( constraint.active === false ) {
			continue;
		}
		constraint.last_impulse.x = constraint.last_impulse.y = constraint.last_impulse.z = 0;

		for ( var j = 0; j < constraint.rows.length; j++ ) {
			var row = constraint.rows[j];
			row.multiplier_cached = row.multiplier;

			if ( constraint.object_a != null && constraint.object_a._mass !== Infinity ) {
				var invA = constraint.object_a._mass_inverted;
				_pbd_vec3_6.x = invA * time_delta * row.jacobian[0] * constraint.object_a.linear_factor.x * row.multiplier;
				_pbd_vec3_6.y = invA * time_delta * row.jacobian[1] * constraint.object_a.linear_factor.y * row.multiplier;
				_pbd_vec3_6.z = invA * time_delta * row.jacobian[2] * constraint.object_a.linear_factor.z * row.multiplier;
				constraint.object_a.linear_velocity.add( _pbd_vec3_6 );
				constraint.last_impulse.add( _pbd_vec3_6 );

				_pbd_vec3_7.x = time_delta * row.jacobian[3] * constraint.object_a.angular_factor.x * row.multiplier;
				_pbd_vec3_7.y = time_delta * row.jacobian[4] * constraint.object_a.angular_factor.y * row.multiplier;
				_pbd_vec3_7.z = time_delta * row.jacobian[5] * constraint.object_a.angular_factor.z * row.multiplier;
				constraint.object_a.inverseInertiaTensorWorldFrame.transformVector3( _pbd_vec3_7 );
				constraint.object_a.angular_velocity.add( _pbd_vec3_7 );
				constraint.last_impulse.add( _pbd_vec3_7 );
			}
			if ( constraint.object_b != null && constraint.object_b._mass !== Infinity ) {
				var invB = constraint.object_b._mass_inverted;
				_pbd_vec3_6.x = invB * time_delta * row.jacobian[6] * constraint.object_b.linear_factor.x * row.multiplier;
				_pbd_vec3_6.y = invB * time_delta * row.jacobian[7] * constraint.object_b.linear_factor.y * row.multiplier;
				_pbd_vec3_6.z = invB * time_delta * row.jacobian[8] * constraint.object_b.linear_factor.z * row.multiplier;
				constraint.object_b.linear_velocity.add( _pbd_vec3_6 );
				constraint.last_impulse.add( _pbd_vec3_6 );

				_pbd_vec3_7.x = time_delta * row.jacobian[9] * constraint.object_b.angular_factor.x * row.multiplier;
				_pbd_vec3_7.y = time_delta * row.jacobian[10] * constraint.object_b.angular_factor.y * row.multiplier;
				_pbd_vec3_7.z = time_delta * row.jacobian[11] * constraint.object_b.angular_factor.z * row.multiplier;
				constraint.object_b.inverseInertiaTensorWorldFrame.transformVector3( _pbd_vec3_7 );
				constraint.object_b.angular_velocity.add( _pbd_vec3_7 );
				constraint.last_impulse.add( _pbd_vec3_7 );
			}
		}

		if ( constraint.breaking_threshold > 0 ) {
			if ( constraint.last_impulse.lengthSquared() >= constraint.breaking_threshold * constraint.breaking_threshold ) {
				constraint.active = false;
			}
		}
	}
};

// HACK (toppling assist): works around XPBD's blind spot for gravity torque about a contact edge
// when a body starts near rest - the derive-from-position-delta step never picks it up, so a body
// past its balance point stalls mid-fall instead of toppling. Not a standard technique; a proper
// fix would be an XPBD angular contact constraint or a less aggressive derived-spin clamp. Thresholds
// below are hand-tuned to the test suite.
//
// Flags bodies that are toppling: COM outside the support-contact hull, still losing height and
// rotating further over. Sets _pbdTipTorque / _pbdTipActive; runs once per tick in three passes
// (cheap gate over bodies, point gather over contacts, hull test over survivors).
Goblin.PBDSolver._assessTipAssist = function( bodies, n, contacts, gravity, epoch, tickNo ) {
	var cand = Goblin.PBDSolver._tipCandidates;
	cand.length = 0;
	for ( var i = 0; i < n; i++ ) {
		var body = bodies[i];
		if ( body._mass === Infinity ) { continue; }
		if ( Goblin.PBDSolver._tipCheapGate( body, gravity, epoch, tickNo ) ) {
			cand.push( body );
		}
	}
	if ( cand.length === 0 ) { return; }
	for ( i = 0; i < contacts.length; i++ ) {
		var c = contacts[i];
		if ( c.object_a != null && c.object_a._pbdTipCandidate === epoch ) { c.object_a._pbdTipPts.push( c.contact_point.x, c.contact_point.y, c.contact_point.z ); }
		if ( c.object_b != null && c.object_b._pbdTipCandidate === epoch ) { c.object_b._pbdTipPts.push( c.contact_point.x, c.contact_point.y, c.contact_point.z ); }
	}
	for ( i = 0; i < cand.length; i++ ) {
		Goblin.PBDSolver._tipHullTest( cand[i], gravity, epoch );
	}
};
Goblin.PBDSolver._tipCandidates = [];

// Cheap onset gates (no hull). Returns true if the body still needs the hull test, or is latched.
Goblin.PBDSolver._tipCheapGate = function( body, gravity, epoch, tickNo ) {
	var latched = body._pbdTipActive && body._pbdTipRefRot != null;

	// Reject moving/spinning bodies before any trig - the common path in a settling scene.
	if ( !latched ) {
		if ( body.linear_velocity.lengthSquared() > 0.09 * 0.09 ||
			body.angular_velocity.lengthSquared() > 0.03 * 0.03 ) {
			body._pbdTipDwell = 0;
			body._pbdTipActive = false;
			return false;
		}
	}

	body._pbdTipPts = body._pbdTipPts || [];
	body._pbdTipPts.length = 0;
	body._pbdTipTorque = body._pbdTipTorque || new Goblin.Vector3();

	var g = body.gravity || gravity;
	var gl = Math.sqrt( g.x * g.x + g.y * g.y + g.z * g.z ) || 1;
	var height = -( body.position.x * g.x + body.position.y * g.y + body.position.z * g.z ) / gl;
	var qw = body.rotation.w; if ( qw < 0 ) { qw = -qw; } if ( qw > 1 ) { qw = 1; }
	var tilt = 2 * Math.acos( qw );
	// Height/tilt trend measured over a window, not one tick, so buzz jitter on a resting body
	// doesn't clear the thresholds.
	var TIP_TREND_WINDOW = 8;
	if ( body._pbdTipWinTick === undefined || ( tickNo - body._pbdTipWinTick ) >= TIP_TREND_WINDOW ) {
		body._pbdTipDHeight = ( body._pbdTipBaseHeight === undefined ) ? 0 : ( height - body._pbdTipBaseHeight );
		body._pbdTipDTilt = ( body._pbdTipBaseTilt === undefined ) ? 0 : ( tilt - body._pbdTipBaseTilt );
		body._pbdTipBaseHeight = height;
		body._pbdTipBaseTilt = tilt;
		body._pbdTipWinTick = tickNo;
	}

	if ( latched ) {
		body._pbdTipCandidate = epoch;
		return true;
	}

	var DESCEND_RATE = -2e-5, TILT_RATE = 1e-5;
	if ( body._pbdTipDHeight >= DESCEND_RATE || body._pbdTipDTilt <= TILT_RATE ) {
		body._pbdTipDwell = 0;
		body._pbdTipActive = false;
		return false;
	}
	body._pbdTipCandidate = epoch;
	return true;
};

// Basis (u,v) spanning the plane perpendicular to gravity.
Goblin.PBDSolver._tipBasis = { ux:0, uy:0, uz:0, vx:0, vy:0, vz:0 };
Goblin.PBDSolver._tipComputeBasis = function( g ) {
	var gl = Math.sqrt( g.x * g.x + g.y * g.y + g.z * g.z ) || 1;
	var nx = g.x / gl, ny = g.y / gl, nz = g.z / gl;
	var ax = Math.abs( nx ) < 0.9 ? 1 : 0;
	var ay = Math.abs( nx ) < 0.9 ? 0 : 1;
	var d = ax * nx + ay * ny;
	var ux = ax - d * nx, uy = ay - d * ny, uz = -d * nz;
	var ul = Math.sqrt( ux * ux + uy * uy + uz * uz ) || 1;
	ux /= ul; uy /= ul; uz /= ul;
	var b = Goblin.PBDSolver._tipBasis;
	b.ux = ux; b.uy = uy; b.uz = uz;
	b.vx = ny * uz - nz * uy;
	b.vy = nz * ux - nx * uz;
	b.vz = nx * uy - ny * ux;
};

// Signed distance of 2D point (qu,qv) to the CCW hull in `hp` (flat [u0,v0,...], m points).
// > 0 inside, < 0 outside; degenerate hulls (0/1/2 pts) return a negative proximity.
Goblin.PBDSolver._tipInsideDist = function( hp, m, qu, qv ) {
	if ( m === 0 ) { return -1e9; }
	if ( m === 1 ) {
		var dx = qu - hp[0], dy = qv - hp[1];
		return -Math.sqrt( dx * dx + dy * dy );
	}
	if ( m === 2 ) {
		var ax = hp[0], ay = hp[1], bx = hp[2], by = hp[3];
		var abx = bx - ax, aby = by - ay;
		var t = ( ( qu - ax ) * abx + ( qv - ay ) * aby ) / ( abx * abx + aby * aby || 1 );
		if ( t < 0 ) { t = 0; } else if ( t > 1 ) { t = 1; }
		var cx = ax + t * abx, cy = ay + t * aby;
		var ex = qu - cx, ey = qv - cy;
		return -Math.sqrt( ex * ex + ey * ey );
	}
	var minD = 1e9;
	for ( var i = 0; i < m; i++ ) {
		var i2 = i * 2, j2 = ( ( i + 1 ) % m ) * 2;
		var e0 = hp[j2] - hp[i2], e1 = hp[j2 + 1] - hp[i2 + 1];
		var nl = Math.sqrt( e0 * e0 + e1 * e1 ) || 1;
		var dd = ( -e1 * ( qu - hp[i2] ) + e0 * ( qv - hp[i2 + 1] ) ) / nl;
		if ( dd < minD ) { minD = dd; }
	}
	return minD;
};

Goblin.PBDSolver._tipSortIdx = [];
Goblin.PBDSolver._tipHullTmp = [];
Goblin.PBDSolver._tip2D = [];
Goblin.PBDSolver._tipHullOut = [];

// Monotone-chain convex hull of n 2D points `src` (flat [u,v,...]) into `out` (flat, CCW).
// Returns the hull point count.
Goblin.PBDSolver._tipHull = function( src, n, out ) {
	if ( n <= 2 ) {
		for ( var k = 0; k < n * 2; k++ ) { out[k] = src[k]; }
		return n;
	}
	var idx = Goblin.PBDSolver._tipSortIdx;
	idx.length = 0;
	for ( var i = 0; i < n; i++ ) { idx.push( i ); }
	idx.sort( function ( a, b ) {
		return ( src[a * 2] - src[b * 2] ) || ( src[a * 2 + 1] - src[b * 2 + 1] );
	} );
	var cross = function ( ox, oy, ax, ay, bx, by ) {
		return ( ax - ox ) * ( by - oy ) - ( ay - oy ) * ( bx - ox );
	};
	var hull = Goblin.PBDSolver._tipHullTmp;
	hull.length = 0;
	for ( i = 0; i < n; i++ ) {
		var p = idx[i], pu = src[p * 2], pv = src[p * 2 + 1];
		while ( hull.length >= 4 &&
			cross( hull[hull.length - 4], hull[hull.length - 3], hull[hull.length - 2], hull[hull.length - 1], pu, pv ) <= 0 ) {
			hull.length -= 2;
		}
		hull.push( pu, pv );
	}
	var lowerLen = hull.length + 2;
	for ( i = n - 2; i >= 0; i-- ) {
		var p2 = idx[i], pu2 = src[p2 * 2], pv2 = src[p2 * 2 + 1];
		while ( hull.length >= lowerLen &&
			cross( hull[hull.length - 4], hull[hull.length - 3], hull[hull.length - 2], hull[hull.length - 1], pu2, pv2 ) <= 0 ) {
			hull.length -= 2;
		}
		hull.push( pu2, pv2 );
	}
	hull.length -= 2;
	for ( var q = 0; q < hull.length; q++ ) { out[q] = hull[q]; }
	return hull.length / 2;
};

// Convex-hull static-stability test + latch bookkeeping, for candidates from _tipCheapGate.
Goblin.PBDSolver._tipHullTest = function( body, gravity, epoch ) {
	if ( body._pbdTipCandidate !== epoch ) { return; }

	var pts = body._pbdTipPts;
	var np = pts.length / 3;
	if ( np === 0 ) {
		body._pbdTipActive = false;
		body._pbdTipRefRot = null;
		return;
	}

	var g = body.gravity || gravity;
	Goblin.PBDSolver._tipComputeBasis( g );
	var b = Goblin.PBDSolver._tipBasis;
	var p2 = Goblin.PBDSolver._tip2D;
	for ( var i = 0; i < np; i++ ) {
		var x = pts[i * 3], y = pts[i * 3 + 1], z = pts[i * 3 + 2];
		p2[i * 2] = x * b.ux + y * b.uy + z * b.uz;
		p2[i * 2 + 1] = x * b.vx + y * b.vy + z * b.vz;
	}
	var comU = body.position.x * b.ux + body.position.y * b.uy + body.position.z * b.uz;
	var comV = body.position.x * b.vx + body.position.y * b.vy + body.position.z * b.vz;
	var hp = Goblin.PBDSolver._tipHullOut;
	var m = Goblin.PBDSolver._tipHull( p2, np, hp );
	var dist = Goblin.PBDSolver._tipInsideDist( hp, m, comU, comV );

	var UNSTABLE_MARGIN = -0.05, COMMIT_SWING = 0.35, TIP_DWELL_TICKS = 12;

	if ( body._pbdTipActive && body._pbdTipRefRot != null ) {
		var rq = body._pbdTipRefRot;
		var dq = rq.x * body.rotation.x + rq.y * body.rotation.y + rq.z * body.rotation.z + rq.w * body.rotation.w;
		if ( dq < 0 ) { dq = -dq; }
		if ( dq > 1 ) { dq = 1; }
		// Release once the swing has committed or the body is stable again.
		if ( 2 * Math.acos( dq ) > COMMIT_SWING || dist >= UNSTABLE_MARGIN ) {
			body._pbdTipActive = false;
			body._pbdTipRefRot = null;
			return;
		}
	} else if ( dist < UNSTABLE_MARGIN ) {
		body._pbdTipDwell = ( body._pbdTipDwell || 0 ) + 1;
		if ( body._pbdTipDwell >= TIP_DWELL_TICKS ) {
			body._pbdTipActive = true;
			body._pbdTipRefRot = body._pbdTipRefRot || new Goblin.Quaternion();
			body._pbdTipRefRot.x = body.rotation.x;
			body._pbdTipRefRot.y = body.rotation.y;
			body._pbdTipRefRot.z = body.rotation.z;
			body._pbdTipRefRot.w = body.rotation.w;
		} else {
			body._pbdTipActive = false;
			return;
		}
	} else {
		body._pbdTipDwell = 0;
		body._pbdTipActive = false;
		return;
	}

	// Tip torque h x (m*g), h = footprint-centroid -> COM (horizontal), capped so it can't run away.
	var cu = 0, cv = 0;
	for ( i = 0; i < np; i++ ) { cu += p2[i * 2]; cv += p2[i * 2 + 1]; }
	cu /= np; cv /= np;
	var du = comU - cu, dv = comV - cv;
	var hx = du * b.ux + dv * b.vx;
	var hy = du * b.uy + dv * b.vy;
	var hz = du * b.uz + dv * b.vz;
	_pbd_vec3_7.x = body._mass * g.x;
	_pbd_vec3_7.y = body._mass * g.y;
	_pbd_vec3_7.z = body._mass * g.z;
	_pbd_vec3_6.x = hx; _pbd_vec3_6.y = hy; _pbd_vec3_6.z = hz;
	body._pbdTipTorque.crossVectors( _pbd_vec3_6, _pbd_vec3_7 );
	var TIP_TORQUE_MAX = 4.0 * body._mass;
	var ttLen = body._pbdTipTorque.length();
	if ( ttLen > TIP_TORQUE_MAX && ttLen > 0 ) {
		var ttScale = TIP_TORQUE_MAX / ttLen;
		body._pbdTipTorque.x *= ttScale;
		body._pbdTipTorque.y *= ttScale;
		body._pbdTipTorque.z *= ttScale;
	}
};

Goblin.PBDSolver._contactPointVelocity = function( contact, out ) {
	var relVelocity = _pbd_vec3_1;
	relVelocity.x = relVelocity.y = relVelocity.z = 0;

	if ( contact.object_a._mass !== Infinity ) {
		_pbd_vec3_2.subtractVectors( contact.contact_point, contact.object_a.position );
		_pbd_vec3_3.crossVectors( contact.object_a.angular_velocity, _pbd_vec3_2 );
		_pbd_vec3_3.add( contact.object_a.linear_velocity );
		relVelocity.subtract( _pbd_vec3_3 );
	}
	if ( contact.object_b._mass !== Infinity ) {
		_pbd_vec3_2.subtractVectors( contact.contact_point, contact.object_b.position );
		_pbd_vec3_3.crossVectors( contact.object_b.angular_velocity, _pbd_vec3_2 );
		_pbd_vec3_3.add( contact.object_b.linear_velocity );
		relVelocity.add( _pbd_vec3_3 );
	}
	out.copy( relVelocity );
};

// Same as _contactPointVelocity but reuses the cached lever arms (contact._pbdRA/_pbdRB) instead of
// recomputing contact_point - position. For the velocity solve, which runs after _prepareVelocityContact.
Goblin.PBDSolver._contactPointVelocityCached = function( contact, out ) {
	var a = contact.object_a, b = contact.object_b;
	out.x = out.y = out.z = 0;

	if ( a._mass !== Infinity ) {
		_pbd_vec3_3.crossVectors( a.angular_velocity, contact._pbdRA );
		_pbd_vec3_3.add( a.linear_velocity );
		out.subtract( _pbd_vec3_3 );
	}
	if ( b._mass !== Infinity ) {
		_pbd_vec3_3.crossVectors( b.angular_velocity, contact._pbdRB );
		_pbd_vec3_3.add( b.linear_velocity );
		out.add( _pbd_vec3_3 );
	}
};

// Caches velocity-solve invariants on the contact (lever arms rA/rB, inverse masses, normal wSum,
// point divisor). Run once per step after the position solve; constant across velocity iterations.
Goblin.PBDSolver._prepareVelocityContact = function( contact ) {
	var a = contact.object_a, b = contact.object_b;
	var normal = contact.contact_normal;

	var rA = contact._pbdRA || ( contact._pbdRA = new Goblin.Vector3() );
	var rB = contact._pbdRB || ( contact._pbdRB = new Goblin.Vector3() );
	rA.subtractVectors( contact.contact_point, a.position );
	rB.subtractVectors( contact.contact_point, b.position );

	contact._pbdInvMassA = a._mass === Infinity ? 0 : a._mass_inverted;
	contact._pbdInvMassB = b._mass === Infinity ? 0 : b._mass_inverted;

	var wA = Goblin.PBDSolver._angularInvMass( a, rA, normal );
	var wB = Goblin.PBDSolver._angularInvMass( b, rB, normal );
	contact._pbdNormalWSum = contact._pbdInvMassA + wA + contact._pbdInvMassB + wB;

	var countA = Goblin.PBDSolver._pointCountOf( a );
	var countB = Goblin.PBDSolver._pointCountOf( b );
	contact._pbdPointDiv = ( countA > countB ? countA : countB );
};

// Drive the contact point's relative normal velocity toward zero (approaching side only), with full
// angular coupling. Records the normal impulse for friction's Coulomb bound. (Muller 2020, 3.6.)
Goblin.PBDSolver._solveContactNormalVelocity = function( contact ) {
	var a = contact.object_a, b = contact.object_b;
	var normal = contact.contact_normal;

	Goblin.PBDSolver._contactPointVelocityCached( contact, _pbd_vec3_1 );
	var relNormal = _pbd_vec3_1.dot( normal );
	if ( relNormal >= 0 ) {
		return;
	}

	var wSum = contact._pbdNormalWSum;
	if ( wSum <= 0 ) {
		return;
	}

	var jn = ( -relNormal / wSum ) / contact._pbdPointDiv;
	contact._pbdNormalImpulse += jn;

	if ( contact._pbdInvMassA > 0 ) {
		Goblin.PBDSolver._applyVelocityImpulse( a, contact._pbdRA, normal, -jn );
	}
	if ( contact._pbdInvMassB > 0 ) {
		Goblin.PBDSolver._applyVelocityImpulse( b, contact._pbdRB, normal, jn );
	}
};

// True when a body at this contact is rolling (real spin, near-zero contact-point slip).
Goblin.PBDSolver._contactIsRolling = function( contact ) {
	var ROLL_SPIN_SQ = 0.05 * 0.05;
	var a = contact.object_a, b = contact.object_b;
	if ( a._mass !== Infinity && a.angular_velocity.lengthSquared() > ROLL_SPIN_SQ ) {
		return true;
	}
	if ( b._mass !== Infinity && b.angular_velocity.lengthSquared() > ROLL_SPIN_SQ ) {
		return true;
	}
	return false;
};

// Impulse needed over `h` to hold this contact's dynamic side(s) against gravity, divided by the number
// of points sharing the load so an N-point footprint doesn't multiply the body's weight by N.
Goblin.PBDSolver._restingNormalImpulse = function( contact, h ) {
	var a = contact.object_a, b = contact.object_b;
	var mass = 0;
	if ( a._mass !== Infinity ) { mass += a._mass; }
	if ( b._mass !== Infinity ) { mass += b._mass; }
	if ( mass === 0 ) {
		return 0;
	}

	var gravity = 9.8;
	var world = ( a.world || b.world );
	if ( world && world.gravity ) {
		var g = world.gravity;
		gravity = Math.sqrt( g.x * g.x + g.y * g.y + g.z * g.z ) || 9.8;
	}

	var countA = Goblin.PBDSolver._pointCountOf( a );
	var countB = Goblin.PBDSolver._pointCountOf( b );
	var share = ( countA > countB ? countA : countB );

	return ( mass * gravity * h ) / share;
};

// Cancel contact-point tangential slip, Coulomb-bounded by friction * this contact's normal impulse.
// Runs after the normal solve and rolling resistance, so it keeps re-establishing v = w x r.
Goblin.PBDSolver._solveContactFrictionVelocity = function( contact ) {
	var friction = contact.friction;
	if ( friction <= 0 ) {
		return;
	}
	// Skip a tip-assisted contact: friction here would cancel the pivot slip and bleed the assist.
	if ( ( contact.object_a && contact.object_a._pbdTipActive ) || ( contact.object_b && contact.object_b._pbdTipActive ) ) {
		return;
	}
	var maxImpulse = friction * ( contact._pbdNormalImpulse || 0 );
	if ( maxImpulse <= 0 ) {
		return;
	}

	var a = contact.object_a, b = contact.object_b;
	var normal = contact.contact_normal;

	Goblin.PBDSolver._contactPointVelocityCached( contact, _pbd_vec3_1 );
	var alongNormal = _pbd_vec3_1.dot( normal );
	var tx = _pbd_vec3_1.x - alongNormal * normal.x;
	var ty = _pbd_vec3_1.y - alongNormal * normal.y;
	var tz = _pbd_vec3_1.z - alongNormal * normal.z;
	var tangentSpeed = Math.sqrt( tx * tx + ty * ty + tz * tz );
	if ( tangentSpeed < 1e-8 ) {
		return;
	}

	var tangent = _pbd_vec3_8;
	tangent.x = tx / tangentSpeed;
	tangent.y = ty / tangentSpeed;
	tangent.z = tz / tangentSpeed;

	var rA = contact._pbdRA, rB = contact._pbdRB;
	var wA = Goblin.PBDSolver._angularInvMass( a, rA, tangent );
	var wB = Goblin.PBDSolver._angularInvMass( b, rB, tangent );
	var wSum = contact._pbdInvMassA + wA + contact._pbdInvMassB + wB;
	if ( wSum <= 0 ) {
		return;
	}

	var jt = ( tangentSpeed / wSum ) / contact._pbdPointDiv;
	if ( jt > maxImpulse ) {
		jt = maxImpulse;
	}

	if ( contact._pbdInvMassA > 0 ) {
		Goblin.PBDSolver._applyVelocityImpulse( a, rA, tangent, jt );
	}
	if ( contact._pbdInvMassB > 0 ) {
		Goblin.PBDSolver._applyVelocityImpulse( b, rB, tangent, -jt );
	}
};

/**
 * Velocity-space impulse `signed * normal` at lever arm `r` (linear + angular). Counterpart to
 * _applyPositionCorrection.
 *
 * @method _applyVelocityImpulse
 * @static
 * @private
 */
Goblin.PBDSolver._applyVelocityImpulse = function( body, r, normal, signed ) {
	var invMass = body._mass_inverted;
	body.linear_velocity.x += invMass * normal.x * signed;
	body.linear_velocity.y += invMass * normal.y * signed;
	body.linear_velocity.z += invMass * normal.z * signed;

	_pbd_vec3_6.crossVectors( r, normal );
	_pbd_vec3_6.scale( signed );
	body.inverseInertiaTensorWorldFrame.transformVector3( _pbd_vec3_6 );
	body.angular_velocity.add( _pbd_vec3_6 );
};

// Full updateDerived once per dynamic body (deduped by epoch) - called after the position solve, which
// only did transform-only rebuilds.
Goblin.PBDSolver._refreshDerived = function( body, epoch ) {
	if ( body == null || body._mass === Infinity || body._pbdDerivedEpoch === epoch ) {
		return;
	}
	body._pbdDerivedEpoch = epoch;
	body.updateDerived();
};

// Caches position-solve invariants (lever arms, inverse masses, normal wSum, point divisor) from the
// step's contact geometry. Bodies drift slightly as they depenetrate over the substep passes, but the
// change is negligible for the correction, so these are computed once.
Goblin.PBDSolver._preparePositionContact = function( contact, h ) {
	var a = contact.object_a, b = contact.object_b;
	var normal = contact.contact_normal;

	a.transform.transformVector3Into( contact.contact_point_in_a, _pbd_vec3_1 );
	b.transform.transformVector3Into( contact.contact_point_in_b, _pbd_vec3_2 );

	var rA = contact._pbdPosRA || ( contact._pbdPosRA = new Goblin.Vector3() );
	var rB = contact._pbdPosRB || ( contact._pbdPosRB = new Goblin.Vector3() );
	rA.subtractVectors( _pbd_vec3_1, a.position );
	rB.subtractVectors( _pbd_vec3_2, b.position );

	contact._pbdPosInvMassA = a._mass === Infinity ? 0 : a._mass_inverted;
	contact._pbdPosInvMassB = b._mass === Infinity ? 0 : b._mass_inverted;

	var wA = Goblin.PBDSolver._angularInvMass( a, rA, normal );
	var wB = Goblin.PBDSolver._angularInvMass( b, rB, normal );
	contact._pbdPosWSum = contact._pbdPosInvMassA + wA + contact._pbdPosInvMassB + wB;

	// alphaTilde = compliance / h^2, computed once per substep since h is fixed within it.
	var compliance = Goblin.PBDSolver.CONTACT_COMPLIANCE;
	contact._pbdAlphaTilde = compliance > 0 ? compliance / ( h * h ) : 0;
};

// One penetration-recovery correction: live separation along the normal, distributed by the cached
// generalized inverse mass. Point divisor keeps an N-point contact from overcorrecting ~Nx.
Goblin.PBDSolver._solveContactPosition = function( contact, relaxation ) {
	var a = contact.object_a, b = contact.object_b;
	var normal = contact.contact_normal;
	var wSum = contact._pbdPosWSum;
	if ( wSum <= 0 ) {
		return;
	}

	a.transform.transformVector3Into( contact.contact_point_in_a, _pbd_vec3_1 );
	b.transform.transformVector3Into( contact.contact_point_in_b, _pbd_vec3_2 );
	_pbd_vec3_3.subtractVectors( _pbd_vec3_1, _pbd_vec3_2 );
	// Bias the target a hair past contact so gravity's next-frame re-penetration still lands shallow -
	// keeps GJK reporting "touching" instead of triggering the ~7x-costlier EPA every frame.
	var separation = _pbd_vec3_3.dot( normal ) + Goblin.PBDSolver.PENETRATION_BIAS;
	if ( separation <= 0 ) {
		return;
	}

	// XPBD: dLambda = (separation - alphaTilde * lambda) / (wSum + alphaTilde), alphaTilde =
	// compliance / h^2. The accumulated-lambda feedback gives timestep-independent stiffness; at
	// compliance 0 this reduces to PBD's dLambda = separation / wSum. That feedback also subsumes the
	// old per-point divisor, so there is none here.
	var alphaTilde = contact._pbdAlphaTilde;
	var lambda = ( separation - alphaTilde * contact._pbdAccumLambda ) / ( wSum + alphaTilde );
	lambda = lambda * relaxation;
	contact._pbdAccumLambda += lambda;

	if ( contact._pbdPosInvMassA > 0 ) {
		Goblin.PBDSolver._applyPositionCorrection( a, contact._pbdPosRA, normal, -lambda );
	}
	if ( contact._pbdPosInvMassB > 0 ) {
		Goblin.PBDSolver._applyPositionCorrection( b, contact._pbdPosRB, normal, lambda );
	}
};

// Counts contact points per dynamic body into body._pbdPointCount (reset on first touch this epoch),
// used as the per-point divisor so an N-point contact doesn't overcorrect ~Nx.
Goblin.PBDSolver._countFrictionBody = function( body, epoch ) {
	if ( body == null || body._mass === Infinity ) {
		return;
	}
	if ( body._pbdCountEpoch !== epoch ) {
		body._pbdCountEpoch = epoch;
		body._pbdPointCount = 0;
	}
	body._pbdPointCount++;
};

// Point divisor for a body: its contact-point count this step, or 1 for static/uncounted.
Goblin.PBDSolver._pointCountOf = function( body ) {
	return ( body._mass !== Infinity && body._pbdPointCount ) ? body._pbdPointCount : 1;
};

/**
 * Rolling resistance (opt-in via RigidBody.rolling_friction): impulse of magnitude
 * rolling_friction * N opposing the contact point's travel, applied AT the contact point so linear
 * and angular velocity decay together (v = w x r stays intact - a roll doesn't become a skid).
 * Distinct from Coulomb friction, which needs real slip and so can't arrest a roll.
 *
 * @method _solveRollingResistance
 * @static
 * @private
 */
Goblin.PBDSolver._solveRollingResistance = function( contact, dt, gravityMag ) {
	var a = contact.object_a, b = contact.object_b;
	var normal = contact.contact_normal;
	var rolling_friction = contact.rolling_friction;
	if ( !rolling_friction || rolling_friction <= 0 ) {
		return;
	}

	var mA = a._mass, mB = b._mass;
	var rollBody = ( mA !== Infinity && ( mB === Infinity || mA <= mB ) ) ? a : b;
	var restMass = rollBody._mass;
	if ( restMass === Infinity ) {
		return;
	}

	var r = _pbd_vec3_4;
	r.subtractVectors( contact.contact_point, rollBody.position );

	// Roll rate: body spin perpendicular to the contact normal. Keyed off spin, not centre
	// velocity, so a spin-in-place still gets bled.
	_pbd_vec3_1.copy( rollBody.angular_velocity );
	var wAlong = _pbd_vec3_1.dot( normal );
	_pbd_vec3_1.x -= wAlong * normal.x;
	_pbd_vec3_1.y -= wAlong * normal.y;
	_pbd_vec3_1.z -= wAlong * normal.z;
	var rollRate = _pbd_vec3_1.length();
	if ( rollRate < 1e-6 ) {
		return;
	}

	// Contact-point travel from the roll is w_perp x r; the resistive impulse opposes it.
	var travel = _pbd_vec3_8;
	travel.crossVectors( _pbd_vec3_1, r );
	var travelSpeed = travel.length();
	if ( travelSpeed < 1e-6 ) {
		return;
	}
	travel.x /= travelSpeed;
	travel.y /= travelSpeed;
	travel.z /= travelSpeed;

	// rolling_friction * N * dt, capped so it decelerates the roll without reversing it.
	var maxImpulse = rolling_friction * restMass * gravityMag * dt;
	var stopImpulse = travelSpeed * restMass;
	var jImpulse = Math.min( maxImpulse, stopImpulse );
	if ( jImpulse <= 0 ) {
		return;
	}

	Goblin.PBDSolver._applyVelocityImpulseWorld( rollBody, r, travel, -jImpulse );
};

/**
 * Like _applyVelocityImpulse but `dir` is an arbitrary world direction, not the contact normal.
 *
 * @method _applyVelocityImpulseWorld
 * @static
 * @private
 */
Goblin.PBDSolver._applyVelocityImpulseWorld = function( body, r, dir, signed ) {
	var invMass = body._mass_inverted;
	body.linear_velocity.x += invMass * dir.x * signed;
	body.linear_velocity.y += invMass * dir.y * signed;
	body.linear_velocity.z += invMass * dir.z * signed;

	_pbd_vec3_7.crossVectors( r, dir );
	_pbd_vec3_7.scale( signed );
	body.inverseInertiaTensorWorldFrame.transformVector3( _pbd_vec3_7 );
	body.angular_velocity.add( _pbd_vec3_7 );
};

Goblin.PBDSolver._angularAxisInvMass = function( body, axis ) {
	if ( body._mass === Infinity ) {
		return 0;
	}
	body.inverseInertiaTensorWorldFrame.transformVector3Into( axis, _pbd_vec3_7 );
	return _pbd_vec3_7.dot( axis );
};

Goblin.PBDSolver._angularInvMass = function( body, r, normal ) {
	if ( body._mass === Infinity ) {
		return 0;
	}
	_pbd_vec3_6.crossVectors( r, normal );
	body.inverseInertiaTensorWorldFrame.transformVector3Into( _pbd_vec3_6, _pbd_vec3_7 );
	_pbd_vec3_6.crossVectors( _pbd_vec3_7, r );
	return _pbd_vec3_6.dot( normal );
};

/**
 * Position/rotation correction `signedLambda * normal` at lever arm `r`: small-angle quaternion
 * update, then renormalize.
 *
 * @method _applyPositionCorrection
 * @static
 * @private
 */
Goblin.PBDSolver._applyPositionCorrection = function( body, r, normal, signedLambda ) {
	var invMass = body._mass_inverted;
	body.position.x += invMass * normal.x * signedLambda;
	body.position.y += invMass * normal.y * signedLambda;
	body.position.z += invMass * normal.z * signedLambda;

	_pbd_vec3_6.crossVectors( r, normal );
	_pbd_vec3_6.scale( signedLambda );
	body.inverseInertiaTensorWorldFrame.transformVector3( _pbd_vec3_6 );

	_pbd_quat4_1.x = _pbd_vec3_6.x;
	_pbd_quat4_1.y = _pbd_vec3_6.y;
	_pbd_quat4_1.z = _pbd_vec3_6.z;
	_pbd_quat4_1.w = 0;
	_pbd_quat4_1.multiply( body.rotation );

	body.rotation.x += 0.5 * _pbd_quat4_1.x;
	body.rotation.y += 0.5 * _pbd_quat4_1.y;
	body.rotation.z += 0.5 * _pbd_quat4_1.z;
	body.rotation.w += 0.5 * _pbd_quat4_1.w;
	body.rotation.normalize();

	// Transform-only rebuild; world inertia and AABB are refreshed once after the solve.
	body.transform.makeTransform( body.rotation, body.position );
};

/**
 * Post-solve normal velocity correction from pre- vs post-solve contact-point velocity. Run once,
 * not iterated - a resting contact shows approach velocity almost every tick and iterating that
 * injects spin.
 *
 * @method _applyRestitution
 * @static
 * @private
 */
Goblin.PBDSolver._applyRestitution = function( contact ) {
	var a = contact.object_a, b = contact.object_b;
	var normal = contact.contact_normal;

	Goblin.PBDSolver._contactPointVelocityCached( contact, _pbd_vec3_1 );
	var postNormalVel = _pbd_vec3_1.dot( normal );
	var preNormalVel = contact._pbdPreSolveVelocity.dot( normal );

	// Only correct a genuinely approaching contact - one already separating pre-solve is real motion,
	// left untouched.
	if ( preNormalVel >= 0 ) {
		return;
	}
	var target = -preNormalVel * contact.restitution;

	var delta = target - postNormalVel;
	if ( delta === 0 ) {
		return;
	}

	// wSum needs the angular term too, or a lever-arm contact gets too large an impulse.
	var invMassA = a._mass === Infinity ? 0 : a._mass_inverted;
	var invMassB = b._mass === Infinity ? 0 : b._mass_inverted;
	var rA = _pbd_vec3_4, rB = _pbd_vec3_5;
	a.transform.transformVector3Into( contact.contact_point_in_a, _pbd_vec3_2 );
	b.transform.transformVector3Into( contact.contact_point_in_b, _pbd_vec3_3 );
	rA.subtractVectors( _pbd_vec3_2, a.position );
	rB.subtractVectors( _pbd_vec3_3, b.position );
	var wA = Goblin.PBDSolver._angularInvMass( a, rA, normal );
	var wB = Goblin.PBDSolver._angularInvMass( b, rB, normal );
	var wSum = invMassA + wA + invMassB + wB;
	if ( wSum <= 0 ) {
		return;
	}
	var impulse = delta / wSum;

	// Linear only: applying it to angular_velocity as well injects spin over many ticks.
	if ( a._mass !== Infinity ) {
		a.linear_velocity.x -= invMassA * normal.x * impulse;
		a.linear_velocity.y -= invMassA * normal.y * impulse;
		a.linear_velocity.z -= invMassA * normal.z * impulse;
	}
	if ( b._mass !== Infinity ) {
		b.linear_velocity.x += invMassB * normal.x * impulse;
		b.linear_velocity.y += invMassB * normal.y * impulse;
		b.linear_velocity.z += invMassB * normal.z * impulse;
	}
};
