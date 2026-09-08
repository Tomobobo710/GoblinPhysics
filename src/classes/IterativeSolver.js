/**
 * Adapted from BulletPhysics's btIterativeSolver. A projected-Gauss-Seidel velocity solver - one of
 * potentially several Goblin.Solver implementations (see Solver.js); World only depends on the
 * Goblin.Solver interface, not on this class specifically.
 *
 * @class IterativeSolver
 * @extends Goblin.Solver
 * @constructor
 */
Goblin.IterativeSolver = function() {
	Goblin.Solver.call( this );

	this.existing_contact_ids = {};

	/**
	 * Holds contact constraints generated from contact manifolds
	 *
	 * @property contact_constraints
	 * @type {Array}
	 */
	this.contact_constraints = [];

	/**
	 * Holds friction constraints generated from contact manifolds
	 *
	 * @property friction_constraints
	 * @type {Array}
	 */
	this.friction_constraints = [];

	/**
	 * array of all constraints being solved
	 *
	 * @property all_constraints
	 * @type {Array}
	 */
	this.all_constraints = [];

	/**
	 * maximum solver iterations per time step
	 *
	 * @property max_iterations
	 * @type {number}
	 */
	this.max_iterations = 10;

	/**
	 * maximum solver iterations per time step to resolve contacts
	 *
	 * @property penetrations_max_iterations
	 * @type {number}
	 */
	this.penetrations_max_iterations = 5;

	/**
	 * used to relax the contact position solver, 0 is no position correction and 1 is full correction
	 *
	 * @property relaxation
	 * @type {number}
	 * @default 0.9
	 */
	this.relaxation = 0.9;

	/**
	 * Overlap (in world units) left uncorrected at a resting contact. Bodies settle slightly
	 * interpenetrated and stop generating position correction, so a settled pile stops being nudged.
	 * Measured AFTER GjkEpa.margins, which already offsets reported depths.
	 *
	 * 0 reproduces the single-relaxation behavior.
	 *
	 * @property penetration_slop
	 * @type {number}
	 */
	this.penetration_slop = 0;

	/**
	 * Largest overlap correctable in one step. A deep overlap then recovers over several steps
	 * instead of one shove big enough to fling the body.
	 *
	 * Infinity reproduces the single-relaxation behavior.
	 *
	 * @property max_position_correction
	 * @type {number}
	 */
	this.max_position_correction = 0.01;

	/**
	 * Multiple of `max_position_correction` an overlap must exceed to count as a tunnel-in candidate
	 * (see _buildFlatPenetration). An isolated overlap past this depth is resolved in full for prompt
	 * recovery; anything shallower, or any overlap on a body that has several deep contacts at once
	 * (a settling pile), stays on the per-step cap. Set high enough that a resting stack's solver
	 * noise never crosses it - ~5x the cap.
	 *
	 * @property deep_contact_multiple
	 * @type {number}
	 */
	this.deep_contact_multiple = 5;

	/**
	 * Consecutive frames a contact must stay deep (see `deep_contact_multiple`) before it is granted
	 * full per-step correction. A genuine tunnel-in stays deep for many frames while it digs out; a
	 * settling pile's shuffle only blips a contact deep for a frame or two, and that blip must not
	 * earn an off-centre full-strength shove (which yaws the box). 1 disables the streak requirement.
	 *
	 * A pile's landing-impact phase is chaotic for a good fraction of a second, and during it a box
	 * can hold an isolated deep contact for a dozen-plus frames by chance; this is set past that so
	 * only a body that is genuinely stuck inside another (a tunnel-in that never resolves on its own)
	 * ever qualifies.
	 *
	 * @property deep_streak_frames
	 * @type {number}
	 */
	this.deep_streak_frames = 16;

	/**
	 * When true, position correction is projected onto the contact normal's vertical-vs-tangential
	 * split and the tangential part is scaled by tangential_correction. Separating two bodies should
	 * move them apart, not slide them past each other; unconstrained tangential correction shows up
	 * as lateral drift that friction cannot oppose (it never becomes velocity).
	 *
	 * 1 reproduces the single-relaxation behavior.
	 *
	 * @property tangential_correction
	 * @type {number}
	 */
	this.tangential_correction = 1;

	/**
	 * When true, penetration correction is applied as a pseudo-velocity integrated once per BODY,
	 * rather than as a separate position write per CONTACT CONSTRAINT.
	 *
	 * The per-constraint form applies a full correction for every contact a body has, so a box with
	 * ten contacts is displaced ten times in a step. In a deep stack those displacements accumulate
	 * sideways and show up as lateral drift that the velocity solver never sees and friction cannot
	 * oppose. The pseudo-velocity form uses the accumulated push/turn the penetration LCP already
	 * solves for, so each body moves once by the amount that actually resolves its overlap.
	 *
	 * @property split_impulse
	 * @type {boolean}
	 */
	this.split_impulse = false;

	/**
	 * weighting used in the Gauss-Seidel successive over-relaxation solver
	 *
	 * @property sor_weight
	 * @type {number}
	 */
	this.sor_weight = 0.85;

	/**
	 * how much of the solution to start with on the next solver pass
	 *
	 * @property warmstarting_factor
	 * @type {number}
	 */
	this.warmstarting_factor = 0.95;

	/**
	 * mass-normalized impulse below which a solver sweep counts as converged and the iteration loop
	 * stops early. Too loose and deep stacks exit with a large residual that shows up as bodies
	 * slowly rotating in place.
	 *
	 * @property convergence_epsilon
	 * @type {number}
	 */
	this.convergence_epsilon = 0.02;


	var solver = this;
	/**
	 * used to remove contact constraints from the system when their contacts are destroyed
	 *
	 * @method onContactDeactivate
	 * @private
	 */
	this.onContactDeactivate = function() {
		this.removeListener( 'deactivate', solver.onContactDeactivate );

		// Swap-remove using the constraint's tracked index instead of indexOf()'s O(n) scan.
		Goblin.IterativeSolver._swapRemove( solver.contact_constraints, this );

		delete solver.existing_contact_ids[ this.contact.uid ];
	};
	/**
	 * used to remove friction constraints from the system when their contacts are destroyed
	 *
	 * @method onFrictionDeactivate
	 * @private
	 */
	this.onFrictionDeactivate = function() {
		this.removeListener( 'deactivate', solver.onFrictionDeactivate );

		Goblin.IterativeSolver._swapRemove( solver.friction_constraints, this );
	};
};
Goblin.IterativeSolver.prototype = Object.create( Goblin.Solver.prototype );
Goblin.IterativeSolver.prototype.constructor = Goblin.IterativeSolver;

/**
 * Applies gravity once for the whole tick and integrates every body once - the same single-shot
 * integration World.step used to do inline before Solver.integrate existed. PGS has no need for
 * substeps the way XPBD does (see PBDSolver.prototype.integrate for why that one differs).
 *
 * @method integrate
 * @param rigid_bodies {Array}
 * @param gravity {Vector3}
 * @param time_delta {Number}
 */
Goblin.IterativeSolver.prototype.integrate = function( rigid_bodies, gravity, time_delta ) {
	var i, loop_count, body;

	for ( i = 0, loop_count = rigid_bodies.length; i < loop_count; i++ ) {
		body = rigid_bodies[i];
		if ( body._mass !== Infinity ) {
			_tmp_vec3_1.scaleVector( body.gravity || gravity, body._mass * time_delta );
			body.accumulated_force.add( _tmp_vec3_1 );
		}
	}

	for ( i = 0, loop_count = rigid_bodies.length; i < loop_count; i++ ) {
		rigid_bodies[i].integrate( time_delta );
	}
};

/**
 * Removes `item` from `array` in O(1) via its tracked _arrayIndex. Requires _pushTracked additions.
 *
 * @method _swapRemove
 * @static
 */
Goblin.IterativeSolver._swapRemove = function( array, item ) {
	var idx = item._arrayIndex;
	var last = array.length - 1;
	if ( idx !== last ) {
		array[idx] = array[last];
		array[idx]._arrayIndex = idx;
	}
	array.length = last;
	item._arrayIndex = -1;
};

/**
 * Pushes `item` onto `array`, recording its index for later O(1) removal via _swapRemove.
 *
 * @method _pushTracked
 * @static
 */
Goblin.IterativeSolver._pushTracked = function( array, item ) {
	item._arrayIndex = array.length;
	array.push( item );
};

// addConstraint/removeConstraint are inherited from Goblin.Solver - joint bookkeeping is the same
// flat-array logic regardless of how contacts are solved.

/**
 * Converts contact manifolds into contact constraints
 *
 * @method processContactManifolds
 * @param contact_manifolds {Array} contact manifolds to process
 */
Goblin.IterativeSolver.prototype.processContactManifolds = function( contact_manifolds ) {
	var i, j,
		manifold,
		contacts_length,
		contact,
		constraint;

	manifold = contact_manifolds.first;

	while( manifold ) {
		contacts_length = manifold.points.length;

		for ( i = 0; i < contacts_length; i++ ) {
			contact = manifold.points[i];

			var existing_constraint = this.existing_contact_ids.hasOwnProperty( contact.uid );

			if ( !existing_constraint ) {
				this.existing_contact_ids[contact.uid] = true;

				// Build contact constraint
				constraint = Goblin.ObjectPool.getObject( 'ContactConstraint' );
				constraint.buildFromContact( contact );
				// Which actual sub-shapes generated this (see NarrowPhase's _shapeKeyA/B stamping) —
				// used by solveNormalBlocks' pairing so a compound's independent contacts, which all
				// share one manifold, are never block-paired with each other.
				constraint._shapeKeyA = contact._shapeKeyA;
				constraint._shapeKeyB = contact._shapeKeyB;
				Goblin.IterativeSolver._pushTracked( this.contact_constraints, constraint );
				constraint.addListener( 'deactivate', this.onContactDeactivate );

				// Build friction constraint
				constraint = Goblin.ObjectPool.getObject( 'FrictionConstraint' );
				constraint.buildFromContact( contact );
				Goblin.IterativeSolver._pushTracked( this.friction_constraints, constraint );
				constraint.addListener( 'deactivate', this.onFrictionDeactivate );
			}
		}

		manifold = manifold.next_manifold;
	}

	// @TODO just for now
	this.all_constraints.length = 0;
	Array.prototype.push.apply( this.all_constraints, this.friction_constraints );
	Array.prototype.push.apply( this.all_constraints, this.constraints );
	Array.prototype.push.apply( this.all_constraints, this.contact_constraints );
};

Goblin.IterativeSolver.prototype.prepareConstraints = function( time_delta ) {
	var num_constraints = this.all_constraints.length,
		constraint,
		row,
		i, j;

	for ( i = 0; i < num_constraints; i++ ) {
		constraint = this.all_constraints[i];
		if ( constraint.active === false ) {
			continue;
		}

		constraint.update( time_delta );
		for ( j = 0; j < constraint.rows.length; j++ ) {
			row = constraint.rows[j];
			row.multiplier = 0;
			row.computeB( constraint ); // Objects' inverted mass & inertia tensors & Jacobian
			row.computeD();
			row.computeEta( constraint, time_delta ); // Amount of work needed for the constraint
		}
	}
};

Goblin.IterativeSolver.prototype._buildFlatPenetration = function() {
	var cc = this.contact_constraints;
	var n = cc.length;

	if ( !this._flatPenCap || this._flatPenCap < n ) {
		this._flatPenCap = Math.max( 64, n * 2 );
		this._flatPenJacobian = new Float64Array( this._flatPenCap * 12 );
		this._flatPenB = new Float64Array( this._flatPenCap * 12 );
		this._flatPenD = new Float64Array( this._flatPenCap );
		this._flatPenLower = new Float64Array( this._flatPenCap );
		this._flatPenUpper = new Float64Array( this._flatPenCap );
		this._flatPenMultiplier = new Float64Array( this._flatPenCap );
		this._flatPenDepth = new Float64Array( this._flatPenCap );
		this._flatPenBodyA = new Int32Array( this._flatPenCap );
		this._flatPenBodyB = new Int32Array( this._flatPenCap );
		this._flatPenRowRef = new Array( this._flatPenCap );
	}
	if ( !this._flatPenBodyCap || this._flatPenBodyCap < n * 2 + 2 ) {
		this._flatPenBodyCap = Math.max( 64, n * 2 + 2 );
		this._flatPenBodyPush = new Float64Array( this._flatPenBodyCap * 3 );
		this._flatPenBodyTurn = new Float64Array( this._flatPenBodyCap * 3 );
		this._flatPenBodyLinearFactor = new Float64Array( this._flatPenBodyCap * 3 );
		this._flatPenBodyAngularFactor = new Float64Array( this._flatPenBodyCap * 3 );
		this._flatPenBodyRef = new Array( this._flatPenBodyCap );
	}

	var flatJacobian = this._flatPenJacobian, flatB = this._flatPenB, flatD = this._flatPenD,
		flatLower = this._flatPenLower, flatUpper = this._flatPenUpper, flatMultiplier = this._flatPenMultiplier,
		flatDepth = this._flatPenDepth, flatBodyA = this._flatPenBodyA, flatBodyB = this._flatPenBodyB,
		flatRowRef = this._flatPenRowRef,
		flatBodyPush = this._flatPenBodyPush, flatBodyTurn = this._flatPenBodyTurn,
		flatBodyLinearFactor = this._flatPenBodyLinearFactor, flatBodyAngularFactor = this._flatPenBodyAngularFactor,
		flatBodyRef = this._flatPenBodyRef;

	var bodySlots = this._flatPenBodyMap = {};
	var bodyCount = 0;

	function bodySlotFor( body ) {
		if ( body == null || body._mass === Infinity ) {
			return -1;
		}
		var slot = bodySlots[ body.id ];
		if ( slot === undefined ) {
			slot = bodyCount++;
			bodySlots[ body.id ] = slot;
			flatBodyRef[slot] = body;
			var base3 = slot * 3;
			flatBodyPush[base3] = body.push_velocity.x;
			flatBodyPush[base3 + 1] = body.push_velocity.y;
			flatBodyPush[base3 + 2] = body.push_velocity.z;
			flatBodyTurn[base3] = body.turn_velocity.x;
			flatBodyTurn[base3 + 1] = body.turn_velocity.y;
			flatBodyTurn[base3 + 2] = body.turn_velocity.z;
			flatBodyLinearFactor[base3] = body.linear_factor.x;
			flatBodyLinearFactor[base3 + 1] = body.linear_factor.y;
			flatBodyLinearFactor[base3 + 2] = body.linear_factor.z;
			flatBodyAngularFactor[base3] = body.angular_factor.x;
			flatBodyAngularFactor[base3 + 1] = body.angular_factor.y;
			flatBodyAngularFactor[base3 + 2] = body.angular_factor.z;
		}
		return slot;
	}

	// Deep-contact count per body slot, for the tunnel-in vs stack-landing discriminator below.
	if ( !this._flatPenDeepCount || this._flatPenDeepCount.length < this._flatPenBodyCap ) {
		this._flatPenDeepCount = new Int32Array( this._flatPenBodyCap );
	}
	var deepCountByBody = this._flatPenDeepCount;

	var over_cap = this.max_position_correction;
	// A contact only counts as "deep" (tunnel-in candidate) well above the per-step cap - a settled
	// pile's contacts blip a hair over the cap from solver noise, and treating one of those as a
	// tunnel-in over-corrects it into lateral creep. Real tunnel-ins are an order of magnitude past it.
	var deep_threshold = over_cap * this.deep_contact_multiple;

	for ( var i = 0; i < n; i++ ) {
		var constraint = cc[i];
		var row = constraint.rows[0];
		var jb = i * 12;
		flatJacobian.set( row.jacobian, jb );
		flatB.set( row.B, jb );
		flatD[i] = row.D;
		flatLower[i] = row.lower_limit;
		flatUpper[i] = row.upper_limit;
		flatMultiplier[i] = row.multiplier;

		// Slop only, for now - the per-step cap is applied in the second pass, once we know how many
		// deep contacts each body has. Negative depths (separated pairs, within GjkEpa.margins) pass
		// through unchanged: the solve clamps them to zero via flatLower, and forcing them to zero here
		// measurably destabilizes resting contacts.
		var depth = constraint.contact.penetration_depth;
		if ( depth > 0 ) {
			depth -= this.penetration_slop;
			if ( depth < 0 ) { depth = 0; }
		}
		flatDepth[i] = depth;

		var sa = bodySlotFor( constraint.object_a );
		var sb = bodySlotFor( constraint.object_b );
		flatBodyA[i] = sa;
		flatBodyB[i] = sb;
		flatRowRef[i] = row;

		// A per-contact streak of consecutive deep frames. A real tunnel-in stays deep for many
		// frames while it digs out; a settling pile's shuffle only blips one contact deep for a frame
		// or two. Requiring a streak keeps that shuffle on the per-step cap (no off-centre full-
		// correction shove -> no yaw) while still fast-tracking a genuine punch-through.
		if ( depth > deep_threshold ) {
			constraint._deepStreak = ( constraint._deepStreak || 0 ) + 1;
			if ( constraint._deepStreak >= this.deep_streak_frames ) {
				if ( sa >= 0 ) { deepCountByBody[sa]++; }
				if ( sb >= 0 ) { deepCountByBody[sb]++; }
			}
		} else {
			constraint._deepStreak = 0;
		}
		flatDepth[i] = depth;   // (re-store: unchanged, keeps the assignment adjacent to the streak logic)
	}

	// Second pass: cap the per-step correction for a deep overlap ONLY when one of its bodies is in
	// more than one deep contact at once. That is the signature of a stack settling under its own
	// weight (or a pile's landing-impact frame): every box reports a fat overlap, the coupled LCP
	// solves one big consistent correction, and applying it in a single step makes the whole pile jump
	// and ring. An isolated deep overlap - a fast body that punched through a wall or a head - has no
	// such neighbours and is safe to resolve in full, which is what makes it stop looking buried.
	for ( var k = 0; k < n; k++ ) {
		if ( flatDepth[k] <= over_cap ) { continue; }
		var ka = flatBodyA[k], kb = flatBodyB[k];
		// Free pass (resolve in full) only for a deep overlap that is BOTH persistent (its own streak
		// has qualified - flatRowRef's constraint) AND isolated (neither body is in a second qualified
		// deep contact). Everything else is capped: a settling pile, a one-frame shuffle blip, or the
		// shallow-but-over-cap contacts that ring an isolated tunnel-in.
		var qualified = ( cc[k]._deepStreak || 0 ) >= this.deep_streak_frames;
		var lone = ( ka < 0 || deepCountByBody[ka] <= 1 ) && ( kb < 0 || deepCountByBody[kb] <= 1 );
		if ( !( qualified && lone && flatDepth[k] > deep_threshold ) ) {
			flatDepth[k] = over_cap;
		}
	}

	// Clear the scratch counts we used (only the slots we touched).
	for ( var c = 0; c < bodyCount; c++ ) { deepCountByBody[c] = 0; }

	this._flatPenCount = n;
	this._flatPenBodyCount = bodyCount;
};

Goblin.IterativeSolver.prototype._flushFlatPenetration = function() {
	var n = this._flatPenCount,
		flatMultiplier = this._flatPenMultiplier,
		flatRowRef = this._flatPenRowRef,
		i;
	for ( i = 0; i < n; i++ ) {
		flatRowRef[i].multiplier = flatMultiplier[i];
	}

	var bodyCount = this._flatPenBodyCount,
		flatBodyPush = this._flatPenBodyPush, flatBodyTurn = this._flatPenBodyTurn,
		flatBodyRef = this._flatPenBodyRef;
	for ( i = 0; i < bodyCount; i++ ) {
		var body = flatBodyRef[i];
		var base3 = i * 3;
		body.push_velocity.x = flatBodyPush[base3];
		body.push_velocity.y = flatBodyPush[base3 + 1];
		body.push_velocity.z = flatBodyPush[base3 + 2];
		body.turn_velocity.x = flatBodyTurn[base3];
		body.turn_velocity.y = flatBodyTurn[base3 + 1];
		body.turn_velocity.z = flatBodyTurn[base3 + 2];
	}
};

/**
 * Splits a position correction into its component along `normal` and the component across it, and
 * scales only the latter by `scale`. Separating two bodies should move them apart along the contact
 * normal; any across-normal component slides them, which is drift the velocity solver never sees.
 *
 * @method _dampTangential
 * @static
 * @private
 */
Goblin.IterativeSolver._dampTangential = function( correction, normal, scale ) {
	var along = correction.x * normal.x + correction.y * normal.y + correction.z * normal.z;
	var nx = normal.x * along, ny = normal.y * along, nz = normal.z * along;
	correction.x = nx + ( correction.x - nx ) * scale;
	correction.y = ny + ( correction.y - ny ) * scale;
	correction.z = nz + ( correction.z - nz ) * scale;
};

/**
 * Integrates each body's accumulated penetration pseudo-velocity into its position and orientation
 * ONCE, then clears it. Counterpart to the per-constraint position write - see `split_impulse`.
 *
 * push_velocity/turn_velocity already hold the solved correction for the whole step (the penetration
 * LCP accumulated them across every contact the body has), so applying them per body resolves the
 * overlap without displacing a heavily-contacted body once per contact.
 *
 * @method _applyPseudoVelocity
 * @private
 */
Goblin.IterativeSolver.prototype._applyPseudoVelocity = function() {
	var bodies = this._flatPenBodyRef, n = this._flatPenBodyCount, i;
	var relaxation = this.relaxation;
	var tangential_correction = this.tangential_correction;

	for ( i = 0; i < n; i++ ) {
		var body = bodies[i];
		if ( body == null || body._mass === Infinity ) { continue; }

		var push = body.push_velocity, turn = body.turn_velocity;

		_tmp_vec3_2.x = push.x * relaxation;
		_tmp_vec3_2.y = push.y * relaxation;
		_tmp_vec3_2.z = push.z * relaxation;

		// Damp sideways displacement without weakening the separating (vertical) part. The body's net
		// push direction stands in for a per-contact normal here, since this correction is the sum over
		// all of its contacts.
		if ( tangential_correction !== 1 ) {
			var len = Math.sqrt( _tmp_vec3_2.x * _tmp_vec3_2.x + _tmp_vec3_2.y * _tmp_vec3_2.y + _tmp_vec3_2.z * _tmp_vec3_2.z );
			if ( len > Goblin.EPSILON ) {
				_tmp_vec3_1.x = _tmp_vec3_2.x / len;
				_tmp_vec3_1.y = _tmp_vec3_2.y / len;
				_tmp_vec3_1.z = _tmp_vec3_2.z / len;
				Goblin.IterativeSolver._dampTangential( _tmp_vec3_2, _tmp_vec3_1, tangential_correction );
			}
		}

		body.position.x += _tmp_vec3_2.x;
		body.position.y += _tmp_vec3_2.y;
		body.position.z += _tmp_vec3_2.z;

		// Rotation stays on the per-constraint path. Summing every contact's angular correction
		// before applying it loses the cancellation between opposing contacts, and the small residual
		// that leaves is applied every step in a consistent direction - over a long run it compounds
		// into visible tilt. Translation has no such issue because opposing pushes sum to zero.

		push.x = push.y = push.z = 0;
		turn.x = turn.y = turn.z = 0;
	}
};

Goblin.IterativeSolver.prototype.resolveContacts = function() {
	var iteration,
		constraint,
		row, i,
		invmass;

	// Separate buffers from _buildFlatConstraints: this loop uses push_velocity/turn_velocity
	// (position correction) rather than solver_impulse (velocity solve), and only visits
	// contact_constraints, not the full all_constraints list.
	this._buildFlatPenetration();
	var flatJacobian = this._flatPenJacobian, flatB = this._flatPenB, flatD = this._flatPenD,
		flatLower = this._flatPenLower, flatUpper = this._flatPenUpper, flatMultiplier = this._flatPenMultiplier,
		flatDepth = this._flatPenDepth, flatBodyA = this._flatPenBodyA, flatBodyB = this._flatPenBodyB,
		flatBodyPush = this._flatPenBodyPush, flatBodyTurn = this._flatPenBodyTurn,
		flatBodyLinearFactor = this._flatPenBodyLinearFactor, flatBodyAngularFactor = this._flatPenBodyAngularFactor;
	var count = this._flatPenCount;
	var max_impulse = 0, jdot, delta_lambda;

	// Solve penetrations
	for ( iteration = 0; iteration < this.penetrations_max_iterations; iteration++ ) {
		max_impulse = 0;
		for ( i = 0; i < count; i++ ) {
			var jb = i * 12;
			var ba = flatBodyA[i], bb = flatBodyB[i];

			jdot = 0;
			if ( ba >= 0 ) {
				var iba = ba * 3;
				jdot += (
					flatJacobian[jb] * flatBodyLinearFactor[iba] * flatBodyPush[iba] +
					flatJacobian[jb + 1] * flatBodyLinearFactor[iba + 1] * flatBodyPush[iba + 1] +
					flatJacobian[jb + 2] * flatBodyLinearFactor[iba + 2] * flatBodyPush[iba + 2] +
					flatJacobian[jb + 3] * flatBodyAngularFactor[iba] * flatBodyTurn[iba] +
					flatJacobian[jb + 4] * flatBodyAngularFactor[iba + 1] * flatBodyTurn[iba + 1] +
					flatJacobian[jb + 5] * flatBodyAngularFactor[iba + 2] * flatBodyTurn[iba + 2]
				);
			}
			if ( bb >= 0 ) {
				var ibb = bb * 3;
				jdot += (
					flatJacobian[jb + 6] * flatBodyLinearFactor[ibb] * flatBodyPush[ibb] +
					flatJacobian[jb + 7] * flatBodyLinearFactor[ibb + 1] * flatBodyPush[ibb + 1] +
					flatJacobian[jb + 8] * flatBodyLinearFactor[ibb + 2] * flatBodyPush[ibb + 2] +
					flatJacobian[jb + 9] * flatBodyAngularFactor[ibb] * flatBodyTurn[ibb] +
					flatJacobian[jb + 10] * flatBodyAngularFactor[ibb + 1] * flatBodyTurn[ibb + 1] +
					flatJacobian[jb + 11] * flatBodyAngularFactor[ibb + 2] * flatBodyTurn[ibb + 2]
				);
			}

			delta_lambda = ( flatDepth[i] - jdot ) / flatD[i] || 0;
			var cache = flatMultiplier[i];
			var mult = Math.max( flatLower[i], Math.min( cache + delta_lambda, flatUpper[i] ) );
			flatMultiplier[i] = mult;
			delta_lambda = mult - cache;
			max_impulse = Math.max( max_impulse, delta_lambda );

			if ( ba >= 0 ) {
				var wba = ba * 3;
				flatBodyPush[wba] += delta_lambda * flatB[jb];
				flatBodyPush[wba + 1] += delta_lambda * flatB[jb + 1];
				flatBodyPush[wba + 2] += delta_lambda * flatB[jb + 2];
				flatBodyTurn[wba] += delta_lambda * flatB[jb + 3];
				flatBodyTurn[wba + 1] += delta_lambda * flatB[jb + 4];
				flatBodyTurn[wba + 2] += delta_lambda * flatB[jb + 5];
			}
			if ( bb >= 0 ) {
				var wbb = bb * 3;
				flatBodyPush[wbb] += delta_lambda * flatB[jb + 6];
				flatBodyPush[wbb + 1] += delta_lambda * flatB[jb + 7];
				flatBodyPush[wbb + 2] += delta_lambda * flatB[jb + 8];
				flatBodyTurn[wbb] += delta_lambda * flatB[jb + 9];
				flatBodyTurn[wbb + 1] += delta_lambda * flatB[jb + 10];
				flatBodyTurn[wbb + 2] += delta_lambda * flatB[jb + 11];
			}
		}

		if ( max_impulse >= -Goblin.EPSILON && max_impulse <= Goblin.EPSILON ) {
			break;
		}
	}

	this._flushFlatPenetration();

	// With split impulse the LINEAR correction is applied once per body from the accumulated
	// pseudo-velocity; the per-constraint loop below then handles rotation only.
	var split_linear = this.split_impulse;
	if ( split_linear ) {
		this._applyPseudoVelocity();
	}

	// Apply position/rotation solver
	//
	// The linear part is split into "along the contact normal" (separating the bodies, which is the
	// point) and "across it" (sliding them, which is not). Only the tangential part is scaled by
	// tangential_correction, so damping drift does not weaken separation.
	var tangential_correction = this.tangential_correction;
	var split_tangential = tangential_correction !== 1;

	for ( i = 0; i < this.contact_constraints.length; i++ ) {
		constraint = this.contact_constraints[i];
		row = constraint.rows[0];

		// Contact normal for this row, used to split the correction below.
		var cn = constraint.contact ? constraint.contact.contact_normal : null;

		if ( constraint.object_a != null && constraint.object_a._mass !== Infinity ) {
			invmass = constraint.object_a._mass_inverted;
			_tmp_vec3_2.x = invmass * row.jacobian[0] * constraint.object_a.linear_factor.x * row.multiplier * this.relaxation;
			_tmp_vec3_2.y = invmass * row.jacobian[1] * constraint.object_a.linear_factor.y * row.multiplier * this.relaxation;
			_tmp_vec3_2.z = invmass * row.jacobian[2] * constraint.object_a.linear_factor.z * row.multiplier * this.relaxation;
			if ( split_tangential && cn !== null ) {
				Goblin.IterativeSolver._dampTangential( _tmp_vec3_2, cn, tangential_correction );
			}
			if ( !split_linear ) {
				constraint.object_a.position.x += _tmp_vec3_2.x;
				constraint.object_a.position.y += _tmp_vec3_2.y;
				constraint.object_a.position.z += _tmp_vec3_2.z;
			}

			_tmp_vec3_1.x = row.jacobian[3] * constraint.object_a.angular_factor.x * row.multiplier * this.relaxation;
			_tmp_vec3_1.y = row.jacobian[4] * constraint.object_a.angular_factor.y * row.multiplier * this.relaxation;
			_tmp_vec3_1.z = row.jacobian[5] * constraint.object_a.angular_factor.z * row.multiplier * this.relaxation;
			constraint.object_a.inverseInertiaTensorWorldFrame.transformVector3( _tmp_vec3_1 );

			_tmp_quat4_1.x = _tmp_vec3_1.x;
			_tmp_quat4_1.y = _tmp_vec3_1.y;
			_tmp_quat4_1.z = _tmp_vec3_1.z;
			_tmp_quat4_1.w = 0;
			_tmp_quat4_1.multiply( constraint.object_a.rotation );

			constraint.object_a.rotation.x += 0.5 * _tmp_quat4_1.x;
			constraint.object_a.rotation.y += 0.5 * _tmp_quat4_1.y;
			constraint.object_a.rotation.z += 0.5 * _tmp_quat4_1.z;
			constraint.object_a.rotation.w += 0.5 * _tmp_quat4_1.w;
			constraint.object_a.rotation.normalize();
		}

		if ( constraint.object_b != null && constraint.object_b._mass !== Infinity ) {
			invmass = constraint.object_b._mass_inverted;
			_tmp_vec3_2.x = invmass * row.jacobian[6] * constraint.object_b.linear_factor.x * row.multiplier * this.relaxation;
			_tmp_vec3_2.y = invmass * row.jacobian[7] * constraint.object_b.linear_factor.y * row.multiplier * this.relaxation;
			_tmp_vec3_2.z = invmass * row.jacobian[8] * constraint.object_b.linear_factor.z * row.multiplier * this.relaxation;
			if ( split_tangential && cn !== null ) {
				Goblin.IterativeSolver._dampTangential( _tmp_vec3_2, cn, tangential_correction );
			}
			if ( !split_linear ) {
				constraint.object_b.position.x += _tmp_vec3_2.x;
				constraint.object_b.position.y += _tmp_vec3_2.y;
				constraint.object_b.position.z += _tmp_vec3_2.z;
			}

			_tmp_vec3_1.x = row.jacobian[9] * constraint.object_b.angular_factor.x * row.multiplier * this.relaxation;
			_tmp_vec3_1.y = row.jacobian[10] * constraint.object_b.angular_factor.y * row.multiplier * this.relaxation;
			_tmp_vec3_1.z = row.jacobian[11] * constraint.object_b.angular_factor.z * row.multiplier * this.relaxation;
			constraint.object_b.inverseInertiaTensorWorldFrame.transformVector3( _tmp_vec3_1 );

			_tmp_quat4_1.x = _tmp_vec3_1.x;
			_tmp_quat4_1.y = _tmp_vec3_1.y;
			_tmp_quat4_1.z = _tmp_vec3_1.z;
			_tmp_quat4_1.w = 0;
			_tmp_quat4_1.multiply( constraint.object_b.rotation );

			constraint.object_b.rotation.x += 0.5 * _tmp_quat4_1.x;
			constraint.object_b.rotation.y += 0.5 * _tmp_quat4_1.y;
			constraint.object_b.rotation.z += 0.5 * _tmp_quat4_1.z;
			constraint.object_b.rotation.w += 0.5 * _tmp_quat4_1.w;
			constraint.object_b.rotation.normalize();
		}

		row.multiplier = 0;
	}
};

/**
 * Packs every active row of `all_constraints` into flat typed-array SoA buffers, once per step, so
 * the hot per-iteration loops (`solveConstraints`, `resolveContacts`, `solveNormalBlocks`) never walk
 * `constraint.object_a.linear_factor.x`-style property chains — those are megamorphic across
 * RigidBody/RigidBodyProxy and, at ~4000 rows x 10 iterations/step, dominate solver time even though
 * the underlying math was already using typed arrays (jacobian/B/solver_impulse). This only touches
 * the repeated-every-iteration read/write path; row construction (computeB/computeD/computeEta,
 * restitution, warm-start setup) is untouched and still runs exactly as before, once per row per step,
 * before this flattening happens.
 *
 * Per row: jacobian[12], B[12] copied in; D, eta, lower_limit, upper_limit, factor, multiplier copied
 * in/out. Body state (solver_impulse[6], linear_factor[3], angular_factor[3], "is infinite mass")
 * is deduplicated per unique body across all rows into a separate per-body block, referenced by index
 * — a body touched by many contacts (the mesh, a stacked box) only gets one slot, and solver_impulse
 * writes during the iteration go directly into that shared slot exactly like the object-based version
 * did (writes are visible across rows referencing the same body within an iteration, matching
 * Gauss-Seidel semantics of the original code).
 *
 * @method _buildFlatConstraints
 * @private
 */
Goblin.IterativeSolver.prototype._buildFlatConstraints = function() {
	var all = this.all_constraints;
	var n = all.length;

	// Count active rows and assign each a flat index; assign each unique body a flat body-slot index,
	// keyed by body.id (a plain object map, not ES6 Map, to match this codebase's existing style —
	// see existing_contact_ids above).
	var rowCount = 0;
	var bodySlots = this._flatBodyMap = {};
	var bodyCount = 0;

	var i, j, constraint, row, bodyA, bodyB;
	for ( i = 0; i < n; i++ ) {
		constraint = all[i];
		if ( constraint.active === false ) { continue; }
		for ( j = 0; j < constraint.rows.length; j++ ) {
			rowCount++;
		}
	}

	// (Re)allocate flat buffers only when they need to grow, reused across steps otherwise.
	if ( !this._flatCap || this._flatCap < rowCount ) {
		this._flatCap = Math.max( 64, rowCount * 2 );
		this._flatJacobian = new Float64Array( this._flatCap * 12 );
		this._flatB = new Float64Array( this._flatCap * 12 );
		this._flatD = new Float64Array( this._flatCap );
		this._flatEta = new Float64Array( this._flatCap );
		this._flatLower = new Float64Array( this._flatCap );
		this._flatUpper = new Float64Array( this._flatCap );
		this._flatFactor = new Float64Array( this._flatCap );
		this._flatMultiplier = new Float64Array( this._flatCap );
		this._flatBodyA = new Int32Array( this._flatCap );
		this._flatBodyB = new Int32Array( this._flatCap );
		this._flatTotalMass = new Float64Array( this._flatCap );
		this._flatRowRef = new Array( this._flatCap ); // back-reference to the real ConstraintRow, to write results back
		this._flatConstraintRef = new Array( this._flatCap ); // back-reference to the real Constraint, for .factor/.active
	}
	if ( !this._flatBodyCap || this._flatBodyCap < n * 2 + 2 ) {
		this._flatBodyCap = Math.max( 64, n * 2 + 2 );
		this._flatBodyImpulse = new Float64Array( this._flatBodyCap * 6 );
		this._flatBodyLinearFactor = new Float64Array( this._flatBodyCap * 3 );
		this._flatBodyAngularFactor = new Float64Array( this._flatBodyCap * 3 );
		this._flatBodyRef = new Array( this._flatBodyCap );
	}

	var flatJacobian = this._flatJacobian, flatB = this._flatB, flatD = this._flatD, flatEta = this._flatEta,
		flatLower = this._flatLower, flatUpper = this._flatUpper, flatFactor = this._flatFactor,
		flatMultiplier = this._flatMultiplier, flatBodyA = this._flatBodyA, flatBodyB = this._flatBodyB,
		flatRowRef = this._flatRowRef, flatConstraintRef = this._flatConstraintRef,
		flatBodyImpulse = this._flatBodyImpulse, flatBodyLinearFactor = this._flatBodyLinearFactor,
		flatBodyAngularFactor = this._flatBodyAngularFactor, flatBodyRef = this._flatBodyRef,
		flatTotalMass = this._flatTotalMass;

	function bodySlotFor( body ) {
		if ( body == null || body._mass === Infinity ) {
			return -1;
		}
		// Keyed by body.id, not object identity: a compound child's RigidBodyProxy aliases its
		// parent's id, and several transient proxies for the same body must share one solver_impulse slot.
		var slot = bodySlots[ body.id ];
		if ( slot === undefined ) {
			slot = bodyCount++;
			bodySlots[ body.id ] = slot;
			flatBodyRef[slot] = body;
			var base3 = slot * 3;
			flatBodyLinearFactor[base3] = body.linear_factor.x;
			flatBodyLinearFactor[base3 + 1] = body.linear_factor.y;
			flatBodyLinearFactor[base3 + 2] = body.linear_factor.z;
			flatBodyAngularFactor[base3] = body.angular_factor.x;
			flatBodyAngularFactor[base3 + 1] = body.angular_factor.y;
			flatBodyAngularFactor[base3 + 2] = body.angular_factor.z;
			var base6 = slot * 6;
			flatBodyImpulse[base6] = body.solver_impulse[0];
			flatBodyImpulse[base6 + 1] = body.solver_impulse[1];
			flatBodyImpulse[base6 + 2] = body.solver_impulse[2];
			flatBodyImpulse[base6 + 3] = body.solver_impulse[3];
			flatBodyImpulse[base6 + 4] = body.solver_impulse[4];
			flatBodyImpulse[base6 + 5] = body.solver_impulse[5];
		}
		return slot;
	}

	var r = 0;
	for ( i = 0; i < n; i++ ) {
		constraint = all[i];
		if ( constraint.active === false ) { continue; }
		bodyA = bodySlotFor( constraint.object_a );
		bodyB = bodySlotFor( constraint.object_b );

		for ( j = 0; j < constraint.rows.length; j++ ) {
			row = constraint.rows[j];
			var jb = r * 12;
			flatJacobian.set( row.jacobian, jb );
			flatB.set( row.B, jb );
			flatD[r] = row.D;
			flatEta[r] = row.eta;
			flatLower[r] = row.lower_limit;
			flatUpper[r] = row.upper_limit;
			flatFactor[r] = constraint.factor;
			flatMultiplier[r] = row.multiplier;
			flatBodyA[r] = bodyA;
			flatBodyB[r] = bodyB;
			// Used only to normalize max_impulse's convergence check; masses don't change mid-step.
			flatTotalMass[r] = ( bodyA >= 0 ? constraint.object_a._mass : 0 ) + ( bodyB >= 0 ? constraint.object_b._mass : 0 );
			flatRowRef[r] = row;
			flatConstraintRef[r] = constraint;
			r++;
		}
	}

	this._flatRowCount = r;
	this._flatBodyCount = bodyCount;
};

/**
 * Writes flat-buffer state (multiplier, solver_impulse) back onto the real ConstraintRow/RigidBody
 * objects the rest of the engine (applyConstraints, position solve, warm-start caching) reads from.
 *
 * @method _flushFlatConstraints
 * @private
 */
Goblin.IterativeSolver.prototype._flushFlatConstraints = function() {
	var rowCount = this._flatRowCount,
		flatMultiplier = this._flatMultiplier,
		flatRowRef = this._flatRowRef,
		i;
	for ( i = 0; i < rowCount; i++ ) {
		flatRowRef[i].multiplier = flatMultiplier[i];
	}

	var bodyCount = this._flatBodyCount,
		flatBodyImpulse = this._flatBodyImpulse,
		flatBodyRef = this._flatBodyRef;
	for ( i = 0; i < bodyCount; i++ ) {
		var body = flatBodyRef[i];
		var base6 = i * 6;
		body.solver_impulse[0] = flatBodyImpulse[base6];
		body.solver_impulse[1] = flatBodyImpulse[base6 + 1];
		body.solver_impulse[2] = flatBodyImpulse[base6 + 2];
		body.solver_impulse[3] = flatBodyImpulse[base6 + 3];
		body.solver_impulse[4] = flatBodyImpulse[base6 + 4];
		body.solver_impulse[5] = flatBodyImpulse[base6 + 5];
	}
};

/**
 * Cheap variant of _flushFlatConstraints/_buildFlatConstraints used around solveNormalBlocks: only
 * `multiplier` and `solver_impulse` change during the solve, so only those need to round-trip through
 * the real objects — jacobian/B/D/eta/lower/upper/factor are set once in prepareConstraints and never
 * change mid-step, so re-copying them every iteration (what a full rebuild would do) is pure waste.
 *
 * @method _syncFlatToReal
 * @private
 */
Goblin.IterativeSolver.prototype._syncFlatToReal = function() {
	this._flushFlatConstraints();
};

/**
 * @method _syncRealToFlat
 * @private
 */
Goblin.IterativeSolver.prototype._syncRealToFlat = function() {
	var rowCount = this._flatRowCount,
		flatMultiplier = this._flatMultiplier,
		flatRowRef = this._flatRowRef,
		i;
	for ( i = 0; i < rowCount; i++ ) {
		flatMultiplier[i] = flatRowRef[i].multiplier;
	}

	var bodyCount = this._flatBodyCount,
		flatBodyImpulse = this._flatBodyImpulse,
		flatBodyRef = this._flatBodyRef;
	for ( i = 0; i < bodyCount; i++ ) {
		var body = flatBodyRef[i];
		var base6 = i * 6;
		flatBodyImpulse[base6] = body.solver_impulse[0];
		flatBodyImpulse[base6 + 1] = body.solver_impulse[1];
		flatBodyImpulse[base6 + 2] = body.solver_impulse[2];
		flatBodyImpulse[base6 + 3] = body.solver_impulse[3];
		flatBodyImpulse[base6 + 4] = body.solver_impulse[4];
		flatBodyImpulse[base6 + 5] = body.solver_impulse[5];
	}
};

Goblin.IterativeSolver.prototype.solveConstraints = function() {
	var num_constraints = this.all_constraints.length,
		constraint,
		num_rows,
		row,
		warmth,
		i, j;

	var iteration,
		delta_lambda,
		max_impulse = 0, // Track the largest impulse per iteration; if the impulse is <= EPSILON then early out
		jdot;

	// Warm starting
	for ( i = 0; i < num_constraints; i++ ) {
		constraint = this.all_constraints[i];
		if ( constraint.active === false ) {
			continue;
		}

		for ( j = 0; j < constraint.rows.length; j++ ) {
			row = constraint.rows[j];
			warmth = row.multiplier_cached * this.warmstarting_factor;
			row.multiplier = warmth;

			if ( constraint.object_a && constraint.object_a._mass !== Infinity ) {
				constraint.object_a.solver_impulse[0] += warmth * row.B[0];
				constraint.object_a.solver_impulse[1] += warmth * row.B[1];
				constraint.object_a.solver_impulse[2] += warmth * row.B[2];

				constraint.object_a.solver_impulse[3] += warmth * row.B[3];
				constraint.object_a.solver_impulse[4] += warmth * row.B[4];
				constraint.object_a.solver_impulse[5] += warmth * row.B[5];
			}
			if ( constraint.object_b && constraint.object_b._mass !== Infinity ) {
				constraint.object_b.solver_impulse[0] += warmth * row.B[6];
				constraint.object_b.solver_impulse[1] += warmth * row.B[7];
				constraint.object_b.solver_impulse[2] += warmth * row.B[8];

				constraint.object_b.solver_impulse[3] += warmth * row.B[9];
				constraint.object_b.solver_impulse[4] += warmth * row.B[10];
				constraint.object_b.solver_impulse[5] += warmth * row.B[11];
			}
		}
	}

	this._buildFlatConstraints();
	this._buildNormalBlockPairsFlat();
	var flatJacobian = this._flatJacobian, flatB = this._flatB, flatD = this._flatD, flatEta = this._flatEta,
		flatLower = this._flatLower, flatUpper = this._flatUpper, flatFactor = this._flatFactor,
		flatMultiplier = this._flatMultiplier, flatBodyA = this._flatBodyA, flatBodyB = this._flatBodyB,
		flatBodyImpulse = this._flatBodyImpulse, flatBodyLinearFactor = this._flatBodyLinearFactor,
		flatBodyAngularFactor = this._flatBodyAngularFactor,
		flatConstraintRef = this._flatConstraintRef, flatTotalMass = this._flatTotalMass;
	var rowCount = this._flatRowCount;

	var debugResiduals = Goblin.IterativeSolver._debugTrackResiduals;

	for ( iteration = 0; iteration < this.max_iterations; iteration++ ) {
		max_impulse = 0;
		var liveRows = 0;

		for ( var r = 0; r < rowCount; r++ ) {
			var jb = r * 12;
			var ba = flatBodyA[r], bb = flatBodyB[r];

			jdot = 0;
			if ( ba >= 0 ) {
				var iba = ba * 6, lfa = ba * 3, afa = ba * 3;
				jdot += (
					flatJacobian[jb] * flatBodyLinearFactor[lfa] * flatBodyImpulse[iba] +
					flatJacobian[jb + 1] * flatBodyLinearFactor[lfa + 1] * flatBodyImpulse[iba + 1] +
					flatJacobian[jb + 2] * flatBodyLinearFactor[lfa + 2] * flatBodyImpulse[iba + 2] +
					flatJacobian[jb + 3] * flatBodyAngularFactor[afa] * flatBodyImpulse[iba + 3] +
					flatJacobian[jb + 4] * flatBodyAngularFactor[afa + 1] * flatBodyImpulse[iba + 4] +
					flatJacobian[jb + 5] * flatBodyAngularFactor[afa + 2] * flatBodyImpulse[iba + 5]
				);
			}
			if ( bb >= 0 ) {
				var ibb = bb * 6, lfb = bb * 3, afb = bb * 3;
				jdot += (
					flatJacobian[jb + 6] * flatBodyLinearFactor[lfb] * flatBodyImpulse[ibb] +
					flatJacobian[jb + 7] * flatBodyLinearFactor[lfb + 1] * flatBodyImpulse[ibb + 1] +
					flatJacobian[jb + 8] * flatBodyLinearFactor[lfb + 2] * flatBodyImpulse[ibb + 2] +
					flatJacobian[jb + 9] * flatBodyAngularFactor[afb] * flatBodyImpulse[ibb + 3] +
					flatJacobian[jb + 10] * flatBodyAngularFactor[afb + 1] * flatBodyImpulse[ibb + 4] +
					flatJacobian[jb + 11] * flatBodyAngularFactor[afb + 2] * flatBodyImpulse[ibb + 5]
				);
			}

			delta_lambda = ( ( flatEta[r] - jdot ) / flatD[r] || 0 ) * flatFactor[r];
			var cache = flatMultiplier[r],
				multiplier_target = cache + delta_lambda;

			multiplier_target = this.sor_weight * multiplier_target + ( 1 - this.sor_weight ) * cache;

			var mult = Math.max( flatLower[r], Math.min( multiplier_target, flatUpper[r] ) );
			flatMultiplier[r] = mult;

			delta_lambda = mult - cache;

			var rowImpulse = Math.abs( delta_lambda ) / flatTotalMass[r];
			max_impulse = Math.max( max_impulse, rowImpulse );
			if ( debugResiduals && rowImpulse > 0.1 ) { liveRows++; }

			if ( ba >= 0 ) {
				var wba = ba * 6;
				flatBodyImpulse[wba] += delta_lambda * flatB[jb];
				flatBodyImpulse[wba + 1] += delta_lambda * flatB[jb + 1];
				flatBodyImpulse[wba + 2] += delta_lambda * flatB[jb + 2];
				flatBodyImpulse[wba + 3] += delta_lambda * flatB[jb + 3];
				flatBodyImpulse[wba + 4] += delta_lambda * flatB[jb + 4];
				flatBodyImpulse[wba + 5] += delta_lambda * flatB[jb + 5];
			}
			if ( bb >= 0 ) {
				var wbb = bb * 6;
				flatBodyImpulse[wbb] += delta_lambda * flatB[jb + 6];
				flatBodyImpulse[wbb + 1] += delta_lambda * flatB[jb + 7];
				flatBodyImpulse[wbb + 2] += delta_lambda * flatB[jb + 8];
				flatBodyImpulse[wbb + 3] += delta_lambda * flatB[jb + 9];
				flatBodyImpulse[wbb + 4] += delta_lambda * flatB[jb + 10];
				flatBodyImpulse[wbb + 5] += delta_lambda * flatB[jb + 11];
			}
		}

		// Block-solve paired normal contacts (2-point manifolds) as a coupled 2x2 each sweep — see
		// solveNormalBlocksFlat. Its impulses count toward convergence too, otherwise the loop can
		// exit while the block solve is still making large corrections.
		max_impulse = Math.max( max_impulse, this.solveNormalBlocksFlat() );

		if ( Goblin.IterativeSolver._debugMaxImpulseTrace ) { Goblin.IterativeSolver._debugMaxImpulseTrace.push( max_impulse ); }
		if ( debugResiduals && iteration === this.max_iterations - 1 ) {
			Goblin.IterativeSolver._debugLiveRowsAtEnd = liveRows;
			Goblin.IterativeSolver._debugTotalRowsAtEnd = rowCount;
		}
		if ( max_impulse <= this.convergence_epsilon ) {
			break;
		}
	}
	Goblin.IterativeSolver._lastIterationCount = iteration + 1;

	this._flushFlatConstraints();
};

/**
 * Block-solves the normal rows of a 2-point contact manifold as a coupled 2x2 LCP (Catto/Box2D
 * style). Two normal ContactConstraints on the same body pair are the ends of one resting manifold;
 * solved sequentially, each one's impulse applies a torque that violates the other, leaving an
 * antisymmetric residual that spins a low-inertia body up from rest and that more iterations only
 * worsen. Solving both normals together, with the [0, inf] one-sided limit enumerated over four
 * cases, cancels the cross-coupling in one shot.
 *
 * Operates on the flat typed arrays (_flatJacobian/_flatB/_flatD/_flatEta/_flatMultiplier/
 * _flatBodyImpulse), addressed by flat row index rather than by Constraint/ConstraintRow object.
 *
 * @method solveNormalBlocksFlat
 */
/**
 * Precomputes, as pairs of FLAT ROW INDICES, which contact_constraints are the two points of one
 * shape-pair's contact — keyed by the actual sub-shapes involved (see NarrowPhase's _shapeKeyA/B),
 * not just the top-level body pair, so a compound body's several independent contacts against the
 * same other body (which all share one manifold) are never wrongly block-paired with each other.
 *
 * Must run after _buildFlatConstraints (needs _flatRowRef to map a ConstraintRow back to its flat
 * index).
 *
 * @method _buildNormalBlockPairsFlat
 */
Goblin.IterativeSolver.prototype._buildNormalBlockPairsFlat = function() {
	var cc = this.contact_constraints;
	var n = cc.length;
	var rowCount = this._flatRowCount;
	var flatRowRef = this._flatRowRef;

	// Row -> flat index, built fresh each step; only covers active rows already in the flat set.
	var rowToFlat = {};
	var i;
	for ( i = 0; i < rowCount; i++ ) {
		rowToFlat[ flatRowRef[i]._uid || ( flatRowRef[i]._uid = ++Goblin.IterativeSolver._rowUidCounter ) ] = i;
	}

	if ( !this._normalBlockPairsCap || this._normalBlockPairsCap < n ) {
		this._normalBlockPairsCap = Math.max( 64, n * 2 );
		this._normalBlockPairs = new Int32Array( this._normalBlockPairsCap );
	}
	var pairs = this._normalBlockPairs;
	var pairCount = 0;

	// Only pairs the first two contacts sharing a shape-pair key; a manifold's 3rd/4th point (if any)
	// falls through to the plain per-row solve.
	var byKey = {};
	for ( i = 0; i < n; i++ ) {
		var c = cc[i];
		if ( c.active === false || c._shapeKeyA == null || c._shapeKeyB == null ) {
			continue;
		}
		var row0 = c.rows[0];
		var flatIdx = rowToFlat[ row0._uid ];
		if ( flatIdx === undefined ) {
			continue;
		}
		// Key on the BODY pair as well as the shape pair. Shape ids alone collide whenever many bodies
		// share one shape instance (a stack of identical boxes), which would block-pair two contacts
		// that have no body in common - the 2x2 solve then couples unrelated rows and injects energy.
		var bodyIdA = c.object_a ? c.object_a.id : -1;
		var bodyIdB = c.object_b ? c.object_b.id : -1;
		var bodyKey = bodyIdA < bodyIdB ? ( bodyIdA + ':' + bodyIdB ) : ( bodyIdB + ':' + bodyIdA );
		var shapeKey = c._shapeKeyA.id < c._shapeKeyB.id ? ( c._shapeKeyA.id + '_' + c._shapeKeyB.id ) : ( c._shapeKeyB.id + '_' + c._shapeKeyA.id );
		var key = bodyKey + '|' + shapeKey;
		if ( byKey.hasOwnProperty( key ) ) {
			var partnerIdx = byKey[key];
			pairs[pairCount++] = partnerIdx;
			pairs[pairCount++] = flatIdx;
			delete byKey[key];
		} else {
			byKey[key] = flatIdx;
		}
	}
	this._normalBlockPairCount = pairCount;
};
Goblin.IterativeSolver._rowUidCounter = 0;

Goblin.IterativeSolver.prototype.solveNormalBlocksFlat = function() {
	var pairs = this._normalBlockPairs;
	var n = this._normalBlockPairCount;
	var flatJacobian = this._flatJacobian, flatB = this._flatB, flatD = this._flatD, flatEta = this._flatEta,
		flatMultiplier = this._flatMultiplier, flatBodyA = this._flatBodyA, flatBodyB = this._flatBodyB,
		flatBodyImpulse = this._flatBodyImpulse, flatBodyLinearFactor = this._flatBodyLinearFactor,
		flatBodyAngularFactor = this._flatBodyAngularFactor, flatUpper = this._flatUpper,
		flatTotalMass = this._flatTotalMass;

	// Largest mass-normalized multiplier change this pass, so solveConstraints can fold it into its
	// convergence test (same normalization the sequential sweep uses).
	var maxImpulse = 0;
	function noteDelta( row, delta ) {
		var m = flatTotalMass[row];
		var v = m > 0 ? Math.abs( delta ) / m : Math.abs( delta );
		if ( v > maxImpulse ) { maxImpulse = v; }
	}

	for ( var a = 0; a < n; a += 2 ) {
		var r1 = pairs[a], r2 = pairs[a + 1];
		var jb1 = r1 * 12, jb2 = r2 * 12;
		var ba1 = flatBodyA[r1], bb1 = flatBodyB[r1];
		var ba2 = flatBodyA[r2], bb2 = flatBodyB[r2];

		var jdot1 = Goblin.IterativeSolver._flatRowJdot( flatJacobian, flatBodyImpulse, flatBodyLinearFactor, flatBodyAngularFactor, jb1, ba1, bb1 );
		var jdot2 = Goblin.IterativeSolver._flatRowJdot( flatJacobian, flatBodyImpulse, flatBodyLinearFactor, flatBodyAngularFactor, jb2, ba2, bb2 );

		var K11 = flatD[r1], K22 = flatD[r2];
		if ( K11 <= 0 || K22 <= 0 ) {
			continue;
		}
		var K12 = Goblin.IterativeSolver._flatRowCross( flatJacobian, flatB, jb1, jb2 );

		var x1 = flatMultiplier[r1], x2 = flatMultiplier[r2];
		// "velocity if these two rows' impulses were zero"
		var av1 = jdot1 - ( K11 * x1 + K12 * x2 );
		var av2 = jdot2 - ( K12 * x1 + K22 * x2 );
		// want A x' + a = eta  ->  A x' = (eta - a)
		var rhs1 = flatEta[r1] - av1;
		var rhs2 = flatEta[r2] - av2;

		var nx1, nx2;
		var det = K11 * K22 - K12 * K12;

		// Case 1: both active
		if ( Math.abs( det ) > 1e-12 ) {
			nx1 = ( rhs1 * K22 - rhs2 * K12 ) / det;
			nx2 = ( rhs2 * K11 - rhs1 * K12 ) / det;
			if ( nx1 >= 0 && nx2 >= 0 && nx1 <= flatUpper[r1] && nx2 <= flatUpper[r2] ) {
				Goblin.IterativeSolver._flatApplyBlock( flatBodyImpulse, flatB, jb1, ba1, bb1, nx1 - x1 );
				Goblin.IterativeSolver._flatApplyBlock( flatBodyImpulse, flatB, jb2, ba2, bb2, nx2 - x2 );
				noteDelta( r1, nx1 - x1 ); noteDelta( r2, nx2 - x2 );
				flatMultiplier[r1] = nx1; flatMultiplier[r2] = nx2;
				continue;
			}
		}
		// Case 2: only point 1
		nx1 = rhs1 / K11;
		if ( nx1 >= 0 && nx1 <= flatUpper[r1] && ( K12 * nx1 + av2 ) >= flatEta[r2] ) {
			Goblin.IterativeSolver._flatApplyBlock( flatBodyImpulse, flatB, jb1, ba1, bb1, nx1 - x1 );
			Goblin.IterativeSolver._flatApplyBlock( flatBodyImpulse, flatB, jb2, ba2, bb2, 0 - x2 );
			noteDelta( r1, nx1 - x1 ); noteDelta( r2, 0 - x2 );
			flatMultiplier[r1] = nx1; flatMultiplier[r2] = 0;
			continue;
		}
		// Case 3: only point 2
		nx2 = rhs2 / K22;
		if ( nx2 >= 0 && nx2 <= flatUpper[r2] && ( K12 * nx2 + av1 ) >= flatEta[r1] ) {
			Goblin.IterativeSolver._flatApplyBlock( flatBodyImpulse, flatB, jb1, ba1, bb1, 0 - x1 );
			Goblin.IterativeSolver._flatApplyBlock( flatBodyImpulse, flatB, jb2, ba2, bb2, nx2 - x2 );
			noteDelta( r1, 0 - x1 ); noteDelta( r2, nx2 - x2 );
			flatMultiplier[r1] = 0; flatMultiplier[r2] = nx2;
			continue;
		}
		// Case 4: neither
		if ( av1 >= flatEta[r1] && av2 >= flatEta[r2] ) {
			Goblin.IterativeSolver._flatApplyBlock( flatBodyImpulse, flatB, jb1, ba1, bb1, 0 - x1 );
			Goblin.IterativeSolver._flatApplyBlock( flatBodyImpulse, flatB, jb2, ba2, bb2, 0 - x2 );
			noteDelta( r1, 0 - x1 ); noteDelta( r2, 0 - x2 );
			flatMultiplier[r1] = 0; flatMultiplier[r2] = 0;
		}
	}

	return maxImpulse;
};

Goblin.IterativeSolver._flatApplyBlock = function( flatBodyImpulse, flatB, jb, ba, bb, delta_lambda ) {
	if ( delta_lambda === 0 ) {
		return;
	}
	if ( ba >= 0 ) {
		var wba = ba * 6;
		flatBodyImpulse[wba] += delta_lambda * flatB[jb];
		flatBodyImpulse[wba + 1] += delta_lambda * flatB[jb + 1];
		flatBodyImpulse[wba + 2] += delta_lambda * flatB[jb + 2];
		flatBodyImpulse[wba + 3] += delta_lambda * flatB[jb + 3];
		flatBodyImpulse[wba + 4] += delta_lambda * flatB[jb + 4];
		flatBodyImpulse[wba + 5] += delta_lambda * flatB[jb + 5];
	}
	if ( bb >= 0 ) {
		var wbb = bb * 6;
		flatBodyImpulse[wbb] += delta_lambda * flatB[jb + 6];
		flatBodyImpulse[wbb + 1] += delta_lambda * flatB[jb + 7];
		flatBodyImpulse[wbb + 2] += delta_lambda * flatB[jb + 8];
		flatBodyImpulse[wbb + 3] += delta_lambda * flatB[jb + 9];
		flatBodyImpulse[wbb + 4] += delta_lambda * flatB[jb + 10];
		flatBodyImpulse[wbb + 5] += delta_lambda * flatB[jb + 11];
	}
};

Goblin.IterativeSolver._flatRowJdot = function( flatJacobian, flatBodyImpulse, flatBodyLinearFactor, flatBodyAngularFactor, jb, ba, bb ) {
	var jdot = 0;
	if ( ba >= 0 ) {
		var iba = ba * 6, lfa = ba * 3, afa = ba * 3;
		jdot += (
			flatJacobian[jb] * flatBodyLinearFactor[lfa] * flatBodyImpulse[iba] +
			flatJacobian[jb + 1] * flatBodyLinearFactor[lfa + 1] * flatBodyImpulse[iba + 1] +
			flatJacobian[jb + 2] * flatBodyLinearFactor[lfa + 2] * flatBodyImpulse[iba + 2] +
			flatJacobian[jb + 3] * flatBodyAngularFactor[afa] * flatBodyImpulse[iba + 3] +
			flatJacobian[jb + 4] * flatBodyAngularFactor[afa + 1] * flatBodyImpulse[iba + 4] +
			flatJacobian[jb + 5] * flatBodyAngularFactor[afa + 2] * flatBodyImpulse[iba + 5]
		);
	}
	if ( bb >= 0 ) {
		var ibb = bb * 6, lfb = bb * 3, afb = bb * 3;
		jdot += (
			flatJacobian[jb + 6] * flatBodyLinearFactor[lfb] * flatBodyImpulse[ibb] +
			flatJacobian[jb + 7] * flatBodyLinearFactor[lfb + 1] * flatBodyImpulse[ibb + 1] +
			flatJacobian[jb + 8] * flatBodyLinearFactor[lfb + 2] * flatBodyImpulse[ibb + 2] +
			flatJacobian[jb + 9] * flatBodyAngularFactor[afb] * flatBodyImpulse[ibb + 3] +
			flatJacobian[jb + 10] * flatBodyAngularFactor[afb + 1] * flatBodyImpulse[ibb + 4] +
			flatJacobian[jb + 11] * flatBodyAngularFactor[afb + 2] * flatBodyImpulse[ibb + 5]
		);
	}
	return jdot;
};

Goblin.IterativeSolver._flatRowCross = function( flatJacobian, flatB, jb1, jb2 ) {
	return (
		flatJacobian[jb1] * flatB[jb2] + flatJacobian[jb1 + 1] * flatB[jb2 + 1] + flatJacobian[jb1 + 2] * flatB[jb2 + 2] +
		flatJacobian[jb1 + 3] * flatB[jb2 + 3] + flatJacobian[jb1 + 4] * flatB[jb2 + 4] + flatJacobian[jb1 + 5] * flatB[jb2 + 5] +
		flatJacobian[jb1 + 6] * flatB[jb2 + 6] + flatJacobian[jb1 + 7] * flatB[jb2 + 7] + flatJacobian[jb1 + 8] * flatB[jb2 + 8] +
		flatJacobian[jb1 + 9] * flatB[jb2 + 9] + flatJacobian[jb1 + 10] * flatB[jb2 + 10] + flatJacobian[jb1 + 11] * flatB[jb2 + 11]
	);
};

Goblin.IterativeSolver.prototype.applyConstraints = function( time_delta ) {
	var num_constraints = this.all_constraints.length,
		constraint,
		num_rows,
		row,
		i, j,
		invmass;

	for ( i = 0; i < num_constraints; i++ ) {
		constraint = this.all_constraints[i];
		if ( constraint.active === false ) {
			continue;
		}
		num_rows = constraint.rows.length;

		constraint.last_impulse.x = constraint.last_impulse.y = constraint.last_impulse.z = 0;

		for ( j = 0; j < num_rows; j++ ) {
			row = constraint.rows[j];
			row.multiplier_cached = row.multiplier;

			if ( constraint.object_a != null && constraint.object_a._mass !== Infinity ) {
				invmass = constraint.object_a._mass_inverted;
				_tmp_vec3_2.x = invmass * time_delta * row.jacobian[0] * constraint.object_a.linear_factor.x * row.multiplier;
				_tmp_vec3_2.y = invmass * time_delta * row.jacobian[1] * constraint.object_a.linear_factor.y * row.multiplier;
				_tmp_vec3_2.z = invmass * time_delta * row.jacobian[2] * constraint.object_a.linear_factor.z * row.multiplier;
				constraint.object_a.linear_velocity.add( _tmp_vec3_2 );
				constraint.last_impulse.add( _tmp_vec3_2 );

				_tmp_vec3_1.x = time_delta * row.jacobian[3] * constraint.object_a.angular_factor.x * row.multiplier;
				_tmp_vec3_1.y = time_delta * row.jacobian[4] * constraint.object_a.angular_factor.y * row.multiplier;
				_tmp_vec3_1.z = time_delta * row.jacobian[5] * constraint.object_a.angular_factor.z * row.multiplier;
				constraint.object_a.inverseInertiaTensorWorldFrame.transformVector3( _tmp_vec3_1 );
				constraint.object_a.angular_velocity.add( _tmp_vec3_1 );
				constraint.last_impulse.add( _tmp_vec3_1 );
			}

			if ( constraint.object_b != null && constraint.object_b._mass !== Infinity ) {
				invmass = constraint.object_b._mass_inverted;
				_tmp_vec3_2.x = invmass * time_delta * row.jacobian[6] * constraint.object_b.linear_factor.x * row.multiplier;
				_tmp_vec3_2.y = invmass * time_delta * row.jacobian[7] * constraint.object_b.linear_factor.y * row.multiplier;
				_tmp_vec3_2.z = invmass * time_delta * row.jacobian[8] * constraint.object_b.linear_factor.z * row.multiplier;
				constraint.object_b.linear_velocity.add(_tmp_vec3_2 );
				constraint.last_impulse.add( _tmp_vec3_2 );

				_tmp_vec3_1.x = time_delta * row.jacobian[9] * constraint.object_b.angular_factor.x * row.multiplier;
				_tmp_vec3_1.y = time_delta * row.jacobian[10] * constraint.object_b.angular_factor.y * row.multiplier;
				_tmp_vec3_1.z = time_delta * row.jacobian[11] * constraint.object_b.angular_factor.z * row.multiplier;
				constraint.object_b.inverseInertiaTensorWorldFrame.transformVector3( _tmp_vec3_1 );
				constraint.object_b.angular_velocity.add( _tmp_vec3_1 );
				constraint.last_impulse.add( _tmp_vec3_1 );
			}
		}

		if ( constraint.breaking_threshold > 0 ) {
			if ( constraint.last_impulse.lengthSquared() >= constraint.breaking_threshold * constraint.breaking_threshold ) {
				constraint.active = false;
			}
		}
	}

	// Kill the resting-contact residual "buzz": a body settled on a contact keeps a small standing linear
	// velocity that never damps to zero (an equilibrium artifact) and leaks into anything resting on it.
	// Only a body slow (tiny linear AND angular velocity) for several consecutive frames is zeroed, so the
	// active settling transient and any rolling/spinning body are never touched.
	var BUZZ_LIN = 0.08, BUZZ_ANG = 0.08, BUZZ_FRAMES = 8;
	var BUZZ_LIN_SQ = BUZZ_LIN * BUZZ_LIN, BUZZ_ANG_SQ = BUZZ_ANG * BUZZ_ANG;
	for ( i = 0; i < this.contact_constraints.length; i++ ) {
		constraint = this.contact_constraints[i];
		if ( constraint.active === false ) { continue; }
		// Was building a fresh 2-element [object_a, object_b] array here every constraint, every step,
		// purely to loop over "the two bodies" — 2247 throwaway array allocations/step in this scene for
		// no reason, since the two sides can just be checked inline without a wrapper array.
		for ( var pi = 0; pi < 2; pi++ ) {
			var bod = pi === 0 ? constraint.object_a : constraint.object_b;
			if ( bod == null || bod._mass === Infinity ) { continue; }
			if ( bod.linear_velocity.lengthSquared() < BUZZ_LIN_SQ &&
				bod.angular_velocity.lengthSquared() < BUZZ_ANG_SQ ) {
				bod._buzzSlowFrames = ( bod._buzzSlowFrames || 0 ) + 1;
				if ( bod._buzzSlowFrames >= BUZZ_FRAMES ) {
					bod.linear_velocity.x = bod.linear_velocity.y = bod.linear_velocity.z = 0;
				}
			} else {
				bod._buzzSlowFrames = 0;
			}
		}
	}
};