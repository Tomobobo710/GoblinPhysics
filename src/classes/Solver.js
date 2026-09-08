/**
 * Base interface for the constraint solver World drives each step. World only ever calls the 6
 * methods defined here (see World.step and World.addConstraint/removeConstraint) - any object
 * implementing them can be assigned as world.solver, regardless of the algorithm underneath
 * (Gauss-Seidel/PGS in IterativeSolver, position-based in PBDSolver, or anything else). Concrete
 * solvers extend this via `Object.create( Goblin.Solver.prototype )` and override the 6 step methods;
 * addConstraint/removeConstraint are algorithm-agnostic bookkeeping and are inherited as-is.
 *
 * @class Solver
 * @constructor
 */
Goblin.Solver = function() {
	/**
	 * User-added constraints (joints: hinge, point, slider, weld, etc.) - populated via
	 * addConstraint/removeConstraint, distinct from whatever a solver derives from contact manifolds.
	 *
	 * @property constraints
	 * @type {Array}
	 */
	this.constraints = [];

	this.world = null;
};

/**
 * Applies gravity/external forces and integrates every rigid body's position and rotation for this
 * world tick. Owned by the solver (not World) so an algorithm that needs a structurally different
 * integration scheme - e.g. XPBD substepping, where gravity is applied and position is integrated
 * fresh N times per tick, each followed by its own mini contact-resolve, rather than once - can do so.
 * World.step calls this BEFORE processContactManifolds/prepareConstraints/resolveContacts, and force
 * generators have already run for this tick by the time it's called (see World.step). rigid_bodies of
 * infinite mass must be left untouched, matching RigidBody.integrate's own guard.
 *
 * @method integrate
 * @param rigid_bodies {Array} every body in the world
 * @param gravity {Vector3} the world's default gravity (a body's own `.gravity`, if set, overrides this)
 * @param time_delta {Number} elapsed time for this whole world tick, in seconds
 */
Goblin.Solver.prototype.integrate = function( rigid_bodies, gravity, time_delta ) {
	throw new Error( 'Solver.integrate is not implemented' );
};

/**
 * Converts this step's contact manifolds into whatever internal representation the solver uses.
 * Called once per step, before prepareConstraints.
 *
 * @method processContactManifolds
 * @param contact_manifolds {ContactManifoldList} this step's contact manifolds
 */
Goblin.Solver.prototype.processContactManifolds = function( contact_manifolds ) {
	throw new Error( 'Solver.processContactManifolds is not implemented' );
};

/**
 * Precomputes whatever per-constraint state the solver needs before solving (e.g. Jacobians for a
 * velocity solver, rest lengths for a position solver).
 *
 * @method prepareConstraints
 * @param time_delta {Number} elapsed time for this step, in seconds
 */
Goblin.Solver.prototype.prepareConstraints = function( time_delta ) {
	throw new Error( 'Solver.prepareConstraints is not implemented' );
};

/**
 * Resolves existing penetration (a separate pass from the main velocity/position solve in
 * IterativeSolver; a position-based solver may fold this into solveConstraints and leave this a no-op).
 *
 * @method resolveContacts
 */
Goblin.Solver.prototype.resolveContacts = function() {
	throw new Error( 'Solver.resolveContacts is not implemented' );
};

/**
 * Runs the solver's main iteration and updates body velocities and/or positions.
 *
 * @method solveConstraints
 */
Goblin.Solver.prototype.solveConstraints = function() {
	throw new Error( 'Solver.solveConstraints is not implemented' );
};

/**
 * Applies the solved result (impulses, position corrections) to the rigid bodies.
 *
 * @method applyConstraints
 * @param time_delta {Number} elapsed time for this step, in seconds
 */
Goblin.Solver.prototype.applyConstraints = function( time_delta ) {
	throw new Error( 'Solver.applyConstraints is not implemented' );
};

/**
 * Optional whole-tick hook. A solver that defines this owns the entire tick - integration, collision
 * detection and solving - instead of being driven through the six methods above, and World.step calls
 * it in their place.
 *
 * This exists for substepping algorithms. XPBD's stability comes from interleaving integrate ->
 * detect -> solve -> derive-velocity N times per tick with a timestep of dt/N; the six-method split
 * cannot express that, because World.step runs collision detection exactly once, between integrate
 * and solve. A solver stuck in that shape can only integrate the whole tick, penetrate deeply, and
 * then try to dig itself out once - which is not XPBD and does not behave like it.
 *
 * Solvers that do not define `step` (IterativeSolver) are unaffected and keep the six-method path.
 *
 * @method step
 * @param rigid_bodies {Array} every body in the world
 * @param gravity {Vector3} the world's default gravity
 * @param time_delta {Number} elapsed time for this whole world tick, in seconds
 * @param broadphase {Goblin.Broadphase} the world's broadphase, for pair generation
 * @param narrowphase {Goblin.NarrowPhase} the world's narrowphase, for contact generation
 */
Goblin.Solver.prototype.step = null;

/**
 * Adds a user constraint (joint) to the solver. Generic array bookkeeping - concrete solvers only
 * need to override this if they track joints differently than a flat array.
 *
 * @method addConstraint
 * @param constraint {Goblin.Constraint} constraint to be added
 */
Goblin.Solver.prototype.addConstraint = function( constraint ) {
	if ( this.constraints.indexOf( constraint ) === -1 ) {
		this.constraints.push( constraint );
	}
};

/**
 * Removes a user constraint (joint) from the solver.
 *
 * @method removeConstraint
 * @param constraint {Goblin.Constraint} constraint to be removed
 */
Goblin.Solver.prototype.removeConstraint = function( constraint ) {
	var idx = this.constraints.indexOf( constraint );
	if ( idx !== -1 ) {
		this.constraints.splice( idx, 1 );
	}
};
