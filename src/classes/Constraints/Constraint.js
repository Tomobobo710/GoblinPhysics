/**
 * Base class for velocity constraints solved by the IterativeSolver. Not used directly - concrete
 * constraints (ContactConstraint, FrictionConstraint, HingeConstraint, PointConstraint,
 * SliderConstraint, WeldConstraint) extend this via `Object.create( Goblin.Constraint.prototype )`
 * and populate `rows` with the ConstraintRow(s) that express their particular restriction.
 *
 * @class Constraint
 * @constructor
 */
Goblin.Constraint = (function() {
	var constraint_count = 0;

	return function() {
		this.id = constraint_count++;

		this.active = true;

		this.object_a = null;

		this.object_b = null;

		this.limit = new Goblin.ConstraintLimit();

		this.motor = new Goblin.ConstraintMotor();

		this.rows = [];

		this.factor = 1;

		this.last_impulse = new Goblin.Vector3();

		this.breaking_threshold = 0;

		this.listeners = {};
	};
})();
Goblin.EventEmitter.apply( Goblin.Constraint );

/**
 * Marks this constraint inactive and emits a `deactivate` event, so the solver drops it from
 * `all_constraints` and any listeners (e.g. a manifold's contact/friction constraint pair) can
 * clean up in response.
 *
 * @method deactivate
 */
Goblin.Constraint.prototype.deactivate = function() {
	this.active = false;
	this.emit( 'deactivate' );
};

/**
 * Recomputes this constraint's row(s) (jacobian, bias) from current body state. Called once per
 * solver iteration before the rows are consumed. No-op on the base class; concrete constraints
 * override this.
 *
 * @method update
 */
Goblin.Constraint.prototype.update = function(){};