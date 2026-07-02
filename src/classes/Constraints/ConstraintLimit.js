/**
 * Optional lower/upper bound on a constraint's separating value (e.g. a HingeConstraint's angle
 * about its axis). Owned by the constraint it limits; only allocates its ConstraintRow lazily,
 * on demand, when the bound is actually violated.
 *
 * @class ConstraintLimit
 * @constructor
 * @param limit_lower {Number} lower bound, or null/undefined to leave that side unconstrained
 * @param limit_upper {Number} upper bound, or null/undefined to leave that side unconstrained
 */
Goblin.ConstraintLimit = function( limit_lower, limit_upper ) {
	this.erp = 0.3;
	this.constraint_row = null;

	this.set( limit_lower, limit_upper );
};

/**
 * Updates the lower/upper bounds and re-derives `enabled` from whether either bound is set.
 *
 * @method set
 * @param limit_lower {Number} lower bound, or null/undefined to leave that side unconstrained
 * @param limit_upper {Number} upper bound, or null/undefined to leave that side unconstrained
 */
Goblin.ConstraintLimit.prototype.set = function( limit_lower, limit_upper ) {
	this.limit_lower = limit_lower;
	this.limit_upper = limit_upper;

	this.enabled = this.limit_lower != null || this.limit_upper != null;
};

/**
 * Allocates this limit's ConstraintRow from the object pool. Called by the owning constraint the
 * first time the limit is actually violated in a given step.
 *
 * @method createConstraintRow
 */
Goblin.ConstraintLimit.prototype.createConstraintRow = function() {
	this.constraint_row = Goblin.ConstraintRow.createConstraintRow();
};