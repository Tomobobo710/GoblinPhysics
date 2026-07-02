/**
 * Optional powered drive on a constraint (e.g. a HingeConstraint spinning its axis under a bounded
 * torque up to a target speed). Owned by the constraint it drives; only allocates its
 * ConstraintRow lazily, on demand, once enabled.
 *
 * @class ConstraintMotor
 * @constructor
 * @param torque {Number} maximum torque/force the motor may apply, or null/undefined to disable
 * @param max_speed {Number} target speed the motor drives toward, or null/undefined to disable
 */
Goblin.ConstraintMotor = function( torque, max_speed ) {
	this.constraint_row = null;
	this.set( torque, max_speed);
};

/**
 * Updates the motor's torque/speed and re-derives `enabled` from whether both are set.
 *
 * @method set
 * @param torque {Number} maximum torque/force the motor may apply, or null/undefined to disable
 * @param max_speed {Number} target speed the motor drives toward, or null/undefined to disable
 */
Goblin.ConstraintMotor.prototype.set = function( torque, max_speed ) {
	this.enabled = torque != null && max_speed != null;
	this.torque = torque;
	this.max_speed = max_speed;
};

/**
 * Allocates this motor's ConstraintRow from the object pool. Called by the owning constraint the
 * first time the motor is enabled.
 *
 * @method createConstraintRow
 */
Goblin.ConstraintMotor.prototype.createConstraintRow = function() {
	this.constraint_row = Goblin.ConstraintRow.createConstraintRow();
};