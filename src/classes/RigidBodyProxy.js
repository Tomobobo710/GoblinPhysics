Goblin.RigidBodyProxy = function() {
	this.parent = null;
	this.id = null;

	this.shape = null;

	this.aabb = new Goblin.AABB();

	this._mass = null;
	this._mass_inverted = null;

	this.position = new Goblin.Vector3();
	this.rotation = new Goblin.Quaternion();

	this.transform = new Goblin.Matrix4();
	this.transform_inverse = new Goblin.Matrix4();

	this.restitution = null;
	this.friction = null;
	this.rolling_friction = null;
};

Object.defineProperty(
	Goblin.RigidBodyProxy.prototype,
	'mass',
	{
		get: function() {
			return this._mass;
		},
		set: function( n ) {
			this._mass = n;
			this._mass_inverted = 1 / n;
			this.inertiaTensor = this.shape.getInertiaTensor( n );
		}
	}
);

Goblin.RigidBodyProxy.prototype.setFrom = function( parent, shape_data ) {
	this.parent = parent;
	this.id = parent.id;
	this.shape = shape_data.shape;
	this.shape_data = shape_data;
	this._mass = parent._mass;

	// The child's world pose (parent * child-local) only changes when the parent moves. Cache it on the
	// child keyed by parent._transformVersion so a static compound (this scene's 3000-child ground) skips
	// the per-frame makeTransform + invert + aabb.transform - the dominant compound-narrowphase cost.
	var pv = parent._transformVersion;
	if ( shape_data._worldPoseVersion === pv && pv !== undefined ) {
		this.position.copy( shape_data._worldPosition );
		this.rotation.copy( shape_data._worldRotation );
		this.transform.copy( shape_data._worldTransform );
		this.transform_inverse.copy( shape_data._worldTransformInverse );
		this.aabb.copy( shape_data._worldChildAabb );
	} else {
		parent.transform.transformVector3Into( shape_data.position, this.position );
		this.rotation.multiplyQuaternions( parent.rotation, shape_data.rotation );
		this.transform.makeTransform( this.rotation, this.position );
		this.transform.invertInto( this.transform_inverse );
		this.aabb.transform( this.shape.aabb, this.transform );

		if ( pv !== undefined ) {
			( shape_data._worldPosition || ( shape_data._worldPosition = new Goblin.Vector3() ) ).copy( this.position );
			( shape_data._worldRotation || ( shape_data._worldRotation = new Goblin.Quaternion() ) ).copy( this.rotation );
			( shape_data._worldTransform || ( shape_data._worldTransform = new Goblin.Matrix4() ) ).copy( this.transform );
			( shape_data._worldTransformInverse || ( shape_data._worldTransformInverse = new Goblin.Matrix4() ) ).copy( this.transform_inverse );
			( shape_data._worldChildAabb || ( shape_data._worldChildAabb = new Goblin.AABB() ) ).copy( this.aabb );
			shape_data._worldPoseVersion = pv;
		}
	}

	this.restitution = parent.restitution;
	this.friction = parent.friction;
	this.rolling_friction = parent.rolling_friction;
	this._transformVersion = ( this._transformVersion || 0 ) + 1;
};

Goblin.RigidBodyProxy.prototype.findSupportPoint = Goblin.RigidBody.prototype.findSupportPoint;

Goblin.RigidBodyProxy.prototype.getRigidBody = function() {
	var body = this.parent;
	while ( body.parent ) {
		body = this.parent;
	}
	return body;
};