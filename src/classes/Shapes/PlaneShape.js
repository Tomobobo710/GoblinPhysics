/**
 * @class PlaneShape
 * @param orientation {Number} index of axis which is the plane's normal ( 0 = X, 1 = Y, 2 = Z )
 * @param half_width {Number} half width of the plane
 * @param half_length {Number} half height of the plane
 * @constructor
 */
Goblin.PlaneShape = function( orientation, half_width, half_length ) {
	/**
	 * index of axis which is the plane's normal ( 0 = X, 1 = Y, 2 = Z )
	 * when 0, width is Y and length is Z
	 * when 1, width is X and length is Z
	 * when 2, width is X and length is Y
	 *
	 * @property half_width
	 * @type {Number}
	 */
	this.orientation = orientation;

	/**
	 * half width of the plane
	 *
	 * @property half_height
	 * @type {Number}
	 */
	this.half_width = half_width;

	/**
	 * half length of the plane
	 *
	 * @property half_length
	 * @type {Number}
	 */
	this.half_length = half_length;

    this.aabb = new Goblin.AABB();
    this.calculateLocalAABB( this.aabb );


	if ( this.orientation === 0 ) {
		this._half_width = 0;
		this._half_height = this.half_width;
		this._half_depth = this.half_length;
	} else if ( this.orientation === 1 ) {
		this._half_width = this.half_width;
		this._half_height = 0;
		this._half_depth = this.half_length;
	} else {
		this._half_width = this.half_width;
		this._half_height = this.half_length;
		this._half_depth = 0;
	}
};

/**
 * Calculates this shape's local AABB and stores it in the passed AABB object
 *
 * @method calculateLocalAABB
 * @param aabb {AABB}
 */
Goblin.PlaneShape.prototype.calculateLocalAABB = function( aabb ) {
    if ( this.orientation === 0 ) {
        this._half_width = 0;
        this._half_height = this.half_width;
        this._half_depth = this.half_length;

        aabb.min[0] = 0;
        aabb.min[1] = -this.half_width;
        aabb.min[2] = -this.half_length;

        aabb.max[0] = 0;
        aabb.max[1] = this.half_width;
        aabb.max[2] = this.half_length;
    } else if ( this.orientation === 1 ) {
        this._half_width = this.half_width;
        this._half_height = 0;
        this._half_depth = this.half_length;

        aabb.min[0] = -this.half_width;
        aabb.min[1] = 0;
        aabb.min[2] = -this.half_length;

        aabb.max[0] = this.half_width;
        aabb.max[1] = 0;
        aabb.max[2] = this.half_length;
    } else {
        this._half_width = this.half_width;
        this._half_height = this.half_length;
        this._half_depth = 0;

        aabb.min[0] = -this.half_width;
        aabb.min[1] = -this.half_length;
        aabb.min[2] = 0;

        aabb.max[0] = this.half_width;
        aabb.max[1] = this.half_length;
        aabb.max[2] = 0;
    }
};

Goblin.PlaneShape.prototype.getInertiaTensor = function( mass ) {
	var width_squared = this.half_width * this.half_width * 4,
		length_squared = this.half_length * this.half_length * 4,
		element = 0.0833 * mass,

		x = element * length_squared,
		y = element * ( width_squared + length_squared ),
		z = element * width_squared;

	if ( this.orientation === 0 ) {
		return mat3.createFrom(
			y, 0, 0,
			0, x, 0,
			0, 0, z
		);
	} else if ( this.orientation === 1 ) {
		return mat3.createFrom(
			x, 0, 0,
			0, y, 0,
			0, 0, z
		);
	} else {
		return mat3.createFrom(
			y, 0, 0,
			0, z, 0,
			0, 0, x
		);
	}
};

/**
 * Given `direction`, find the point in this body which is the most extreme in that direction.
 * This support point is calculated in world coordinates and stored in the second parameter `support_point`
 *
 * @method findSupportPoint
 * @param direction {vec3} direction to use in finding the support point
 * @param support_point {vec3} vec3 variable which will contain the supporting point after calling this method
 */
Goblin.PlaneShape.prototype.findSupportPoint = function( direction, support_point ) {
	/*
	 support_point = [
	 sign( direction.x ) * _half_width,
	 sign( direction.y ) * _half_height,
	 sign( direction.z ) * _half_depth
	 ]
	 */

	// Calculate the support point in the local frame
	if ( direction[0] < 0 ) {
		support_point[0] = -this._half_width;
	} else {
		support_point[0] = this._half_width;
	}

	if ( direction[1] < 0 ) {
		support_point[1] = -this._half_height;
	} else {
		support_point[1] = this._half_height;
	}

	if ( direction[2] < 0 ) {
		support_point[2] = -this._half_depth;
	} else {
		support_point[2] = this._half_depth;
	}
};

/**
 * Checks if a ray segment intersects with the shape
 *
 * @method rayIntersect
 * @property start {vec3} start point of the segment
 * @property end {vec3{ end point of the segment
 * @return {RayIntersection|null} if the segment intersects, a RayIntersection is returned, else `null`
 */
Goblin.PlaneShape.prototype.rayIntersect = (function(){
	var normal = vec3.create(),
		ab = vec3.create(),
		point = vec3.create(),
		t;

	return function( start, end ) {
		if ( this.orientation === 0 ) {
			normal[0] = 1;
			normal[1] = normal[2] = 0;
		} else if ( this.orientation === 1 ) {
			normal[1] = 1;
			normal[0] = normal[2] = 0;
		} else {
			normal[2] = 1;
			normal[0] = normal[1] = 0;
		}

		vec3.subtract( end, start, ab );
		t = -vec3.dot( normal, start ) / vec3.dot( normal, ab );

		if ( t < 0 || t > 1 ) {
			return null;
		}

		vec3.scale( ab, t, point );
		vec3.add( point, start );

		if ( point[0] < -this._half_width || point[0] > this._half_width ) {
			return null;
		}

		if ( point[1] < -this._half_height || point[1] > this._half_height ) {
			return null;
		}

		if ( point[2] < -this._half_depth || point[2] > this._half_depth ) {
			return null;
		}

		var intersection = Goblin.ObjectPool.getObject( 'RayIntersection' );
		intersection.object = this;
		intersection.t = t;
		vec3.set( point, intersection.point );

		return intersection;
	};
})();