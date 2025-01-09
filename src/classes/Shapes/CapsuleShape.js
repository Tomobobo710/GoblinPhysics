/**
 * A capsule shape consisting of a cylinder with hemispheres at both ends
 * 
 * @class CapsuleShape
 * @param radius {Number} radius of both the cylinder portion and hemisphere caps
 * @param total_height {Number} total height of the capsule including both hemisphere caps
 * @constructor
 */
Goblin.CapsuleShape = function(radius, total_height) {
    /**
     * radius of both the cylinder and hemisphere caps
     * 
     * @property radius
     * @type {Number}
     */
    this.radius = radius;

    /**
     * total height of the capsule including hemisphere caps
     * 
     * @property total_height
     * @type {Number}
     */
    this.total_height = total_height;

    /**
     * half of the total capsule height
     * 
     * @property half_height
     * @type {Number}
     */
    this.half_height = total_height * 0.5;

    /**
     * height of just the cylinder portion (excluding hemispheres)
     * 
     * @property cylinder_height
     * @type {Number}
     */
    this.cylinder_height = total_height - (2 * radius);

    /**
     * half height of just the cylinder portion
     * 
     * @property cylinder_half_height
     * @type {Number}
     */
    this.cylinder_half_height = this.cylinder_height * 0.5;

    // Validate the shape is possible
    if (this.cylinder_height < 0) {
        throw new Error("Total height must be greater than 2 * radius");
    }

    this.aabb = new Goblin.AABB();
    this.calculateLocalAABB(this.aabb);
};

/**
 * Calculates this shape's local AABB (Axis-Aligned Bounding Box) and stores it in the passed AABB object
 * 
 * @method calculateLocalAABB
 * @param aabb {AABB} AABB object to store the results
 */
Goblin.CapsuleShape.prototype.calculateLocalAABB = function(aabb) {
    aabb.min.x = aabb.min.z = -this.radius;
    aabb.min.y = -this.half_height;

    aabb.max.x = aabb.max.z = this.radius;
    aabb.max.y = this.half_height;
};

/**
 * Calculates and returns the inertia tensor for the capsule
 * Combines cylinder and sphere inertia based on their respective volumes and mass distribution
 *
 * @method getInertiaTensor
 * @param mass {Number} total mass of the capsule
 * @return {Matrix3} 3x3 matrix representing the inertia tensor
 */
Goblin.CapsuleShape.prototype.getInertiaTensor = function(mass) {
    // Calculate volumes for mass distribution
    var cylinder_volume = Math.PI * this.radius * this.radius * this.cylinder_height;
    var sphere_volume = (4/3) * Math.PI * this.radius * this.radius * this.radius;
    var total_volume = cylinder_volume + sphere_volume;
    
    // Distribute mass proportionally based on volume
    var cylinder_mass = mass * (cylinder_volume / total_volume);
    var sphere_mass = mass * (sphere_volume / total_volume);
    
    // Calculate cylinder contribution to inertia
    var cylinder_x = (1/12) * cylinder_mass * (3 * this.radius * this.radius + this.cylinder_height * this.cylinder_height);
    var cylinder_y = 0.5 * cylinder_mass * this.radius * this.radius;
    
    // Calculate sphere contribution to inertia
    var sphere_element = (2/5) * sphere_mass * this.radius * this.radius;
    
    // Combine inertias into final tensor
    return new Goblin.Matrix3(
        cylinder_x + sphere_element, 0, 0,
        0, cylinder_y + sphere_element, 0,
        0, 0, cylinder_x + sphere_element
    );
};

/**
 * Given a direction, finds the point in this capsule which is the most extreme in that direction.
 * This support point is calculated in local coordinates and stored in the second parameter
 * Used primarily in collision detection algorithms like GJK
 *
 * @method findSupportPoint
 * @param direction {vec3} direction to use in finding the support point
 * @param support_point {vec3} vec3 variable which will contain the supporting point after calling this method
 */
Goblin.CapsuleShape.prototype.findSupportPoint = function(direction, support_point) {
    if (direction.x === 0 && direction.z === 0) {
        // Direction is purely vertical, support point is at the top or bottom cap
        support_point.x = support_point.z = 0;
        support_point.y = direction.y > 0 ? this.half_height : -this.half_height;
    } else {
        // Handle cylindrical portion
        var sigma = Math.sqrt(direction.x * direction.x + direction.z * direction.z);
        var r_s = this.radius / sigma;
        support_point.x = r_s * direction.x;
        support_point.z = r_s * direction.z;
        
        // Handle spherical ends
        if (direction.y > 0) {
            support_point.y = this.cylinder_half_height;
            support_point.y += this.radius * (direction.y / direction.length());
        } else {
            support_point.y = -this.cylinder_half_height;
            support_point.y += this.radius * (direction.y / direction.length());
        }
    }
};

/**
 * Helper method that checks if a ray segment intersects with a sphere.
 * Used by rayIntersect to handle the capsule's hemispherical caps.
 * 
 * @method sphereIntersect
 * @param start {vec3} start point of the ray segment
 * @param end {vec3} end point of the ray segment
 * @param radius {Number} radius of the sphere to check intersection with
 * @return {RayIntersection|null} if the segment intersects, a RayIntersection is returned, else null
 * @private
 */
Goblin.CapsuleShape.prototype.sphereIntersect = function(start, end, radius) {
    var direction = new Goblin.Vector3();
    direction.subtractVectors(end, start);
    var length = direction.length();
    direction.scale(1 / length); // normalize

    var a = start.dot(direction);
    var b = start.dot(start) - radius * radius;

    // Exit early if ray starts outside sphere and points away
    if (a >= 0 && b >= 0) {
        return null;
    }

    var discr = a * a - b;
    // Check for ray miss
    if (discr < 0) {
        return null;
    }

    // Calculate intersection point
    var discr_sqrt = Math.sqrt(discr);
    var t = -a - discr_sqrt;
    if (t < 0) {
        t = -a + discr_sqrt;
    }

    // Verify intersection is within segment length
    if (t > length) {
        return null;
    }

    var intersection = Goblin.ObjectPool.getObject('RayIntersection');
    intersection.point.scaleVector(direction, t);
    intersection.t = t;
    intersection.point.add(start);
    return intersection;
};

/**
 * Checks if a ray segment intersects with the capsule shape.
 * Performs intersection tests against both the cylindrical portion and hemispherical caps,
 * returning the closest intersection point if any exists.
 *
 * @method rayIntersect
 * @param start {vec3} start point of the ray segment
 * @param end {vec3} end point of the ray segment
 * @return {RayIntersection|null} if the segment intersects, a RayIntersection is returned, else null
 */
Goblin.CapsuleShape.prototype.rayIntersect = (function(){
    var p = new Goblin.Vector3(),
        q = new Goblin.Vector3(),
        temp = new Goblin.Vector3();

    return function(start, end) {
        // Test cylinder intersection first
        p.y = this.cylinder_half_height;  // Top of cylinder
        q.y = -this.cylinder_half_height; // Bottom of cylinder

        // Calculate ray properties relative to cylinder
        var d = new Goblin.Vector3();
        d.subtractVectors(q, p);          // Cylinder axis vector

        var m = new Goblin.Vector3();
        m.subtractVectors(start, p);      // Vector from cylinder top to ray start

        var n = new Goblin.Vector3();
        n.subtractVectors(end, start);    // Ray direction vector

        // Compute intermediate values for intersection test
        var md = m.dot(d),
            nd = n.dot(d),
            dd = d.dot(d);

        var nn = n.dot(n),
            mn = m.dot(n),
            a = dd * nn - nd * nd,
            k = m.dot(m) - this.radius * this.radius,
            c = dd * k - md * md,
            cylinder_intersection = null;
        var t;

        // Handle ray parallel to cylinder axis
        if (Math.abs(a) < Goblin.EPSILON) {
            if (c > 0) {
                cylinder_intersection = null;
            } else {
                if (md < 0) {
                    t = -mn / nn;         // Intersect with p endcap
                } else if (md > dd) {
                    t = (nd - mn) / nn;   // Intersect with q endcap
                } else {
                    t = 0;                // Ray starts inside cylinder
                }
                
                if (t >= 0 && t <= 1) {
                    cylinder_intersection = Goblin.ObjectPool.getObject('RayIntersection');
                    cylinder_intersection.t = t * n.length();
                    cylinder_intersection.point.scaleVector(n, t);
                    cylinder_intersection.point.add(start);
                }
            }
        } else {
            // Standard cylinder intersection test
            var b = dd * mn - nd * md,
                discr = b * b - a * c;

            if (discr >= 0) {
                t = (-b - Math.sqrt(discr)) / a;
                if (t >= 0 && t <= 1) {
                    var hit_y = md + t * nd;
                    if (hit_y >= 0 && hit_y <= dd) {
                        cylinder_intersection = Goblin.ObjectPool.getObject('RayIntersection');
                        cylinder_intersection.t = t * n.length();
                        cylinder_intersection.point.scaleVector(n, t);
                        cylinder_intersection.point.add(start);
                    }
                }
            }
        }

        // Now test hemisphere intersections
        var sphere_intersection = null;
        
        // Test top hemisphere
        temp.copy(start);
        temp.y -= this.cylinder_half_height;
        var top_intersection = this.sphereIntersect(temp, end, this.radius);
        if (top_intersection) {
            top_intersection.point.y += this.cylinder_half_height;
            sphere_intersection = top_intersection;
        }

        // Test bottom hemisphere
        temp.copy(start);
        temp.y += this.cylinder_half_height;
        var bottom_intersection = this.sphereIntersect(temp, end, this.radius);
        if (bottom_intersection) {
            bottom_intersection.point.y -= this.cylinder_half_height;
            if (!sphere_intersection || bottom_intersection.t < sphere_intersection.t) {
                sphere_intersection = bottom_intersection;
            }
        }

        // Return the closest intersection between cylinder and hemispheres
        var intersection = null;
        if (cylinder_intersection && sphere_intersection) {
            intersection = cylinder_intersection.t < sphere_intersection.t ? 
                         cylinder_intersection : sphere_intersection;
        } else {
            intersection = cylinder_intersection || sphere_intersection;
        }

        if (intersection) {
            intersection.object = this;
            // Calculate surface normal at intersection point
            if (Math.abs(intersection.point.y) >= this.cylinder_half_height) {
                // Point is on hemisphere cap
                temp.copy(intersection.point);
                temp.y += intersection.point.y > 0 ? -this.cylinder_half_height : this.cylinder_half_height;
                intersection.normal.normalizeVector(temp);
            } else {
                // Point is on cylinder wall
                intersection.normal.x = intersection.point.x;
                intersection.normal.y = 0;
                intersection.normal.z = intersection.point.z;
                intersection.normal.scale(1 / this.radius);
            }
        }

        return intersection;
    };
})();