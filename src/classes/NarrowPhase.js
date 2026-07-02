/**
 * Takes possible contacts found by a broad phase and determines if they are legitimate contacts
 *
 * @class NarrowPhase
 * @constructor
 */
Goblin.NarrowPhase = function() {
	/**
	 * holds all contacts which currently exist in the scene
	 *
	 * @property contact_manifolds
	 * @type Goblin.ContactManifoldList
	 */
	this.contact_manifolds = new Goblin.ContactManifoldList();
};

/**
 * Iterates over all contact manifolds, updating penetration depth & contact locations
 *
 * @method updateContactManifolds
 */
Goblin.NarrowPhase.prototype.updateContactManifolds = function() {
	var current = this.contact_manifolds.first,
		prev = null;

	while ( current !== null ) {
		current.update();

		if ( current.points.length === 0 ) {
			Goblin.ObjectPool.freeObject( 'ContactManifold', current );
			if ( prev == null ) {
				this.contact_manifolds.first = current.next_manifold;
			} else {
				prev.next_manifold = current.next_manifold;
			}
			current = current.next_manifold;
		} else {
			prev = current;
			current = current.next_manifold;
		}
	}
};

Goblin.NarrowPhase.prototype.midPhase = function( object_a, object_b ) {
	var compound,
		other;

	if ( object_a.shape instanceof Goblin.CompoundShape ) {
		compound = object_a;
		other = object_b;
	} else {
		compound = object_b;
		other = object_a;
	}

	var proxy = Goblin.ObjectPool.getObject( 'RigidBodyProxy' ),
		child_shape, contact;
	for ( var i = 0; i < compound.shape.child_shapes.length; i++ ) {
		child_shape = compound.shape.child_shapes[i];
		proxy.setFrom( compound, child_shape );

		if ( proxy.shape instanceof Goblin.CompoundShape || other.shape instanceof Goblin.CompoundShape ) {
			this.midPhase( proxy, other );
		} else {
			contact = this.getContact( proxy, other );
			if ( contact != null ) {
				var parent_a, parent_b;
				if ( contact.object_a === proxy ) {
					contact.object_a = compound;
					parent_a = proxy;
					parent_b = other;
				} else {
					contact.object_b = compound;
					parent_a = other;
					parent_b = proxy;
				}

				if ( parent_a instanceof Goblin.RigidBodyProxy ) {
					while ( parent_a.parent ) {
						if ( parent_a instanceof Goblin.RigidBodyProxy ) {
							parent_a.shape_data.transform.transformVector3( contact.contact_point_in_a );
						}
						parent_a = parent_a.parent;
					}
				}

				if ( parent_b instanceof Goblin.RigidBodyProxy ) {
					while ( parent_b.parent ) {
						if ( parent_b instanceof Goblin.RigidBodyProxy ) {
							parent_b.shape_data.transform.transformVector3( contact.contact_point_in_b );
						}
						parent_b = parent_b.parent;
					}
				}

				contact.object_a = parent_a;
				contact.object_b = parent_b;
				this.addContact( parent_a, parent_b, contact );
			}
		}
	}
	Goblin.ObjectPool.freeObject( 'RigidBodyProxy', proxy );
};

Goblin.NarrowPhase.prototype.meshCollision = (function(){
	var b_to_a = new Goblin.Matrix4(),
		tri_b = new Goblin.TriangleShape( new Goblin.Vector3(), new Goblin.Vector3(), new Goblin.Vector3() ),
		b_aabb = new Goblin.AABB(),
		b_right_aabb = new Goblin.AABB(),
		b_left_aabb = new Goblin.AABB();

	function meshMesh( object_a, object_b, addContact ) {
		// get matrix which converts from object_b's space to object_a
		b_to_a.copy( object_a.transform_inverse );
		b_to_a.multiply( object_b.transform );

		// traverse both objects' AABBs while they overlap, if two overlapping leaves are found then perform Triangle/Triangle intersection test
		var nodes = [ object_a.shape.hierarchy, object_b.shape.hierarchy ];
		//debugger;
		while ( nodes.length ) {
			var a_node = nodes.shift(),
				b_node = nodes.shift();

			if ( a_node.isLeaf() && b_node.isLeaf() ) {
				// Both sides are triangles, do intersection test
                // convert node_b's triangle into node_a's frame
                b_to_a.transformVector3Into( b_node.object.a, tri_b.a );
                b_to_a.transformVector3Into( b_node.object.b, tri_b.b );
                b_to_a.transformVector3Into( b_node.object.c, tri_b.c );
                _tmp_vec3_1.subtractVectors( tri_b.b, tri_b.a );
                _tmp_vec3_2.subtractVectors( tri_b.c, tri_b.a );
                tri_b.normal.crossVectors( _tmp_vec3_1, _tmp_vec3_2 );
                tri_b.normal.normalize();

				var contact = Goblin.TriangleTriangle( a_node.object, tri_b );
                if ( contact != null ) {
					object_a.transform.rotateVector3( contact.contact_normal );

                    object_a.transform.transformVector3( contact.contact_point );

                    object_a.transform.transformVector3( contact.contact_point_in_b );
                    object_b.transform_inverse.transformVector3( contact.contact_point_in_b );

                    contact.object_a = object_a;
                    contact.object_b = object_b;

                    contact.restitution = ( object_a.restitution + object_b.restitution ) / 2;
                    contact.friction = ( object_a.friction + object_b.friction ) / 2;
                    /*console.log( contact );
                    debugger;*/

                    addContact( object_a, object_b, contact );
                }
			} else if ( a_node.isLeaf() ) {
				// just a_node is a leaf
				b_left_aabb.transform( b_node.left.aabb, b_to_a );
				if ( a_node.aabb.intersects( b_left_aabb ) ) {
					nodes.push( a_node, b_node.left );
				}
				b_right_aabb.transform( b_node.right.aabb, b_to_a );
				if ( a_node.aabb.intersects( b_right_aabb ) ) {
					nodes.push( a_node, b_node.right );
				}
			} else if ( b_node.isLeaf() ) {
				// just b_node is a leaf
				b_aabb.transform( b_node.aabb, b_to_a );
				if ( b_aabb.intersects( a_node.left.aabb ) ) {
					nodes.push( a_node.left, b_node );
				}
				if ( b_aabb.intersects( a_node.right.aabb ) ) {
					nodes.push( a_node.right, b_node );
				}
			} else {
				// neither node is a branch
				b_left_aabb.transform( b_node.left.aabb, b_to_a );
				b_right_aabb.transform( b_node.right.aabb, b_to_a );
				if ( a_node.left.aabb.intersects( b_left_aabb ) ) {
					nodes.push( a_node.left, b_node.left );
				}
				if ( a_node.left.aabb.intersects( b_right_aabb ) ) {
					nodes.push( a_node.left, b_node.right );
				}
				if ( a_node.right.aabb.intersects( b_left_aabb ) ) {
					nodes.push( a_node.right, b_node.left );
				}
				if ( a_node.right.aabb.intersects( b_right_aabb ) ) {
					nodes.push( a_node.right, b_node.right );
				}
			}
		}
	}

	function triangleConvex( triangle, mesh, convex ) {
		// Create proxy to convert convex into mesh's space
		var proxy = Goblin.ObjectPool.getObject( 'RigidBodyProxy' );

		var child_shape = new Goblin.CompoundShapeChild( triangle, new Goblin.Vector3(), new Goblin.Quaternion() );
		proxy.setFrom( mesh, child_shape );

		var simplex = Goblin.GjkEpa.GJK( proxy, convex ),
			contact;
		if ( Goblin.GjkEpa.result != null ) {
			contact = Goblin.GjkEpa.result;
		} else if ( simplex != null ) {
			contact = Goblin.GjkEpa.EPA( simplex );
		}

		Goblin.ObjectPool.freeObject( 'RigidBodyProxy', proxy );

		return contact;
	}

	var meshConvex = (function(){
		var convex_to_mesh = new Goblin.Matrix4(),
			convex_aabb_in_mesh = new Goblin.AABB();

		return function meshConvex( mesh, convex, addContact ) {
			// Find matrix that converts convex into mesh space
			convex_to_mesh.copy( convex.transform );
			convex_to_mesh.multiply( mesh.transform_inverse );

			convex_aabb_in_mesh.transform( convex.aabb, mesh.transform_inverse );

			// Traverse the BHV in mesh
			var pending_nodes = [ mesh.shape.hierarchy ],
				node;
			while ( ( node = pending_nodes.shift() ) ) {
				if ( node.aabb.intersects( convex_aabb_in_mesh ) ) {
					if ( node.isLeaf() ) {
						// Check node for collision
						var contact = triangleConvex( node.object, mesh, convex );
						if ( contact != null ) {
							var _mesh = mesh;
							while ( _mesh.parent != null ) {
								_mesh = _mesh.parent;
							}
							contact.object_a = _mesh;
							addContact( _mesh, convex, contact );
						}
					} else {
						pending_nodes.push( node.left, node.right );
					}
				}
			}
		};
	})();

	return function meshCollision( object_a, object_b ) {
		var a_is_mesh = object_a.shape instanceof Goblin.MeshShape,
			b_is_mesh = object_b.shape instanceof Goblin.MeshShape;

		if ( a_is_mesh && b_is_mesh ) {
			meshMesh( object_a, object_b, this.addContact.bind( this ) );
		} else {
			if ( a_is_mesh ) {
				meshConvex( object_a, object_b, this.addContact.bind( this ) );
			} else {
				meshConvex( object_b, object_a, this.addContact.bind( this ) );
			}
		}
	};
})();

/**
 * Tests two objects for contact
 *
 * @method getContact
 * @param {RigidBody} object_a
 * @param {RigidBody} object_b
 */
Goblin.NarrowPhase.prototype.getContact = function( object_a, object_b ) {
	if ( object_a.shape instanceof Goblin.CompoundShape || object_b.shape instanceof Goblin.CompoundShape ) {
		this.midPhase( object_a, object_b );
		return;
	}

	if ( object_a.shape instanceof Goblin.MeshShape || object_b.shape instanceof Goblin.MeshShape ) {
		this.meshCollision( object_a, object_b );
		return;
	}

	var contact;

	if ( object_a.shape instanceof Goblin.SphereShape && object_b.shape instanceof Goblin.SphereShape ) {
		// Sphere - Sphere contact check
		contact = Goblin.SphereSphere( object_a, object_b );
	} else if (
		object_a.shape instanceof Goblin.SphereShape && object_b.shape instanceof Goblin.BoxShape ||
		object_a.shape instanceof Goblin.BoxShape && object_b.shape instanceof Goblin.SphereShape
	) {
		// Sphere - Box contact check
		contact = Goblin.BoxSphere( object_a, object_b );
	} else {
		// contact check based on GJK
		var simplex = Goblin.GjkEpa.GJK( object_a, object_b );
		if ( Goblin.GjkEpa.result != null ) {
			contact = Goblin.GjkEpa.result;
		} else if ( simplex != null ) {
			contact = Goblin.GjkEpa.EPA( simplex );
		}
	}

	return contact;
};

Goblin.NarrowPhase.prototype.addContact = function( object_a, object_b, contact ) {
	this.contact_manifolds.getManifoldForObjects( object_a, object_b ).addContact( contact );
};

/**
 * Loops over the passed array of object pairs which may be in contact
 * valid contacts are put in this object's `contacts` property
 *
 * @param possible_contacts {Array}
 */
Goblin.NarrowPhase.prototype.generateContacts = function( possible_contacts ) {
	var i,
		contact,
		possible_contacts_length = possible_contacts.length;

	// Make sure all of the manifolds are up to date
	this.updateContactManifolds();

	for ( i = 0; i < possible_contacts_length; i++ ) {
		contact = this.getContact( possible_contacts[i][0], possible_contacts[i][1] );
		if ( contact != null ) {
			this.addContact( possible_contacts[i][0], possible_contacts[i][1], contact );
		}
	}

	this.symmetrizeRoundContacts();
	this.collapseSphereContacts();
};

/**
 * Collapses any sphere manifold to a single contact point under the sphere's center. A sphere touches
 * a surface at exactly one point along the contact normal, but the persistent ContactManifold caches
 * up to four points, each recomputed from a local anchor fixed on the sphere surface. As the sphere
 * rolls those anchors rotate with it, so cached points trail behind the true bottom while still
 * shallowly penetrating, forming an off-center cluster whose lever arm under the vertical normal is a
 * phantom torque that pumps the sphere's spin and keeps it rolling. This keeps the single deepest
 * point, re-places it (and its local anchors) at the sphere center projected onto the surface along
 * the normal, so it is recomputed from the center each frame and rolling cannot smear it.
 *
 * @method collapseSphereContacts
 */
Goblin.NarrowPhase.prototype.collapseSphereContacts = function() {
	var manifold = this.contact_manifolds.first;
	while ( manifold ) {
		var hasSphere = manifold.object_a.shape instanceof Goblin.SphereShape ||
			manifold.object_b.shape instanceof Goblin.SphereShape;

		if ( hasSphere && manifold.points.length > 0 ) {
			// Keep the single deepest-penetrating point (the real current contact); destroy the rest.
			var deepest = 0;
			for ( var i = 1; i < manifold.points.length; i++ ) {
				if ( manifold.points[i].penetration_depth > manifold.points[deepest].penetration_depth ) {
					deepest = i;
				}
			}
			for ( i = manifold.points.length - 1; i >= 0; i-- ) {
				if ( i !== deepest ) {
					manifold.points[i].destroy();
					manifold.points.splice( i, 1 );
				}
			}

			var p = manifold.points[0];
			// The contact's own object ordering is authoritative — a manifold matches its body pair
			// in either order, so its object_a may be the contact's object_b (see BoxSphere, which
			// always makes the sphere its contact's object_a).
			var sphereIsA = p.object_a.shape instanceof Goblin.SphereShape;
			var sphereObj = sphereIsA ? p.object_a : p.object_b;
			var r = sphereObj.shape.radius;
			// Contact normal points from A to B. The point on the sphere surface that is in contact is
			// the sphere center moved by radius along the normal toward the OTHER body.
			var sign = sphereIsA ? 1 : -1;   // toward the other body from the sphere's center
			var nx = p.contact_normal.x * sign, ny = p.contact_normal.y * sign, nz = p.contact_normal.z * sign;

			// True world contact point: on the sphere surface, directly along the normal from center.
			// Split the small penetration so the point sits midway in the overlap, matching the
			// engine's convention (see BoxSphere / ContactManifold.update midpoint).
			var half_pen = p.penetration_depth * 0.5;
			p.contact_point.x = sphereObj.position.x + nx * ( r - half_pen );
			p.contact_point.y = sphereObj.position.y + ny * ( r - half_pen );
			p.contact_point.z = sphereObj.position.z + nz * ( r - half_pen );

			// Re-anchor each body's local point fresh at its own surface along the normal — the sphere
			// at its true surface point, the other body at that point pushed through the full overlap —
			// so ContactManifold.update recomputes the correct penetration from their separation rather
			// than from a stale surface point that rotates with the rolling sphere.
			_tmp_vec3_1.set(
				sphereObj.position.x + nx * r,
				sphereObj.position.y + ny * r,
				sphereObj.position.z + nz * r
			);
			sphereObj.transform_inverse.transformVector3Into( _tmp_vec3_1, sphereIsA ? p.contact_point_in_a : p.contact_point_in_b );
			var other = sphereIsA ? p.object_b : p.object_a;
			_tmp_vec3_1.set(
				sphereObj.position.x + nx * ( r - p.penetration_depth ),
				sphereObj.position.y + ny * ( r - p.penetration_depth ),
				sphereObj.position.z + nz * ( r - p.penetration_depth )
			);
			other.transform_inverse.transformVector3Into( _tmp_vec3_1, sphereIsA ? p.contact_point_in_b : p.contact_point_in_a );
		}

		manifold = manifold.next_manifold;
	}
};

/**
 * Snaps a round or tapered body's side-rest contact points onto its true line contact. A cylinder,
 * capsule, or cone lying on its side touches a flat surface along a line fixed by the shape's own
 * geometry (see each shape's `getRestAxis`), but GJK/EPA place their points off that line; any such
 * point applies a torque about the line every step, spinning the body up into an endless roll. For any
 * shape exposing `getRestAxis(localNormal)` this transforms that rest line into world space and
 * re-projects each contact point onto it (clamped to the segment), correcting only the in-plane
 * component and leaving the along-normal penetration untouched. Guarded to the genuine side-rest case:
 * at least two points sharing a near-identical normal, with that normal roughly perpendicular to the
 * body's own axis; anything else is left as EPA produced it.
 *
 * @method symmetrizeRoundContacts
 */
Goblin.NarrowPhase.prototype.symmetrizeRoundContacts = function() {
	var manifold = this.contact_manifolds.first;
	var local_n = new Goblin.Vector3(),
		axis_local_0 = new Goblin.Vector3(), axis_local_1 = new Goblin.Vector3(),
		axis_world_0 = new Goblin.Vector3(), axis_world_1 = new Goblin.Vector3();

	while ( manifold ) {
		var round = null;
		if ( typeof manifold.object_a.shape.getRestAxis === 'function' ) {
			round = manifold.object_a;
		} else if ( typeof manifold.object_b.shape.getRestAxis === 'function' ) {
			round = manifold.object_b;
		}

		if ( round !== null && manifold.points.length >= 2 ) {
			var points = manifold.points;
			var n0 = points[0].contact_normal;

			// All normals must point essentially the same way (a single flat contact face, not
			// several faces of a corner/wedge).
			var consistent = true;
			for ( var i = 1; i < points.length; i++ ) {
				var ni = points[i].contact_normal;
				var ndot = n0.x * ni.x + n0.y * ni.y + n0.z * ni.z;
				if ( ndot <= 0.999 ) { consistent = false; break; }
			}

			// Normal in the round body's local space, to pick the radial direction for shapes whose
			// rest axis depends on which way is down locally, and to detect orientation. The barrel
			// line is valid only for a genuine side rest (local normal roughly perpendicular to the
			// body's own axis); a body standing on its flat end has a near-axial local normal and a
			// rim-circle of points that this line logic must not touch. Require |local_n.y| small.
			round.transform_inverse.rotateVector3Into( n0, local_n );
			var isSideRest = Math.abs( local_n.y ) < 0.7;

			if ( consistent && isSideRest ) {
				var axis = round.shape.getRestAxis( local_n );
				axis_local_0.x = axis[0].x; axis_local_0.y = axis[0].y; axis_local_0.z = axis[0].z;
				axis_local_1.x = axis[1].x; axis_local_1.y = axis[1].y; axis_local_1.z = axis[1].z;
				round.transform.transformVector3Into( axis_local_0, axis_world_0 );
				round.transform.transformVector3Into( axis_local_1, axis_world_1 );

				var ax = axis_world_1.x - axis_world_0.x,
					ay = axis_world_1.y - axis_world_0.y,
					az = axis_world_1.z - axis_world_0.z;
				var axisLenSq = ax * ax + ay * ay + az * az;

				for ( i = 0; i < points.length; i++ ) {
					var p = points[i];
					var n = p.contact_normal;

					// Project the point onto the true rest line, clamped to the segment.
					var t = 0;
					if ( axisLenSq > 1e-9 ) {
						t = ( ( p.contact_point.x - axis_world_0.x ) * ax +
							( p.contact_point.y - axis_world_0.y ) * ay +
							( p.contact_point.z - axis_world_0.z ) * az ) / axisLenSq;
						if ( t < 0 ) {
							t = 0;
						} else if ( t > 1 ) {
							t = 1;
						}
					}
					var targetX = axis_world_0.x + ax * t,
						targetY = axis_world_0.y + ay * t,
						targetZ = axis_world_0.z + az * t;

					// Only correct the in-plane (perpendicular-to-normal) component; leave the
					// along-normal (penetration) component exactly as EPA computed it, so we don't
					// fight the height/penetration solve.
					var dx = targetX - p.contact_point.x,
						dy = targetY - p.contact_point.y,
						dz = targetZ - p.contact_point.z;
					var dn = dx * n.x + dy * n.y + dz * n.z;
					dx -= dn * n.x; dy -= dn * n.y; dz -= dn * n.z;

					// Shift only the world-space contact point the jacobians are built from this step.
					// The local anchors contact_point_in_a/_in_b are left alone: they drive next
					// step's penetration/height computation, and rewriting them from the tangentially
					// shifted point corrupts the height solve. Leaving them makes this a per-step
					// correction, re-applied fresh from EPA's latest points rather than a persistent edit.
					p.contact_point.x += dx; p.contact_point.y += dy; p.contact_point.z += dz;
				}
			}
		}

		manifold = manifold.next_manifold;
	}
};

Goblin.NarrowPhase.prototype.removeBody = function( body ) {
	var manifold = this.contact_manifolds.first;

	while ( manifold != null ) {
		if ( manifold.object_a === body || manifold.object_b === body ) {
			for ( var i = 0; i < manifold.points.length; i++ ) {
				manifold.points[i].destroy();
			}
			manifold.points.length = 0;
		}

		manifold = manifold.next;
	}
};