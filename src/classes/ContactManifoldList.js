/**
 * List/Manager of ContactManifolds
 *
 * @Class ContactManifoldList
 * @constructor
 */
Goblin.ContactManifoldList = function() {
	/**
	 * The first ContactManifold in the list
	 *
	 * @property first
	 * @type {ContactManifold}
	 */
	this.first = null;

	// Pair-id -> manifold, kept alongside the list for O(1) getManifoldForObjects lookup.
	this._byKey = {};
};

Goblin.ContactManifoldList.pairKey = function( object_a, object_b ) {
	return object_a.id < object_b.id ? ( object_a.id + '_' + object_b.id ) : ( object_b.id + '_' + object_a.id );
};

/**
 * Inserts a ContactManifold into the list
 *
 * @method insert
 * @param {ContactManifold} contact_manifold contact manifold to insert into the list
 */
Goblin.ContactManifoldList.prototype.insert = function( contact_manifold ) {
	// The list is completely unordered, throw the manifold at the beginning
	contact_manifold.next_manifold = this.first;
	this.first = contact_manifold;
	this._byKey[ Goblin.ContactManifoldList.pairKey( contact_manifold.object_a, contact_manifold.object_b ) ] = contact_manifold;
};

/**
 * Removes a ContactManifold from the key index. Callers unlinking from the list directly must also call this.
 *
 * @method remove
 * @param {ContactManifold} contact_manifold
 */
Goblin.ContactManifoldList.prototype.remove = function( contact_manifold ) {
	delete this._byKey[ Goblin.ContactManifoldList.pairKey( contact_manifold.object_a, contact_manifold.object_b ) ];
};

/**
 * Returns (and possibly creates) a ContactManifold for the two rigid bodies
 *
 * @method getManifoldForObjects
 * @param object_a {RigidBody} first body
 * @param object_b {RigidBody} second body
 * @return {ContactManifold}
 */
Goblin.ContactManifoldList.prototype.getManifoldForObjects = function( object_a, object_b ) {
	var manifold = this._byKey[ Goblin.ContactManifoldList.pairKey( object_a, object_b ) ] || null;

	if ( manifold === null ) {
		// A manifold for these two objects does not exist, create one
		manifold = Goblin.ObjectPool.getObject( 'ContactManifold' );
		manifold.object_a = object_a;
		manifold.object_b = object_b;
		// A pooled manifold may carry a stale mesh-leaf cache from its previous pairing; clear it.
		manifold._cachedTriangles = null;
		manifold._cacheValid = false;
		this.insert( manifold );
	}

	return manifold;
};