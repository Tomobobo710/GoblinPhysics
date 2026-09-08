(function(){
	function getSurfaceArea( aabb ) {
		var x = aabb.max.x - aabb.min.x,
			y = aabb.max.y - aabb.min.y,
			z = aabb.max.z - aabb.min.z;
		return x * ( y + z ) + y * z;
	}

	/**
	 * Tree node for a BVH
	 *
	 * @class BVHNode
	 * @param [object] {Object} leaf object in the BVH tree
	 * @constructor
	 * @private
	 */
	var BVHNode = function( object ) {
		this.aabb = new Goblin.AABB();
		this.area = 0;

		this.parent = null;
		this.left = null;
		this.right = null;

		this.morton = null;

		this.object = object || null;
	};
	BVHNode.prototype = {
		isLeaf: function() {
			return this.object != null;
		},

		computeBounds: function( global_aabb ) {
			if ( this.isLeaf() ) {
				this.aabb.copy( this.object.aabb );
			} else {
				this.aabb.combineAABBs( this.left.aabb, this.right.aabb );
			}

			this.area = getSurfaceArea( this.aabb );
		},

		valueOf: function() {
			return this.area;
		}
	};

	/**
	 * Bottom-up BVH construction based on "Efficient BVH Construction via Approximate Agglomerative Clustering", Yan Gu 2013
	 *
	 * @Class AAC
	 * @static
	 * @private
	 */
	var AAC = (function(){
		function part1By2( n ) {
			n = ( n ^ ( n << 16 ) ) & 0xff0000ff;
			n = ( n ^ ( n << 8 ) ) & 0x0300f00f;
			n = ( n ^ ( n << 4 ) ) & 0x030c30c3;
			n = ( n ^ ( n << 2 ) ) & 0x09249249;
			return n;
		}
		function morton( x, y, z ) {
			return ( part1By2( z ) << 2 ) + ( part1By2( y ) << 1 ) + part1By2( x );
		}

		var _tmp_aabb = new Goblin.AABB();

		var AAC = function( global_aabb, leaves ) {
			var global_width = global_aabb.max.x - global_aabb.min.x,
				global_height = global_aabb.max.y - global_aabb.min.y,
				global_depth = global_aabb.max.z - global_aabb.min.z,
				max_value = 1 << 9,
				scale_x = max_value / global_width,
				scale_y = max_value / global_height,
				scale_z = max_value / global_depth;

			// Compute the morton code for each leaf
			for ( var i = 0; i < leaves.length; i++ ) {
				var leaf = leaves[i],
					// find center of aabb
					x = ( leaf.aabb.max.x - leaf.aabb.min.x ) / 2 + leaf.aabb.min.x,
					y = ( leaf.aabb.max.y - leaf.aabb.min.y ) / 2 + leaf.aabb.min.y,
					z = ( leaf.aabb.max.z - leaf.aabb.min.z ) / 2 + leaf.aabb.min.z;

				leaf.morton = morton(
					( x + global_aabb.min.x ) * scale_x,
					( y + global_aabb.min.y ) * scale_y,
					( z + global_aabb.min.z ) * scale_z
				);
			}

			// Sort leaves based on morton code
			leaves.sort( AAC.mortonSort );
			// Each axis is quantized to 9 bits, interleaved into bits [0..26] — 26 is the highest bit
			// that can differ between two leaves; starting higher wastes early recursion levels on
			// no-op splits.
			var tree = AAC.buildTree( leaves, 26 );
			AAC.combineCluster( tree, 1 );
			return tree;
		};
		AAC.mortonSort = function( a, b ) {
			if ( a.morton < b.morton ) {
				return -1;
			} else if ( a.morton > b.morton ) {
				return 1;
			} else {
				return 0;
			}
		};
		// Gu et al. 2013's reduction function: f(n) = C * n^alpha (C=0.5, alpha=0.5) — combineCluster
		// should shrink a bucket to ~0.5*sqrt(n) clusters, not n/2.
		AAC.clusterReductionCount = function( cluster_size ) {
			var c = 0.5, a = 0.5;
			return Math.max( c * Math.pow( cluster_size, a ), 1 );
		};
		AAC.buildTree = function( nodes, bit ) {
			var cluster = [];

			if ( nodes.length < AAC.max_bucket_size ) {
				cluster.push.apply( cluster, nodes );
				AAC.combineCluster( cluster, AAC.clusterReductionCount( AAC.max_bucket_size ) );
			} else {
				var left = [],
					right = [];

				if ( bit < 1 ) {
					// no more bits, just cut bucket in half
					left = nodes.slice( 0, nodes.length / 2 );
					right = nodes.slice( nodes.length / 2 );
				} else {
					var bit_value = 1 << bit;
					for ( var i = 0; i < nodes.length; i++ ) {
						var node = nodes[i];
						if ( node.morton & bit_value ) {
							right.push( node );
						} else {
							left.push( node );
						}
					}
				}
				cluster.push.apply( cluster, AAC.buildTree( left, bit - 1 ) );
				cluster.push.apply( cluster, AAC.buildTree( right, bit - 1 ) );
				AAC.combineCluster( cluster, AAC.clusterReductionCount( cluster.length ) );
			}

			return cluster;
		};
		AAC.combineCluster = function( cluster, max_clusters ) {
			if ( cluster.length <= 1 ) {
				return cluster;
			}

			// find the best match for each object
			var merge_queue = new Goblin.MinHeap(),
				merged_node;
			for ( var i = 0; i < cluster.length; i++ ) {
				merged_node = new BVHNode();
				merged_node.left = cluster[i];
				merged_node.right = AAC.findBestMatch( cluster, cluster[i] );
				merged_node.computeBounds();
				merge_queue.push( merged_node );
			}

			var best_cluster;
			while( cluster.length > max_clusters ) {
				best_cluster = merge_queue.pop();
				cluster.splice( cluster.indexOf( best_cluster.left ), 1 );
				cluster.splice( cluster.indexOf( best_cluster.right ), 1 );
				cluster.push( best_cluster );

				// update the merge queue
				// @TODO don't clear the whole heap every time, only need to update any nodes which touched best_cluster.left / best_cluster.right
				merge_queue.heap.length = 0;
				for ( i = 0; i < cluster.length; i++ ) {
					merged_node = new BVHNode();
					merged_node.left = cluster[i];
					merged_node.right = AAC.findBestMatch( cluster, cluster[i] );
					merged_node.computeBounds();
					merge_queue.push( merged_node );
				}
			}
		};
		AAC.findBestMatch = function( cluster, object ) {
			var area,
				best_area = Infinity,
				best_idx = 0;
			for ( var i = 0; i < cluster.length; i++ ) {
				if ( cluster[i] === object ) {
					continue;
				}
				_tmp_aabb.combineAABBs( object.aabb, cluster[i].aabb );
				area = getSurfaceArea( _tmp_aabb );

				if ( area < best_area ) {
					best_area = area;
					best_idx = i;
				}
			}

			return cluster[best_idx];
		};
		AAC.max_bucket_size = 20;
		return AAC;
	})();

	/**
	 * Creates a bounding volume hierarchy around a group of objects which have AABBs
	 *
	 * @class BVH
	 * @param bounded_objects {Array} group of objects to be hierarchized
	 * @constructor
	 */
	Goblin.BVH = function( bounded_objects ) {
		// Create a node for each object
		var leaves = [],
			global_aabb = new Goblin.AABB();

		for ( var i = 0; i < bounded_objects.length; i++ ) {
			global_aabb.combineAABBs( global_aabb, bounded_objects[i].aabb );
			var leaf = new BVHNode( bounded_objects[i] );
			leaf.computeBounds();
			leaves.push( leaf );
		}

		this.tree = AAC( global_aabb, leaves )[0];

		this.flat = Goblin.BVH.flatten( this.tree );
	};

	/**
	 * Flattens a BVHNode tree into a cache-friendly, index-based layout for hot traversal loops
	 * (`Float32Array` of AABBs + `Int32Array` of child indices, instead of chasing `.left`/`.right`
	 * object pointers scattered across the heap). Traversing 2M individual GC'd node objects for a
	 * 1M-triangle mesh means every node visit during a BVH walk is a fresh cache miss; a flat array
	 * walk streams through contiguous memory instead. The original pointer-based `.tree` is left
	 * intact and still used by the mesh-mesh/ray-intersect paths — this is purely an additive fast
	 * path for the convex-vs-mesh hot loop, so a bug here can't affect the already-correct tree walk.
	 *
	 * Leaf/internal distinction is encoded in `children`: a leaf stores `-1 - leafIndex` (always < -1
	 * counting from -1, so index 0 encodes as -1, distinguishable from "no node"); an internal node
	 * stores its right-child flat index directly (>= 0) with the left child always immediately
	 * following its parent in the array (standard depth-first flattening), so only one child index
	 * needs to be stored per node.
	 *
	 * @method flatten
	 * @static
	 * @param root {BVHNode}
	 * @return {Object} { aabbs: Float32Array, rightOrLeaf: Int32Array, leafObjects: Array, nodeCount: Number }
	 */
	Goblin.BVH.flatten = function( root ) {
		// Iterative, not recursive: a large mesh's node count is O(N) even though depth is O(log N),
		// and a naive recursive walk blows the JS call stack on a large tree.

		// First pass: count nodes so the typed arrays can be allocated exactly once.
		var nodeCount = 0;
		var stack = [ root ];
		while ( stack.length > 0 ) {
			var n = stack.pop();
			nodeCount++;
			if ( !n.isLeaf() ) {
				stack.push( n.left, n.right );
			}
		}

		var aabbs = new Float32Array( nodeCount * 6 );
		var rightOrLeaf = new Int32Array( nodeCount );
		var leafObjects = new Array( nodeCount );

		// Second pass: depth-first pre-order assignment of flat indices, so each internal node's left
		// child always lands at parentIndex + 1 (only the right child's index needs to be stored).
		// Uses an explicit stack of {node, index, rightPending} frames to stay iterative; `rightOrLeaf`
		// for an internal node is patched in once its right subtree's root index is known, via a
		// pending-patch list keyed by the parent's flat index.
		var next = 0;
		var pendingParent = []; // parallel arrays: flat index of parent awaiting its right child's index
		var workStack = [ { node: root, parent: -1 } ];
		while ( workStack.length > 0 ) {
			var frame = workStack.pop();
			var node = frame.node;
			var i = next++;
			var base = i * 6;
			aabbs[base] = node.aabb.min.x;
			aabbs[base + 1] = node.aabb.min.y;
			aabbs[base + 2] = node.aabb.min.z;
			aabbs[base + 3] = node.aabb.max.x;
			aabbs[base + 4] = node.aabb.max.y;
			aabbs[base + 5] = node.aabb.max.z;

			if ( frame.parent >= 0 && frame.isRightChild ) {
				rightOrLeaf[frame.parent] = i;
			}

			if ( node.isLeaf() ) {
				rightOrLeaf[i] = -1 - i;
				leafObjects[i] = node.object;
			} else {
				// Push right first so left is processed next (pop = LIFO), landing left at i + 1.
				workStack.push( { node: node.right, parent: i, isRightChild: true } );
				workStack.push( { node: node.left, parent: i, isRightChild: false } );
			}
		}

		return { aabbs: aabbs, rightOrLeaf: rightOrLeaf, leafObjects: leafObjects, nodeCount: nodeCount };
	};

	Goblin.BVH.AAC = AAC;
})();