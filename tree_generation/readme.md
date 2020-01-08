## KDTree-Inspired Atlas Structure for Choosing Maps

#### Objective

The Atlas is a collection of multiple maps with a shared coordinate system, used for large-scale localization. Compared to a single large map, localization with small maps can improve the time/memory efficiency. 

In our scheme, we plan to use the GPS data to find the zone that the vehicle is in, and localize with the corresponding map. When the vehicle is about to leave the map, we find the new map it will come into and do relocalization on it.

This program uses a data structure like quad-tree for compact storage and efficient query of the current zone. We firstly generate the 2D tile occupancy matrix from the point-cloud map, and construct a space-efficient structure for real-time queries.

#### Setting

Suppose the 2D plane is divided into 1*1 tiles and some are occupied. For maps, we expect that occupied tiles are concentrated in a well-formed area, and should deal with real-time queries on whether a tile is occupied. Then, we can decide if the current point is in a certain map.

For multiple maps, if the decision problem is efficient enough, we may just query one by one and find the map(s) it is in.

In this problem, we are concerned more on query costs than construction costs, as the map can be constructed offline while the queries are online.

#### Algorithm Details

##### Construction

To save query time, we compute the AABB(axis-aligned bounding box) of occupied tiles. If a tile is not within the AABB, we can instantly decide that it is not occupied.

Firstly we mark all occupied 1*1 (Level-0, L0) tiles with black, and assume other L0 squares within the AABB are marked with white in default. Then, we merge 4 aligned squares (vertex coordinates aligned with powers of 2) with the same color into a higher-level one and remove them afterwards, until there are no tiles satisfying the condition to merge. 

It is tricky that white L0 squares do not need to stored in memory, because all tiles are occupied with some squares: the ones not occupied with any other squares are occupied with white L0 squares.

The tiles are stored in memory by the STL::map structure. It takes O(log n) time for a query in a given level, a little worse than linear (for quadtree). But it is easier to store this structure into a file, so is it more intuitive to understand. 



##### Query

For any queried tile, firstly we check if it is in the AABB. If true, we check if it is occupied by a square from high levels to low levels. As each tile is covered by exactly one square, if the occupied tiles have a regular shape, then most tiles are covered by few high-level squares and can be decided efficiently.

For one query within the AABB, the time complexity is O( log^2(L/r) ) in which L is the scale of the map and r is the tile resolution. 



#### Interfaces

###### kdt::init(ifstream&, int *scale_level*)

To generate the structure from a file. The coordinate range of all tiles must be within [-2^(scale_level-1), 2^(scale_level-1)-1].

###### kdt::exists(int x, int y)

To query if the tile(x,y) is occupied.

###### kdt::save(ofstream&)

To save the structure into a file.

###### kdt::load(ifstream&)

To load the structure from a saved file.

#### I/O Formats

##### Input raw map file:

Each line contains two integers (all numbers are separated by space in this program), coordinate of an occupied tile.

*Example: map1.txt*



##### Dumped file:

The first line contains 4 integers: scale_level, maximum_square_level, x_offset, y_offset

The second line contains 4 integers: xmin, xmax, ymin, ymax (after offset)

The third line contains 2 integers: #occupied tiles, #squares.

Then, each block describes one level of squares in an ascending order. 

The first line contains two integers: current_level, #squares in this level.

Each following line contains 3 integers that describes a square: x, y, color (white=0, black=1).

*Example: tree.txt*