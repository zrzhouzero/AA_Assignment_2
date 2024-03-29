package pathFinder;

import map.Coordinate;
import map.PathMap;

import java.util.*;

public class DijkstraPathFinder implements PathFinder {

    private PathMap map;
    private ArrayList<PathRecorder> allPossiblePath;

    public DijkstraPathFinder(PathMap map) {
        this.map = map;
    } // end of DijkstraPathFinder()

    @Override
    public List<Coordinate> findPath() {
        // when there are no way points, simply call findPathBetween method to improve the performance
        // when there are way points, call findPathAmong to consider all the way points
        // as for more than one origins or destinations, we need to select the minimum one from all the possible paths
        this.allPossiblePath = new ArrayList<>();

        if (this.map.waypointCells.size() == 0) {
            for (Coordinate co : map.originCells) {
                for (Coordinate cd : map.destCells) {
                    allPossiblePath.add(findPathBetween(co, cd));
                }
            }
        } else {
            for (Coordinate co : map.originCells) {
                for (Coordinate cd : map.destCells) {
                    allPossiblePath.add(findPathAmong(co, cd, map.waypointCells));
                }
            }
        }

        int minimumIndex = findMinimumIndex();

        if (allPossiblePath.get(minimumIndex).isFound()) {
            System.out.println("Shortest Distance: " + allPossiblePath.get(minimumIndex).getShortestDistance());
            return allPossiblePath.get(minimumIndex).getPath();
        } else {
            return new ArrayList<>();
        }
    } // end of findPath()


    @Override
    public int coordinatesExplored() {
        HashSet<Coordinate> resultSet = new HashSet<>();
        for (PathRecorder p : allPossiblePath) {
            resultSet.addAll(p.getNodesVisited());
        }
        return resultSet.size();
    } // end of cellsExplored()

    /**
     * find the index of the minimum distance from allPossiblePath
     *
     * @return this minimum index
     */
    private int findMinimumIndex() {
        int minimumIndex = 0;
        for (int i = 1; i < allPossiblePath.size(); i++) {
            if (allPossiblePath.get(minimumIndex).getShortestDistance() > allPossiblePath.get(i).getShortestDistance()) {
                minimumIndex = i;
            }
        }
        return minimumIndex;
    }

    /**
     * @param origin      the origin point coordinate
     * @param destination the destination point coordinate
     * @param wayPoints   all the way points must be visited
     * @return PathRecorder that records the shortest path, the coordinates visited, and the distance of the shortest path
     */
    private PathRecorder findPathAmong(Coordinate origin, Coordinate destination, List<Coordinate> wayPoints) {
        // if there is no path between origin and destination
        if (!findPathBetween(origin, destination).isFound()) return new PathRecorder();
        // if there is no path between origin and any way points
        boolean isReachable = true;
        for (Coordinate c : wayPoints) {
            if (!findPathBetween(origin, c).isFound()) {
                isReachable = false;
                break;
            }
        }
        if (!isReachable) return new PathRecorder();

        // get all way points permutations
        ArrayList<ArrayList<Coordinate>> allPossibleRoute = arrangementGenerator(origin, destination, wayPoints);

        // search for the shortest path among all the routes
        int shortestDistance = Integer.MAX_VALUE;
        PathRecorder result = new PathRecorder();

        for (ArrayList<Coordinate> route: allPossibleRoute) {
            PathRecorder routeRecorder = new PathRecorder();
            for (int i = 0; i < route.size() - 1; i++) {
                routeRecorder.mergeRecorder(findPathBetween(route.get(i), route.get(i + 1)));
            }
            if (routeRecorder.getShortestDistance() < shortestDistance) {
                shortestDistance = routeRecorder.getShortestDistance();
                result = routeRecorder;
            }
        }
        return result;
    }

    private ArrayList<ArrayList<Coordinate>> arrangementGenerator(Coordinate origin, Coordinate destination, List<Coordinate> wayPoints) {
        Permutation.permutation(wayPoints, 0, wayPoints.size());
        ArrayList<ArrayList<Coordinate>> result = Permutation.getResult();
        for (ArrayList<Coordinate> c : result) {
            c.add(0, origin);
            c.add(destination);
        }
        return result;
    }

    /**
     * @param origin      the origin point coordinate
     * @param destination the destination point coordinate
     * @return PathRecorder that records the shortest path, the coordinates visited, and the distance of the shortest path
     */
    private PathRecorder findPathBetween(Coordinate origin, Coordinate destination) {
        if (!map.isIn(origin)) {
            return new PathRecorder();
        }

        List<Coordinate> path = new ArrayList<>();

        // the resultList store the shortest path to each coordinate
        List<PathFinderRecorder> resultList = new ArrayList<>();
        // the queue store all the coordinates which are needed to be inspected
        Queue<PathFinderRecorder> queue = new PriorityQueue<>(new PathFinderRecorderComparator());

        // the originRecorder is to record the origin coordinate
        // because we do NOT have to move to the same coordinate, so we can simply add it to the queue
        // to recognise the start coordinate, we set its previous coordinate to null
        PathFinderRecorder originRecorder = new PathFinderRecorder(origin, 0, null);
        queue.add(originRecorder);

        // while there still are coordinates in the queue
        while (queue.size() > 0) {
            // poll the first item from the queue, and it is the shortest path
            PathFinderRecorder temp = queue.poll();
            resultList.add(temp);

            // if this polled item is the destination coordinate, then stop inspecting the other coordinates
            if (temp.getCurrentCoordinate().equals(destination)) {
                break;
            }

            // if the shortest path is added to the resultList, any other approach except for the shortest path will be removed
            queue.removeIf(p -> p.getCurrentCoordinate().equals(temp.getCurrentCoordinate()));

            // initialise adjacent coordinates
            PathFinderRecorder[] recorders = new PathFinderRecorder[4];
            recorders[0] = null; // coordinate above
            recorders[1] = null; // coordinate below
            recorders[2] = null; // coordinate to the right
            recorders[3] = null; // coordinate to the left

            // if the adjacent coordinates are in the map, find them and give them values
            if (temp.getCurrentCoordinate().getRow() + 1 < map.sizeR) {
                recorders[0] = new PathFinderRecorder(map.cells[temp.getCurrentCoordinate().getRow() + 1][temp.getCurrentCoordinate().getColumn()],
                        map.cells[temp.getCurrentCoordinate().getRow() + 1][temp.getCurrentCoordinate().getColumn()].getTerrainCost() + temp.getShortestDistance(),
                        temp.getCurrentCoordinate());
            }
            if (temp.getCurrentCoordinate().getRow() - 1 >= 0) {
                recorders[1] = new PathFinderRecorder(map.cells[temp.getCurrentCoordinate().getRow() - 1][temp.getCurrentCoordinate().getColumn()],
                        map.cells[temp.getCurrentCoordinate().getRow() - 1][temp.getCurrentCoordinate().getColumn()].getTerrainCost() + temp.getShortestDistance(),
                        temp.getCurrentCoordinate());
            }
            if (temp.getCurrentCoordinate().getColumn() + 1 < map.sizeC) {
                recorders[2] = new PathFinderRecorder(map.cells[temp.getCurrentCoordinate().getRow()][temp.getCurrentCoordinate().getColumn() + 1],
                        map.cells[temp.getCurrentCoordinate().getRow()][temp.getCurrentCoordinate().getColumn() + 1].getTerrainCost() + temp.getShortestDistance(),
                        temp.getCurrentCoordinate());
            }
            if (temp.getCurrentCoordinate().getColumn() - 1 >= 0) {
                recorders[3] = new PathFinderRecorder(map.cells[temp.getCurrentCoordinate().getRow()][temp.getCurrentCoordinate().getColumn() - 1],
                        map.cells[temp.getCurrentCoordinate().getRow()][temp.getCurrentCoordinate().getColumn() - 1].getTerrainCost() + temp.getShortestDistance(),
                        temp.getCurrentCoordinate());
            }

            // if the adjacent coordinates exist, and they are not contained by the result list, as well as they are passable,
            // add them to the pending queue
            for (PathFinderRecorder p : recorders) {
                if (p != null) {
                    if (!resultList.contains(p) && !p.getCurrentCoordinate().getImpassable()) {
                        queue.add(p);
                    }
                }
            }
        }

        // trace back to find the path in a reversed order (because we trace from the destination from the origin)
        // and calculate the shortest distance of the path
        PathFinderRecorder pendingNode = resultList.get(resultList.size() - 1);
        int distance = pendingNode.getShortestDistance();
        // add the destination coordinate
        path.add(pendingNode.getCurrentCoordinate());

        // trace back and add the coordinate until the origin coordinate is found
        while (pendingNode.getPreviousCoordinate() != null) {
            int index;
            // find the index of the previous coordinate
            for (index = 0; index < resultList.size(); index++) {
                if (resultList.get(index).getCurrentCoordinate().equals(pendingNode.getPreviousCoordinate())) {
                    break;
                }
            }
            pendingNode = resultList.get(index);
            path.add(pendingNode.getCurrentCoordinate());
        }

        // reverse the path to make it ascending
        Collections.reverse(path);

        // add all the coordinates visited to a hash set so there is no duplicates
        HashSet<Coordinate> nodeVisited = new HashSet<>();
        for (PathFinderRecorder p : resultList) {
            nodeVisited.add(p.getCurrentCoordinate());
        }
        for (PathFinderRecorder p : queue) {
            nodeVisited.add(p.getCurrentCoordinate());
        }

        boolean isFound = (path.contains(origin) && path.contains(destination));

        // using a PathRecorder to store the result
        return new PathRecorder(path, distance, nodeVisited, isFound);
    }

} // end of class DijsktraPathFinder

/**
 * A class to store the information when perform searching including
 * the current coordinate,
 * the shortest distance from the origin to the current coordinate,
 * and the previous coordinate the shortest path have reached.
 *
 * @author zhouzhirou
 */
class PathFinderRecorder {

    private Coordinate currentCoordinate;
    private Integer shortestDistance;
    private Coordinate previousCoordinate;

    PathFinderRecorder(Coordinate currentCoordinate, Integer shortestDistance, Coordinate previousCoordinate) {
        this.currentCoordinate = currentCoordinate;
        this.shortestDistance = shortestDistance;
        this.previousCoordinate = previousCoordinate;
    }

    Coordinate getCurrentCoordinate() {
        return currentCoordinate;
    }

    Coordinate getPreviousCoordinate() {
        return previousCoordinate;
    }

    Integer getShortestDistance() {
        return shortestDistance;
    }

    @Override
    public String toString() {
        if (previousCoordinate == null) {
            return currentCoordinate.toString() + " " + shortestDistance.toString() + " NULL";
        } else {
            return currentCoordinate.toString() + " " + shortestDistance.toString() + " " + previousCoordinate.toString();
        }
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this) return true;
        if (obj == null || obj.getClass() != this.getClass()) return false;
        return ((PathFinderRecorder) obj).getCurrentCoordinate().equals(this.getCurrentCoordinate());
    }

    @Override
    public int hashCode() {
        return currentCoordinate.hashCode();
    }

}

/**
 * A comparator class to compare the shortest distance from PathFinderRecorder class
 *
 * @author zhouzhirou
 */
class PathFinderRecorderComparator implements Comparator<PathFinderRecorder> {
    @Override
    public int compare(PathFinderRecorder o1, PathFinderRecorder o2) {
        return Integer.compare(o1.getShortestDistance(), o2.getShortestDistance());
    }
}

/**
 * A class to store the path result, it contains
 * the shortest path,
 * the corresponding distance,
 * and all the coordinate visited.
 * <p>
 * Two PathRecorders can merge into one.
 *
 * @author zhouzhirou
 */
class PathRecorder {

    private List<Coordinate> path;
    private Integer shortestDistance;
    private HashSet<Coordinate> nodesVisited;
    private boolean isFound;

    PathRecorder() {
        this.path = new ArrayList<>();
        this.shortestDistance = 0;
        this.nodesVisited = new HashSet<>();
        this.isFound = true;
    }

    PathRecorder(List<Coordinate> path, Integer shortestDistance, HashSet<Coordinate> nodesVisited, boolean isFound) {
        this.path = path;
        this.shortestDistance = shortestDistance;
        this.nodesVisited = nodesVisited;
        this.isFound = isFound;
    }

    boolean isFound() {
        return isFound;
    }

    void mergeRecorder(PathRecorder p) {
        addPath(p.getPath());
        addShortestDistance(p.getShortestDistance());
        addNodesVisited(p.getNodesVisited());
        this.isFound = this.isFound && p.isFound;
    }

    Integer getShortestDistance() {
        return shortestDistance;
    }

    HashSet<Coordinate> getNodesVisited() {
        return nodesVisited;
    }

    List<Coordinate> getPath() {
        return path;
    }

    private void addShortestDistance(Integer distance) {
        this.shortestDistance += distance;
    }

    private void addNodesVisited(HashSet<Coordinate> nodesVisited) {
        this.nodesVisited.addAll(nodesVisited);
    }

    private void addPath(List<Coordinate> path) {
        if (this.path.size() == 0) {
            this.path.addAll(path);
        } else {
            // the destination of the previous iteration is the origin of the current one,
            // so as it is added, we do not have to add it again
            path.remove(0);
            this.path.addAll(path);
        }
    }

}

/**
 * A class to generate all way points permutations
 */
class Permutation {

    private static ArrayList<ArrayList<Coordinate>> result = new ArrayList<>();

    static void permutation(List<Coordinate> wayPoints, int start, int end) {
        if (start == end) {
            ArrayList<Coordinate> coordinates = new ArrayList<>();
            for (int i = 0; i < end; i++) {
                coordinates.add(i, wayPoints.get(i));
            }
            result.add(coordinates);
        } else {
            for (int i = start; i < end; i++) {
                Coordinate temp = wayPoints.get(start);
                wayPoints.set(start, wayPoints.get(i));
                wayPoints.set(i, temp);
                permutation(wayPoints, start + 1, end);
                wayPoints.set(i, wayPoints.get(start));
                wayPoints.set(start, temp);
            }
        }
    }

    static ArrayList<ArrayList<Coordinate>> getResult() {
        return result;
    }

}