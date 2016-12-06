/**
 * Code for Assignment 6 in UNCG's CSC 330 (Fall 2016).
 * 
 * This is a slightly modified version of the Graph implementation from the
 * Weiss textbook.
 */

import java.io.FileReader;
import java.io.IOException;
import java.util.StringTokenizer;

import java.util.Collection;
import java.util.List;
import java.util.Queue;
import java.util.Map;
import java.util.LinkedList;
import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.Stack;

import weiss.nonstandard.PairingHeap;

// Used to signal violations of preconditions for
// various shortest path algorithms.
class GraphException extends RuntimeException {

    public GraphException(String name) {
        super(name);
    }
}

// Represents an edge in the graph.
class Edge {

    public Vertex dest;   // Second vertex in Edge
    public double cost;   // Edge cost

    public Edge(Vertex d, double c) {
        dest = d;
        cost = c;
    }
}

// Represents an entry in the priority queue for Dijkstra's algorithm.
class Path implements Comparable<Path> {

    public Vertex dest;   // w
    public double cost;   // d(w)

    public Path(Vertex d, double c) {
        dest = d;
        cost = c;
    }

    public int compareTo(Path rhs) {
        double otherCost = rhs.cost;

        return cost < otherCost ? -1 : cost > otherCost ? 1 : 0;
    }
}

// Represents a vertex in the graph.
class Vertex {

    public String name;   // Vertex name
    public List<Edge> adj;    // Adjacent vertices
    public double dist;   // Cost
    public Vertex prev;   // Previous vertex on shortest path
    public int scratch;// Extra variable used in algorithm

    public Vertex(String nm) {
        name = nm;
        adj = new LinkedList<Edge>();
        reset();
    }

    public void reset() {
        dist = Graph.INFINITY;
        prev = null;
        pos = null;
        scratch = 0;
    }

    public PairingHeap.Position<Path> pos;  // Used for dijkstra2 (Chapter 23)
}

// Graph class: evaluate shortest paths.
//
// CONSTRUCTION: with no parameters.
//
// ******************PUBLIC OPERATIONS**********************
// void addEdge( String v, String w, double cvw )
//                              --> Add additional edge
// void printPath( String w )   --> Print path after alg is run
// void unweighted( String s )  --> Single-source unweighted
// void dijkstra( String s )    --> Single-source weighted
// void negative( String s )    --> Single-source negative weighted
// void acyclic( String s )     --> Single-source acyclic
// ******************ERRORS*********************************
// Some error checking is performed to make sure graph is ok,
// and to make sure graph satisfies properties needed by each
// algorithm.  Exceptions are thrown if errors are detected.
public class Graph {

    public static final double INFINITY = Double.MAX_VALUE;
    private Map<String, Vertex> vertexMap = new HashMap<String, Vertex>();

    /**
     * Add a new edge to the graph.
     */
    public void addEdge(String sourceName, String destName, double cost) {
        Vertex v = getVertex(sourceName);
        Vertex w = getVertex(destName);
        v.adj.add(new Edge(w, cost));
    }

    /**
     * Driver routine to handle unreachables and print total cost. It calls
     * recursive routine to print shortest path to destNode after a shortest
     * path algorithm has run.
     */
    public void printPath(String destName) {
        Vertex w = vertexMap.get(destName);
        if (w == null) {
            throw new NoSuchElementException("Destination vertex not found");
        } else if (w.dist == INFINITY) {
            System.out.println(destName + " is unreachable");
        } else {
            System.out.print("(Cost is: " + w.dist + ") ");
            printPath(w);
            System.out.println();
        }
    }

    /**
     * If vertexName is not present, add it to vertexMap. In either case, return
     * the Vertex.
     */
    public Vertex getVertex(String vertexName) {
        Vertex v = vertexMap.get(vertexName);
        if (v == null) {
            v = new Vertex(vertexName);
            vertexMap.put(vertexName, v);
        }
        return v;
    }

    /**
     * Recursive routine to print shortest path to dest after running shortest
     * path algorithm. The path is known to exist.
     */
    public void printPath(Vertex dest) {
        if (dest.prev != null) {
            printPath(dest.prev);
            System.out.print(" to ");
        }
        System.out.print(dest.name);
    }

    /**
     * Initializes the vertex output info prior to running any shortest path
     * algorithm.
     */
    private void clearAll() {
        for (Vertex v : vertexMap.values()) {
            v.reset();
        }
    }

    /**
     * Single-source unweighted shortest-path algorithm.
     */
    public void unweighted(String startName) {
        clearAll();

        Vertex start = vertexMap.get(startName);
        if (start == null) {
            throw new NoSuchElementException("Start vertex not found");
        }

        Queue<Vertex> q = new LinkedList<Vertex>();
        q.add(start);
        start.dist = 0;

        while (!q.isEmpty()) {
            Vertex v = q.remove();

            for (Edge e : v.adj) {
                Vertex w = e.dest;
                if (w.dist == INFINITY) {
                    w.dist = v.dist + 1;
                    w.prev = v;
                    q.add(w);
                }
            }
        }
    }

    /**
     * Single-source weighted shortest-path algorithm.
     */
    public void dijkstra(String startName) {
        PriorityQueue<Path> pq = new PriorityQueue<Path>();

        Vertex start = vertexMap.get(startName);
        if (start == null) {
            throw new NoSuchElementException("Start vertex not found");
        }

        clearAll();
        pq.add(new Path(start, 0));
        start.dist = 0;

        int nodesSeen = 0;
        while (!pq.isEmpty() && nodesSeen < vertexMap.size()) {
            Path vrec = pq.remove();
            Vertex v = vrec.dest;
            if (v.scratch != 0) // already processed v
            {
                continue;
            }

            v.scratch = 1;
            nodesSeen++;

            for (Edge e : v.adj) {
                Vertex w = e.dest;
                double cvw = e.cost;

                if (cvw < 0) {
                    throw new GraphException("Graph has negative edges");
                }

                if (w.dist > v.dist + cvw) {
                    w.dist = v.dist + cvw;
                    w.prev = v;
                    pq.add(new Path(w, w.dist));
                }
            }
        }
    }

    /**
     * Single-source weighted shortest-path algorithm using pairing heaps.
     */
    public void dijkstra2(String startName) {
        PairingHeap<Path> pq = new PairingHeap<Path>();

        Vertex start = vertexMap.get(startName);
        if (start == null) {
            throw new NoSuchElementException("Start vertex not found");
        }

        clearAll();
        start.pos = pq.insert(new Path(start, 0));
        start.dist = 0;

        while (!pq.isEmpty()) {
            Path vrec = pq.deleteMin();
            Vertex v = vrec.dest;

            for (Edge e : v.adj) {
                Vertex w = e.dest;
                double cvw = e.cost;

                if (cvw < 0) {
                    throw new GraphException("Graph has negative edges");
                }

                if (w.dist > v.dist + cvw) {
                    w.dist = v.dist + cvw;
                    w.prev = v;

                    Path newVal = new Path(w, w.dist);
                    if (w.pos == null) {
                        w.pos = pq.insert(newVal);
                    } else {
                        pq.decreaseKey(w.pos, newVal);
                    }
                }
            }
        }
    }

    /**
     * Single-source negative-weighted shortest-path algorithm.
     */
    public void negative(String startName) {
        clearAll();

        Vertex start = vertexMap.get(startName);
        if (start == null) {
            throw new NoSuchElementException("Start vertex not found");
        }

        Queue<Vertex> q = new LinkedList<Vertex>();
        q.add(start);
        start.dist = 0;
        start.scratch++;

        while (!q.isEmpty()) {
            Vertex v = q.remove();
            if (v.scratch++ > 2 * vertexMap.size()) {
                throw new GraphException("Negative cycle detected");
            }

            for (Edge e : v.adj) {
                Vertex w = e.dest;
                double cvw = e.cost;

                if (w.dist > v.dist + cvw) {
                    w.dist = v.dist + cvw;
                    w.prev = v;
                    // Enqueue only if not already on the queue
                    if (w.scratch++ % 2 == 0) {
                        q.add(w);
                    } else {
                        w.scratch--;  // undo the enqueue increment    
                    }
                }
            }
        }
    }

    /**
     * Single-source negative-weighted acyclic-graph shortest-path algorithm.
     */
    public void acyclic(String startName) {
        Vertex start = vertexMap.get(startName);
        if (start == null) {
            throw new NoSuchElementException("Start vertex not found");
        }

        clearAll();
        Queue<Vertex> q = new LinkedList<Vertex>();
        start.dist = 0;

        // Compute the indegrees
        Collection<Vertex> vertexSet = vertexMap.values();
        for (Vertex v : vertexSet) {
            for (Edge e : v.adj) {
                e.dest.scratch++;
            }
        }

        // Enqueue vertices of indegree zero
        for (Vertex v : vertexSet) {
            if (v.scratch == 0) {
                q.add(v);
            }
        }

        int iterations;
        for (iterations = 0; !q.isEmpty(); iterations++) {
            Vertex v = q.remove();

            for (Edge e : v.adj) {
                Vertex w = e.dest;
                double cvw = e.cost;

                if (--w.scratch == 0) {
                    q.add(w);
                }

                if (v.dist == INFINITY) {
                    continue;
                }

                if (w.dist > v.dist + cvw) {
                    w.dist = v.dist + cvw;
                    w.prev = v;
                }
            }
        }

        if (iterations != vertexMap.size()) {
            throw new GraphException("Graph has a cycle!");
        }
    }

    /**
     * Process a request; return false if end of file.
     */
    public static boolean processRequest(Scanner in, Graph g) {
        try {
            System.out.print("Enter start node:");
            String startName = in.nextLine();

            System.out.print("Enter destination node:");
            String destName = in.nextLine();

            System.out.print("Enter algorithm (u, d, n, a ): ");
            String alg = in.nextLine();

            if (alg.equals("u")) {
                g.unweighted(startName);
            } else if (alg.equals("d")) {
                g.dijkstra(startName);
                g.printPath(destName);
                g.dijkstra2(startName);
            } else if (alg.equals("n")) {
                g.negative(startName);
            } else if (alg.equals("a")) {
                g.acyclic(startName);
            }

            g.printPath(destName);
        } catch (NoSuchElementException e) {
            return false;
        } catch (GraphException e) {
            System.err.println(e);
        }
        return true;
    }

    /**
     * Method to read a graph from a file - basically extracted from the "main"
     * method in Weiss's code.
     *
     * @param fileName - name of the file to read
     * @return the graph, or null if there was an error reading (no file, etc.)
     */
    public static Graph readGraph(String fileName) {
        Graph g = new Graph();

        try {
            FileReader fin = new FileReader(fileName);
            Scanner graphFile = new Scanner(fin);

            // Read the edges and insert
            String line;
            while (graphFile.hasNextLine()) {
                line = graphFile.nextLine();
                StringTokenizer st = new StringTokenizer(line);

                try {
                    if (st.countTokens() != 3) {
                        System.err.println("Skipping ill-formatted line " + line);
                        continue;
                    }
                    String source = st.nextToken();
                    String dest = st.nextToken();
                    int cost = Integer.parseInt(st.nextToken());
                    g.addEdge(source, dest, cost);
                } catch (NumberFormatException e) {
                    System.err.println("Skipping ill-formatted line " + line);
                }
            }
        } catch (IOException e) {
            System.err.println(e);
            return null;
        }

        return g;
    }

    public static void interactiveTest(Graph g) {
        System.out.println("File read...");
        System.out.println(g.vertexMap.size() + " vertices");

        Scanner in = new Scanner(System.in);
        while (processRequest(in, g))
             ;
    }

    /**
     * printGraph prints the graph to standard output in a readable format.
     */
    public void printGraph() {
        for (Vertex v : vertexMap.values()) {
            System.out.print(v.name+" -- ");
            
            if (v.adj.size() == 0) {
                System.out.println("no edges out");
            } else if (v.adj.size() == 1) {
                System.out.println("edge out to:");
            } else {
                System.out.println("edges out to:");
            }
            
            for (Edge e : v.adj) {
                System.out.println("    "+e.dest.name);
            }            
        }
    }

    /**
     * getTransposed() graph creates the transpose of the graph and returns
     * it as a new graph. The transpose contains all of the vertices of the
     * original graph, but edges are in the opposite direction (all other
     * properties, such as edge cost, are preserved).
     * @return the transposed graph
     */
    public Graph getTransposedGraph() {
        Graph result = new Graph();
        for (Vertex v : vertexMap.values()) {
            result.getVertex(v.name);
            for (Edge e : v.adj)
                result.addEdge(e.dest.name, v.name, e.cost);
        }
        
        return result;
    }    

    // Recursive workhorse function for basic depth first search.
    private void dfs_r(Vertex v) {
//        System.out.println("  "+v.name);
        for (Edge e : v.adj) {
            if (e.dest.scratch == 0) {
                e.dest.scratch = 1;
                e.dest.prev = v;
                dfs_r(e.dest);
            }
        }
    }

    /**
     * Depth first search routine that is callable from outside the Graph class.
     * This is a driver function for dfs_r, the recursive function that really
     * does all the work.
     *
     * @param v The vertex to start from
     */
    public void dfs(Vertex start) {
        clearAll();
        start.scratch = 1;
        dfs_r(start);
    }

    /**
     * Prints connected components to standard output using depth-first search.
     * Designed for undirected graphs, so really should only be called on
     * graphs that have bidirectional edges (an edge from w to v whenever
     * there is an edge from v to w.
     */
    public void connected() {
        clearAll();
        for (Vertex v : vertexMap.values()) {
            if (v.scratch == 0) {
                System.out.println("\nComponent:");
                v.scratch = 1;
                dfs_r(v);  // Need to use a version of dfs that prints vertices
            }
        }
    }

    // private workhorse function for first dfs in strongly connected
    // components algorithm (regular dfs pushing vertices on stack when
    // finished with them
    private void dfs_scc1(Vertex v, Stack<String> sccStack) {
        for (Edge e : v.adj) {
            if (e.dest.scratch == 0) {
                e.dest.scratch = 1;
                e.dest.prev = v;
                dfs_scc1(e.dest, sccStack);
            }
        }
        sccStack.push(v.name);
    }

    // private workhorse function for second dfs in strongly connected
    // components algorithm (called on transpose graph, but otherwise is
    // a standard dfs printing out vertices when they are seen
    private void dfs_scc2(Vertex v) {
        System.out.println(v.name);
        for (Edge e : v.adj) {
            if (e.dest.scratch == 0) {
                e.dest.scratch = 1;
                e.dest.prev = v;
                dfs_scc2(e.dest);
            }
        }
    }

    /**
     * printStronglyConnected() prints out the strongly connected components
     * of the graph. Uses Kosaraju's algorithm which has time complexity
     * O(|V|+|E|).
     */
    public void printStronglyConnected() {
        clearAll();

        // Step 1: dfs on original graph, pushing vertex names on a stack
        // in order of postorder traversal of dfs tree
        Stack<String> sccStack = new Stack<>();
        for (Vertex v : vertexMap.values()) {
            if (v.scratch == 0) {
                v.scratch = 1;
                dfs_scc1(v, sccStack);
            }
        }

        // Step 2: Compute the transpose of the graph
        Graph transposed = getTransposedGraph();
        
        // Step 3: dfs on transposed graph, picking start vertices in order
        // of popping off the stack created in Step 1
        transposed.clearAll();
        while (!sccStack.empty()) {
            Vertex v = transposed.getVertex(sccStack.pop());
            if (v.scratch == 0) {
                System.out.println("\nComponent:");
                v.scratch = 1;
                transposed.dfs_scc2(v);
            }
        }
    }

    /**
     * A main routine that: 1. Reads a file containing edges (supplied as a
     * command-line parameter); 2. Forms the graph; 3. Repeatedly prompts for
     * two vertices and runs the shortest path algorithm. The data file is a
     * sequence of lines of the format source destination cost
     */
    public static void main(String[] args) {
        Graph g = readGraph("graph1.txt");

        // The commented-out lines below are various things we tried in class
        // interactiveTest(g);
        // g.connected();
        // g.unweighted("D");
        g.dfs(g.getVertex("D"));
        g.printPath(g.getVertex("E"));
        System.out.println();
    }
}
