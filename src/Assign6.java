/**
 * Code for Assignment 6 in UNCG's CSC 330 (Fall 2016).
 *
 * @author Steve Tate
 */

public class Assign6 {
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        Graph g = Graph.readGraph("a6graph1.in");
        g.printStronglyConnected();
    }
    
}
