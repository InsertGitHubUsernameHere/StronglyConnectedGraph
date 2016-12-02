/**
 * Code for Assignment 6 in UNCG's CSC 330 (Fall 2016).
 * 
 * This is really the same as the Assign6.java file, just using a different
 * input file. Put in a separate file so there wouldn't be conflicts with
 * student Assign6.java files.
 *
 * @author Steve Tate
 */

public class A6Test {
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        Graph g = Graph.readGraph("a6test.in");
        g.printStronglyConnected();
    }
    
}
