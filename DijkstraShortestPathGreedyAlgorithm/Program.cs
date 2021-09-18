using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace DijkstraShortestPathGreedyAlgorithm
{
    class Program
    {
        const int infinity = 9999999;
        public class Edge
        {
            public Edge(int cost, int[] nodes)
            {
                this.cost = cost;
                if (this.cost < 0)
                    throw new ArgumentOutOfRangeException("Cost can not be ngeative");
                if (this.cost >= infinity)
                    throw new ArgumentOutOfRangeException($"Cost can not be greater than or equal to {infinity}");

                this.nodes = nodes;
            }
            public readonly int cost;
            public readonly int[] nodes = new int[2];
        }
        public class Node
        {
            public readonly int No;
            public readonly List<Edge> edges;

            public Node(int no, List<Edge> edges)
            {
                No = no;
                this.edges = edges;
            }
        }

        public class DijkstraShortestPathGreedyAlgorithm
        {
            private Dictionary<Node, int> _distanceToAchiveEveryNode = new Dictionary<Node, int>();
            private Dictionary<int, int> _parentOfNode = new Dictionary<int, int>();
            private List<Node> _visitedNodes = new List<Node>();

            private List<Node> _graph;

            public DijkstraShortestPathGreedyAlgorithm(List<Node> graph)
            {
                _graph = graph;
            }

            public void IntializeForAll(Node source)
            {
                foreach (var node in _graph)
                {
                    var distance = infinity;

                    if (node == source)
                    {
                        distance = 0;
                    }
                    _distanceToAchiveEveryNode.Add(node, distance);
                    _parentOfNode.Add(node.No, infinity);
                }
            }
            public void PrintPath()
            {
                foreach (var node in _graph)
                {
                    if (_parentOfNode[node.No] == infinity)
                    {
                        _parentOfNode[node.No] = 0;
                        _distanceToAchiveEveryNode[node] = 0;
                    }
                    Console.WriteLine($"Parent of node ({node.No}) = " + _parentOfNode[node.No] + " and final distance to reach it (" + _distanceToAchiveEveryNode[node] + ")");
                }
            }
            private Node PickNextNode()
            {
                Node nextNode = null;
                int minDist = infinity;
                foreach (var kv in _distanceToAchiveEveryNode)
                {
                    if (kv.Value <= minDist && !_visitedNodes.Contains(kv.Key))
                    {
                        minDist = kv.Value;
                        nextNode = kv.Key;
                    }
                }
                return nextNode;
            }
            void UpdateCostForNodeNeighbors(int currentNodeCost, Node from, Node node)
            {
                foreach (var edge in node.edges)
                {
                    foreach (var no in edge.nodes)
                    {
                        if(no != node.No && no != from.No)
                        {
                            var neighborNode = _graph.FirstOrDefault(d => d.No == no);
                            var newCost = currentNodeCost + edge.cost;
                            if(newCost <= _distanceToAchiveEveryNode[neighborNode])
                            {
                                _distanceToAchiveEveryNode[neighborNode] = newCost;
                                _parentOfNode[neighborNode.No] = node.No;

                            }
                        }
                    }

                }
            }


            public void Run(Node start)
            {
                Node current = start;
                _visitedNodes.Add(current);
                var distanceForCurrent = _distanceToAchiveEveryNode[current];
                int minDistance = infinity;
                Node minCostNode = null;
                foreach (var edge in current.edges)
                {
                    var newDistance = distanceForCurrent + edge.cost;
                    var edgNode = _graph.FirstOrDefault(d => edge.nodes.Contains(d.No) && d.No != current.No);
                    if (!_visitedNodes.Contains(edgNode))
                    {
                        if (newDistance <= minDistance)
                        {
                            minDistance = newDistance;
                            minCostNode = edgNode;
                        }
                        if (newDistance <= _distanceToAchiveEveryNode[edgNode])
                        {
                            _distanceToAchiveEveryNode[edgNode] = newDistance;
                            _parentOfNode[edgNode.No] = current.No;

                        }
                    }
                }
                if (minCostNode != null)
                {
                    _parentOfNode[minCostNode.No] = current.No;
                    UpdateCostForNodeNeighbors(minDistance, current, minCostNode);
                }
                var nextNode = PickNextNode();
                if (nextNode != null)
                {
                    Run(nextNode);
                }

            }
        }

        static void Main(string[] args)
        {
            var graph = new List<Node>();
            var v01 = new Edge(4, new int[] { 0, 1 });
            var v07 = new Edge(8, new int[] { 0, 7 });
            var n0 = new Node(0, new List<Edge>() { v01, v07 });
            graph.Add(n0);

            var v17 = new Edge(11, new int[] { 1, 7 });
            var v12 = new Edge(8, new int[] { 1, 2 });
            var n1 = new Node(1, new List<Edge>() { v01, v17, v12 });
            graph.Add(n1);

            var v78 = new Edge(7, new int[] { 7, 8 });
            var v76 = new Edge(1, new int[] { 7, 6 });
            var n7 = new Node(7, new List<Edge>() { v17, v07, v78, v76 });
            graph.Add(n7);

            var v28 = new Edge(2, new int[] { 2, 8 });
            var v23 = new Edge(7, new int[] { 2, 3 });
            var v25 = new Edge(4, new int[] { 2, 5 });

            var n2 = new Node(2, new List<Edge>() { v12, v28, v23, v25 });
            graph.Add(n2);

            var v68 = new Edge(6, new int[] { 6, 8 });
            var v56 = new Edge(2, new int[] { 6, 5 });
            var v67 = new Edge(1, new int[] { 6, 7 });

            var n6 = new Node(6, new List<Edge>() { v67, v68, v56 });
            graph.Add(n6);

            var n8 = new Node(8, new List<Edge>() { v28, v78, v68 });
            graph.Add(n8);

            var v45 = new Edge(10, new int[] { 4, 5 });
            var v35 = new Edge(14, new int[] { 3, 5 });

            var n5 = new Node(5, new List<Edge>() { v56, v45, v35, v25 });
            graph.Add(n5);

            var v34 = new Edge(9, new int[] { 3, 4 });
            var n3 = new Node(3, new List<Edge>() { v23, v34, v35, });
            graph.Add(n3);

            var n4 = new Node(4, new List<Edge>() { v34, v45 });
            graph.Add(n4);

            DijkstraShortestPathGreedyAlgorithm algo = new DijkstraShortestPathGreedyAlgorithm(graph);
            var source = n0;
            algo.IntializeForAll(source);
            algo.Run(source);
            algo.PrintPath();
            Console.ReadLine();



        }
    }
}
