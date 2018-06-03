// This file implements Dijkstra Algorithm on randomly created graph
// and calculates average shortest distance b/w nodes.

#include <iostream>
#include <iomanip>
#include <ctime> // for time
#include <cstdlib> // for srand()
#include <vector>
#include <list>
#include <unordered_map>

using namespace std;

// This class implements a random graph using adjacency matrix implementation.
// The graph randomly generates edges based on density parameter passed to the graph.
// This also assigns positive weights from given weight ranges to the randomly generated edges.
// When an edge has positive value (non-zero non-negative), the edge exists.
// Rational behind using adjancency matrix: Graph has been chosen to be reprented using adjacency
// matrix, for the faster lookup for edge weight during run of Dijkstra shortest path algorithm.
// The lookup cost with this implementation is constant time of O(1), instead of O(n/density);
// Alternate, approach could have been using adjacency list with an additional hash map which will store
// the weight for the edge, but that will also take additional space proportional to number of edges;
// Key for the hash would be pair of vertices connecting edge and value would be the distance.
// I chose to went with former.
class undirected_graph {
    public:

        // ctor to generate graph with adjacency matrix representation     
        undirected_graph(int num_vertices, double density, double dist_range_start, double dist_range_end) :
            n_vertices (num_vertices), edge_density(density), dist_range_start(dist_range_start),
            dist_range_end(dist_range_end) {
            // check if density is within range
            if (density <= 0 || density >= 1.0) {
                throw exception("value of density is out of range!");
            }

            if (dist_range_start < 1.0 || dist_range_end < dist_range_start + 1) {
                throw exception("Invalid distance range!");
            }

            n_edges = 0; // assign number of edges to zero
            // alloc memory for graph adjacency matrix representation
            graph = new double*[n_vertices];
            for (int i = 0; i < n_vertices; ++i) {
                graph[i] = new double[n_vertices];
            }

            // call function to randomly generate edge with weight of the graph
            generate_edge_randomly(edge_density, dist_range_start, dist_range_end);

            // allocate memory for storing node value
            node_val = new double[n_vertices];

            // initialize node value with -1
            // value of -1 means that node value is not yet calculated from current source
            // at the end of Dijkstra algorithm, if value is still -1.0, that means the node
            // is not reachable from source
            for (int i = 0; i < n_vertices; ++i) {
                node_val[i] = -1.0;
            }
        }

        // dtor to free resources consumed by graph
        ~undirected_graph() {
            // free memory occupied by node_val
            if (node_val != nullptr) {
                delete[] node_val;
            }

            // free memory occupied by adjacency matrix
            for (int i = 0; i < n_vertices; ++i) {
                delete[] graph[i];
            }

            delete[] graph;
        }

        // returns number of vertices in the graph
        int get_num_vertices() {
            return n_vertices;
        }

        // returns number of edge in the graph
        int get_num_edges() {
            return n_edges;
        }

        // returns true if there is an edge b/w vertex index idx1 to vertex idx2
        // else false
        bool adjacent(int idx1, int idx2) {
            if (idx1 < 0 || idx1 >= n_vertices || idx2 < 0 || idx2 >= n_vertices) {
                throw exception("invalid parameter!");
            }

            return (graph[idx1][idx2] != 0);
        }

        // returns all adjacent nodes for a given node
        vector<int>* get_neighbors(int idx) {
            vector<int>* neighbor_nodes = new vector<int>;
            for (int i = 0; i < n_vertices; ++i) {
                // check if there is an edge b/w idx and i
                if (graph[idx][i] != 0) {
                    neighbor_nodes->push_back(i);
                }
            }
            return neighbor_nodes;
        }

        // add an edge from node x to y if it is not there
        void add(int x, int y) {
            if (x < 0 || x >= n_vertices || y < 0 || y >= n_vertices) {
                throw exception("invalid parameter!");
            }

            if (graph[x][y] == 0) {
                graph[x][y] = graph[y][x] = generate_weight_in_range(dist_range_start, dist_range_end);
            }
        }

        // removes an edge from node x to y if it is there
        void remove(int x, int y) {
            if (x < 0 || x >= n_vertices || y < 0 || y >= n_vertices) {
                throw exception("invalid parameter!");
            }

            if (graph[x][y] != 0) {
                graph[x][y] = graph[y][x] = 0;
            }
        }

        // returns value associated with edge (x, y)
        double get_edge_value(int x, int y) {
            if (x < 0 || x >= n_vertices || y < 0 || y >= n_vertices) {
                throw exception("invalid parameter!");
            }
            return graph[x][y];
        }

        // set value assocatiated with edge (x, y) to v
        void set_edge_value(int x, int y, double v) {
            if (x < 0 || x >= n_vertices || y < 0 || y >= n_vertices) {
                throw exception("invalid parameter!");
            }
            graph[x][y] = v;  
        }

        // returns node distance from current source
        double get_node_value(int x) {
            if (x < 0 || x >= n_vertices) {
                throw exception("invalid parameter!");   
            }
            return node_val[x];
        }

        // set the node value to given value
        void set_node_value(int x, double a) {
            if (x < 0 || x >= n_vertices) {
                throw exception("invalid parameter!");   
            }
            node_val[x] = a;   
        }

        // prints adjacency matrix representing the graph
        void print() {
            cout << "\tnumber of vertices: " << n_vertices << " , number of edges: " << n_edges << " , edge_density: "
                 << edge_density << endl;
            cout << "\tidx:  ";
            for (int i = 0; i < n_vertices; ++i) {
                cout << setw(2) << i << "  ";   
            }
            cout << endl;
            for (int i = 0; i < n_vertices; ++i) {
                cout << "\t" << setw(3) << i << ":  ";
                for (int j = 0; j < n_vertices; ++j) {
                    cout << setw(2) << graph[i][j] << "  ";
                }
                cout << endl;
             }
         }

         void reset_node_values() {
             // reset node values to -1 for each vertex
             for (int i = 0; i < n_vertices; ++i) {
                node_val[i] = -1.0;
            }
         }

    private:
        int n_vertices; // number of vertices in the graph, will be numbered from 0 to n_vertices - 1
        int n_edges; // number of edges in the graph
        double **graph; // adjacency matrix for the graph
        double edge_density; // edge_density of the graph
        double dist_range_start;
        double dist_range_end;
        double *node_val; // stores the node value with respect to current source

        void generate_edge_randomly(double density_in, double dist_range_start, double dist_range_end) {
            // initialize random number generator
            srand(time(0));

            // multiply density by 100 to have a number b/w (0, 100)
            int density = static_cast<int>(100 * density_in);

            // generate the edge randomly for the graph
            for (int i = 0; i < n_vertices; ++i) {
                for (int j = i; j < n_vertices; ++j) {
                    if (i == j) {
                        graph[i][j] = 0;
                    } else {
                        if ((rand() % 101) <= density) {
                            // get the weight for the edge
                            graph[i][j] = graph[j][i] = generate_weight_in_range(dist_range_start, dist_range_end);
                            ++n_edges;
                        } else {
                            graph[i][j] = graph[j][i] = 0;
                        }
                    }
                }
            }
        }

        double generate_weight_in_range(double dist_range_start, double dist_range_end) {
            int range_length =  static_cast<int>(dist_range_end) - static_cast<int>(dist_range_start) + 1;
            return (rand() % range_length) + dist_range_start;
        }
};

// This class represents an element in the priority queue used by shortest
// path algorithm. This has three field, edge_source_node_idx (par_idx),
// edge_dest_node_idx (idx) and value (distance to reach destination node via
// source node from source node index).
class queue_element {
    public:
        queue_element(int par_idx, int node_idx, double node_value) :
            par_idx(par_idx), idx (node_idx), value (node_value) { }

        queue_element(const queue_element &elm) {
            par_idx = elm.par_idx;
            idx = elm.idx;
            value = elm.value;
        }

        int get_par_idx() {
            return par_idx;
        }

        int get_node_idx() {
            return idx;
        }

        double get_node_val() {
            return value;
        }

        void set_par_idx(int new_par_idx) {
            par_idx = new_par_idx;
        }

        void set_node_idx(int new_idx) {
             idx = new_idx;
        }

        void set_node_val(double new_val) {
            value = new_val;
        }
 
    private:
        int par_idx; // source node index of edge
        int idx;  // destination node index of edge
        double value; // value to reach destination node of edge from source node of algorithm
};

// this class implements priority queue using vector which
// is always maintained in min-heap configuration after every operation.
// queue_element.value property is used to maintain min-heap configuration
// of priority queue
class priority_queue {
    public:
         priority_queue() { }

        ~priority_queue() {
            while (p_queue.size() > 0) {
                p_queue.pop_back();
            }
        }

        // change priority of element to given value and maintain
        // the resultant configuration as min-heap, it is a no-op
        // if given value is greater than or equal to current value
        // of the element.
        void chg_priority(queue_element &elm, double value) {
            // get index of elem in p_queue
            int q_idx = get_idx(elm);
            if (q_idx == -1) {
                throw exception("Element not found!");
            }

            double prev_val = p_queue[q_idx].get_node_val();
            if (prev_val > value) {
                // update current value to node
                p_queue[q_idx].set_node_val(value);
                p_queue[q_idx].set_par_idx(elm.get_par_idx());
                // adjust the element to appropriate position in the priority queue
                decrease_value(q_idx);
            }
        }

        // check if given element exists in the priority queue
        bool contains(queue_element &elm) {
            if (get_idx(elm) != -1) {
                return true;
            } else {
                return false;
            }
        }

        // insert the given element into priority queue
        void insert(queue_element &elm) {
            p_queue.push_back(elm);
            decrease_value(p_queue.size() - 1);
        }

        // removes the top element and maintains min-heap property
        // of priority queue
        void minPriority() {
            // move the last element to front
            if (p_queue.size() > 1) {
               p_queue[0] = p_queue[p_queue.size() - 1]; 
            }
            
            // remove last element
            p_queue.pop_back();
            if (p_queue.size() > 1) {
                min_heapify(0);
            }
        }

        // returns the size of priority queue
        int size() {
            return p_queue.size();
        }

        // returns reference to the top element of priority queue
        queue_element& top() {
            if (size() == 0) {
                throw exception("Empty queue!");
            }

            return p_queue[0];
        }

    private:
        vector<queue_element> p_queue; // it is always maintained as min-heap.

        // returns index of parent for given element index in the vector
        int par(int idx) {
            return (idx - 1) / 2;
        }

        // returns index of left child for given element index in the vector
        int left_child(int idx) {
            return (2 * idx + 1);
        }

        // returns index of right child for given element index
        int right_child(int idx) {
            return (2 * idx + 1);
        }

        // adjust the node with given index to correct position in the 
        // vector to maintain min-heap property of priority queue.
        // this is called when node element value has been decreased or
        // during insertion of the element.
        void decrease_value(int idx) {
            if (idx == 0 ) {
                return;
            } 

            int par_idx = par(idx);
            if (p_queue[idx].get_node_val() < p_queue[par_idx].get_node_val()) {
                swap(p_queue[idx], p_queue[par_idx]);
                decrease_value(par_idx);
            }
        }

        // adjust the node with given index to correct positon in the vector
        // to maintain min-heap property of priority queue.
        // this is called when node top node has been removed from priority queue.
        void min_heapify(int idx) {
            if (idx >= p_queue.size()) {
                return;
            }

            int lc_idx = left_child(idx); // left child index
            int rc_idx = right_child(idx); // right child index

            int lowest_idx = idx; // lowest_idx, is the idx of element with lowest node value 
                                  // among idx, left child and right child nodes
            if (lc_idx < p_queue.size() && p_queue[lc_idx].get_node_val() < p_queue[idx].get_node_val()) {
                lowest_idx = lc_idx;
            }

            if (rc_idx < p_queue.size() && p_queue[rc_idx].get_node_val() < p_queue[lowest_idx].get_node_val()) {
                lowest_idx = rc_idx;
            }

            if (lowest_idx != idx) {
                swap(p_queue[idx], p_queue[lowest_idx]);
                min_heapify(lowest_idx);
            }
        }

        // return the index of given element in the vector
        int get_idx(queue_element &elm) {
            for (int i = 0; i < p_queue.size(); ++i) {
                if (p_queue[i].get_node_idx() == elm.get_node_idx()) {
                    return i;
                }
            }
            return -1;
        }

        // this swaps the content of given two elements
        void swap (queue_element &elem1, queue_element &elem2) {
            queue_element tmp = elem1;
            elem1 = elem2;
            elem2 = tmp;
        }
};

// this class implements Dijkstra shortest path algorithm on randomly generated
// graph; It uses priority queue for storing edges in the open set and uses
// unordered_map to store the nodes (closed set) whose distance have been calculated.
class shortest_path {
    public:
        // ctor to take input for randomly generated graph on which shorted path algorithm
        // will be run and crate the graph
        shortest_path(int n_vertices, double edge_density,
             double dist_range_start, double dist_range_end) : n_vertices(n_vertices),
             edge_density(edge_density), dist_range_start(dist_range_start), dist_range_end(dist_range_end) {
             g = new undirected_graph(n_vertices, edge_density, dist_range_start, dist_range_end);
             g->print();
         }

        // free the resources used by graph
         ~shortest_path() {
             delete g;
         }

         // returns the list of vertices in the graph
         void get_vertices(std::list<int> &vert_list) {
             for (int i = 0; i < n_vertices; ++i) {
                 vert_list.push_back(i);
             }
         }

         // this method returns the list of nodes containing the path if there is a path exists 
         // from source node (represented by src_idx) to destination node (represented by dst_idx);
         // throws and exception with message "path not found", if no path exists b/w source and
         // destination node.
         std::list<int> *get_path(int src_idx, int dst_idx) {
             std::unordered_map<int, int> closed_set; // key is child_idx, value is par_idx
                                                 // if par_idx is -1, that means child_idx was source
             priority_queue open_set; // priority queue to store discovered edges

             // reset node values to -1, so that calculted node values from source node be stored.
             g->reset_node_values();

             // insert source node in closed set
             closed_set.insert(std::make_pair(src_idx, -1)); // there is no parent for source node
             // set source node value to zero
             g->set_node_value(src_idx, 0);

             double cur_val = 0;
             int cur_node_idx = src_idx;
             double edge_dist = 0;
             int cur_dst_node_idx;
             while (cur_node_idx != dst_idx) {
                // get the list of adjacent nodes
                 vector<int> *neighbor_nodes = g->get_neighbors(cur_node_idx);
                 for (int i = 0; i < neighbor_nodes->size(); ++i) {
                     cur_dst_node_idx = neighbor_nodes->at(i);
                     if (closed_set.find(cur_dst_node_idx) != closed_set.end()) {
                         // current dst node is already in closed set, skip it
                         continue;
                     }
                     edge_dist = g->get_edge_value(cur_node_idx, cur_dst_node_idx);
                     queue_element elm = queue_element(cur_node_idx, cur_dst_node_idx, cur_val + edge_dist);
                     if (open_set.contains(queue_element(elm)) == false) {
                         // insert the edge from cur_node_idx -> cur_dst_node_idx : value
                         open_set.insert(elm);
                     } else {
                        // if edge already exists in the open set, update its value to current value if it is lower than
                        // existing value of the destination node
                        open_set.chg_priority(elm, cur_val + edge_dist); 
                     }                    
                 }

                 // no edges to discover in open set
                 if (open_set.size() == 0) {
                     break;
                 }
                 
                 // get the min element from open_set
                 queue_element top = open_set.top();
                 cur_node_idx = top.get_node_idx();
                 cur_val = top.get_node_val();
                 // add this node to close set
                 closed_set.insert(make_pair(cur_node_idx, top.get_par_idx()));
                 // update node value to cur_val
                 g->set_node_value(cur_node_idx, cur_val);
                 // remove top element from open set
                 open_set.minPriority();
             }

             // if cur_node_idx is not destinatin index, that means path is not found.
             if (cur_node_idx != dst_idx) {
                 closed_set.clear();
                 throw exception("no path found!");
             } else {
                 // get the nodes included in the path
                 list<int> *node_list = new list<int>;
                 node_list->push_front(dst_idx);
                 int cur_idx = dst_idx;
                 while (cur_idx != src_idx) {
                     auto it = closed_set.find(cur_idx);
                     cur_idx = it->second;
                     node_list->push_front(cur_idx);
                 }
                 // remove the elements in closed set
                 closed_set.clear();
                 return node_list;
             }
         }

         // returns the path length b/w src_idx and dst_idx;
         // this API should only be called after get_path() call
         double path_size(int src_idx, int dst_idx) {
             return g->get_node_value(dst_idx);
         }

    private:
        int n_vertices;  // number of vertices for the graph to be created
        double edge_density; // edge_density for the graph to be created, should be greater than 0 and less than 1.0
        double dist_range_start; // start value of distance range for the edge in graph
        double dist_range_end; // end value for distance range for the edge in the graph
        undirected_graph *g; // graph instance on which shortes path algo will run
};

void calculate_avg_path(double n_vertices, double edge_density) {
    shortest_path sp(n_vertices, edge_density, 1.0, 10.0);
    double sum_distance = 0;
    int num_path = 0;
    double path_length = 0;

    for (int i = 1; i < n_vertices; ++i)
    {
        try
        {
            list<int> *node_list = sp.get_path(0, i);
            path_length = sp.path_size(0, i);
            cout << "list of nodes forming path from 0 to " << i << " and path_length: " << path_length << endl;
            for (auto it = node_list->begin(); it != node_list->end(); ++it) {
                cout << *it << ", ";
            }
            cout << endl;
            node_list->clear();
            delete node_list;

            sum_distance += path_length;
            ++num_path;
        }
        catch (const exception &ex) {
            cout << "ex: " << ex.what() << endl;
        }
    }
    if (num_path != 0) {
        cout << "Average path length for graph with " << n_vertices << " nodes and edge_density: " << edge_density << endl; 
        cout << "avg_path_length = " << sum_distance << " / " << num_path << " = " << sum_distance / num_path << endl;
    }
    else {
        cout << "No path found!";
    }
}

// Program to test shortest path algorithm on two different input.
int main() {
    cout << "--------------- test case 1 - begin ---------------------------" << endl;
    calculate_avg_path(50, 0.2);
    cout << "--------------- test case 1 - end -----------------------------" << endl << endl << endl;

    cout << "--------------- test case 2 - begin ---------------------------" << endl;
    calculate_avg_path(50, 0.4);  
    cout << "--------------- test case 2 - end -----------------------------" << endl << endl << endl; 
    return 0;
}