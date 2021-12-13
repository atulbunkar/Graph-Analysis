
//Submitted by : 214101011, Mtech, Sem-1.
#include<bits/stdc++.h>
#include <iostream>
#include <cstdlib>
#include<fstream>
#include<math.h>

using namespace std;

struct AdjListNode{         //stores a node in adjlist
    int dest;
    int wt;
    struct AdjListNode* next;
    char type ;
    vector<int> traverse ;
};
 

struct AdjList{             //creates a new adjlist node
    struct AdjListNode *head;
};

class Graph
{
    private:
        int V;                  //node count
        int Time = 0;           //timer for dfs
        int time = 0;
        struct AdjList* array;    

        int* stime;         //start time of nodes(DFS , Tarjan)
        int* etime;         //end time of nodes
        bool* visited;      //marks visited nodes

        bool* onStack;      //array tells which nodes are on stack (tarjan)
        int* low;           //low link array(tarjan)
        stack<int> st;      
        int SCC = 0;                 //count of SCC of graph
        vector<vector<int>> scc;    //stores SCC components

        
        //helper func for dfs() / soln 1.
        void traverse_dfs(int v);       //recusive dfs 
        void mark_edge(int v , ostream& myfile , vector<vector<bool>>& vstd , int call);   //mark edgetype and print dot code

        //helper for tarjan() / soln2
        void tar_jan(int i);                            //run tarjan algo
        void printComponents(const std::string& name);  //print components  

        //helper for soln 3
        bool path_present(int s , int d);   //checks if d can reach from s node


        //helper for soln4.
        bool isSemiCon(int call, vector<vector<int>>& inter_scc_edges,const std::string& name);     //all functions called from here
        void topologicalSort();                     //topological sort on SCC graph
        void dfs_topological(int v);                //same
        bool checklinearorder();                    //checks if a linear path present in scc graph
        void printComponentGraph(int V1,const std::string& name); //print component graph


    public:
        Graph(int V){           //constructor
            this->V = V;
            stime = new int[V+1];
            etime = new int[V+1];
            visited = new bool[V+1];
            array = new AdjList[V+1];
            onStack = new bool[V+1];
            low = new int[V+1];

            for (int i = 0; i <= V; i++){
                stime[i] = 0;
                etime[i] = 1000;
                visited[i] = false;
                onStack[i] = false;
                array[i].head = NULL;
            }   
        }
        
        AdjListNode* newAdjListNode(int src ,int dest , int wt){
            AdjListNode* newNode = new AdjListNode;
            newNode->dest = dest;
            newNode->next = NULL;       //pts to next adj list vertex
            newNode->wt = wt;
            return newNode;
        }
        
        void addEdge(int src, int dest,int wt ){        //add edge to graph
            AdjListNode* newNode = newAdjListNode(src,dest,wt);
            newNode->next = array[src].head;
            array[src].head = newNode;
        }
    
        //Main tasks 
        void dfs(int call ,const std::string& name);              //soln 1 - call tells which soln called this function.
        void tarjan(int call, const std::string& name);          //soln 2
        
        void minGraph();                                         //soln 3
                     
        bool soln4(const std::string& name);                     //soln 4

        void shortestPath(int src,int ddest );                   //soln 5

};

//------------------------------------------------------

//Soln 1 - DFS
void Graph::dfs(int call ,const std::string& name){       //DFS on graph and produce image of it.

    for(int v = 0 ; v <= V ; v++){
        stime[v] = 0;
        etime[v] = 0;
        visited[v] = false;
    }
    //DFS and getting stime , etime
    for(int v =1 ; v <= V; v++){
        if(!(visited[v])){      //checks if vertice already visited
            traverse_dfs(v);   
        }
    }
    for(int v = 0 ; v <= V ; v++){
        visited[v] = false;                      //stores info of vertice already visited                          
    }
    vector<vector<bool>> vstd(V+1 , vector<bool>(V+1, false));  //stores info of edge already marked.

    ofstream myfile1;       //to print DFS graph.
    myfile1.open(name);

    myfile1<<"digraph G{\n" ;                      //DOT code.
    myfile1<<"node[style=\"rounded\",shape=record];\n" ;

    //marking edges based on stime,etime
    for(int v =1 ; v <= V; v++){
        if(!(visited[v])){
            mark_edge(v ,myfile1 , vstd, call);   
        }
    }
    myfile1<< "}\n";
    myfile1.close();

    //cmd line to get png of DFS tree
    string cmd = "dot.exe -Tpng " + name +" -o " + name + ".png";  
    system((const char*)cmd.c_str());
    return ;
}

void Graph::traverse_dfs(int v){    //get start time and end time of all nodes during dfs
    AdjListNode* vertice = array[v].head ;
    visited[v] = true;

    stime[v] = ++Time;
   
    while(vertice){
        int u = vertice->dest;          //edge going: v---->u.

        if(!(visited[u])){
            traverse_dfs(u);
        }
        vertice = vertice->next ; 
        etime[v] = ++Time;
    }
    if(etime[v] == 0)etime[v] = ++Time; //for vertices with no outgoing edges.
    return;
}

void Graph::mark_edge(int v , ostream& myfile , vector<vector<bool>>& vstd , int call){    //print dot code according to edge type
                                                               // Structure of node :| start_time | Node key | end_time | 
    AdjListNode* vertice = array[v].head ;
    visited[v] = true;
     while(vertice){
        int u = vertice->dest;    

        if(vstd[v][u]){               //checks if edge already marked.
            vertice = vertice->next ;
            continue;    
        }
        vstd[v][u] = true;
        if(!(visited[u])){

            if(call == 3)                               //means if creating dot file for gmin/soln-3 graph.
                    myfile<< v << " -> " << u << "\n" ;
            else
                myfile<< v << " -> " << u << "[label=\"" << "tree(wt= " << vertice->wt << ")\", color = " << "black"  << " ];\n" ;

            mark_edge(u , myfile , vstd , call);
        }
        else{
            if(stime[v] > stime[u] && etime[v] < etime[u] || v == u ){
                if(call == 3)
                    myfile<< v << " -> " << u << "\n" ;
                else
                    myfile<< v << " -> " << u << "[label=\"" << "back(wt= " << vertice->wt << ")\", color = " << "green" << " ];\n" ;
            }
            else if( stime[v] < stime[u] && etime[v] > etime[u] && call != 3 ){
        
                myfile<< v << " -> " << u << "[label=\"" << "forward(wt= " << vertice->wt << ")\", color = " << "red" << " ];\n" ;
            }
            else if(stime[v]  > etime[u] && etime[v] > stime[v]  ){
                if(call == 3)
                    myfile<< v << " -> " << u << "\n" ;
                else
                    myfile<< v << " -> " << u << "[label=\"" << "cross(wt= " << vertice->wt << ")\", color = " << "blue" << " ];\n" ;
            }
        }
        vertice = vertice->next ;
    }
    if(call != 3)myfile<< v << "[label = \"<f1> s_time : "<< stime[v]  <<" |<f2>  " << v << " |<f3>   e_time : " << etime[v] << " \"] \n";
    return;
}

//------------------------------------------------------

//Soln 2 - Tarjan Algorithm
void Graph::tarjan(int call ,const std::string& name){      
    if(scc.size() == 0 ){

        for(int i = 0 ; i <= V ; i++){          //initialize 
            visited[i] = onStack[i] = false;    //tells which node is in stack
            stime[i] = 0;
        }

        for(int i = 1 ; i <= V ; i++){          //for all nodes , call tarjan
            if(visited[i] == false)tar_jan(i); 
        }

    }

    if(call == 2)printComponents(name); 

    return;
}

void Graph::tar_jan(int v){         //Tarjan algo implementation
    visited[v] = true;
    stime[v] = low[v] = time++;
    onStack[v] = true;          
    st.push(v);

    AdjListNode* vertice = array[v].head ;  //traverse the vertices and get low value of nodes
    while(vertice){
        int u = vertice->dest;
        if((visited[u] == true) && (onStack[u] == true))
        {
            low[v] = min(low[v] , stime[u]);
        }
        else{

            if(visited[u] == false)
            {
                tar_jan(u);
                if(onStack[u] == true)
                low[v] = min(low[v] , low[u]);
            }
        }
        vertice = vertice->next;
    }

    if(stime[v] == low[v]){     //found a new SCC
        SCC++;
        vector<int> one_scc;
        int x;
        
        while(true)     //store all nodes with same low values to one array of 2d scc vector .
        {
            x = st.top();   
            st.pop() ;
            onStack[x] = false;     
            one_scc.push_back(x);
            if(x == v)break;        //if node value == low, we got all nodes in that component
        }
        scc.push_back(one_scc);
    }
}

void Graph::printComponents(const std::string& name){   //prints all SCC components separately showing all nodes in each SCC.
    ofstream myfile2 ;
    myfile2.open(name); 
    cout << "\nNo. of SCC are : " << scc.size() <<endl;

    myfile2<<"digraph G{\n" ; 

    for(int i = 0; i < scc.size() ; i++){   
        myfile2<<"subgraph cluster_" << i << " {" <<endl ;
        myfile2<< "label=\" SCC "<< i+1 <<"\";  \n";
        cout<< "SCC " << i+1 << ":   ";

        int first = scc[i][0];

        if(scc[i].size() == 1){
            myfile2 << first  << "\n}\n";
            cout << first << endl;
            continue;
        }
        int j;
        for(j =0 ; j < scc[i].size()-1 ; j++){
            cout<< scc[i][j] << " ";

            for(int k = j+1 ; k < scc[i].size() ; k++){
                
                AdjListNode* v1 = array[scc[i][j]].head ;
                AdjListNode* v2 = array[scc[i][k]].head ;

                while(v1){
                    if(v1->dest == scc[i][k] )
                        myfile2<< scc[i][j]  << "->" << scc[i][k] <<";\n" ;
                    v1 = v1->next;
                }

                while(v2){
                    if(v2->dest == scc[i][j] )
                        myfile2<< scc[i][k]  << "->" << scc[i][j] <<";\n" ; 
                    v2 = v2->next;
                }
            }
        }
        cout<< scc[i][j] << " ";
        cout << endl;
        myfile2 << "}\n";
    }
    myfile2 << "}\n";
    myfile2.close();

    string cmd = "dot.exe -Tpng " + name + " -o " + name + ".png";  
    system((const char*)cmd.c_str());
}

//-------------------------------------------------------
//soln 3 - 
void Graph::minGraph(){              //It uses functions from soln 2 and soln 4.
    int edgeCount = 0;

    Graph g_min(V);                                // g_min is new graph to which edges are added from main graph in minimal way.
                                                 //g_min will have n edges for n vertice graph.
    if(scc.size() == 0)tarjan(3,"ignore3");     //get SCCs in vector scc.
    
    for(auto cc : scc){                               //loop all nodes in each SCC and add edge to g_min in cyclic way.
        for(int i = 0 ; i < cc.size()-1 ; i++){
            g_min.addEdge(cc[i+1] , cc[i], 0 );        //adding edges within a SCC to form a simple cycle ( n nodes have n edges btw them )
            edgeCount++;
        }
        if(cc.size()>1){
            g_min.addEdge(cc[0] , cc[cc.size()-1] ,0);  //connecting last and first node
            edgeCount++ ;
        }
    }
   
    vector<vector<int>> inter_scc_edges(V+1, vector<int>() ) ;  //holds edges from one scc to another.(if present in original graph)

    bool x =isSemiCon(3 , inter_scc_edges ,"ignore");    // fill the inter_scc_edges vector with edges between SCCs.

    for(int i = 1 ; i < inter_scc_edges.size() ; i++ ){           //looping all src SCC
       
        for(int j = 0 ; j < inter_scc_edges[i].size() ; j++ ){            //looping all dest SCC

            if(!g_min.path_present(i, inter_scc_edges[i][j] )){     //check if path already present to prevent redundant edge addition
                g_min.addEdge(i, inter_scc_edges[i][j] , 0);     //add edge between inter component nodes.  
                edgeCount++;
            }
        }
    }
    
    g_min.dfs(3,"gmin_Graph");                         //prints g_min graph. Check g_min is subgraph of g with minimal edges. check g_graph and g3_graph.png
    
    g_min.tarjan(2,"gmin_Components");                   //prints separate components of g_min. Check g_components and g3_components.png .
    
    x = g_min.soln4("gmin_Component_Graph");            //prints gmin_component_graph . Check component graph is same of g and g_min in png.

    return;
}


bool Graph::path_present(int s , int d){        //check if there is path from src to dest node using BFS
    if (s == d)
      return true;
 
    for (int i = 0; i <= V; i++)
        visited[i] = false;
 
    list<int> queue;
 
    visited[s] = true;
    queue.push_back(s);
 
    while (!queue.empty()){
        
        s = queue.front();
        queue.pop_front();
    
        AdjListNode* adj_s = array[s].head ;

        while(adj_s)
        {
            int i = adj_s->dest;

            if (i == d)
                return true;
 
            if (!visited[i])
            {
                visited[i] = true;
                queue.push_back(i);
            }
            adj_s = adj_s->next;
        }
    }

    return false;
}


//-------------------------------------------------------
//soln 4 -
bool Graph::soln4(const std::string& name){
     if(scc.size() ==0 )tarjan(4,"ignore4");     //first we get all SCC from tarjan algo and store then in scc vector(if not already done)

     if(scc.size() == 1){                    //if Only one SCC, so , it is semiconnected.
        int V1 = 1;
        printComponentGraph(V1,name);
        return true;
    }

    vector<vector<int>> inter_scc_edges(V+1, vector<int>()) ;  //not needed for this soln , just filling params.(see soln 3)
    return isSemiCon(4 ,  inter_scc_edges, name);
}

bool Graph::isSemiCon(int call , vector<vector<int>>& inter_scc_edges , const std::string& name ){
                       
    Graph g4(scc.size());   //creating a graph of SCCs, of size = no. of SCC. (one node represent one SCC)
    bool flag1 , flag2;
    
    for(int u = 0 ; u < scc.size()-1 ; u++){        //looping one SCC(u)

        for(int v = u+1 ; v < scc.size() ; v++){    //looping another SCC(v)
            flag1 = true;
            flag2 = true;
            for(int u1 : scc[u]){                        //looping one scc vertices
                
                for(int v1:scc[v]){                     //looping another scc vertices

                    AdjListNode* src = array[u1].head ;     //to find if any edge goes from one SCC to another or vice versa.
                    AdjListNode* src2 = array[v1].head ;

                    while(src and flag1){
                        if(src->dest == v1){
                            flag1 = false;              //edge found so flag = false.
                            g4.addEdge(u+1,v+1,0);      //adding edge(u,v) if scc(u)'s edge goes to scc(v) (with wt =0)
                            inter_scc_edges[u1].push_back(v1);
                            break;
                        }
                        src = src->next;
                    }

                    while(src2 and flag2){
                        if(src2->dest == u1){
                            flag2 = false;
                            g4.addEdge(v+1,u+1,0);    //adding edge(v,u) if scc(v)'s edge goes to scc(u) (with wt =0)
                            inter_scc_edges[v1].push_back(u1);
                            break;
                        }
                        src2 = src2->next;
                    }

                if(!flag1 or !flag2)break; //if any one edge found either from u->v or v->u . We stop and move to next 2 SCC.
                }
                if(!flag1 or !flag2)break;
            } 
        }
    }//end of all for loop

    if(call == 3)return true;    //if call made from soln 3 to fill inter_scc_edges , we dont need to move ahead.
    
    g4.printComponentGraph(scc.size(),name);       //print component graph. (SCC as nodes and edges between components)

    g4.topologicalSort();       //topological sort the SCC graph
 
    //check(for all i), if edge btw vi -> vi+1 in topological sorted graph. (here vi , vi+1 ...vk are each nodes made from one SCC each.)
    return g4.checklinearorder();
}
  
// The function to do Topological Sort.
void Graph::topologicalSort(){     //Get topological Sorting of SCC components

    for (int i = 0; i <= V; i++)
        visited[i] = false;
  
    for (int i = 1; i <= V; i++)
        if (visited[i] == false)
            dfs_topological(i);
  
}   

void Graph::dfs_topological(int v){ //Get topological Sorting of SCC components
    
    visited[v] = true;
    AdjListNode* src = array[v].head;

    while(src){
        int ddest = src->dest;
        if (!visited[ddest])
            dfs_topological(ddest);

        src =src->next;
    }

    st.push(v);  
}

bool Graph::checklinearorder(){     //check if all SCC are linearly connected. SCC(i)->SCC(i+1)->.....->SCC(k)
    
    int Vi = st.top();  
    st.pop();

    while(!st.empty()){
        int Vi_plus_1 = st.top();

        st.pop();

        AdjListNode* Vi_ = array[Vi].head;

        if(Vi_ == NULL)return false;

        if(Vi_->dest != Vi_plus_1){

            return false;
        }

        Vi = Vi_plus_1;
    }
    return true;
}

void Graph::printComponentGraph(int V1 ,const std::string& name){       //print ComponentGraph image
    ofstream myfile4;
    myfile4.open(name); 

    vector<bool> vsstd(V+1 , false) ;

    myfile4<<"digraph G{\n" ;                      //DOT code.
    myfile4<<"node[shape=circle];\n" ;

    int v;
    for(v =0 ; v < V1 ; v++){
        if(vsstd[v+1])continue;
        vsstd[v+1] = true;

        myfile4<< v+1 << "[label = \"SCC "<< v+1 << " \"] \n";

        AdjListNode* adj_v = array[v+1].head ;
       
        while(adj_v){
            myfile4 << v+1 << " -> " << adj_v->dest << ";\n" ;      
            adj_v = adj_v->next;
        }
    }

    myfile4<< "}\n";
    myfile4.close();

   // cmd line to convert gv file to png.
   string cmd = "dot.exe -Tpng "+ name + " -o " + name + ".png";  
    system((const char*)cmd.c_str());

    return;
}


//-------------------------------------------------------
//soln 5 -

void Graph::shortestPath(int src ,int t ){          //prints shortes path btw 2 nodes using djikstra.

    if(src > V || t > V || src < 1 || t < 1){
        cout<< "Either Source node or target Node is out of range!! \n";
        return;
    }

    priority_queue< pair<int, int>, vector<pair<int, int>> , greater<pair<int, int>> > pq;  //-> min queue
  
    vector<int> dist(V+1, INT_MAX);
  
    pq.push(make_pair(0, src));     //pq -> <dist, node> 
    dist[src] = 0;
  
    while (!pq.empty()){
        
        int u = pq.top().second;
        pq.pop();
  
        AdjListNode* adj_u = array[u].head;

        while(adj_u)
        {
            int v = adj_u->dest ;
            int weight = adj_u->wt ;
  
            //  If there is shorted path to v through u.
            if (dist[v] > dist[u] + weight)
            {
                // Updating distance of v
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));
            }

            adj_u = adj_u->next;
        }
    }
    cout << "\nShortest Distance from " <<  src << " to " << t << " = " ;
    if(dist[t] == INT_MAX)
        cout<< "INFINITE/NOT REACHABLE " <<endl;
    else
        cout<< dist[t] <<endl;
}

//-----------------------------------------------------------


// Driver program

int main()
{
    cout<< "\n---------------DS Assn 4 - GRAPH --------------------- \n\n";
    int n,m;
    int u,v,w;
    string input;

    ifstream file;
    cout<<"Enter full input file name(including.txt)\n";
    cin >> input;

    file.open(input);
    if(!file.is_open()){
        cout << " file " << input <<" did not open !! Exiting\n!!";
        return 0;
    }

    file >> n >>m ;
    cout<< "#Nodes : " << n << ", #Edges : " << m <<endl;
    if(n == 0){
        cout << "Empty Graph !!! Exiting !!!\n";
        return 1;
    }

    Graph g(n);     //creating a graph with n nodes.
    cout<< "Creating graph from the input file........\n";
    while(file>>u>>v>>w){       //reading input and adding edges to the graph.
        g.addEdge(u,v,w);
    }
    
    int soln;
    cout<< "------Graph created !-------\n";
    

while(true){

    cout<< "\nChoose which solution to run(int 1 to 5) , or enter 6 to exit program :\n";
    cout<<"1.Run DFS\n2.Get components using Tarjan()\n3.Get minimal edge graph\n4.check semiconnected or not\n5.Find shortest distance btw 2 nodes\n";
    cout<<"Enter choice: ";
    cin>> soln;

    switch(soln) {
        case 1:
            g.dfs(1,"g_Graph");
            cout << "\nDFS graph created, check out image : g_Graph.png\n";
            cout<<"\nStructure of node :| start_time | Node key | end_time | \n";
            break;
        case 2:{
            g.tarjan(2,"g_components"); 
            cout << "\nAll Strong components is shown in image : g_components.png\n";
            }
            break;
        case 3:{
            g.minGraph();
            cout<< "\nThe minimal edge graph g_min is created and 3 images(gmin_Graph , gmin_components , gmin_Component_Graph) are created.\n";
            }
            break;
        case 4:{
            if(g.soln4("g_Component_Graph"))  cout<< " YES ! The graph is Semi Connected !";
            else cout << " NO ! The graph is not Semi Connected !";
            cout << "\nComponent graph is shown in image : g_Component_Graph.png\n";
            }
            break;
        case 5:{
            int s,d;
            cout<< "\nEnter Source Node: \n";
            cin>> s ;
            while(cin.fail()) {
               cout << "Enter only Interger !!"  << endl;
               cin.clear();
               cin.ignore(256,'\n');
               cin >> s;
             }
            cout<< "\nEnter target Node: \n";
            cin >> d;
            while(cin.fail()) {
               cout << "Enter only Interger !!"  << std::endl;
               cin.clear();
               cin.ignore(256,'\n');
               cin >> d;
             }
            g.shortestPath(s,d);  
            }  
            break;
        default:
            cout<<"Exiting the program !!!\n";
            return 0;
    }

}
    return 0;
}

//end of assignment !!
