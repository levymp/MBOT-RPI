#include <planning/priority.hpp>
#include <iostream>

int main()
{
    // start up a list
    std::vector<Grid_Astar> stored_nodes;
    
    // add two elements
    stored_nodes.push_back(Grid_Astar());
    stored_nodes.push_back(Grid_Astar());

    // Check initialization works
    std::cout << "TESTING INITIALIZATION VALUES" << std::endl;
    print_struct(stored_nodes[0]);
    
    // Check changing values
    std::cout << "CHANING VALUES" << std::endl;
    Grid_Astar& tmp = stored_nodes[0];
    tmp.cell_pos = Point<int>(1, 1);
    tmp.distance = 540;
    tmp.priority = 1.2;
    tmp.visited = true;
    tmp.parent = &stored_nodes[1];
    print_struct(stored_nodes[0]);
    
    // Check parent storage works
    std::cout << "PRINTING PARENT INFO" << std::endl;
    print_struct(*stored_nodes[0].parent);

    // add some values to the stored nodes (starting at third index)
    for(int i= stored_nodes.size();i < 6; i++)
    {
        std::cout << i << std::endl;
        stored_nodes.push_back(Grid_Astar());
        stored_nodes[i].priority =(float) -i * 100.0;
    }
    std::cout << "SIZE: " <<stored_nodes.size() << std::endl; 

    // test out min heap
    std::vector<Grid_Astar*> visit_q;
    
    
    for(int i = 0; i < (int)stored_nodes.size(); i++)
    {
        visit_q.push_back(&stored_nodes[i]);
    }

    // make heap
    std::make_heap(visit_q.begin(), visit_q.end(), compare_priority());
    
    std::cout << "TESTING PRIORITY MIN HEAP" << std::endl;
    for(int i= 0; i < (int)visit_q.size(); i++)
    {
        std::cout << visit_q[i]->priority << '\t' << visit_q[i]->distance << '\t' << i << std::endl;
    }

    // pop heap (moves min value to the back)
    std::pop_heap(visit_q.begin(), visit_q.end(), compare_priority());

    std::cout << "AFTER POP HEAP" << std::endl; 
    for(int i= 0; i < (int)visit_q.size(); i++)
    {
        std::cout << visit_q[i]->priority << '\t' << visit_q[i]->distance << '\t' << i << std::endl; 
    }

    Grid_Astar* min = visit_q.back();
    // printing min value
    std::cout << "PRINTING MIN VALUE:" << std::endl; 
    print_struct(*min);

    // pop min (back) value
    visit_q.pop_back();
    
    while(!visit_q.empty())
    {
        Grid_Astar* min = dequeue(visit_q);
        std::cout << min -> priority << std::endl;
    }

    std::cout << "TESTING ENQUEUE AND DEQUEUE TOGETHER" << std::endl; 

    static const float arr[] = {5.2, 9.8, -3, 50, 64.5, 10004, 50 };
    std::vector <float> priorities (arr, arr + sizeof(arr)/sizeof(arr[0]));
    
    for(int i=0; i < (int)priorities.size(); i++)
    {
        std::cout << priorities[i] << "\t";
    }
    std::cout << std::endl;


    // starting index for stored_nodes
    int m = stored_nodes.size();
    for(int i =0; i < (int)priorities.size(); i++ )
    {
        stored_nodes.push_back(Grid_Astar());
        stored_nodes[m].priority = priorities[i];
        enqueue(visit_q, &stored_nodes[m]);
        m++;
    }

    // print out all values
    while(!visit_q.empty())
    {
        Grid_Astar* min = dequeue(visit_q);
        std::cout << min -> priority << std::endl;
    }

    return 0;
}


