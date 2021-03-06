So you will implement two algorithms. For both, the configuration space is assumed to be bounded  
by a 300 × 200 rectangle, as given in the first line of the input file, in counterclockwise orientation:  
(0, 0), (300, 0), (300, 200), (0, 200)  
This boundary will not change. Every other line of the input file encodes a simply connected  
polygon in counterclockwise orientation. A full input file that you can use for testing is as follows,  
which contains 3 obstacles (save the lines into a file for your testing). The last line has two points  
representing the start and goal configurations.  
(0, 0), (300, 0), (300, 200), (0, 200)  
(33, 40), (67, 55), (75, 109), (128, 97), (84, 149), (44, 94)  
(189, 102), (250, 165), (158, 132)  
(180, 23), (274, 46), (225, 97), (212, 61), (148, 83)  
(100, 90), (290, 60).  
The figure below shows the environment and the configurations x I and x G   

As the output, for each input file, each of your algorithms will output two items: a graph and the  
solution path. The graph should be written down as a list of vertices with integer IDs, and a list  
of edges based on these IDs, e.g., the following is a triangle graph  
1 : (20, 35), 2 : (55, 80), 3 : (40, 100)  
(1, 2), (1, 3), (2, 3)  
And the path should be written out as a list of vertices. The graph and the output paths should be  
written as three lines in an output file.  
