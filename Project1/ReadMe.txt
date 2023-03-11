Information about the code:

        | a | d | g |
Matrix  | b | e | h |  is represented by the array, [a, b, c, d, e, f, g, h, i], throughout the code.
        | c | f | i |
Hence, the array is the representation of the node state for the 8 puzzle layout.

Provide the intial state and the final state of the 8 puzzle problem to the variables initial_state and final_state respectively.
They are under the if __name__ == "__main__" condition which is also the entry point to the program. Follow the representation above.

The library used in the code is "copy", specifically deepcopy() function.
I had to deep copy the node state arrays when moving the blank tile to create a new node state. So, I used the library.

Also, the implementation of BFS in the code is done such that once the final state is "created", no more nodes are created or explored.
