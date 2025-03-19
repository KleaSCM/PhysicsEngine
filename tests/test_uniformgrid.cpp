// #include <iostream>
// #include <cassert>
// #include <vector>
// #include "UniformGridBroadPhase.h"
// #include "RigidBody.h"

// int main() {
//     // Create three rigid bodies with known positions.
//     // Using cellSize = 2.0f, bodies with positions within [0,2) will go to cell (0,0,0),
//     // while a body at position (3,0,0) will be in cell (1,0,0).
//     RigidBody body1, body2, body3;
//     body1.position = Vector3(0.0f, 0.0f, 0.0f);
//     body2.position = Vector3(1.0f, 0.0f, 0.0f);
//     body3.position = Vector3(3.0f, 0.0f, 0.0f);

//     // For broad-phase, we don't need to set mass or other properties for this test.
//     // (Assume they're all active dynamic bodies.)

//     // Create a vector of pointers to the bodies.
//     std::vector<RigidBody*> bodies;
//     bodies.push_back(&body1);
//     bodies.push_back(&body2);
//     bodies.push_back(&body3);

//     // Instantiate the UniformGridBroadPhase with a cell size of 2.0f.
//     UniformGridBroadPhase grid(2.0f);

//     // Update the grid with the current bodies.
//     grid.Update(bodies);

//     // Retrieve the potential collision pairs from the grid.
//     std::vector<std::pair<RigidBody*, RigidBody*>> potentialPairs = grid.GetPotentialPairs();

//     // Expected:
//     // - body1 and body2 should be in the same cell (cell (0,0,0)).
//     // - body3 is in cell (1,0,0).
//     // So potential pairs should include:
//     //   (body1, body2) from the same cell, and
//     //   (body1, body3) and (body2, body3) from neighbor cell interactions.
//     // In total, we expect 3 potential pairs.
//     std::cout << "Number of potential pairs: " << potentialPairs.size() << std::endl;
//     assert(potentialPairs.size() == 3);

//     // Verify that each expected pair is present.
//     bool found12 = false, found13 = false, found23 = false;
//     for (const auto& pair : potentialPairs) {
//         RigidBody* a = pair.first;
//         RigidBody* b = pair.second;
//         if ((a == &body1 && b == &body2) || (a == &body2 && b == &body1)) {
//             found12 = true;
//         }
//         if ((a == &body1 && b == &body3) || (a == &body3 && b == &body1)) {
//             found13 = true;
//         }
//         if ((a == &body2 && b == &body3) || (a == &body3 && b == &body2)) {
//             found23 = true;
//         }
//     }
//     assert(found12 && found13 && found23);

//     std::cout << "UniformGridBroadPhase test passed!\n";
//     return 0;
// }
