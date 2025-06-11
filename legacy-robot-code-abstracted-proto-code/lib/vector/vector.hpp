#ifndef _VECTOR_HPP_
#define _VECTOR_HPP_

#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <string>

class Vector {
    public:
    float i;
    float j;

    Vector(float new_i=0, float new_j=0);

    std::string display();
    float magnitude();
    float heading();
    Vector unit();
    float dot(Vector vec2);
    Vector scale(float scalar);

    /* heading in radians */
    static Vector from_heading(float heading, float magnitude) {
        return Vector(cos(heading)*magnitude, sin(heading)*magnitude);
    };
    static Vector sum_vectors(std::vector<Vector> vec_list) {
        float new_i = 0, new_j = 0;
        for (Vector vec : vec_list) {
            new_i += vec.i;
            new_j += vec.j;
        }
        return Vector(new_i, new_j);
    };
};

#endif //VECTOR_H