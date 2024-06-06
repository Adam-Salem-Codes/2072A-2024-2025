
#include <vector>
#include "lemlib/api.hpp"


namespace util {

    float radToDeg(float rad);

    float degToRad(float deg);

    float avg(std::vector<int> values);

    float dist(float x1, float y1, float x2, float y2);

    lemlib::Pose dist(lemlib::Pose p1, lemlib::Pose p2);

    lemlib::Pose angleBetweenPoints(lemlib::Pose p1, lemlib::Pose p2);
};