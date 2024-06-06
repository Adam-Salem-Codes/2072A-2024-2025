// Adam Salem - Adam-Salem-Codes@github - 2024-2025

#include "util.h"

namespace util
{
    /**
     * Converts radians to degrees.
     *
     * @param rad The radians to convert.
     *
     * @return The converted degrees.
     *
     * @throws None
     */
    float radToDeg(float rad)
    {
        return rad * 180 / M_PI;
    }

    /**
     * Converts degrees to radians.
     *
     * @param deg The degrees to convert.
     *
     * @return The converted radians.
     *
     * @throws None
     */
    float degToRad(float deg)
    {
        return deg * M_PI / 180;
    }

    /**
     * Calculates the average of a vector of integers.
     *
     * @param values The vector of integers to calculate the average of.
     *
     * @return The average of the vector of integers.
     *
     * @throws None
     */
    float avg(std::vector<int> values)
    {
        int sum = 0;
        for (int i = 0; i < values.size(); i++)
        {
            sum += values[i];
        }
        return sum / values.size();
    }
    /**
     * Calculates the Euclidean distance between two points in a 2D plane.
     *
     * @param x1 the x-coordinate of the first point
     * @param y1 the y-coordinate of the first point
     * @param x2 the x-coordinate of the second point
     * @param y2 the y-coordinate of the second point
     *
     * @return the distance between the two points
     *
     * @throws None
     */
    float dist(float x1, float y1, float x2, float y2)
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    /**
     * Calculate the distance between two poses in 2D space.
     *
     * @param p1 the first pose
     * @param p2 the second pose
     *
     * @return a new pose representing the distance between the two input poses
     *
     * @throws None
     */
    lemlib::Pose dist(lemlib::Pose p1, lemlib::Pose p2)
    {
        return lemlib::Pose(p1.x - p2.x, p1.y - p2.y, 0);
    }

    /**
     * Calculate the angle between two points in 2D space.
     *
     * @param p1 the first pose
     * @param p2 the second pose
     *
     * @return a new pose representing the angle between the two input poses
     *
     * @throws None
     */
    float angleBetweenPoints(lemlib::Pose p1, lemlib::Pose p2)
    {
        return atan2(p2.y - p1.y, p2.x - p1.x);
    }

}