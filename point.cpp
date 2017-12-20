#include "point.h"

Point::Point() {

}

Point::Point(float x1 = 0, float y1 = 0) {
    x = x1;
    y = y1;
}

void Point::setx(float x1) {
    x = x1;
}

void Point::sety(float y1) {
    y = y1;
}
