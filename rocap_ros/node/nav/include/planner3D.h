
#ifndef PLANNER_H
#define PLANNER_H

class Coord {
    public:
        Coord(float x = 0, float y = 0, float z = 0) {
            this->x = x;
            this->y = y;
            this->z = z;
        }

        float getX() const {return x;}
        float getY() const {return y;}
        float getZ() const {return z;}

    bool operator==(const Coord&  b) const {
            return this->getX() == b.getX() &&
                   this->getY() == b.getY() &&
                   this->getZ() == b.getZ();
    }

    struct hash {
      size_t operator()(const Coord& key) const{
        // a simple hashing function 
        // explicit casts to size_t to operate on the complete range
        // constanst will be promoted according to C++ standard
        return static_cast<size_t>(key.getX())
          + 1447*static_cast<size_t>(key.getY())
          + 345637*static_cast<size_t>(key.getZ());
      }
    };

    private:
        float x;
        float y;
        float z;
};

#endif