#include "Arduino.h"

template<typename T>
class TwoDTree {
    public: 
        explicit TwoDTree(const uint16_t maxSize);
        ~TwoDTree();

        T& get(uint16_t x, uint16_t y);

        void put(T item);

    private:

    protected:
        uint16_t _maxSize;
        uint16_t _amount;
        T* _root;
        uint16_t _putIndex;
        T _values;

        int8_t comparatorX(T i1, T i2);
        int8_t comparatorY(T i1, T i2);



};