#pragma once
#include "Arduino.h"

template<typename T>
class TwoDTree {
    public: 
        explicit TwoDTree(const uint16_t maxSize) {
            _values = (T *) malloc(sizeof(T) * maxSize);
        }
        ~TwoDTree() {
            free(_values);
        }

        T* get(int x, int y);

        void put(T item);

        boolean contains(int x, int y);

    protected:
        uint16_t _maxSize;
        uint16_t _amount;
        T* _root;
        uint16_t _putIndex;
        T* _values;

        int8_t comparatorX(T i1, T i2);
        int8_t comparatorY(T i1, T i2);

};