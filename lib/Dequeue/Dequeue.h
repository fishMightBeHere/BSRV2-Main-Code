#pragma once
#include "Arduino.h"

template<typename T> 
class Dequeue {
    public:
        explicit Dequeue(uint16_t maxSize) {
            _values = (T *) malloc(sizeof(T) * maxSize);
            _front = -1;
            _rear = 0;
            _size = maxSize;
        }
        ~Dequeue() {
            free(_values);
        }

        bool add(T e);

        bool push(T e);

        T* peek();        
        
        T* peekLast();

        T* remove();

        T* pop();

        uint16_t size();

        bool isEmpty();
        
        bool isFull();

        void clear();

    protected:
        T* _values;
        int _size;
        int _front;
        int _rear;
};