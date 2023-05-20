#pragma once
#include "Arduino.h"

template <typename T>
class Dequeue {
   public:
    explicit Dequeue(uint16_t maxSize) {
        _values = (T*)malloc(sizeof(T) * maxSize);
        _front = -1;
        _rear = 0;
        _size = maxSize;
    }
    ~Dequeue() { free(_values); }

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

template <typename T>
bool Dequeue<T>::add(T e) {
    if (isFull()) return false;

    if (isEmpty()) {
        _front = 0;
        _rear = 0;
    } else if (_rear == _size - 1)
        _rear = 0;
    else
        _rear++;
    _values[_rear] = e;

    return true;
}

template <typename T>
bool Dequeue<T>::push(T e) {
    if (isFull()) return false;

    if (isEmpty()) {
        _front = 0;
        _rear = 0;
    } else if (_front == 0)
        _front = _size - 1;
    else
        _front--;
    _values[_front] = e;

    return true;
}

template <typename T>
T* Dequeue<T>::peek() {
    return &_values[_front];
}

template <typename T>
T* Dequeue<T>::peekLast() {
    return &_values[_rear];
}

template <typename T>
T* Dequeue<T>::remove() {
    if (isEmpty()) return NULL;

    if (_rear == 0)
        _rear = _size - 1;
    else
        _rear--;

    if (isFull()) _front = -1;

    if (_rear == _size - 1)
        return &_values[0];
    else
        return &_values[_rear + 1];
}

template <typename T>
T* Dequeue<T>::pop() {
    if (isEmpty()) return NULL;

    if (_front == _size - 1)
        _front = 0;
    else
        _front++;

    if (isFull()) _rear = -1;

    if (_front == 0)
        return &_values[_size - 1];
    else
        return &_values[_front - 1];
}

template <typename T>
uint16_t Dequeue<T>::size() {
    if (isEmpty())
        return 0;
    else if (_rear >= _front)
        return _rear - _front + 1;
    else
        return _rear + 1 + _size - _front;
}

template <typename T>
bool Dequeue<T>::isEmpty() {
    return (_front == -1 || _rear == -1);
}

template <typename T>
bool Dequeue<T>::isFull() {
    return ((_front == 0 && _rear == _size - 1) || _front == _rear + 1);
}

template <typename T>
void Dequeue<T>::clear() {
    _front = -1;
    _rear = -1;
}