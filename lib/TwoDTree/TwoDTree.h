#pragma once
#include <Arduino.h>

template <typename T>
class TwoDTree {
   public:
    explicit TwoDTree(uint16_t maxSize) { _values = (T*)malloc(sizeof(T) * maxSize); }
    ~TwoDTree() { free(_values); }

    T* get(int x, int y);

    void put(T item);

    boolean contains(int x, int y);

   protected:
    uint16_t _maxSize;
    T* _root = nullptr;
    uint16_t _putIndex = 0;
    T* _values;

    inline int8_t comparatorX(T* i1, T* i2);
    inline int8_t comparatorY(T* i1, T* i2);
};

template <typename T>
T* TwoDTree<T>::get(int x, int y) {
    if (_root == nullptr) {
        return NULL;
    }

    T* currentItem = _root;
    bool k = true;
    while (true) {
        if (currentItem->x == x && currentItem->y == y) {
            return currentItem;
        }

        if (k) {
            if (currentItem->leftNode != nullptr && x < currentItem->x) {
                currentItem = currentItem->leftNode;
            } else if (currentItem->rightNode != nullptr && x >= currentItem->x) {
                currentItem = currentItem->rightNode;
            } else {
                return NULL;
            }
        } else {
            if (currentItem->leftNode != nullptr && y < currentItem->y) {
                currentItem = currentItem->leftNode;
            } else if (currentItem->rightNode != nullptr && y >= currentItem->y) {
                currentItem = currentItem->rightNode;
            } else {
                return NULL;
            }
        }
        k = !k;
    }

    return NULL;
}

template <typename T>
void TwoDTree<T>::put(T item) {
    if (_root == nullptr) {
        _values[_putIndex] = item;
        _root = &_values[_putIndex];
    } else {
        boolean k = true;
        T* compareItem = _root;
        while (true) {
            if (k) {
                if (comparatorX(&item, compareItem) == -1) {
                    if (compareItem->leftNode != nullptr) {
                        compareItem = compareItem->leftNode;
                    } else {
                        _values[_putIndex] = item;
                        compareItem->leftNode = &_values[_putIndex];
                        break;
                    }
                } else {
                    if (compareItem->rightNode != nullptr) {
                        compareItem = compareItem->rightNode;
                    } else {
                        _values[_putIndex] = item;
                        compareItem->rightNode = &_values[_putIndex];
                        break;
                    }
                }
            } else {
                if (comparatorY(&item, compareItem) == -1) {
                    if (compareItem->leftNode != nullptr) {
                        compareItem = compareItem->leftNode;
                    } else {
                        _values[_putIndex] = item;
                        compareItem->leftNode = &_values[_putIndex];
                        break;
                    }
                } else {
                    if (compareItem->rightNode != nullptr) {
                        compareItem = compareItem->rightNode;
                    } else {
                        _values[_putIndex] = item;
                        compareItem->rightNode = &_values[_putIndex];
                        break;
                    }
                }
            }
            k = !k;
        }
    }
    _putIndex++;
}

template <typename T>
boolean TwoDTree<T>::contains(int x, int y) {
    return get(x, y) != NULL ? true : false;
}

template <typename T>
inline int8_t TwoDTree<T>::comparatorX(T* i1, T* i2) {
    return i1->x < i2->x ? -1 : 1;
}

template <typename T>
inline int8_t TwoDTree<T>::comparatorY(T* i1, T* i2) {
    return i1->y < i2->y ? -1 : 1;
}
