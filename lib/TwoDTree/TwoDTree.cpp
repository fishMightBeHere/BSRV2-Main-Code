/*
#include "TwoDTree.h"

template <typename T>
T* TwoDTree<T>::get(int x, int y) {
    if (_root == nullptr) {
        return NULL;
    }

    T* currentItem = &_root;
    bool k = false;
    while (true) {
        if (currentItem->x == x && currentItem->y == y) {
            return currentItem;
        }

        if (k) {
            if (currentItem->leftNode != nullptr && currentItem->x < x) {
                currentItem = currentItem->leftNode;
            } else if (currentItem->rightNode != nullptr && currentItem->x >= x) {
                currentItem = currentItem->rightNode;
            } else {
                return NULL;
            }
        } else {
            if (currentItem->leftNode != nullptr && currentItem->y < y) {
                currentItem = currentItem->leftNode;
            } else if (currentItem->rightNode != nullptr && currentItem->y >= y) {
                currentItem = currentItem->right;
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
    if (_root == NULL) {
        _root = item;
    }

    boolean k = true;
    T* compareItem = &_root;
    while (true) {
        if (k) {
            if (comparatorX(&item, compareItem) == -1 && compareItem->leftNode != nullptr) {
                compareItem = compareItem->leftNode;
            } else {
                _values[_putIndex] = item;
                compareItem->leftNode = &_values[_putIndex];  // item in parameter is stored in stack which will be destroyed, we need the pointer towards the permanent storage in _values
                break; 
            }

            if (comparatorX(&item, compareItem) != -1 && compareItem->rightNode != nullptr) {
                compareItem = compareItem->rightNode;
            } else {
                _values[_putIndex] = item;
                compareItem->rightNode = &_values[_putIndex];
                break;
            }
        } else {
            if (comparatorY(&item, compareItem) == -1 && compareItem->leftNode != nullptr) {
                compareItem = compareItem->leftNode;
            } else {
                _values[_putIndex] = item;
                compareItem->leftNode = &_values[_putIndex];
                break;
            }

            if (comparatorY(&item, compareItem) != -1 && compareItem->rightNode != nullptr) {
                compareItem = compareItem->rightNode;
            } else {
                _values[_putIndex] = item;
                compareItem->rightNode = &_values[_putIndex];
                break;
            }
        }
        k = !k;
    }
    _putIndex++;
}

template <typename T>
boolean TwoDTree<T>::contains(int x, int y) {
    return get(x, y) = NULL ? true : false;
}

template <typename T>
int8_t TwoDTree<T>::comparatorX(T* i1, T* i2) {
    return i1->x < i2->y ? -1 : 1;
}

template <typename T>
int8_t TwoDTree<T>::comparatorY(T* i1, T* i2) {
    return i1->y < i2->y ? -1 : 1;
}*/