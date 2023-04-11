#include "TwoDTree.h"
template<typename T>
TwoDTree<T>::TwoDTree(const uint16_t maxSize) {
    _values = (T *) malloc(sizeof(T) * maxSize);
}

template<typename T>
TwoDTree<T>::~TwoDTree() {
    free(_values);
}

template<typename T>
T& TwoDTree<T>::get(uint16_t x, uint16_t y) {
    if (_root == nullptr) {
        throw "Tree is empty, root dne";
    }

    T currentItem = _root;
    uint16_t k = 0;
    while (true) {
        if (currentItem.x == x && currentItem.y == y) {
            return currentItem;
        }

        if (k%2 == 0) {
            if (currentItem.left != nullptr && currentItem.x < x) {
                currentItem = *currentItem.left;
            } else if (currentItem.right != nullptr && currentItem.x >= x) {
                currentItem = *currentItem.right;
            }
            else {
                throw "point does not exist";
            }
        } else {
            if (currentItem.left != nullptr && currentItem.y < y) {
                currentItem = *currentItem.left;
            } else if (currentItem.right != nullptr && currentItem.y >= y) {
                currentItem = *currentItem.right;
            } else {
                throw "point does not exist";
            }
        }
    }
    
    throw "Point does not exist";
}

template<typename T>
void TwoDTree<T>::put(T item) {
    if (_root == nullptr) {
        _root = item;
    }

    uint16_t k = 0;
    T compareItem = _root;
    while (true) {
        if (k%2 == 0) {
            if (comparatorX(item,compareItem) == -1 && compareItem.left != nullptr) {
                compareItem == *compareItem.left;
            } else {
                compareItem.left = *item;
                _values[_putIndex] = item;
                break;
            }

            if (comparatorX(item, compareItem) != -1 && compareItem.right != nullptr) {
                compareItem == *compareItem.right;
            } else {
                compareItem.right = *item;
                _values[_putIndex] = item;
                break;
            }
        } else {
            if (comparatorY(item,compareItem) == -1 && compareItem.left != nullptr) {
                compareItem == *compareItem.left;
            } else {
                compareItem.left = *item;
                _values[_putIndex] = item;
                break;
            }

            if (comparatorY(item,compareItem) != -1 && compareItem.right != nullptr) {
                compareItem == *compareItem.right;
            } else {
                compareItem.right = *item;
                _values[_putIndex] = item;
                break;
            }
        }
        k++;
    }        
    _putIndex++;
}

template<typename T>
inline
int8_t TwoDTree<T>::comparatorX(T i1, T i2) {
    if (i1.x < i2.x) {
        return -1;
    } 
    return 1;
}

template<typename T>
inline
int8_t TwoDTree<T>::comparatorY(T i1, T i2) {
    if (i1.y < i2.y) {
        return -1;
    } 
    return 1;
}
