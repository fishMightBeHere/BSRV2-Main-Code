#include "TwoDTree.h"

template<typename T>
T& TwoDTree<T>::get(int x, int y) {
    if (_root == nullptr) {
        return NULL;
    }

    T currentItem = _root;
    uint16_t k = 0;
    while (true) {
        if (currentItem.x == x && currentItem.y == y) {
            return currentItem;
        }

        if (k%2 == 0) {
            if (currentItem.leftNode != nullptr && currentItem.x < x) {
                currentItem = *currentItem.leftNode;
            } else if (currentItem.right != nullptr && currentItem.x >= x) {
                currentItem = *currentItem.right;
            }
            else {
                return NULL;
            }
        } else {
            if (currentItem.leftNode != nullptr && currentItem.y < y) {
                currentItem = *currentItem.leftNode;
            } else if (currentItem.right != nullptr && currentItem.y >= y) {
                currentItem = *currentItem.right;
            } else {
                return NULL;
            }
        }
    }
    
    return NULL;
}

template<typename T>
void TwoDTree<T>::put(T item) {
    if (_root == nullptr) {
        _root = item;
    }

    boolean k = true;
    T compareItem = _root;
    while (true) {
        if (k) {
            if (comparatorX(item,compareItem) == -1 && compareItem.leftNode != nullptr) {
                compareItem == *compareItem.leftNode;
            } else {
                compareItem.leftNode = *item;
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
            if (comparatorY(item,compareItem) == -1 && compareItem.leftNode != nullptr) {
                compareItem == *compareItem.leftNode;
            } else {
                compareItem.leftNode = *item;
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
        k = !k;
    }        
    _putIndex++;
}

template<typename T> 
boolean TwoDTree<T>::contains(int x, int y) {
    return get(x,y) = NULL ? true : false;
}


template<typename T>
inline
int8_t TwoDTree<T>::comparatorX(T i1, T i2) {
    return i1.x < i2.y ? -1 : 1;
}

template<typename T>
inline
int8_t TwoDTree<T>::comparatorY(T i1, T i2) {
    return i1.y < i2.y ? -1 : 1;
}
